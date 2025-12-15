
#include "dynalloc.h"

#include <stdint.h>
#include <string.h>

#define ALLOC_MEMORY_ADDRESS_END (ALLOC_MEMORY_ADDRESS) + (ALLOC_MEMORY_SIZE)
#define ALLOC_MEMORY_ADDRESS_MAX ((ALLOC_MEMORY_ADDRESS_END) - 1)

#if ALLOC_MEMORY_SIZE > 0x7FFFFFFF
typedef uint64_t asize_t;
static const uint64_t SizeMask = 0x7fffffffffffffff;
static const uint64_t UsageMask = 0x8000000000000000;
#define ASIZE_LEN 8
#elif ALLOC_MEMORY_SIZE > 0x7FFF
typedef uint32_t asize_t;
static const uint32_t SizeMask = 0x7fffffff;
static const uint32_t UsageMask = 0x80000000;
#define ASIZE_LEN 4
#elif ALLOC_MEMORY_SIZE > 0x7F
typedef uint16_t asize_t;
static const uint16_t SizeMask = 0x7fff;
static const uint16_t UsageMask = 0x8000;
#define ASIZE_LEN 2
#else
typedef uint8_t asize_t;
static const uint8_t SizeMask = 0x7f;
static const uint8_t UsageMask = 0x80;
#define ASIZE_LEN 1
#endif

#define ALIGN_UP(x) (((x) - 1) / (ALLOC_MEMORY_ALIGNMENT) + 1) * (ALLOC_MEMORY_ALIGNMENT)

#pragma pack(push, 1)
typedef struct MemBlockTag_s
{
    // Block status
    asize_t block_state;

    // paddings
#if (ALLOC_MEMORY_ALIGNMENT) - ASIZE_LEN > 0
    uint8_t _pad[(ALLOC_MEMORY_ALIGNMENT) - ASIZE_LEN];
#endif

    // The pointer to the allocated data
    uint8_t data[0];
}MemBlockTag_t;
#pragma pack(pop)

void HeapCorrupted()
{
    Error_Handler();
}

void DoubleFree()
{
    Error_Handler();
}

void BadFree()
{
    Error_Handler();
}

void BadReallocPtr()
{
    Error_Handler();
}

static size_t GetBlockSize(MemBlockTag_t *b)
{
    return b->block_state & SizeMask;
}

static void SetBlockSize(MemBlockTag_t *b, size_t size)
{
    b->block_state &= ~SizeMask;
    b->block_state |= size;
}

static int GetIsInuse(MemBlockTag_t *b)
{
    return b->block_state & UsageMask;
}

static void SetIsInuse(MemBlockTag_t *b, int val)
{
    if (val)
        b->block_state |= UsageMask;
    else
        b->block_state &= ~UsageMask;
}

static void SetBlockState(MemBlockTag_t *b, size_t size, int is_inuse)
{
    SetBlockSize(b, size);
    SetIsInuse(b, is_inuse);
}

static MemBlockTag_t *GetBlockTagFromUserPtr(void *ptr)
{
    MemBlockTag_t *ret = (MemBlockTag_t *)((size_t)ptr - (sizeof * ret));
    if ((size_t)ret + GetBlockSize(ret) >= ALLOC_MEMORY_ADDRESS_END) HeapCorrupted();
    return ret;
}

static void *GetUserPtr(MemBlockTag_t *ptr)
{
    return (void*)&ptr->data[0];
}

static MemBlockTag_t *NextBlock(MemBlockTag_t *cur)
{
    return (MemBlockTag_t *)((size_t)cur + GetBlockSize(cur));
}

static size_t GetContinuousFreeSize(MemBlockTag_t *ptr, size_t target_size)
{
    size_t sum = 0;
    while (!GetIsInuse(ptr))
    {
        sum += GetBlockSize(ptr);
        ptr = NextBlock(ptr);
        if ((size_t)ptr >= ALLOC_MEMORY_ADDRESS_END) break;
        if (sum >= target_size) break;
    }
    return sum;
}

static void MakeFreeSpace(MemBlockTag_t *ptr, size_t size)
{
    SetBlockState(ptr, size, 0);
}

void DynAlloc_Init()
{
    MemBlockTag_t *ptr = (MemBlockTag_t *)ALLOC_MEMORY_ADDRESS;
    __disable_irq();
    MakeFreeSpace(ptr, ALLOC_MEMORY_SIZE);
    __enable_irq();
}

void *DynAlloc_Alloc(size_t size)
{
    MemBlockTag_t *ptr = (MemBlockTag_t *)ALLOC_MEMORY_ADDRESS;
    size_t block_size_to_alloc = ALIGN_UP(size + (sizeof * ptr));
    __disable_irq();
    while (1)
    {
        size_t block_size = GetBlockSize(ptr);
        if (GetIsInuse(ptr))
        {
            ptr = NextBlock(ptr);
            if ((size_t)ptr >= ALLOC_MEMORY_ADDRESS_MAX)
            {
              __enable_irq();
              return NULL;
            }
        }
        else
        {
            size_t free_space = GetContinuousFreeSize(ptr, block_size_to_alloc);
            if (free_space < block_size_to_alloc)
            {
                MakeFreeSpace(ptr, free_space);
                ptr = NextBlock(ptr);
                continue;
            }
            else
            {
                MemBlockTag_t *ret = ptr;
                size_t next_free_space = free_space - block_size_to_alloc;
                if (next_free_space <= sizeof * ptr)
                    block_size_to_alloc = free_space;
                SetBlockState(ret, block_size_to_alloc, 1);
                if (block_size == block_size_to_alloc) return GetUserPtr(ret);
                ptr = NextBlock(ret);
                MakeFreeSpace(ptr, free_space - block_size_to_alloc);
                __enable_irq();
                return GetUserPtr(ret);
            }
        }
    }
}

void *DynAlloc_Realloc(void *user_ptr, size_t new_size)
{
    MemBlockTag_t *ptr;
    size_t block_size_to_alloc;
    size_t block_size;
    size_t avail_space;
    if (user_ptr == NULL) return DynAlloc_Alloc(new_size);
    if (!new_size)
    {
        DynAlloc_Free(user_ptr);
        return NULL;
    }
    if ((size_t)user_ptr < ALLOC_MEMORY_ADDRESS || (size_t)user_ptr > ALLOC_MEMORY_ADDRESS_MAX) BadReallocPtr();
    __disable_irq();
    ptr = GetBlockTagFromUserPtr(user_ptr);
    block_size = GetBlockSize(ptr);
    block_size_to_alloc = ALIGN_UP(new_size + sizeof * ptr);
    if (block_size_to_alloc == block_size)
    {
        __enable_irq();
        return user_ptr;
    }
    if (block_size_to_alloc > block_size)
        avail_space = block_size + GetContinuousFreeSize(NextBlock(ptr), block_size_to_alloc - block_size);
    else
        avail_space = block_size;
    if (avail_space >= block_size_to_alloc)
    {
        size_t tail_size = avail_space - block_size_to_alloc;
        if (tail_size > sizeof * ptr)
        {
            SetBlockSize(ptr, block_size_to_alloc);
            ptr = NextBlock(ptr);
            MakeFreeSpace(ptr, tail_size);
        }
        __enable_irq();
        return user_ptr;
    }
    else
    {
        __enable_irq();
        void *new_addr = DynAlloc_Alloc(new_size);
        if (!new_addr) return NULL;
        memcpy(new_addr, user_ptr, GetBlockSize(ptr));
        DynAlloc_Free(user_ptr);
        return new_addr;
    }
}

void DynAlloc_Free(void *addr)
{
    MemBlockTag_t *ptr;
    MemBlockTag_t *next_ptr;
    if (!addr) return;
    if ((size_t)addr < ALLOC_MEMORY_ADDRESS || (size_t)addr > ALLOC_MEMORY_ADDRESS_MAX)
    {
      BadFree();
      return;
    }
    __disable_irq();
    ptr = GetBlockTagFromUserPtr(addr);
    if (!(GetIsInuse(ptr)))
    {
      __enable_irq();
      DoubleFree();
      return;
    }
    SetIsInuse(ptr, 0);
    next_ptr = NextBlock(ptr);
    if ((size_t)next_ptr <= ALLOC_MEMORY_ADDRESS_MAX && !GetIsInuse(next_ptr))
    {
        size_t free_space = GetBlockSize(ptr) + GetBlockSize(next_ptr);
        SetBlockSize(ptr, free_space);
    }
    __enable_irq();
}
