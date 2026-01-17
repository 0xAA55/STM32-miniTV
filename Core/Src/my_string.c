/*
 * my_string.c
 *
 *  Created on: Jun 1, 2025
 *      Author: 0xaa55
 *  Last modified: Jun 16, 2026
 */

#include "my_string.h"
#include <inttypes.h>

#define MEMOPS_BM (MEMOPS_GRANULARITY - 1)

#if MEMOPS_GRANULARITY != 1

// https://gcc.gnu.org/onlinedocs/gcc/Function-Specific-Option-Pragmas.html
// https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html#index-ftree-loop-distribute-patterns
// https://stackoverflow.com/questions/46996893/gcc-replaces-loops-with-memcpy-and-memset

#pragma GCC optimize ("no-tree-loop-distribute-patterns")
#pragma GCC optimize ("O3")

void *memset(void * dst, int val, size_t len)
{
  memops_t *ptr_dst = dst;
  size_t head = (size_t)ptr_dst & MEMOPS_BM;
  if (head)
  {
    volatile uint8_t *ptr_dst_ = (volatile uint8_t *)ptr_dst;
    if (head > len) head = len;
    len -= head;
    while(head)
    {
      *ptr_dst_ ++ = (uint8_t) val;
      head --;
    }
    ptr_dst = (memops_t *)ptr_dst_;
  }
  if (len >= MEMOPS_GRANULARITY && ((size_t)ptr_dst & MEMOPS_BM) == 0)
  {
    union {
      uint8_t u8[MEMOPS_GRANULARITY];
      memops_t uops;
    } vops;
    for (int i = 0; i < MEMOPS_GRANULARITY; i++) vops.u8[i] = (uint8_t) val;
    while (len >= MEMOPS_GRANULARITY)
    {
      *ptr_dst ++ = vops.uops;
      len -= MEMOPS_GRANULARITY;
    }
  }
  volatile uint8_t *ptr_dst_ = (volatile uint8_t *)ptr_dst;
  while (len)
  {
    *ptr_dst_ ++ = (uint8_t) val;
    len --;
  }
  return dst;
}

void *memcpy(void * dst, const void * src, size_t len)
{
  memops_t *ptr_dst = dst;
  const memops_t *ptr_src = src;
  if (dst == src) return dst;
  size_t head = (size_t)dst & MEMOPS_BM;
  if (head)
  {
    volatile uint8_t *ptr_dst_ = (volatile uint8_t *)ptr_dst;
    volatile uint8_t *ptr_src_ = (volatile uint8_t *)ptr_src;
    while(head && len)
    {
      *ptr_dst_++ = *ptr_src_++;
      head --;
      len --;
    }
    ptr_dst = (memops_t *)ptr_dst_;
    ptr_src = (memops_t *)ptr_src_;
  }
  if (((size_t)ptr_src & MEMOPS_BM) == 0 && ((size_t)ptr_dst & MEMOPS_BM) == 0)
  {
    while (len >= MEMOPS_GRANULARITY)
    {
      *ptr_dst++ = *ptr_src++;
      len -= MEMOPS_GRANULARITY;
    }
  }
  if (len)
  {
    volatile uint8_t *ptr_dst_ = (volatile uint8_t *)ptr_dst;
    volatile uint8_t *ptr_src_ = (volatile uint8_t *)ptr_src;

    while (len)
    {
      *ptr_dst_++ = *ptr_src_++;
      len --;
    }
  }
  return dst;
}

void *memmove(void * dst, const void * src, size_t len)
{
  if (dst == src) return dst;
  if (dst < src)
  {
    return memcpy(dst, src, len);
  }
  else
  {
    memops_t *ptr_dst_end = (memops_t *)((uint8_t*)dst + len);
    memops_t *ptr_src_end = (memops_t *)((uint8_t*)src + len);
    size_t tail = (size_t)ptr_dst_end & MEMOPS_BM;
    if (tail)
    {
      volatile uint8_t *ptr_dst_end_ = (volatile uint8_t *)ptr_dst_end;
      volatile uint8_t *ptr_src_end_ = (volatile uint8_t *)ptr_src_end;
      while (tail && len)
      {
        *--ptr_dst_end_ = *--ptr_src_end_;
        tail --;
        len --;
      }
      ptr_dst_end = (memops_t *)ptr_dst_end_;
      ptr_src_end = (memops_t *)ptr_src_end_;
    }
    if (((size_t)ptr_src_end & MEMOPS_BM) == 0 && ((size_t)ptr_dst_end & MEMOPS_BM) == 0)
    {
      while (len >= MEMOPS_GRANULARITY)
      {
        *--ptr_dst_end = *--ptr_src_end;
        len -= MEMOPS_GRANULARITY;
      }
    }
    if (len)
    {
      volatile uint8_t *ptr_dst_end_ = (volatile uint8_t *)ptr_dst_end;
      volatile uint8_t *ptr_src_end_ = (volatile uint8_t *)ptr_src_end;
      while (len)
      {
        *--ptr_dst_end_ = *--ptr_src_end_;
        len --;
      }
    }
  }
  return dst;
}

#endif
