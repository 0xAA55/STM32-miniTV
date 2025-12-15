
#ifndef INC_DYNALLOC_H_
#define INC_DYNALLOC_H_ 1

#ifndef ALLOC_MEMORY_ADDRESS
#define ALLOC_MEMORY_ADDRESS 0x30000000
#define ALLOC_MEMORY_SIZE ((128 + 128 + 32) * 1024)
#define ALLOC_MEMORY_ALIGNMENT 8
#endif

#include <stddef.h>

// This header file provides `__disable_irq()` and `__enable_irq()`
#include "stm32h7xx_hal.h"

extern void Error_Handler();

void DynAlloc_Init();
void *DynAlloc_Alloc(size_t size);
void *DynAlloc_Realloc(void *user_ptr, size_t new_size);
void DynAlloc_Free(void *addr);

#endif /* INC_DYNALLOC_H_ */
