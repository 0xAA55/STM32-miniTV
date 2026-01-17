/*
 * my_string.h
 *
 *  Created on: Jun 1, 2025
 *      Author: 0xaa55
 */

#ifndef SRC_MY_STRING_H_
#define SRC_MY_STRING_H_

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifndef MEMOPS_GRANULARITY
#define MEMOPS_GRANULARITY 8
#endif

#if MEMOPS_GRANULARITY == 1
typedef uint8_t memops_t;
#elif MEMOPS_GRANULARITY == 2
typedef uint16_t memops_t;
#elif MEMOPS_GRANULARITY == 4
typedef uint32_t memops_t;
#elif MEMOPS_GRANULARITY == 8
typedef uint64_t memops_t;
#else
#error MEMOPS_GRANULARITY must be one of 1, 2, 4, 8
#endif

#ifndef MEMOPS_FUNC
#define MEMOPS_FUNC
#endif

#endif /* SRC_MY_STRING_H_ */
