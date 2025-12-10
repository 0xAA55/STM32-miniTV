/*
 * fastmath.h
 *
 *  Created on: 2025年12月10日
 *      Author: 0xaa55
 */

#ifndef INC_FASTMATH_H_
#define INC_FASTMATH_H_

#include <stdint.h>

// A period is 1024, the return value is in a closed range [1024, -1024]
int FastSin(int x);
int FastCos(int x);
int FastSqrt(int x);

#endif /* INC_FASTMATH_H_ */
