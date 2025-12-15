/*
 * qspixip.h
 *
 *  Created on: 2025年12月13日
 *      Author: 0xaa55
 */

#ifndef INC_QSPIXIP_H_
#define INC_QSPIXIP_H_

#include "stm32h7xx_hal.h"

extern QSPI_HandleTypeDef hqspi;

HAL_StatusTypeDef QSPI_InitFlash(void);
HAL_StatusTypeDef QSPI_EnterMemoryMapMode(void);
HAL_StatusTypeDef QSPI_ExitMemoryMapMode(void);

#endif /* INC_QSPIXIP_H_ */
