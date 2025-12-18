/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_PWCTRL_Pin GPIO_PIN_13
#define LCD_PWCTRL_GPIO_Port GPIOC
#define LCD_SDO_Pin GPIO_PIN_6
#define LCD_SDO_GPIO_Port GPIOA
#define LCD_SDA_Pin GPIO_PIN_7
#define LCD_SDA_GPIO_Port GPIOA
#define BAT_PERC_Pin GPIO_PIN_4
#define BAT_PERC_GPIO_Port GPIOC
#define PWCTRL_Pin GPIO_PIN_9
#define PWCTRL_GPIO_Port GPIOE
#define ENC1_PWR_SW_Pin GPIO_PIN_10
#define ENC1_PWR_SW_GPIO_Port GPIOE
#define ENC1_A_Pin GPIO_PIN_11
#define ENC1_A_GPIO_Port GPIOE
#define ENC1_B_Pin GPIO_PIN_12
#define ENC1_B_GPIO_Port GPIOE
#define ENC2_SW_Pin GPIO_PIN_13
#define ENC2_SW_GPIO_Port GPIOE
#define ENC2_A_Pin GPIO_PIN_14
#define ENC2_A_GPIO_Port GPIOE
#define ENC2_B_Pin GPIO_PIN_15
#define ENC2_B_GPIO_Port GPIOE
#define BAT_CHRG_Pin GPIO_PIN_14
#define BAT_CHRG_GPIO_Port GPIOB
#define BAT_FULL_Pin GPIO_PIN_15
#define BAT_FULL_GPIO_Port GPIOB
#define SDMMC_DETECT_Pin GPIO_PIN_15
#define SDMMC_DETECT_GPIO_Port GPIOD
#define SDMMC_D0_Pin GPIO_PIN_8
#define SDMMC_D0_GPIO_Port GPIOC
#define SDMMC_D1_Pin GPIO_PIN_9
#define SDMMC_D1_GPIO_Port GPIOC
#define SDMMC_D2_Pin GPIO_PIN_10
#define SDMMC_D2_GPIO_Port GPIOC
#define SDMMC_D3_Pin GPIO_PIN_11
#define SDMMC_D3_GPIO_Port GPIOC
#define SDMMC_CK_Pin GPIO_PIN_12
#define SDMMC_CK_GPIO_Port GPIOC
#define SDMMC_CMD_Pin GPIO_PIN_2
#define SDMMC_CMD_GPIO_Port GPIOD
#define LCD_CS_Pin GPIO_PIN_3
#define LCD_CS_GPIO_Port GPIOD
#define LCD_RST_Pin GPIO_PIN_4
#define LCD_RST_GPIO_Port GPIOD
#define LCD_SCK_Pin GPIO_PIN_3
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_4
#define LCD_DC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern volatile uint32_t BAT_ADC_VAL;
extern volatile int BAT_ADC_Sampling;
extern volatile int Enc1;
extern volatile int Enc2;
extern volatile int MainBtnClick;
extern volatile int SecondBtnClick;
extern volatile int BAT_Voltage;
extern volatile int BAT_IsCharging;
extern volatile int BAT_IsFull;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
