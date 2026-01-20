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
#ifdef DEBUG
#define ITCM_CODE
#define ITCM_O3CODE __attribute__((optimize("O3")))
#else
#define ITCM_CODE __attribute__((section(".itcm_code")))
#define ITCM_O3CODE __attribute__((section(".itcm_code"), optimize("O3")))
#endif
#define DTCM_DATA __attribute__((section(".dtcm_data"), aligned(4)))
#define DTCM_BSS __attribute__((section(".dtcm_bss"), aligned(4)))
#define D2_DATA __attribute__((section(".d2_data")))
#define D2_BSS __attribute__((section(".d2_bss")))
#define D3_DATA __attribute__((section(".d3_data")))
#define D3_BSS __attribute__((section(".d3_bss")))
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <limits.h>
#include <setjmp.h>
#include "utf8.h"
#include "utf16.h"
#include "ili9341.h"
#include "graphics.h"
#include "fastmath.h"
#include "qspixip.h"
#include "i2saudio.h"
#include "../../FlashROM/flashmap.h"
#include "../../Phat/Phat/phat.h"
#include "../../avi_read/avi_read/avi_reader.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern const int FramebufferWidth;
extern const int FramebufferHeight;
extern const int GUITextAreaWidth;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define JPEG_CHUNK_SIZE_IN  ((uint32_t)(64 * 1024))
#define JPEG_CHUNK_SIZE_OUT ((uint32_t)(64 * 1024))
#define NUM_FILE_ITEMS 14
#define MAX_FILE_NAMELEN 256
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void OnUSBFail(void);
ITCM_CODE int DenoisedPinRead(uint8_t *buffer, size_t buffer_size, GPIO_TypeDef* GPIO, uint32_t Pin);
ITCM_CODE uint64_t HAL_GetTick64();
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
#define ENC1_SW_Pin GPIO_PIN_10
#define ENC1_SW_GPIO_Port GPIOE
#define ENC1_SW_EXTI_IRQn EXTI15_10_IRQn
#define ENC1_A_Pin GPIO_PIN_11
#define ENC1_A_GPIO_Port GPIOE
#define ENC1_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC1_B_Pin GPIO_PIN_12
#define ENC1_B_GPIO_Port GPIOE
#define ENC1_B_EXTI_IRQn EXTI15_10_IRQn
#define ENC2_SW_Pin GPIO_PIN_13
#define ENC2_SW_GPIO_Port GPIOE
#define ENC2_SW_EXTI_IRQn EXTI15_10_IRQn
#define ENC2_A_Pin GPIO_PIN_14
#define ENC2_A_GPIO_Port GPIOE
#define ENC2_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC2_B_Pin GPIO_PIN_15
#define ENC2_B_GPIO_Port GPIOE
#define ENC2_B_EXTI_IRQn EXTI15_10_IRQn
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
DTCM_BSS extern ADC_HandleTypeDef hadc1;
DTCM_BSS extern CRC_HandleTypeDef hcrc;
DTCM_BSS extern DMA2D_HandleTypeDef hdma2d;
DTCM_BSS extern I2S_HandleTypeDef hi2s2;
DTCM_BSS extern DMA_HandleTypeDef hdma_spi2_tx;
DTCM_BSS extern JPEG_HandleTypeDef hjpeg;
DTCM_BSS extern MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
DTCM_BSS extern MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;
DTCM_BSS extern QSPI_HandleTypeDef hqspi;
DTCM_BSS extern SD_HandleTypeDef hsd1;
DTCM_BSS extern SPI_HandleTypeDef hspi1;
DTCM_BSS extern DMA_HandleTypeDef hdma_spi1_tx;
DTCM_BSS extern DMA_HandleTypeDef hdma_spi1_rx;
DTCM_BSS extern LCD hlcd;
DTCM_DATA extern Pixel565 (*CurDrawFramebuffer)[320];
DTCM_BSS extern avi_reader avir;
DTCM_BSS extern avi_stream_reader avi_meta_stream;
DTCM_BSS extern avi_stream_reader avi_video_stream;
DTCM_BSS extern avi_stream_reader avi_audio_stream;
DTCM_BSS extern volatile uint32_t uwTick;
DTCM_BSS extern volatile uint32_t TickHigh;
DTCM_DATA extern uint32_t uwTickPrio;
DTCM_DATA extern HAL_TickFreqTypeDef uwTickFreq;
DTCM_BSS extern int FsMounted;
DTCM_DATA extern int CurVolume;
DTCM_BSS extern int GUICurMenu;
DTCM_BSS extern int GUICurMenuLevel;
DTCM_BSS extern int GUIMenuAnim;
DTCM_BSS extern int GUIMenuReady;
DTCM_BSS extern int GUICurFileIndex;
DTCM_BSS extern int GUIFirstFileIndex;
DTCM_BSS extern int GUILastFileIndex;
DTCM_BSS extern WChar_t GUIFileName[MAX_FILE_NAMELEN];
DTCM_BSS extern uint8_t GUIFileType;
DTCM_BSS extern volatile int GUIIsUsingFile;
DTCM_BSS extern int GUITextFileSize;
DTCM_BSS extern int GUITextFileTextSize;
DTCM_BSS extern int GUITextFileReadPos;
DTCM_BSS extern int GUITextFileMaxReadPosS;
DTCM_BSS extern int GUITextFileMaxReadPosM;
DTCM_BSS extern int GUITextFileMaxReadPosL;
DTCM_BSS extern uint64_t AVIStartPlayTime;
DTCM_BSS extern uint64_t AVIPausePlayTime;
DTCM_BSS extern int AVIPaused;
DTCM_BSS extern int USB_SDCardReady;
DTCM_BSS extern jmp_buf USBFailJmp;
DTCM_BSS extern jmp_buf SysErrorJmp;
DTCM_BSS extern int BugFileAgreed;

extern Pixel565 Framebuffer1[240][320];
extern Pixel565 Framebuffer2[240][320];
D2_BSS extern uint8_t JPEG_buffer[3][240][320];
extern uint8_t FILE_buffer[65536]; // Same size as the framebuffer
extern Phat_t phat;
extern Phat_DirInfo_t GUICurDir;
extern Phat_FileInfo_t CurFileStream1;
extern Phat_FileInfo_t CurFileStream2;
extern Phat_FileInfo_t CurFileStream3;
extern i2saudio_t i2saudio;
DTCM_BSS extern int GUINotifyShow;
DTCM_BSS extern uint64_t GUINotifyTimeUntil;
DTCM_BSS extern char GUINotifyInfo[256];
DTCM_BSS extern int GUIVolumeShow;
DTCM_BSS extern uint64_t GUIVolumeShowTimeUntil;
DTCM_BSS extern char FormatBuf[256];
DTCM_BSS extern volatile int Enc1;
DTCM_BSS extern volatile int Enc2;
DTCM_BSS extern volatile uint32_t BAT_ADC_VAL;
DTCM_BSS extern volatile int BAT_ADC_Sampling;
DTCM_BSS extern volatile int Enc1Click;
DTCM_BSS extern volatile int Enc2Click;
DTCM_BSS extern volatile int BAT_Voltage;
DTCM_BSS extern volatile int BAT_IsCharging;
DTCM_BSS extern volatile int BAT_IsFull;
DTCM_BSS extern volatile int HWJPEG_is_running;
DTCM_BSS extern volatile uint8_t* HWJPEG_src_pointer;
DTCM_BSS extern volatile uint8_t* HWJPEG_dst_pointer;
DTCM_BSS extern size_t HWJPEG_src_size;
DTCM_BSS extern uint8_t* HWJPEG_dst_buffer;
DTCM_BSS extern volatile JPEG_ConfTypeDef HWJpeg_info;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
