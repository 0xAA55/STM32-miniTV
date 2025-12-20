/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
__attribute__((section(".itcm_code"))) void SysTick_Handler(void);
__attribute__((section(".itcm_code"))) void DMA1_Stream0_IRQHandler(void);
__attribute__((section(".itcm_code"))) void DMA1_Stream1_IRQHandler(void);
__attribute__((section(".itcm_code"))) void DMA1_Stream2_IRQHandler(void);
__attribute__((section(".itcm_code"))) void ADC_IRQHandler(void);
__attribute__((section(".itcm_code"))) void SPI1_IRQHandler(void);
__attribute__((section(".itcm_code"))) void SDMMC1_IRQHandler(void);
__attribute__((section(".itcm_code"))) void OTG_HS_IRQHandler(void);
__attribute__((section(".itcm_code"))) void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
__attribute__((section(".itcm_code"))) void HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc);
__attribute__((section(".itcm_code"))) void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
__attribute__((section(".itcm_code"))) void HAL_SD_IRQHandler(SD_HandleTypeDef *hsd);
__attribute__((section(".itcm_code"))) void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern SD_HandleTypeDef hsd1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern SPI_HandleTypeDef hspi1;
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  __attribute__((section(".dtcm_bss"))) static int enc1_last_bm;
  __attribute__((section(".dtcm_bss"))) static int enc2_last_bm;
  __attribute__((section(".dtcm_bss"))) static int main_btn_is_up;
  __attribute__((section(".dtcm_bss"))) static int second_btn_is_up;
  __attribute__((section(".dtcm_bss"))) static uint8_t enc1_a_buffer[4];
  __attribute__((section(".dtcm_bss"))) static uint8_t enc1_b_buffer[4];
  __attribute__((section(".dtcm_bss"))) static uint8_t enc2_a_buffer[4];
  __attribute__((section(".dtcm_bss"))) static uint8_t enc2_b_buffer[4];
  __attribute__((section(".dtcm_bss"))) static int enc1_a;
  __attribute__((section(".dtcm_bss"))) static int enc1_b;
  __attribute__((section(".dtcm_bss"))) static int enc2_a;
  __attribute__((section(".dtcm_bss"))) static int enc2_b;
  __attribute__((section(".dtcm_bss"))) static int enc1_bm;
  __attribute__((section(".dtcm_bss"))) static int enc2_bm;

  enc1_a = DenoisedPinRead(enc1_a_buffer, sizeof enc1_a_buffer, ENC1_A_GPIO_Port, ENC1_A_Pin);
  enc1_b = DenoisedPinRead(enc1_b_buffer, sizeof enc1_b_buffer, ENC1_B_GPIO_Port, ENC1_B_Pin);
  enc2_a = DenoisedPinRead(enc2_a_buffer, sizeof enc2_a_buffer, ENC2_A_GPIO_Port, ENC2_A_Pin);
  enc2_b = DenoisedPinRead(enc2_b_buffer, sizeof enc2_b_buffer, ENC2_B_GPIO_Port, ENC2_B_Pin);
  enc1_bm = (enc1_a << 1) | (enc1_b);
  enc2_bm = (enc2_a << 1) | (enc2_b);
  BAT_IsCharging = (HAL_GPIO_ReadPin(BAT_CHRG_GPIO_Port, BAT_CHRG_Pin) == GPIO_PIN_RESET);
  BAT_IsFull = (HAL_GPIO_ReadPin(BAT_FULL_GPIO_Port, BAT_FULL_Pin) == GPIO_PIN_RESET);
  switch((enc1_last_bm << 2) | enc1_bm)
  {
    case 0b0001:
    case 0b1000:
      Enc1 -= 1;
      break;
    case 0b0010:
    case 0b0100:
      Enc1 += 1;
      break;
  }
  switch((enc2_last_bm << 2) | enc2_bm)
  {
    case 0b0001:
    case 0b1000:
      Enc2 -= 1;
      break;
    case 0b0010:
    case 0b0100:
      Enc2 += 1;
      break;
  }
  enc1_last_bm = enc1_bm;
  enc2_last_bm = enc2_bm;
  switch(HAL_GPIO_ReadPin(ENC1_PWR_SW_GPIO_Port, ENC1_PWR_SW_Pin))
  {
    default:
      if (main_btn_is_up)
      {
        main_btn_is_up = 0;
        MainBtnClick = 1;
      }
      break;
    case GPIO_PIN_SET:
      main_btn_is_up = 1;
      break;
  }
  switch(HAL_GPIO_ReadPin(ENC2_SW_GPIO_Port, ENC2_SW_Pin))
  {
    default:
      if (second_btn_is_up)
      {
        second_btn_is_up = 0;
        SecondBtnClick = 1;
      }
      break;
    case GPIO_PIN_SET:
      second_btn_is_up = 1;
      break;
  }
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC1_IRQn 0 */

  /* USER CODE END SDMMC1_IRQn 0 */
  HAL_SD_IRQHandler(&hsd1);
  /* USER CODE BEGIN SDMMC1_IRQn 1 */

  /* USER CODE END SDMMC1_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS global interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
