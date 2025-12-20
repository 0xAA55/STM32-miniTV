/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include "utf8.h"
#include "utf16.h"
#include "ili9341.h"
#include "graphics.h"
#include "fastmath.h"
#include "qspixip.h"
#include "dynalloc.h"
#include "../../FlashROM/flashmap.h"
#include "ff.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
__attribute__((section(".dtcm_bss"))) extern ADC_HandleTypeDef hadc1;
__attribute__((section(".dtcm_bss"))) extern DMA2D_HandleTypeDef hdma2d;
__attribute__((section(".dtcm_bss"))) extern I2S_HandleTypeDef hi2s2;
__attribute__((section(".dtcm_bss"))) extern DMA_HandleTypeDef hdma_spi2_tx;
__attribute__((section(".dtcm_bss"))) extern JPEG_HandleTypeDef hjpeg;
__attribute__((section(".dtcm_bss"))) extern QSPI_HandleTypeDef hqspi;
__attribute__((section(".dtcm_bss"))) extern SD_HandleTypeDef hsd1;
__attribute__((section(".dtcm_bss"))) extern SPI_HandleTypeDef hspi1;
__attribute__((section(".dtcm_bss"))) extern DMA_HandleTypeDef hdma_spi1_tx;
__attribute__((section(".dtcm_bss"))) extern DMA_HandleTypeDef hdma_spi1_rx;
__attribute__((section(".dtcm_bss"))) extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
__attribute__((section(".dtcm_bss"))) extern FATFS FatFs;
__attribute__((section(".dtcm_bss"))) extern LCD hlcd;
__attribute__((section(".dtcm_data"))) extern Pixel565 (*CurDrawFramebuffer)[320];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DMA2D_HandleTypeDef hdma2d;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

JPEG_HandleTypeDef hjpeg;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* USER CODE BEGIN PV */
const size_t FramebufferWidth = 320;
const size_t FramebufferHeight = 240;
FATFS FatFs;
LCD hlcd;
Pixel565 Framebuffer1[240][320];
Pixel565 Framebuffer2[240][320];
uint8_t JPEG_buffer[320 * 240 * 2]; // Same size as the framebuffer
Pixel565 (*CurDrawFramebuffer)[320] = Framebuffer1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_JPEG_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2S2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_ADC1_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
static void MX_DMA2D_Init(void);
/* USER CODE BEGIN PFP */
__attribute__((section(".itcm_code"))) void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd);
__attribute__((section(".itcm_code"))) void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd);
__attribute__((section(".itcm_code"))) void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define NUM_FILE_ITEMS 14
#define MAX_FILE_NAMELEN 256
int GUICurMenu = 0;
int GUICurMenuLevel = 0;
int GUIMenuAnim = 0;
int GUIMenuReady = 0;
uint16_t GUIFolderPath[4096];
uint16_t GUIFileList[NUM_FILE_ITEMS][MAX_FILE_NAMELEN];
uint8_t GUIFileIsDir[NUM_FILE_ITEMS];
int CurFileIndex = 0;
int FirstFileIndex = 0;
int FsMounted = 0;
__attribute__((section(".dtcm_bss"))) volatile int BAT_IsCharging;
__attribute__((section(".dtcm_bss"))) volatile int BAT_IsFull;
__attribute__((section(".dtcm_bss"))) volatile int Enc1;
__attribute__((section(".dtcm_bss"))) volatile int Enc2;
__attribute__((section(".dtcm_bss"))) volatile uint32_t BAT_ADC_VAL;
__attribute__((section(".dtcm_bss"))) volatile int BAT_ADC_Sampling;
__attribute__((section(".dtcm_bss"))) volatile int MainBtnClick;
__attribute__((section(".dtcm_bss"))) volatile int SecondBtnClick;
__attribute__((section(".dtcm_bss"))) volatile int BAT_Voltage;
__attribute__((section(".itcm_code")))
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  static int adc_read1;
  static int adc_read2;
  static int adc_read3;
  static int adc_read4;
  if (hadc == &hadc1)
  {
    adc_read4 = adc_read3;
    adc_read3 = adc_read2;
    adc_read2 = adc_read1;
    adc_read1 = HAL_ADC_GetValue(hadc);
    BAT_ADC_VAL = imin(imin(adc_read1, adc_read2), imin(adc_read3, adc_read4));
    BAT_ADC_Sampling = 0;
  }
}
int GetBatteryVolatage(uint32_t adc_val)
{
  int adc_voltage = (int)((adc_val * 3300) >> 16);
  return adc_voltage * 7 / 5;
}
int BatteryVolatageToPowerPercentage(int voltage)
{
  if (BAT_IsCharging && voltage != 0)
  {
    voltage = 4200 - (4200 - voltage) * 6 / 5;
  }
  if (voltage >= 3700)
    return (voltage - 3700) * 80 / (4200 - 3700) + 20;
  else
    return (voltage - 3500) * 20 / (3700 - 3500);
}
int GetPowerPercentage()
{
  return BatteryVolatageToPowerPercentage(BAT_Voltage);
}
void UpdatePowerRead()
{
  if (BAT_ADC_VAL == 0 && BAT_ADC_Sampling == 0)
  {
    BAT_ADC_Sampling = 1;
    HAL_ADC_Start_IT(&hadc1);
    while(BAT_ADC_Sampling) __WFI();
    BAT_Voltage = GetBatteryVolatage(BAT_ADC_VAL);
    BAT_ADC_Sampling = 1;
    HAL_ADC_Start_IT(&hadc1);
  }
  else
  {
    BAT_Voltage = GetBatteryVolatage(BAT_ADC_VAL);
    if (!BAT_ADC_Sampling)
    {
      BAT_ADC_Sampling = 1;
      HAL_ADC_Start_IT(&hadc1);
    }
  }
}
int DenoisedPinRead(uint8_t *buffer, size_t buffer_size, GPIO_TypeDef* GPIO, uint32_t Pin)
{
  uint32_t val = 0;
  uint8_t last;
  uint32_t middle = buffer_size >> 1;
  for (size_t i = 0; i < buffer_size - 1; i++)
  {
    uint8_t next = buffer[i + 1];
    val += next;
    buffer[i] = next;
  }
  last = (HAL_GPIO_ReadPin(GPIO, Pin) == GPIO_PIN_SET ? 1 : 0);
  buffer[buffer_size - 1] = last;
  val += last;
  return val >= middle ? 1 : 0;
}
static Pixel565 DrawPixelBg(int x, int y, int time)
{
  int r = FastCos(x - time / 16) / 2 + 512;
  int g = FastSin(time / 8 + y) / 4 + 512;
  int b = 1024 * 200 / 256;
  r = r * 255 / 1024;
  g = g * 255 / 1024;
  b = b * 255 / 1024;
  return MakePixel565(r, g, b);
}
void SwapFramebuffers()
{
  SCB_CleanDCache();
  LCD_WriteGRAM_DMA(&hlcd, (void*)CurDrawFramebuffer, sizeof Framebuffer1 / sizeof Framebuffer1[0][0]);
  if (CurDrawFramebuffer == Framebuffer1)
    CurDrawFramebuffer = Framebuffer2;
  else
    CurDrawFramebuffer = Framebuffer1;
}
void WaitForPresent()
{
  LCD_WaitToIdle(&hlcd);
}
void UpdateEnc1MainMenuState(int delta_tick, int enc_delta)
{
  int target_menu;
  int menu_speed = 5 * delta_tick;
  enc_delta = imax(-1, imin(enc_delta, 1));
  GUICurMenu += enc_delta;
  if (GUICurMenu < 0) GUICurMenu = 0;
  if (GUICurMenu > 3) GUICurMenu = 3;
  target_menu = GUICurMenu * 1024;
  if (GUIMenuAnim < target_menu)
  {
    if (GUIMenuAnim + menu_speed >= target_menu)
    {
      GUIMenuAnim = target_menu;
    }
    else
    {
      GUIMenuAnim += menu_speed;
    }
  }
  if (GUIMenuAnim > target_menu)
  {
    if (GUIMenuAnim - menu_speed <= target_menu)
    {
      GUIMenuAnim = target_menu;
    }
    else
    {
      GUIMenuAnim -= menu_speed;
    }
  }
  GUIMenuReady = (GUIMenuAnim == target_menu);
}
int IsMainBtnClick()
{
  int ret = MainBtnClick;
  MainBtnClick = 0;
  return ret;
}
int IsSecondBtnClick()
{
  int ret = SecondBtnClick;
  SecondBtnClick = 0;
  return ret;
}
int GetEnc1Delta()
{
  __attribute__((section(".dtcm_bss"))) static int last_enc1;
  int enc1_val = Enc1;
  int ret = enc1_val - last_enc1;
  last_enc1 = enc1_val;
  return ret;
}
int GetEnc2Delta()
{
  __attribute__((section(".dtcm_bss"))) static int last_enc2;
  int enc2_val = Enc2;
  int ret = enc2_val - last_enc2;
  last_enc2 = enc2_val;
  return ret;
}
void Suicide()
{
  HAL_GPIO_WritePin(PWCTRL_GPIO_Port, PWCTRL_Pin, GPIO_PIN_RESET);
}
void OnException()
{
  while(1)
  {
    if (IsMainBtnClick()) Suicide();
  }
}
uint8_t BSP_SD_Init(void)
{
  uint8_t sd_state = MSD_OK;
  /* Check if the SD card is plugged in the slot */
  if (BSP_SD_IsDetected() != SD_PRESENT)
  {
    return MSD_ERROR_SD_NOT_PRESENT;
  }
  /* HAL SD initialization */
  sd_state = HAL_SD_Init(&hsd1);
  return sd_state;
}
void GetCurDirFileList()
{
  DIR dir;
  FRESULT res;
  res = f_opendir(&dir, GUIFolderPath);
  memset(GUIFileList, 0, sizeof GUIFileList);
  for (size_t i = 0; i < NUM_FILE_ITEMS; i++)
  {
    GUIFileIsDir[i] = -1;
  }
  if (res == FR_OK)
  {
    FILINFO fno;
    for (size_t i = 0;; i++)
    {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0) break;
      if (fno.fname[0] == '.') continue;
      if (i >= FirstFileIndex)
      {
        size_t item = i - FirstFileIndex;
        if (item >= NUM_FILE_ITEMS) break;
        strncpyW(GUIFileList[item], fno.fname, MAX_FILE_NAMELEN);
        if (fno.fattrib & AM_DIR) GUIFileIsDir[item] = 1;
      }
    }
    f_closedir(&dir);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_MspInit();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  DynAlloc_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDMMC1_SD_Init();
  MX_JPEG_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_I2S2_Init();
  MX_QUADSPI_Init();
  MX_ADC1_Init();
  MX_USB_OTG_HS_PCD_Init();
  MX_DMA2D_Init();
  /* USER CODE BEGIN 2 */
  QSPI_InitFlash();
  QSPI_EnterMemoryMapMode();
  const LCD_GPIO lcd_gpio = {
    LCD_MakePin(LCD_RST_GPIO_Port, LCD_RST_Pin),
    LCD_MakePin(LCD_CS_GPIO_Port, LCD_CS_Pin),
    LCD_MakePin(LCD_DC_GPIO_Port, LCD_DC_Pin),
    LCD_MakePin(LCD_SCK_GPIO_Port, LCD_SCK_Pin),
    LCD_MakePin(LCD_SDA_GPIO_Port, LCD_SDA_Pin),
    LCD_MakePin(LCD_SDO_GPIO_Port, LCD_SDO_Pin),
  };
  LCD_Init
  (
    &hlcd,
    &hspi1,
    &hdma_spi1_tx,
    &hdma_spi1_rx,
    &lcd_gpio,
    LCD_RGB565,
    LCD_Landscape
  );
  LCD_Config(&hlcd);
  Graphics_Init();
  DrawStandByScreen();
  SwapFramebuffers();
  HAL_GPIO_WritePin(PWCTRL_GPIO_Port, PWCTRL_Pin, GPIO_PIN_SET);
  UseLargeFont();
  UpdatePowerRead();
  WaitForPresent();
  HAL_GPIO_WritePin(LCD_PWCTRL_GPIO_Port, LCD_PWCTRL_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int frame_counter = 0;
  uint32_t last_tick = HAL_GetTick();
  uint32_t delta_tick = 0;
  while (1)
  {
    uint32_t cur_tick = HAL_GetTick();
    static char buf[256];
    int main_btn_click = IsMainBtnClick();
    int second_btn_click = IsSecondBtnClick();
    int enc1_delta = GetEnc1Delta();
    int enc2_delta = GetEnc2Delta();

    UpdatePowerRead();

    switch (GUICurMenuLevel)
    {
      case 0:
      {
        UpdateEnc1MainMenuState(delta_tick, enc1_delta);
        for (int y = 0; y < hlcd.yres; y++)
        {
          for (int x = 0; x < hlcd.xres; x++)
          {
            CurDrawFramebuffer[y][x] = DrawPixelBg(x, y, cur_tick);
          }
        }

        int playbutton_anim_pos = GUIMenuAnim;
        int usbbutton_anim_pos = GUIMenuAnim - 1024;
        int optbutton_anim_pos = GUIMenuAnim - 2048;
        int shutbutton_anim_pos = GUIMenuAnim - 3072;
        int playbutton_size = (512 - imin(256, abs(playbutton_anim_pos)));
        int usbbutton_size = (512 - imin(256, abs(usbbutton_anim_pos)));
        int optbutton_size = (512 - imin(256, abs(optbutton_anim_pos)));
        int shutbutton_size = (512 - imin(256, abs(shutbutton_anim_pos)));
        int playbutton_x = 160 - playbutton_anim_pos * 120 / 1024;
        int usbbutton_x = 160 - usbbutton_anim_pos * 120 / 1024;
        int optbutton_x = 160 - optbutton_anim_pos * 120 / 1024;
        int shutbutton_x = 160 - shutbutton_anim_pos * 120 / 1024;
        Pixel565 ui_c1 = MakePixel565(0, 0, 0);
        Pixel565 ui_c2 = MakePixel565(255, 255, 255);

        DrawTFCardButton(playbutton_x, 120, ui_c1, ui_c2, playbutton_size);
        DrawUSBConnButton(usbbutton_x, 120, ui_c1, ui_c2, usbbutton_size);
        DrawOptionButton(optbutton_x, 120, ui_c1, ui_c2, optbutton_size);
        DrawShutdownButton(shutbutton_x, 120, ui_c1, ui_c2, shutbutton_size);
        DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
        if (GUIMenuReady)
        {
          switch(GUICurMenu)
          {
            case 0:
              strcpy(buf, "打开存储卡");
              break;
            case 1:
              strcpy(buf, "连接USB");
              break;
            case 2:
              strcpy(buf, "选项设置");
              break;
            case 3:
              strcpy(buf, "关闭电源");
              break;
          }
          DrawTextOpaque(120, 180, 200, 20, buf, MakePixel565(255, 255, 255), MakePixel565(0, 0, 0));
        }
        if (main_btn_click)
        {
          strcpyW(GUIFolderPath, (const WCHAR*) L"/");
          GUICurMenuLevel = 1;
        }
      }
      break;
      case 1:
      {
        switch (GUICurMenu)
        {
          case 0: // TF card
            if (!FsMounted)
            {
              uint32_t res;
              if ((res = f_mount(&FatFs, (const WCHAR*)L"0:", 1)) != FR_OK)
              {
                if (res == 3)
                  strcpy(buf, "未检测到SD卡，请插入SD卡");
                else
                  snprintf(buf, sizeof buf, "无法挂载SD卡(%"PRIx32")，请尝试更换SD卡", res);
                DrawStandByScreen();
                DrawTextOpaque(30, 110, 260, 80, buf, MakePixel565(255, 255, 255), MakePixel565(0, 0, 0));
              }
              else
              {
                FsMounted = 1;
                GetCurDirFileList();
              }
            }
            if (FsMounted)
            {
              ClearScreen(MakePixel565(0, 0, 0));
              SetWordWrap(0);
              for(size_t i = 0; i < NUM_FILE_ITEMS; i++)
              {
                int y = i * 17;
                if (GUIFileIsDir[i] == 0)
                  DrawFolderOrFile(0, i * 17, 0);
                else if (GUIFileIsDir[i] == 1)
                  DrawFolderOrFile(0, i * 17, 1);
                else
                  break;
                DrawTextW(17, y, 280, 20, GUIFileList[i], MakePixel565(255, 255, 255));
              }
              SetWordWrap(1);
            }
            if (second_btn_click)
            {
              if (FsMounted)
              {
                f_mount(&FatFs, (const WCHAR*)L":0", 0);
                FsMounted = 0;
              }
              GUICurMenuLevel = 0;
            }
            DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
            if (second_btn_click) GUICurMenuLevel = 0;
            break;
          case 1: // USB
            ClearScreen(MakePixel565(0, 0, 0));
            if (second_btn_click) GUICurMenuLevel = 0;
            break;
          case 2: // Option
            ClearScreen(MakePixel565(0, 0, 0));
            if (second_btn_click) GUICurMenuLevel = 0;
            break;
          case 3: // Shutdown
            for (int y = 0; y < hlcd.yres; y++)
            {
              for (int x = 0; x < hlcd.xres; x++)
              {
                CurDrawFramebuffer[y][x] = DrawPixelBg(x, y, cur_tick);
              }
            }
            Suicide();
            break;
        }
      }
    }

    SwapFramebuffers();
    frame_counter += 1;

    if (BAT_Voltage <= 3400 && !BAT_IsCharging && !BAT_IsFull)
      Suicide();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (cur_tick >= last_tick)
      delta_tick = cur_tick - last_tick;
    else
      delta_tick = 0;
    last_tick = cur_tick;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_SDMMC|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = 16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  hdma2d.LayerCfg[1].ChromaSubSampling = DMA2D_NO_CSS;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief JPEG Initialization Function
  * @param None
  * @retval None
  */
static void MX_JPEG_Init(void)
{

  /* USER CODE BEGIN JPEG_Init 0 */

  /* USER CODE END JPEG_Init 0 */

  /* USER CODE BEGIN JPEG_Init 1 */

  /* USER CODE END JPEG_Init 1 */
  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN JPEG_Init 2 */
  HAL_JPEG_MspInit(&hjpeg);
  /* USER CODE END JPEG_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */
  HAL_QSPI_MspInit(&hqspi);
  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 2;
  /* USER CODE BEGIN SDMMC1_Init 2 */
  HAL_SD_MspInit(&hsd1);
  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  HAL_SPI_MspInit(&hspi1);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_HIGH;
  hpcd_USB_OTG_HS.Init.dma_enable = ENABLE;
  hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_ULPI_PHY;
  hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_PWCTRL_GPIO_Port, LCD_PWCTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWCTRL_GPIO_Port, PWCTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_CS_Pin|LCD_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 PE5 PE6
                           PE7 PE8 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_PWCTRL_Pin */
  GPIO_InitStruct.Pin = LCD_PWCTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_PWCTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 PA4 PA8
                           PA10 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PWCTRL_Pin */
  GPIO_InitStruct.Pin = PWCTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWCTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1_PWR_SW_Pin ENC1_A_Pin ENC1_B_Pin ENC2_SW_Pin
                           ENC2_A_Pin ENC2_B_Pin */
  GPIO_InitStruct.Pin = ENC1_PWR_SW_Pin|ENC1_A_Pin|ENC1_B_Pin|ENC2_SW_Pin
                          |ENC2_A_Pin|ENC2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BAT_CHRG_Pin BAT_FULL_Pin */
  GPIO_InitStruct.Pin = BAT_CHRG_Pin|BAT_FULL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD13
                           PD14 PD0 PD1 PD5
                           PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC_DETECT_Pin */
  GPIO_InitStruct.Pin = SDMMC_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SDMMC_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x00000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RW;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x08000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL2;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RW;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER5;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RW;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER6;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER7;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL2;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO_URO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
