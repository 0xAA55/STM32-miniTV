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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAVE_SETTINGS_ADDRESS (8 * 1024 * 1024 - 4096)
#define SAVED_SETTINGS ((CurSettings_p)(0x90000000 + SAVE_SETTINGS_ADDRESS))
#define FASTFORWARD_TIME 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

JPEG_HandleTypeDef hjpeg;
MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

/* USER CODE BEGIN PV */
CurSettings_t CurSettings;
const int FramebufferWidth = 320;
const int FramebufferHeight = 240;
const int GUITextAreaWidth = FramebufferWidth - 3;
LCD hlcd;
Pixel565 Framebuffer1[240][320];
Pixel565 Framebuffer2[240][320];
uint8_t FILE_buffer[65536];
uint8_t JPEG_buffer[3][240][320];
Pixel565 (*CurDrawFramebuffer)[320] = Framebuffer1;
uint8_t *TXT_buffer = &JPEG_buffer[0][0][0];
const size_t TXT_buffer_size = sizeof JPEG_buffer;
Phat_t phat;
avi_reader avir;
avi_stream_reader avi_meta_stream;
avi_stream_reader avi_video_stream;
avi_stream_reader avi_audio_stream;
i2saudio_t i2saudio;
int FsMounted;
int GUICurMenu;
int GUICurMenuLevel;
int GUIMenuAnim;
int GUIMenuReady;
int GUICurFileIndex;
int GUIFirstFileIndex;
int GUILastFileIndex;
uint32_t GUIFileListRefreshTime;
WChar_t GUIFileName[MAX_FILE_NAMELEN];
uint8_t GUIFileType;
Phat_DirInfo_t GUICurDir;
Phat_FileInfo_t CurFileStream1;
Phat_FileInfo_t CurFileStream2;
Phat_FileInfo_t CurFileStream3;
Phat_FileInfo_t CurFileStream4;
volatile int GUIIsUsingFile;
int GUITextFileSize;
int GUITextFileTextSize;
int GUITextFileReadPos;
int GUITextFileMaxReadPosS;
int GUITextFileMaxReadPosM;
int GUITextFileMaxReadPosL;
int GUICurOptionIndex;
int GUICurOptionSelected;
uint64_t AVIStartPlayTime;
uint64_t AVIPausePlayTime;
int AVIPaused;
int GUINotifyShow;
uint64_t GUINotifyTimeUntil;
char GUINotifyInfo[256];
int GUIVolumeShow;
uint64_t GUIVolumeShowTimeUntil;
char FormatBuf[256];
volatile int BAT_IsCharging;
volatile int BAT_IsFull;
volatile int Enc1;
volatile int Enc2;
volatile uint32_t BAT_ADC_VAL;
volatile int BAT_ADC_Sampling;
volatile int Enc1Click;
volatile int Enc2Click;
volatile int BAT_Voltage;
volatile int HWJPEG_is_running;
volatile uint8_t* HWJPEG_src_pointer;
volatile uint8_t* HWJPEG_dst_pointer;
volatile int HWJPEG_got_info;
size_t HWJPEG_src_size;
volatile JPEG_ConfTypeDef HWJpeg_info;
volatile uint32_t TickHigh;
jmp_buf USBFailJmp;
jmp_buf SysErrorJmp;
int BugFileAgreed;
int SubtitlePrepped;
WChar_t CurSubtitle[256];
srt_t SubtitleParser;
srt_slot_p CurSubtitleSlot;
int CurSubtitleY;
uint64_t LastOperateTime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_MDMA_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_JPEG_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2S2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA2D_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ITCM_O3CODE
static uint16_t BSwap16(uint16_t val)
{
  union {
    uint8_t u8s[2];
    uint16_t u16;
  }u1, u2;

  u1.u16 = val;
  u2.u8s[0] = u1.u8s[1];
  u2.u8s[1] = u1.u8s[0];
  return u2.u16 ;
}
ITCM_O3CODE
void HAL_IncTick(void)
{
  uint32_t old_tick = uwTick;
  uwTick += (uint32_t)uwTickFreq;
  if (uwTick < old_tick) TickHigh ++;
}
ITCM_O3CODE
uint64_t HAL_GetTick64()
{
  uint64_t ret;
  __disable_irq();
  ret = ((uint64_t)TickHigh << 32) | uwTick;
  __enable_irq();
  return ret;
}
ITCM_O3CODE
void HAL_Delay(uint32_t Delay)
{
  uint64_t tickstart = HAL_GetTick64();
  uint64_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += uwTickFreq;
  }

  while ((HAL_GetTick64() - tickstart) < wait) __WFI();
}
ITCM_CODE
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == hlcd.hspi)
  {
    LCD_On_DMA_TX(&hlcd);
  }
}
ITCM_CODE
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == hlcd.hspi)
  {
    LCD_On_DMA_RX(&hlcd);
  }
}
ITCM_CODE
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s == i2saudio.hi2s)
  {
    i2saudio_tx_half_cplt_callback(&i2saudio);
  }
}
ITCM_CODE
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s == i2saudio.hi2s)
  {
    i2saudio_tx_full_cplt_callback(&i2saudio);
  }
}
ITCM_CODE
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  DTCM_BSS static int adc_read1;
  DTCM_BSS static int adc_read2;
  DTCM_BSS static int adc_read3;
  DTCM_BSS static int adc_read4;
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
ITCM_CODE
int GetBatteryVolatage(uint32_t adc_val)
{
  int adc_voltage = (int)((adc_val * 3300) >> 16);
  return adc_voltage * 7 / 5;
}
ITCM_CODE
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
ITCM_CODE
int GetPowerPercentage()
{
  return BatteryVolatageToPowerPercentage(BAT_Voltage);
}
ITCM_CODE
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
ITCM_O3CODE
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
ITCM_CODE
void SwapFramebuffers()
{
  SCB_CleanDCache_by_Addr((uint32_t*)CurDrawFramebuffer, sizeof Framebuffer1);
  LCD_WriteGRAM_DMA(&hlcd, (void*)CurDrawFramebuffer, sizeof Framebuffer1 / sizeof Framebuffer1[0][0]);
  if (CurDrawFramebuffer == Framebuffer1)
    CurDrawFramebuffer = Framebuffer2;
  else
    CurDrawFramebuffer = Framebuffer1;
}
ITCM_CODE
void UnswapFramebuffers()
{
  Pixel565 *AnotherBuffer = (Pixel565 *)Framebuffer1;
  if (AnotherBuffer == (Pixel565 *)CurDrawFramebuffer) AnotherBuffer = (Pixel565 *)Framebuffer2;
  memcpy(CurDrawFramebuffer, AnotherBuffer, sizeof Framebuffer1);
}
ITCM_CODE
void UnsaturateScreen()
{
  for (int y = 0; y < hlcd.yres; y++)
  {
    for (int x = 0; x < hlcd.xres; x++)
    {
      // The logic is equivalent to `if RGB == 255 then set to 254`
      uint16_t color = CurDrawFramebuffer[y][x];
      uint16_t new_color = 0;
      if ((color & 0xF800)  == 0xF800) new_color |= 0xF000;
      else new_color |= color & 0xF800;
      if ((color & 0x07E0)  == 0x07E0) new_color |= 0x07C0;
      else new_color |= color & 0x07E0;
      if ((color & 0x001F)  == 0x001F) new_color |= 0x001E;
      else new_color |= color & 0x001F;
      CurDrawFramebuffer[y][x] = new_color;
    }
  }
}
ITCM_CODE
int SaveSettings()
{
  CurSettings.Signature = 0xAA55;
  memset(FILE_buffer, 0, 4096);
  memcpy(FILE_buffer, &CurSettings, sizeof CurSettings);
  if (QSPI_ExitMemoryMapMode() != HAL_OK) return 0;
  if (QSPI_SectorErase(SAVE_SETTINGS_ADDRESS) != HAL_OK) return 0;
  for(size_t i = 0; i < sizeof CurSettings; i += 256)
  {
    if (QSPI_PageProgram(SAVE_SETTINGS_ADDRESS + i, &FILE_buffer[i]) != HAL_OK) return 0;
  }
  if (QSPI_InitFlash() != HAL_OK) return 0;
  if (QSPI_EnterMemoryMapMode() != HAL_OK) return 0;
  return 1;
}
ITCM_CODE
void LoadSettings()
{
  memcpy(&CurSettings, SAVED_SETTINGS, sizeof CurSettings);
  if (CurSettings.Signature != 0xAA55)
  {
    CurSettings.Signature = 0xAA55;
    CurSettings.CurFastForwardTime = 5000;
    CurSettings.CurVolume = 100;
    CurSettings.SubtitleFontSize = 12;
    CurSettings.StandByTime = 5 * 60;
  }
  else
  {
    if (CurSettings.CurFastForwardTime == 0xFFFFFFFF) CurSettings.CurFastForwardTime = 5000;
    if (CurSettings.CurVolume < 0) CurSettings.CurFastForwardTime = 0;
    if (CurSettings.CurVolume > 100) CurSettings.CurFastForwardTime = 100;
    if (CurSettings.SubtitleFontSize != 12 || CurSettings.SubtitleFontSize != 14 || CurSettings.SubtitleFontSize != 17) CurSettings.SubtitleFontSize = 12;
    if (!CurSettings.StandByTime || CurSettings.StandByTime == 0xFFFFFFFF) CurSettings.StandByTime = 5 * 60;
  }
}
void WaitForPresent()
{
  LCD_WaitToIdle(&hlcd);
}
ITCM_CODE
int IsEnc1Click()
{
  int ret = Enc1Click;
  Enc1Click = 0;
  return ret;
}
ITCM_CODE
int IsEnc2Click()
{
  int ret = Enc2Click;
  Enc2Click = 0;
  return ret;
}
ITCM_CODE
int GetEnc1Delta()
{
  DTCM_BSS static int last_enc1;
  int enc1_val = Enc1 / 2;
  int ret = enc1_val - last_enc1;
  last_enc1 = enc1_val;
  return ret;
}
ITCM_CODE
int GetEnc2Delta()
{
  DTCM_BSS static int last_enc2;
  int enc2_val = Enc2 / 2;
  int ret = enc2_val - last_enc2;
  last_enc2 = enc2_val;
  return ret;
}
ITCM_CODE
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  DTCM_BSS static int enc1_last_bm;
  DTCM_BSS static int enc2_last_bm;
  DTCM_BSS static int enc1_sw_is_up;
  DTCM_BSS static int enc2_sw_is_up;
  DTCM_BSS static int enc1_a;
  DTCM_BSS static int enc1_b;
  DTCM_BSS static int enc2_a;
  DTCM_BSS static int enc2_b;
  DTCM_BSS static int enc1_bm;
  DTCM_BSS static int enc2_bm;

  enc1_a = (HAL_GPIO_ReadPin(ENC1_A_GPIO_Port, ENC1_A_Pin) == GPIO_PIN_SET);
  enc1_b = (HAL_GPIO_ReadPin(ENC1_B_GPIO_Port, ENC1_B_Pin) == GPIO_PIN_SET);
  enc2_a = (HAL_GPIO_ReadPin(ENC2_A_GPIO_Port, ENC2_A_Pin) == GPIO_PIN_SET);
  enc2_b = (HAL_GPIO_ReadPin(ENC2_B_GPIO_Port, ENC2_B_Pin) == GPIO_PIN_SET);
  enc1_bm = (enc1_a << 1) | (enc1_b);
  enc2_bm = (enc2_a << 1) | (enc2_b);
  switch((enc1_last_bm << 2) | enc1_bm)
  {
    case 0b0001:
    case 0b1000:
      Enc1 += 1;
      break;
    case 0b0010:
    case 0b0100:
      Enc1 -= 1;
      break;
  }
  switch((enc2_last_bm << 2) | enc2_bm)
  {
    case 0b0001:
    case 0b1000:
      Enc2 += 1;
      break;
    case 0b0010:
    case 0b0100:
      Enc2 -= 1;
      break;
  }
  enc1_last_bm = enc1_bm;
  enc2_last_bm = enc2_bm;
  switch(HAL_GPIO_ReadPin(ENC1_SW_GPIO_Port, ENC1_SW_Pin))
  {
    default:
      if (enc1_sw_is_up)
      {
        enc1_sw_is_up = 0;
        Enc1Click = 1;
      }
      break;
    case GPIO_PIN_SET:
      enc1_sw_is_up = 1;
      break;
  }
  switch(HAL_GPIO_ReadPin(ENC2_SW_GPIO_Port, ENC2_SW_Pin))
  {
    default:
      if (enc2_sw_is_up)
      {
        enc2_sw_is_up = 0;
        Enc2Click = 1;
      }
      break;
    case GPIO_PIN_SET:
      enc2_sw_is_up = 1;
      break;
  }

  LastOperateTime = HAL_GetTick64();
}
void Suicide()
{
  HAL_GPIO_WritePin(PWCTRL_GPIO_Port, PWCTRL_Pin, GPIO_PIN_RESET);
}
void Unsuicide()
{
  HAL_GPIO_WritePin(PWCTRL_GPIO_Port, PWCTRL_Pin, GPIO_PIN_SET);
}
ITCM_CODE
void OnUsingBugFileGUI(uint64_t cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click);
void OnException()
{
  PhatState res;
  res = Phat_Init(&phat);
  if (res != PhatState_OK) goto Fail;

  res = Phat_Mount(&phat, 0, 0);
  if (res != PhatState_OK) goto Fail;

  Phat_OpenRootDir(&phat, &GUICurDir);
  res = Phat_OpenFile(&GUICurDir, u"FlashROM.rom", 1, &CurFileStream1);
  if (res != PhatState_OK) goto Fail;

  strcpyW(GUIFileName, GUICurDir.LFN_name);
  GUICurMenuLevel = 1;
  GUICurMenu = 0;
  GUIIsUsingFile = 1;
  GUIFileType = 3;
  BugFileAgreed = 1;
  QSPI_ExitMemoryMapMode();

  OnUsingBugFileGUI(0, 0, 0, 0, 0, 0);
  return;
Fail:
  while(1)
  {
    if (IsEnc1Click()) Suicide();
    __WFI();
  }
}
ITCM_CODE
void ShowNotifyV(uint32_t duration, const char *format, va_list ap)
{
  char *from;
  size_t size;
  uint64_t new_time_until = HAL_GetTick64() + duration;
  from = GUINotifyInfo;
  size = sizeof GUINotifyInfo;
  if (GUINotifyShow)
  {
    while(*from && size)
    {
      from ++;
      size --;
    }
    if (!size)
    {
      from = GUINotifyInfo;
      size = sizeof GUINotifyInfo;
    }
  }
  if (from > GUINotifyInfo && *(from - 1) != '\n')
  {
    if (size)
    {
      *from ++ = '\n';
      size --;
    }
    if (!size)
    {
      from = GUINotifyInfo;
      size = sizeof GUINotifyInfo;
    }
  }
  vsnprintf(from, size, format, ap);
  if (new_time_until > GUINotifyTimeUntil) GUINotifyTimeUntil = new_time_until;
  GUINotifyShow = 1;
}
ITCM_CODE
void ShowNotify(uint32_t duration, const char *format, ...)
{
  va_list ap;
  va_start(ap, format);
  ShowNotifyV(duration, format, ap);
  va_end(ap);
}
ITCM_CODE
void ShowVolume(uint32_t duration)
{
  GUIVolumeShowTimeUntil = HAL_GetTick64() + duration;
  GUIVolumeShow = 1;
}
ITCM_CODE
uint8_t GetCurFileType()
{
  uint8_t ret;
  if (GUICurDir.attributes & ATTRIB_DIRECTORY)
    ret = 0;
  else
  {
    uint16_t *LFN_name = GUICurDir.LFN_name;
    uint16_t *dot = LFN_name;
    while(*dot) dot++;
    while(dot > LFN_name && *dot != u'.') dot --;
    if (dot == LFN_name)
      ret = 1;
    else
    {
      ret = 1;
      if (!strcmpW(dot, u".avi")) ret = 2;
      if (!strcmpW(dot, u".rom")) ret = 3;
      if (!strcmpW(dot, u".bin")) ret = 3;
      if (!strcmpW(dot, u".srt")) ret = 4;
      if (!strcmpW(dot, u".ass")) ret = 5;
    }
  }
  return ret;
}
ITCM_CODE
void OnMainMenu(uint64_t cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
{
  int target_menu;
  int menu_speed = 5 * delta_tick;
  GUICurMenu += enc1_delta;
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

  for (int y = 0; y < hlcd.yres; y++)
  {
    for (int x = 0; x < hlcd.xres; x++)
    {
      CurDrawFramebuffer[y][x] = DrawPixelBg(x, y, cur_tick);
    }
  }

  if (enc2_delta)
  {
    CurSettings.CurVolume += enc2_delta * 5;
    if (CurSettings.CurVolume < 0) CurSettings.CurVolume = 0;
    if (CurSettings.CurVolume > 100) CurSettings.CurVolume = 100;
    ShowVolume(200);
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
        strcpy(FormatBuf, "打开存储卡");
        break;
      case 1:
        strcpy(FormatBuf, "连接USB");
        break;
      case 2:
        strcpy(FormatBuf, "选项设置");
        break;
      case 3:
        strcpy(FormatBuf, "关闭电源");
        break;
    }
    DrawTextOpaque(120, 180, 200, 20, FormatBuf, MakePixel565(255, 255, 255), MakePixel565(0, 0, 0));
    if (enc1_click)
    {
      GUICurMenuLevel = 1;
      switch(GUICurMenu)
      {
        case 0:
          GUICurFileIndex = 0;
          GUIFirstFileIndex = 0;
          GUIIsUsingFile = 0;
          break;
        case 1:
          switch (setjmp(USBFailJmp))
          {
          case 0:
            MX_USB_DEVICE_Init();
            break;
          case 1:
            GUICurMenuLevel = 0;
            ShowNotify(1000, "请确保插入存储卡，连接电脑USB后，再进入此功能。");
          default:
            GUICurMenuLevel = 0;
            ShowNotify(1000, "系统错误。");
            break;
          }
          break;
        case 2:
          break;
        case 3:
          break;
      }
    }
  }
}
ITCM_CODE
static void PrepareTextFile()
{
  PhatState res;
  FileSize_t filesize;
  FileSize_t to_be_copied;
  uint8_t *dst_ptr;
  const int OneScreenLinesS = FramebufferHeight / 12;
  const int OneScreenLinesM = FramebufferHeight / 14;
  const int OneScreenLinesL = FramebufferHeight / 17;
  res = Phat_OpenFile(&GUICurDir, GUIFileName, 1, &CurFileStream1);
  if (res != PhatState_OK)
  {
    ShowNotify(1000, "无法打开文件（%s）", Phat_StateToString(res));
    return;
  }
  Phat_GetFileSize(&CurFileStream1, &filesize);
  if (filesize > TXT_buffer_size - 2)
  {
    Phat_CloseFile(&CurFileStream1);
    ShowNotify(1000, "文件太大了。");
    return;
  }
  to_be_copied = filesize;
  dst_ptr = TXT_buffer;
  while(to_be_copied)
  {
    FileSize_t to_copy = to_be_copied;
    if (to_copy > sizeof FILE_buffer) to_copy = sizeof FILE_buffer;
    res = Phat_ReadFile(&CurFileStream1, FILE_buffer, to_copy, NULL);
    if (res != PhatState_OK && res != PhatState_EndOfFile)
    {
      Phat_CloseFile(&CurFileStream1);
      ShowNotify(1000, "读取文件失败。");
      return;
    }
    memcpy(dst_ptr, FILE_buffer, to_copy);
    dst_ptr += to_copy;
    to_be_copied -= to_copy;
  }
  TXT_buffer[filesize] = 0;
  TXT_buffer[filesize + 1] = 0;
  if (*(uint16_t*)TXT_buffer == 0xFFFE)
  {
    uint32_t max_height;
    uint16_t *ptr = (uint16_t*)TXT_buffer;
    while(*ptr)
    {
      *ptr = BSwap16(*ptr);
      ptr ++;
    }
    UseSmallFont();
    GetTextSizeW((uint16_t*)TXT_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosS = max_height / 12;
    UseMediumFont();
    GetTextSizeW((uint16_t*)TXT_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosM = max_height / 14;
    UseLargeFont();
    GetTextSizeW((uint16_t*)TXT_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosL = max_height / 17;
  }
  else
  {
    uint32_t max_height;
    UseSmallFont();
    GetTextSize((char*)TXT_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosS = max_height / 12;
    UseMediumFont();
    GetTextSize((char*)TXT_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosM = max_height / 14;
    UseLargeFont();
    GetTextSize((char*)TXT_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosL = max_height / 17;
  }
  Phat_CloseFile(&CurFileStream1);
  GUITextFileReadPos = 0;
  GUITextFileSize = filesize;
  GUIIsUsingFile = 1;
  if (GUITextFileMaxReadPosS > OneScreenLinesS) GUITextFileMaxReadPosS -= OneScreenLinesS; else GUITextFileMaxReadPosS = 0;
  if (GUITextFileMaxReadPosM > OneScreenLinesM) GUITextFileMaxReadPosM -= OneScreenLinesM; else GUITextFileMaxReadPosM = 0;
  if (GUITextFileMaxReadPosL > OneScreenLinesL) GUITextFileMaxReadPosL -= OneScreenLinesL; else GUITextFileMaxReadPosL = 0;
}
/**
  * @brief  Init the DMA2D for YCbCr to RGB565 Conversion .
  * @param  xsize: Image width
  * @param  ysize: Image Height
  * @param  ChromaSampling: YCbCr Chroma sampling : 4:2:0, 4:2:2 or 4:4:4
  * @retval None
  */
ITCM_CODE
HAL_StatusTypeDef DMA2D_Init(uint16_t ImageWidth, uint16_t ImageHeight, uint32_t ColorSpace, uint32_t ChromaSampling)
{
  HAL_StatusTypeDef ret;
  uint32_t input_mode;
  uint32_t css_mode = DMA2D_NO_CSS;

  UNUSED(ImageHeight);

  switch(ColorSpace)
  {
  case JPEG_GRAYSCALE_COLORSPACE:
    input_mode = DMA2D_INPUT_L8;
    break;
  case JPEG_YCBCR_COLORSPACE:
    input_mode = DMA2D_INPUT_YCBCR;
    switch(ChromaSampling)
    {
    case JPEG_444_SUBSAMPLING: css_mode = DMA2D_NO_CSS; break;
    case JPEG_422_SUBSAMPLING: css_mode = DMA2D_CSS_422; break;
    case JPEG_420_SUBSAMPLING: css_mode = DMA2D_CSS_420; break;
    default: return HAL_ERROR;
    }
    break;
  default: return HAL_ERROR;
  }

  /*##-1- Configure the DMA2D Mode, Color Mode and output offset #############*/
  hdma2d.Init.Mode          = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode     = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset  = FramebufferWidth - ImageWidth;
  hdma2d.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion*/
  hdma2d.Init.RedBlueSwap   = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */

  /*##-2- DMA2D Callbacks Configuration ######################################*/
  hdma2d.XferCpltCallback  = NULL;

  /*##-3- Foreground Configuration ###########################################*/
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d.LayerCfg[1].InputColorMode = input_mode;
  hdma2d.LayerCfg[1].ChromaSubSampling = css_mode;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR; /* No ForeGround Red/Blue swap */
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA; /* No ForeGround Alpha inversion */

  hdma2d.Instance = DMA2D;

  /*##-4- DMA2D Initialization     ###########################################*/
  ret = HAL_DMA2D_Init(&hdma2d);
  if (ret != HAL_OK) return ret;
  return HAL_DMA2D_ConfigLayer(&hdma2d, 1);
}
/**
  * @brief  Copy the Decoded image to the display Frame buffer.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Pointer to destination buffer
  * @param  ImageWidth: image width
  * @param  ImageHeight: image Height
  * @retval None
  */
ITCM_CODE
HAL_StatusTypeDef DMA2D_CopyBuffer(uint32_t *pSrc, uint32_t *pDst, uint16_t ImageWidth, uint16_t ImageHeight)
{
  HAL_StatusTypeDef ret;
  uint32_t xPos, yPos;
  uint8_t *destination;

  /*##-1- calculate the destination transfer address  ############*/
  xPos = (FramebufferWidth - HWJpeg_info.ImageWidth) / 2;
  yPos = (FramebufferHeight - HWJpeg_info.ImageHeight) / 2;

  destination = (uint8_t*)pDst + (yPos * FramebufferHeight + xPos) * 2;

  /* wait for the previous DMA2D transfer to ends */
  ret = HAL_DMA2D_PollForTransfer(&hdma2d, 33);
  if (ret != HAL_OK) return ret;

  /* copy the new decoded frame to the LCD Frame buffer*/
  ret = HAL_DMA2D_Start(&hdma2d, (uint32_t)pSrc, (uint32_t)destination, ImageWidth, ImageHeight);
  if (ret != HAL_OK) return ret;
  return HAL_OK;
}
static void QuitVideoFile()
{
  GUIIsUsingFile = 0;
  i2saudio_stop(&i2saudio);
  Phat_CloseFile(&CurFileStream1);
  Phat_CloseFile(&CurFileStream2);
  Phat_CloseFile(&CurFileStream3);
  HAL_JPEG_Abort(&hjpeg);
  HAL_JPEG_DeInit(&hjpeg);
  memset((void*)&HWJpeg_info, 0, sizeof HWJpeg_info);
  HWJPEG_is_running = 0;
}
ITCM_CODE
void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{
  UNUSED(hjpeg);
  HWJPEG_is_running = 0;
}
ITCM_CODE
void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg)
{
  HAL_JPEG_Abort(hjpeg);
  HAL_JPEG_DeInit(hjpeg);
  HWJPEG_is_running = 0;
  ShowNotify(200, "视频画面解码错误");
}
ITCM_CODE
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData)
{
  size_t decoded, in_size;
  HWJPEG_src_pointer += NbDecodedData;
  decoded = HWJPEG_src_pointer - FILE_buffer;
  in_size = HWJPEG_src_size - decoded;
  if (in_size > JPEG_CHUNK_SIZE_IN) in_size = JPEG_CHUNK_SIZE_IN;
  SCB_CleanDCache_by_Addr((uint32_t *)HWJPEG_src_pointer, in_size);
  HAL_JPEG_ConfigInputBuffer(hjpeg, (uint8_t*)HWJPEG_src_pointer, in_size);
}
ITCM_CODE
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
  size_t next_block_size;
  HWJPEG_dst_pointer += OutDataLength;
  if (!HWJPEG_got_info)
  {
    ShowNotify(200, "视频画面信息解码错误");
    return;
  }
  next_block_size = (sizeof JPEG_buffer) - ((size_t)HWJPEG_dst_pointer - (size_t)JPEG_buffer);
  if (next_block_size > JPEG_CHUNK_SIZE_OUT) next_block_size = JPEG_CHUNK_SIZE_OUT;
  HAL_JPEG_ConfigOutputBuffer(hjpeg, (uint8_t*)HWJPEG_dst_pointer, next_block_size);
}
ITCM_CODE
void HAL_JPEG_InfoReadyCallback(JPEG_HandleTypeDef *hjpeg, JPEG_ConfTypeDef *pInfo)
{
  HWJpeg_info = *pInfo;
  if (HWJpeg_info.ImageWidth > FramebufferWidth || HWJpeg_info.ImageHeight > FramebufferHeight)
  {
    ShowNotify(1000, "视频画面尺寸超过了 %dx%d，不被支持", FramebufferWidth, FramebufferHeight);
    goto FailExit;
  }
  switch(HWJpeg_info.ColorSpace)
  {
  case JPEG_GRAYSCALE_COLORSPACE:
    if (HWJpeg_info.ImageWidth & 3)
    {
      ShowNotify(1000, "灰度格式视频画面宽度 %lu 不是 4 的倍数，不被支持", HWJpeg_info.ImageWidth);
      goto FailExit;
    }
    break;
  case JPEG_YCBCR_COLORSPACE:
    switch(HWJpeg_info.ChromaSubsampling)
    {
    case JPEG_444_SUBSAMPLING:
      if (HWJpeg_info.ImageWidth & 7)
      {
        ShowNotify(1000, "YUV444 格式视频画面宽度 %lu 不是 8 的倍数，不被支持", HWJpeg_info.ImageWidth);
        goto FailExit;
      }
      break;
    case JPEG_422_SUBSAMPLING:
      if (HWJpeg_info.ImageWidth & 15)
      {
        ShowNotify(1000, "YUV422 格式视频画面宽度 %lu 不是 16 的倍数，不被支持", HWJpeg_info.ImageWidth);
        goto FailExit;
      }
      break;
    case JPEG_420_SUBSAMPLING:
      if (HWJpeg_info.ImageWidth & 15)
      {
        ShowNotify(1000, "YUV420 格式视频画面宽度 %lu 不是 16 的倍数，不被支持", HWJpeg_info.ImageWidth);
        goto FailExit;
      }
      break;
    default:
      ShowNotify(1000, "YUV未知采样格式视频不被支持");
      goto FailExit;
    }
    break;
  case JPEG_CMYK_COLORSPACE:
    ShowNotify(1000, "视频画面颜色格式不被支持：CMYK");
    goto FailExit;
  default:
    ShowNotify(1000, "视频画面未知颜色格式不被支持");
    goto FailExit;
  }
  HWJPEG_got_info = 1;
  if (DMA2D_Init(HWJpeg_info.ImageWidth, HWJpeg_info.ImageHeight, HWJpeg_info.ColorSpace, HWJpeg_info.ChromaSubsampling) != HAL_OK)
  {
    ShowNotify(1000, "像素格式转码器初始化失败");
    goto FailExit;
  }
  return;
FailExit:
  QuitVideoFile();
}
ITCM_CODE
int JPEG_Wait_Decode(uint32_t timeout)
{
  const uint32_t wait_until = HAL_GetTick() + timeout;
  while(HWJPEG_is_running)
  {
    __WFI();
    if (hjpeg.hdmaout->Instance->CDAR >= (uint32_t)((uint8_t*)JPEG_buffer + sizeof JPEG_buffer))
    {
      ShowNotify(200, "视频画面数据错误");
      HAL_JPEG_Abort(&hjpeg);
      HAL_JPEG_DeInit(&hjpeg);
      break;
    }
    if (HAL_GetTick() > wait_until)
    {
      ShowNotify(200, "视频画面解码超时");
      goto FailExit;
    }
  }
  return 1;
FailExit:
  return 0;
}
ITCM_CODE
HAL_StatusTypeDef JPEG_HWDecode(void *decode_to)
{
  uint32_t in_size = HWJPEG_src_size;
  if (in_size > JPEG_CHUNK_SIZE_IN) in_size = JPEG_CHUNK_SIZE_IN;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK) goto FailExit;
  if (HAL_JPEG_EnableHeaderParsing(&hjpeg) != HAL_OK) goto FailExit;
  HWJPEG_src_pointer = FILE_buffer;
  HWJPEG_dst_pointer = (uint8_t *)JPEG_buffer;
  HWJPEG_got_info = 0;
  HWJPEG_is_running = 1;
  memset((void*)&HWJpeg_info, 0, sizeof HWJpeg_info);
  SCB_CleanDCache_by_Addr((uint32_t*)FILE_buffer, HWJPEG_src_size);
  if (HAL_JPEG_Decode_DMA(&hjpeg, (uint8_t*)FILE_buffer, in_size, (uint8_t*)HWJPEG_dst_pointer, JPEG_CHUNK_SIZE_OUT) != HAL_OK)
  {
    ShowNotify(200, "解码器外设启动失败");
    goto Skipped;
  }
  if (!JPEG_Wait_Decode(33)) goto Skipped;
  HAL_JPEG_Abort(&hjpeg);
  if (HWJPEG_got_info)
  {
    if (DMA2D_CopyBuffer((uint32_t*)JPEG_buffer, (uint32_t*)CurDrawFramebuffer, HWJpeg_info.ImageWidth, HWJpeg_info.ImageHeight) != HAL_OK)
    {
      ShowNotify(200, "视频画面转码失败");
    }
    if (HAL_DMA2D_PollForTransfer(&hdma2d, 33) != HAL_OK)
    {
      ShowNotify(200, "视频画面转码超时");
      goto Skipped;
    }
    SCB_InvalidateDCache_by_Addr((uint32_t*)CurDrawFramebuffer, sizeof Framebuffer1);
    UnsaturateScreen();
  }
  else
  {
    UnswapFramebuffers();
    HAL_JPEG_DeInit(&hjpeg);
  }
  return HAL_OK;
FailExit:
  HWJPEG_is_running = 0;
  if (hjpeg.ErrorCode & HAL_JPEG_ERROR_TIMEOUT) ShowNotify(1000, "视频画面解码超时");
  if (hjpeg.ErrorCode & HAL_JPEG_ERROR_DMA) ShowNotify(1000, "视频画面解码通信错误");
  if (hjpeg.ErrorCode & HAL_JPEG_ERROR_QUANT_TABLE) ShowNotify(1000, "视频画面解码量化表错误");
  if (hjpeg.ErrorCode & HAL_JPEG_ERROR_HUFF_TABLE) ShowNotify(1000, "视频画面解码解压缩表错误");
  QuitVideoFile();
  return HAL_ERROR;
Skipped:
  HWJPEG_is_running = 0;
  HAL_JPEG_Abort(&hjpeg);
  HAL_JPEG_DeInit(&hjpeg);
  return HAL_OK;
}
ITCM_CODE
void AVIPause()
{
  if (!AVIPaused)
  {
    AVIPaused = 1;
    AVIPausePlayTime = HAL_GetTick64();
  }
}
ITCM_CODE
void AVIResume()
{
  if (AVIPaused)
  {
    uint64_t paused_duration = HAL_GetTick64() - AVIPausePlayTime;
    AVIStartPlayTime += paused_duration;
    AVIPaused = 0;
  }
}
ITCM_CODE
uint64_t AVIGetTime()
{
  return AVIPaused ? AVIPausePlayTime - AVIStartPlayTime : HAL_GetTick64() - AVIStartPlayTime;
}
ITCM_CODE
void AVISetTime(uint64_t time)
{
  if (AVIPaused)
    AVIStartPlayTime = AVIPausePlayTime - time;
  else
    AVIStartPlayTime = HAL_GetTick64() - time;
}
ITCM_CODE
static fssize_t AVIStreamRead(void *buffer, size_t len, void *userdata)
{
  size_t bytes_read;
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;

  if (Phat_ReadFile(stream, buffer, len, &bytes_read) == PhatState_DriverError) QuitVideoFile();
  return bytes_read;
}
ITCM_CODE
static fssize_t AVIStreamTell(void *userdata)
{
  FileSize_t position;
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;
  Phat_GetFilePointer(stream, &position);
  return position;
}
ITCM_CODE
static fssize_t AVIStreamSeek(fsize_t offset, void *userdata)
{
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;
  Phat_SeekFile(stream, offset);
  return AVIStreamTell(userdata);
}
ITCM_CODE
static void AVIPrintf(void *userdata, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  ShowNotifyV(2000, fmt, ap);
  va_end(ap);
  UNUSED(userdata);
}
static void OnVideoCompressed(fsize_t offset, fsize_t length, void *userdata)
{
  size_t bytes_read;
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;
  if (length > sizeof FILE_buffer)
  {
    ShowNotify(200, "帧原始大小 %"PRIfsize_t" 过大", length);
    return;
  }

  Phat_SeekFile(stream, offset);
  if (Phat_ReadFile(stream, FILE_buffer, length, &bytes_read) == PhatState_DriverError) QuitVideoFile();
  if (length == bytes_read)
  {
    HWJPEG_src_size = length;
    JPEG_HWDecode(CurDrawFramebuffer);
  }
  else
  {
    ShowNotify(200, "读取视频帧失败");
  }
}
ITCM_CODE
static void OnAudio(fsize_t offset, fsize_t length, void *userdata)
{
  size_t bytes_read;
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;
  uint16_t channels = avi_audio_stream.stream_info->audio_format.nChannels;

  i2saudio_set_volume(&i2saudio, CurSettings.CurVolume);
  Phat_SeekFile(stream, offset);
  while (length)
  {
    const size_t max_length_process = ((sizeof FILE_buffer) / 3) & 0xFFFFFFFC;
    size_t length_process = length;
    size_t written;
    switch (channels)
    {
    case 1:
      if (length_process > max_length_process)
        length_process = max_length_process;
      break;
    case 2:
      if (length > sizeof FILE_buffer)
        length_process = sizeof FILE_buffer;
      break;
    default:
      return;
    }
    Phat_ReadFile(stream, FILE_buffer, length_process, &bytes_read);
    if (length_process == bytes_read)
    {
      uint16_t *samples;
      size_t sample_count = 0;
      if (channels == 1)
      {
        uint16_t *mono_samples = (uint16_t *)FILE_buffer;
        size_t mono_sample_count = length_process / 2;
        samples = (uint16_t *)FILE_buffer + mono_sample_count;
        for (size_t i = 0; i < mono_sample_count; i++)
        {
          samples[i * 2 + 0] = mono_samples[i];
          samples[i * 2 + 1] = mono_samples[i];
        }
        sample_count = mono_sample_count * 2;
      }
      else
      {
        samples = (uint16_t *)FILE_buffer;
        sample_count = length_process / 2;
      }
      i2saudio_feed_samples_to_play(&i2saudio, samples, sample_count, &written);
      if (written != sample_count)
      {
        ShowNotify(200, "音频丢包");
      }
      length -= length_process;
    }
    else
    {
      ShowNotify(200, "读取音频失败");
      return;
    }
  }
}
void OnUSBFail()
{
  longjmp(USBFailJmp, 1);
}
int SubtitleSeek(uint32_t offset, void *userdata)
{
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;
  PhatState res = Phat_SeekFile(stream, offset);
  return res == PhatState_OK;
}
int SubtitleTell(uint32_t *offset, void *userdata)
{
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;
  FileSize_t fp;
  Phat_GetFilePointer(stream, &fp);
  *offset = fp;
  return 1;
}
size_t SubtitleRead(void *buffer, size_t size, void *userdata)
{
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;
  size_t read = 0;
  Phat_ReadFile(stream, buffer, size, &read);
  return read;
}
static void PrepareSubtitleFile()
{
  PhatState res;
  static WChar_t SubtitleFilename[MAX_FILE_NAMELEN];
  WChar_t *dot = SubtitleFilename;
  strncpyW(SubtitleFilename, GUIFileName, MAX_FILE_NAMELEN);
  while(*dot) dot ++;
  while (dot > SubtitleFilename && *dot != u'.') dot --;
  if (dot == SubtitleFilename)
  {
    SubtitlePrepped = 0;
    return;
  }
  strcpyW(dot, u".srt");
  res = Phat_OpenFile(&GUICurDir, SubtitleFilename, 1, &CurFileStream4);
  if (res != PhatState_OK)
  {
    SubtitlePrepped = 0;
    return;
  }
  srt_init(&SubtitleParser, SubtitleSeek, SubtitleTell, SubtitleRead, &CurFileStream4);
  SubtitlePrepped = 1;
  CurSubtitleSlot = NULL;
}
static void PrepareVideoFile()
{
  PhatState res;
  wave_format_ex *avi_audio_format;
  res = Phat_OpenFile(&GUICurDir, GUIFileName, 1, &CurFileStream1);
  if (res != PhatState_OK)
  {
    ShowNotify(1000, "无法打开文件（%s）", Phat_StateToString(res));
    goto FailExit;
  }
  res = Phat_OpenFile(&GUICurDir, GUIFileName, 1, &CurFileStream2);
  if (res != PhatState_OK)
  {
    ShowNotify(1000, "无法打开文件（%s）", Phat_StateToString(res));
    goto FailExit;
  }
  res = Phat_OpenFile(&GUICurDir, GUIFileName, 1, &CurFileStream3);
  if (res != PhatState_OK)
  {
    ShowNotify(1000, "无法打开文件（%s）", Phat_StateToString(res));
    goto FailExit;
  }
  memset(&avir, 0, sizeof avir);
  memset(&avi_meta_stream, 0, sizeof avi_meta_stream);
  memset(&avi_video_stream, 0, sizeof avi_video_stream);
  memset(&avi_audio_stream, 0, sizeof avi_audio_stream);
  if (!avi_reader_init(&avir, &CurFileStream1, AVIStreamRead, AVIStreamSeek, AVIStreamTell, AVIPrintf, PRINT_FATAL)) goto FailExit;
  if (!avi_map_stream_readers(&avir, &CurFileStream2, &CurFileStream3, OnVideoCompressed, NULL, NULL, OnAudio, &avi_video_stream, &avi_audio_stream)) goto FailExit;
  avi_audio_format = &avi_audio_stream.stream_info->audio_format;
  if (avi_video_stream.stream_info->stream_header.fccHandler != 0x47504A4D) goto BadVideoFormat;
  if (avi_audio_format->wFormatTag != 1 && avi_audio_format->wFormatTag != 0xFFFE) goto BadAudioFormat;
  if (avi_audio_format->wBitsPerSample != 16) goto BadAudioFormat;
  if (avi_audio_format->nChannels != 1 && avi_audio_format->nChannels != 2) goto BadAudioFormat;
  if (avi_audio_format->nBlockAlign != 2 * avi_audio_format->nChannels) goto BadAudioFormat;
  i2saudio_init(&i2saudio, &hi2s2, CurSettings.CurVolume, avi_audio_format->nSamplesPerSec);
  PrepareSubtitleFile();
  AVIPaused = 0;
  DrawStandByScreen();
  GUIIsUsingFile = 1;
  AVIStartPlayTime = HAL_GetTick64();
  return;
BadVideoFormat:
  ShowNotify(1000, "不支持的视频格式");
  goto FailExit;
BadAudioFormat:
  ShowNotify(1000, "不支持的音频格式");
  goto FailExit;
FailExit:
  QuitVideoFile();
}
static void QuitBugFile()
{
  GUIIsUsingFile = 0;
  Phat_CloseFile(&CurFileStream1);
}
static void PrepareBugFile()
{
  PhatState res;
  FileSize_t size;
  size_t bytes_read;
  FlashMap_t *ptr = (FlashMap_t *)FILE_buffer;
  res = Phat_OpenFile(&GUICurDir, GUIFileName, 1, &CurFileStream1);
  if (res != PhatState_OK)
  {
    ShowNotify(1000, "无法打开文件（%s）", Phat_StateToString(res));
    goto FailExit;
  }
  Phat_GetFileSize(&CurFileStream1, &size);
  if (size < sizeof *ptr)
  {
    ShowNotify(1000, "文件太小，不是固件升级文件");
    goto FailExit;
  }
  if (Phat_ReadFile(&CurFileStream1, ptr, sizeof *ptr, &bytes_read) == PhatState_DriverError)
  {
    ShowNotify(1000, "无法读取文件（%s）", Phat_StateToString(res));
    goto FailExit;
  }
  if (bytes_read != sizeof *ptr)
  {
    ShowNotify(1000, "加载文件失败");
    goto FailExit;
  }
  if (ptr->Signature == 0xAA55 && !(ptr->Version & 0xFF000000) && size <= 8 * 1024 * 1024)
  {
    if (!(FLASH_MAP->Version & 0xFF000000) && ptr->Version < FLASH_MAP->Version)
    {
      ShowNotify(1000, "这个固件的版本低于当前固件的版本");
      goto FailExit;
    }
    BugFileAgreed = 0;
    GUIIsUsingFile = 1;
    return;
  }
  ShowNotify(1000, "不是固件升级文件 %x %x %u", ptr->Signature, ptr->Version, size);
FailExit:
  QuitBugFile();
}
static void QuitFileList()
{
  if (FsMounted)
  {
    Phat_DeInit(&phat);
    FsMounted = 0;
  }
  GUICurMenuLevel = 0;
}
static void UpdateLastFileIndex()
{
  PhatState res;
  for(int i = 0;; i++)
  {
    res = Phat_NextDirItem(&GUICurDir);
    if (res == PhatState_EndOfDirectory)
    {
      GUILastFileIndex = i - 1;
      break;
    }
    if (res != PhatState_OK)
    {
      QuitFileList();
      break;
    }
  }
}
void OnFileListGUI(uint64_t cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
{
  PhatState res;
  if (!FsMounted)
  {
    if ((res = Phat_Init(&phat)) != PhatState_OK)
    {
      if (res == PhatState_DriverError)
        strcpy(FormatBuf, "未检测到存储卡。请插入存储卡");
      else
        snprintf(FormatBuf, sizeof FormatBuf, "初始化存储卡失败(%s)，请尝试更换存储卡", Phat_StateToString(res));
      Phat_DeInit(&phat);
      DrawStandByScreen();
      DrawTextOpaque(40, 110, 240, 80, FormatBuf, MakePixel565(255, 255, 255), MakePixel565(0, 0, 0));
    }
    else if ((res = Phat_Mount(&phat, 0, 0)) != PhatState_OK)
    {
      Phat_DeInit(&phat);
      snprintf(FormatBuf, sizeof FormatBuf, "无法挂载存储卡(%s)，请尝试更换存储卡", Phat_StateToString(res));
      DrawStandByScreen();
      DrawTextOpaque(30, 110, 260, 80, FormatBuf, MakePixel565(255, 255, 255), MakePixel565(0, 0, 0));
    }
    else
    {
      FsMounted = 1;
      GUICurFileIndex = 0;
      GUIFirstFileIndex = 0;
      GUILastFileIndex = 0;
      GUIIsUsingFile = 0;
      Phat_OpenRootDir(&phat, &GUICurDir);
      UpdateLastFileIndex();
    }
  }
  if (FsMounted)
  {
    if (cur_tick - GUIFileListRefreshTime > 1000)
    {
      res = Phat_FlushCache(&phat, 1);
      if (res != PhatState_OK)
      {
        ShowNotify(1000, "刷新缓存失败（%s）", Phat_StateToString(res));
        goto OnFsError;
      }
    }
    if (enc1_delta)
    {
      GUICurFileIndex += enc1_delta;
    }
    ClearScreen(MakePixel565(0, 0, 0));
    if (GUICurFileIndex < 0 ) GUICurFileIndex = 0;
    if (GUICurFileIndex > GUILastFileIndex) GUICurFileIndex = GUILastFileIndex;
    if (GUIFirstFileIndex < 0) GUIFirstFileIndex = 0;
    if (GUICurFileIndex > GUIFirstFileIndex + (NUM_FILE_ITEMS - 1))
    {
      GUIFirstFileIndex = GUICurFileIndex - (NUM_FILE_ITEMS - 1);
    }
    if (GUICurFileIndex < GUIFirstFileIndex)
    {
      GUIFirstFileIndex = GUICurFileIndex;
    }
    if (GUIFirstFileIndex > GUILastFileIndex) GUIFirstFileIndex = GUILastFileIndex;
    GUICurDir.cur_diritem = 0;
    for (size_t i = 0; i < GUIFirstFileIndex; i++)
    {
      res = Phat_NextDirItem(&GUICurDir);
      if (res == PhatState_EndOfDirectory) break;
      if (res != PhatState_OK)
      {
        ShowNotify(1000, "列出文件失败（%s）", Phat_StateToString(res));
        goto OnFsError;
      }
    }
    SetWordWrap(0);
    for(size_t i = 0; i < NUM_FILE_ITEMS; i++)
    {
      int file_index = GUIFirstFileIndex + i;
      int y = i * 17;
      uint8_t filetype = 0;
      res = Phat_NextDirItem(&GUICurDir);
      if (res == PhatState_EndOfDirectory) break;
      if (res != PhatState_OK)
      {
        ShowNotify(1000, "列出文件失败（%s）", Phat_StateToString(res));
        goto OnFsError;
      }
      filetype = GetCurFileType();
      DrawFileIcon(0, y, filetype);
      DrawTextW(17, y, 300, 20, GUICurDir.LFN_name, MakePixel565(255, 255, 255));
      if (file_index == GUICurFileIndex)
      {
        InvertRect(17, y + 1, 300, 18, 1);
        strcpyW(GUIFileName, GUICurDir.LFN_name);
        GUIFileType = filetype;
      }
    }
    SetWordWrap(1);
    if (enc1_click)
    {
      switch (GUIFileType)
      {
      case 0: //Folder
        if (!strcmpW(GUIFileName, u"System Volume Information"))
          ShowNotify(500, "无法进入文件夹");
        else
        {
          res = Phat_ChDir(&GUICurDir, GUIFileName);
          if (res != PhatState_OK)
          {
            ShowNotify(1000, "进入文件夹失败（%s）", Phat_StateToString(res));
            goto OnFsError;
          }
          GUIFirstFileIndex = 0;
          GUICurFileIndex = 0;
          UpdateLastFileIndex();
        }
        break;
      case 1: //Text file
        PrepareTextFile();
        break;
      case 2: // Video file
        PrepareVideoFile();
        break;
      case 3: // Bug file
        PrepareBugFile();
        break;
      default: // Unknown file
        ShowNotify(500, "无法打开文件");
        break;
      }
    }
  }
  if (enc2_click)
  {
    if (FsMounted)
    {
      if (GUICurDir.dir_start_cluster <= 2)
        QuitFileList();
      else
      {
        res = Phat_ChDir(&GUICurDir, u"..");
        if (res == PhatState_DirectoryNotFound)
        {
          Phat_OpenRootDir(&phat, &GUICurDir);
          res = PhatState_OK;
        }
        if (res != PhatState_OK)
        {
          ShowNotify(1000, "打开路径失败（%s）", Phat_StateToString(res));
          goto OnFsError;
        }
        else
        {
          GUIFirstFileIndex = 0;
          GUICurFileIndex = 0;
          UpdateLastFileIndex();
        }
      }
    }
    else
    {
      QuitFileList();
    }
  }
  if (enc2_delta)
  {
    CurSettings.CurVolume += enc2_delta * 5;
    if (CurSettings.CurVolume < 0) CurSettings.CurVolume = 0;
    if (CurSettings.CurVolume > 100) CurSettings.CurVolume = 100;
    ShowVolume(200);
  }
  DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
  return;
OnFsError:
  Phat_DeInit(&phat);
  FsMounted = 0;
}
void OnUsingTextFileGUI(uint64_t cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
{
  const int scroll_bar_y = 10;
  const int scroll_bar_max_height = FramebufferHeight - scroll_bar_y;
  int y;
  int text_size = 17;
  int max_read_pos = 0;
  int lines_per_screen;
  int scroll_bar_size;
  int scroll_bar_movable_size;
  int scroll_bar_pos;
  GUITextFileReadPos += enc1_delta;
  GUITextFileTextSize += enc2_delta;
  if (GUITextFileTextSize < 0) GUITextFileTextSize = 0;
  if (GUITextFileTextSize > 2) GUITextFileTextSize = 2;
  ClearScreen(MakePixel565(0, 0, 0));
  switch(GUITextFileTextSize)
  {
    case 0:
      UseLargeFont();
      text_size = 17;
      max_read_pos = GUITextFileMaxReadPosL;
      break;
    case 1:
      UseMediumFont();
      text_size = 14;
      max_read_pos = GUITextFileMaxReadPosM;
      break;
    case 2:
      UseSmallFont();
      text_size = 12;
      max_read_pos = GUITextFileMaxReadPosS;
      break;
  }
  if (GUITextFileReadPos > max_read_pos) GUITextFileReadPos = max_read_pos;
  if (GUITextFileReadPos < 0) GUITextFileReadPos = 0;
  y = -GUITextFileReadPos * text_size;
  if (*(uint16_t*)TXT_buffer == 0xFEFF)
    DrawTextW(0, y, GUITextAreaWidth, FramebufferHeight - y, (uint16_t*)TXT_buffer, MakePixel565(255, 255, 255));
  else
    DrawText(0, y, GUITextAreaWidth, FramebufferHeight - y, (char*)TXT_buffer, MakePixel565(255, 255, 255));
  lines_per_screen = FramebufferHeight / text_size;
  scroll_bar_size = lines_per_screen * scroll_bar_max_height / (max_read_pos + lines_per_screen);
  scroll_bar_movable_size = scroll_bar_max_height - scroll_bar_size;
  scroll_bar_pos = scroll_bar_y + GUITextFileReadPos * scroll_bar_movable_size / max_read_pos;
  FillRect(FramebufferWidth - 2, scroll_bar_pos, 2, scroll_bar_size, MakePixel565(100, 200, 255));
  UseLargeFont();
  if (enc2_click)
  {
    GUIIsUsingFile = 0;
  }
  DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
}
void OnUsingVideoFileGUI(uint64_t cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
{
  uint64_t time = AVIGetTime();
  int have_video = (avi_video_stream.r != 0);
  int have_audio = (avi_audio_stream.r != 0);

  if (!have_video && !have_audio)
  {
    QuitVideoFile();
    return;
  }
  if (enc1_click)
  {
    if (AVIPaused)
    {
      fsize_t target_byte = avi_audio_get_target_byte_offset_by_time(&avi_audio_stream, time);
      if (!avi_audio_seek_to_byte_offset(&avi_audio_stream, target_byte, 1))
      {
        QuitVideoFile();
        return;
      }
      AVIResume();
    }
    else
    {
      AVIPause();
      i2saudio_stop(&i2saudio);
    }
  }
  if (enc1_delta)
  {
    if (enc1_delta < 0)
    {
      if (time > FASTFORWARD_TIME)
      {
        time -= FASTFORWARD_TIME;
        ShowNotify(100, "快退%d秒", FASTFORWARD_TIME / 1000);
      }
      else
      {
        time = 0;
        ShowNotify(100, "已经不能快退了");
      }
    }
    else
    {
      if ((have_video && !avi_video_stream.is_no_more_packets) || (have_audio && !avi_audio_stream.is_no_more_packets))
      {
        time += FASTFORWARD_TIME;
        ShowNotify(100, "快进%d秒", FASTFORWARD_TIME / 1000);
      }
      else
      {
        ShowNotify(100, "已经不能快进了");
      }
    }
    AVISetTime(time);
    if (have_audio && !AVIPaused)
    {
      fsize_t target_byte;
      i2saudio_stop(&i2saudio);
      target_byte = avi_audio_get_target_byte_offset_by_time(&avi_audio_stream, time);
      if (!avi_audio_seek_to_byte_offset(&avi_audio_stream, target_byte, 1))
      {
        QuitVideoFile();
        return;
      }
    }
  }

  SCB_CleanDCache_by_Addr((uint32_t*)CurDrawFramebuffer, sizeof Framebuffer1);

  if (have_video)
  {
    fsize_t target_frame = avi_video_get_frame_number_by_time(&avi_video_stream, time);
    if (!avi_video_seek_to_frame_index(&avi_video_stream, target_frame, 1)) QuitVideoFile();
  }

  if (have_audio && !avi_audio_stream.is_no_more_packets && !AVIPaused)
  {
    while (i2saudio_need_data(&i2saudio))
    {
      if (!avi_stream_reader_move_to_next_packet(&avi_audio_stream, 1))
      {
        QuitVideoFile();
        return;
      }
    }
  }

  if ((have_video && avi_video_stream.is_no_more_packets) || (have_audio && avi_audio_stream.is_no_more_packets))
  {
    if (!AVIPaused) QuitVideoFile();
    else avi_stream_reader_call_callback_functions(&avi_video_stream);
  }
  if (enc2_click)
  {
    QuitVideoFile();
  }
  if (enc2_delta)
  {
    CurSettings.CurVolume += enc2_delta * 5;
    if (CurSettings.CurVolume < 0) CurSettings.CurVolume = 0;
    if (CurSettings.CurVolume > 100) CurSettings.CurVolume = 100;
    ShowVolume(200);
  }

  if (SubtitlePrepped)
  {
    srt_slot_p cur_slot = srt_get_subtitle(&SubtitleParser, time);
    if (!cur_slot)
      CurSubtitleSlot = NULL;
    else if (time > cur_slot->time_end_ms)
      CurSubtitleSlot = NULL;
    if (cur_slot && cur_slot != CurSubtitleSlot)
    {
      size_t read;
      size_t supposed_to_read = cur_slot->length;
      if (supposed_to_read > (sizeof FormatBuf) - 1) supposed_to_read = (sizeof FormatBuf) - 1;
      FormatBuf[supposed_to_read] = 0;
      Phat_SeekFile(&CurFileStream4, cur_slot->offset_in_file);
      Phat_ReadFile(&CurFileStream4, FormatBuf, supposed_to_read, &read);
      if (read == cur_slot->length)
      {
        char *ptr = FormatBuf;
        WChar_t *uptr = CurSubtitle;
        while(*ptr)
        {
          ptr += FLASH_MAP->CP936_to_Unicode(ptr, uptr++, u'?');
        }
        *uptr-- = 0;
        while (uptr > CurSubtitle && (*uptr == '\r' || *uptr == '\n')) *uptr-- = 0;
      }
      else
        cur_slot = NULL;
      CurSubtitleSlot = cur_slot;
      if (CurSubtitleSlot)
      {
        uint32_t w, h;
        UseSmallFont();
        SetJustify(1);
        GetTextSizeW(CurSubtitle, FramebufferWidth, FramebufferHeight, &w, &h);
        SetJustify(0);
        CurSubtitleY = FramebufferHeight - h;
        UseLargeFont();
      }
    }
    if (CurSubtitleSlot && time <= CurSubtitleSlot->time_end_ms)
    {
      UseSmallFont();
      SetJustify(1);
      DrawTextOpaqueW(0, CurSubtitleY, FramebufferWidth, FramebufferHeight - CurSubtitleY, CurSubtitle, MakePixel565(255, 255, 255), MakePixel565(0, 0, 0));
      SetJustify(0);
      UseLargeFont();
    }
  }

  DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
}
void OnUsingBugFileGUI(uint64_t cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
{
  HAL_StatusTypeDef hres;
  PhatState pres;
  uint32_t program_address = 0;
  FileSize_t filesize;
  int *some_8kb_buffer = (int *)JPEG_buffer;
  uint32_t num_blocks;
  uint32_t block;

  ClearScreen(MakePixel565(0, 0, 0));
  if (!BugFileAgreed)
  {
    DrawText(40, 100, 240, 140, "将要进行固件升级\n按下按钮1进行升级\n按下按钮2取消升级\n升级过程中不要移除内存卡\n也不要断电", MakePixel565(255, 255, 255));
    DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
    if (enc1_click)
    {
      BugFileAgreed = 1;
      UseDefaultFont();
    }
    if (enc2_click)
    {
      GUIIsUsingFile = 0;
    }
    return;
  }

  ClearScreen(MakePixel565(0, 0, 0));
  DrawText(40, 100, 240, 140, "VERIFYING", MakePixel565(255, 255, 255));
  SwapFramebuffers();

  pres = Phat_SeekFile(&CurFileStream1, 0);
  if (pres != PhatState_OK)
  {
    snprintf(FormatBuf, sizeof FormatBuf, "SEEK FILE FAILED");
    goto FailExit;
  }

  Phat_GetFileSize(&CurFileStream1, &filesize);

  num_blocks = (filesize - 1) / 4096 + 1;
  for(block = 0; block < num_blocks; block++)
  {
    size_t bytes_to_read = 4096;
    size_t bytes_read;
    int percentage = program_address * 100 / filesize;
    if (program_address + bytes_to_read > filesize) bytes_to_read = filesize - program_address;

    pres = Phat_ReadFile(&CurFileStream1, FILE_buffer, bytes_to_read, &bytes_read);
    if (bytes_to_read != bytes_read)
    {
      snprintf(FormatBuf, sizeof FormatBuf, "VERIFY FAILED: IO ERROR (%s)", Phat_StateToString(pres));
      goto FailExit;
    }

    if (!LCD_IsBusy(&hlcd))
    {
      ClearScreen(MakePixel565(0, 0, 0));
      snprintf(FormatBuf, sizeof FormatBuf, "VERIFYING (%d %%)", percentage);
      DrawText(40, 100, 240, 140, FormatBuf, MakePixel565(255, 255, 255));
      FillRect(40, 120, 240, 2, MakePixel565(127, 127, 127));
      FillRect(40, 120, program_address * 240 / filesize, 2, MakePixel565(255, 255, 255));
      SwapFramebuffers();
    }

    some_8kb_buffer[block] = memcmp(FILE_buffer, (void *)(0x90000000 + program_address), bytes_to_read);

    program_address += bytes_to_read;
  }

  QSPI_ExitMemoryMapMode();

  ClearScreen(MakePixel565(0, 0, 0));
  DrawText(40, 100, 240, 140, "PROGRAMMING", MakePixel565(255, 255, 255));
  SwapFramebuffers();

  program_address = 0;
  pres = Phat_SeekFile(&CurFileStream1, 0);
  if (pres != PhatState_OK)
  {
    snprintf(FormatBuf, sizeof FormatBuf, "SEEK FILE FAILED");
    goto FailExit;
  }

  block = 0;
  while(program_address < filesize)
  {
    size_t bytes_to_read;
    size_t bytes_read;
    int percentage = program_address * 100 / filesize;

    if (some_8kb_buffer[block] != 0)
    {
      if (program_address % 4096 == 0)
      {
        if (!LCD_IsBusy(&hlcd))
        {
          ClearScreen(MakePixel565(0, 0, 0));
          snprintf(FormatBuf, sizeof FormatBuf, "ERASING (%d %%)", percentage);
          DrawText(40, 100, 240, 140, FormatBuf, MakePixel565(255, 255, 255));
          FillRect(40, 120, 240, 2, MakePixel565(127, 127, 127));
          FillRect(40, 120, program_address * 240 / filesize, 2, MakePixel565(255, 255, 255));
          SwapFramebuffers();
        }

        hres = QSPI_SectorErase(program_address);
        if (hres != HAL_OK)
        {
          snprintf(FormatBuf, sizeof FormatBuf, "ERASE SECTOR FAILED: QSPI PERIPHERAL ERROR");
          goto FailExit;
        }
      }

      if (!LCD_IsBusy(&hlcd))
      {
        ClearScreen(MakePixel565(0, 0, 0));
        snprintf(FormatBuf, sizeof FormatBuf, "PROGRAMMING (%d %%)", percentage);
        DrawText(40, 100, 240, 140, FormatBuf, MakePixel565(255, 255, 255));
        FillRect(40, 120, 240, 2, MakePixel565(127, 127, 127));
        FillRect(40, 120, program_address * 240 / filesize, 2, MakePixel565(255, 255, 255));
        SwapFramebuffers();
      }

      bytes_to_read = filesize - program_address;
      if (bytes_to_read > 256) bytes_to_read = 256;
      memset(FILE_buffer, 0xFF, 256);
      pres = Phat_ReadFile(&CurFileStream1, FILE_buffer, bytes_to_read, &bytes_read);
      if (bytes_to_read != bytes_read)
      {
        snprintf(FormatBuf, sizeof FormatBuf, "PROGRAMMING FAILED: IO ERROR (%s)", Phat_StateToString(pres));
        goto FailExit;
      }
      hres = QSPI_PageProgram(program_address, FILE_buffer);
      if (hres != HAL_OK)
      {
        snprintf(FormatBuf, sizeof FormatBuf, "PROGRAMMING FAILED: QSPI PERIPHERAL ERROR");
        goto FailExit;
      }
    }
    else
    {
      if (!LCD_IsBusy(&hlcd))
      {
        ClearScreen(MakePixel565(0, 0, 0));
        snprintf(FormatBuf, sizeof FormatBuf, "SKIPPING (%d %%)", percentage);
        DrawText(40, 100, 240, 140, FormatBuf, MakePixel565(255, 255, 255));
        FillRect(40, 120, 240, 2, MakePixel565(127, 127, 127));
        FillRect(40, 120, program_address * 240 / filesize, 2, MakePixel565(255, 255, 255));
        SwapFramebuffers();
      }
    }
    program_address += 256;
    if (program_address % 4096 == 0) block ++;
  }

  QSPI_InitFlash();
  QSPI_EnterMemoryMapMode();
  UseLargeFont();
  GUIIsUsingFile = 0;
  ShowNotify(2000, "ROM 更新成功");
  return;
FailExit:
  ClearScreen(MakePixel565(0, 0, 0));
  DrawText(40, 100, 240, 140, FormatBuf, MakePixel565(255, 255, 255));
  SwapFramebuffers();
  for(;;)
  {
    if (Enc1Click || Enc2Click) Suicide();
    __WFI();
  }
}

void OnUsingFileGUI(uint64_t cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
{
  if (!FsMounted)
  {
    ShowNotify(1000, "未知错误");
    QuitFileList();
    return;
  }
  switch(GUIFileType)
  {
    default:
      ShowNotify(1000, "未知错误");
      QuitFileList();
      break;
    case 1: //Text file
      OnUsingTextFileGUI(cur_tick, delta_tick, enc1_delta, enc1_click, enc2_delta, enc2_click);
      break;
    case 2: //Video file
      OnUsingVideoFileGUI(cur_tick, delta_tick, enc1_delta, enc1_click, enc2_delta, enc2_click);
      break;
    case 3: //Bug file
      OnUsingBugFileGUI(cur_tick, delta_tick, enc1_delta, enc1_click, enc2_delta, enc2_click);
      break;
  }
}

void OnOptionsGUI(uint64_t cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
{
  ClearScreen(MakePixel565(0, 0, 0));

  FillRect(0, 0, 320, 18, MakePixel565(102, 204, 255));
  DrawText(140, 0, 180, 20, "设置", MakePixel565(0, 0, 0));
  DrawText(0, 20, 320, 240, "设置字幕字体大小\n设置快进、快退步长\n设置待机时间\n管理文件\n本机信息", MakePixel565(255, 255, 255));

  GUICurOptionIndex += enc1_delta;
  if (GUICurOptionIndex < 0) GUICurOptionIndex = 0;
  if (GUICurOptionIndex > 4) GUICurOptionIndex = 4;
  InvertRect(0, 20 + GUICurOptionIndex * 17, 320, 17, 1);

  if (enc2_click) GUICurMenuLevel = 0;
  if (enc2_delta)
  {
    CurSettings.CurVolume += enc2_delta * 5;
    if (CurSettings.CurVolume < 0) CurSettings.CurVolume = 0;
    if (CurSettings.CurVolume > 100) CurSettings.CurVolume = 100;
    ShowVolume(200);
  }
  DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_MDMA_Init();
  MX_SDMMC1_SD_Init();
  MX_JPEG_Init();
  MX_SPI1_Init();
  MX_I2S2_Init();
  MX_QUADSPI_Init();
  MX_ADC1_Init();
  MX_DMA2D_Init();
  MX_CRC_Init();
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
  Unsuicide();
  WaitForPresent();
  HAL_GPIO_WritePin(LCD_PWCTRL_GPIO_Port, LCD_PWCTRL_Pin, GPIO_PIN_SET);
  UseLargeFont();
  UpdatePowerRead();
  HAL_GPIO_EXTI_Callback(ENC1_SW_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int frame_counter = 0;
  uint64_t last_tick = HAL_GetTick64();
  int delta_tick = 0;
  while (1)
  {
    uint64_t cur_tick = HAL_GetTick64();
    int enc1_click = IsEnc1Click();
    int enc2_click = IsEnc2Click();
    int enc1_delta = GetEnc1Delta();
    int enc2_delta = GetEnc2Delta();

    UpdatePowerRead();

    switch (setjmp(SysErrorJmp))
    {
    case 0:
      break;
    case 1:
      GUICurMenuLevel = 0;
      ShowNotify(1000, "系统错误。");
      break;
    }

    switch (GUICurMenuLevel)
    {
      case 0:
        OnMainMenu(cur_tick, delta_tick, enc1_delta, enc1_click, enc2_delta, enc2_click);
        break;
      case 1:
        switch (GUICurMenu)
        {
          case 0: // TF card
            if (!GUIIsUsingFile)
              OnFileListGUI(cur_tick, delta_tick, enc1_delta, enc1_click, enc2_delta, enc2_click);
            else
              OnUsingFileGUI(cur_tick, delta_tick, enc1_delta, enc1_click, enc2_delta, enc2_click);
            break;
          case 1: // USB
            DrawUSBMSCScreen();
            DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
            if (enc2_click)
            {
              GUICurMenuLevel = 0;
            }
            break;
          case 2: // Option
            OnOptionsGUI(cur_tick, delta_tick, enc1_delta, enc1_click, enc2_delta, enc2_click);
            break;
          case 3: // Shutdown
            DrawStandByScreen();
            Suicide();
            if (enc2_click)
            {
              GUICurMenuLevel = 0;
              Unsuicide();
            }
            break;
        }
    }

    if (GUIVolumeShow)
    {
      if (cur_tick <= GUIVolumeShowTimeUntil)
        DrawVolume(CurSettings.CurVolume);
      else
      {
        SaveSettings();
        GUIVolumeShow = 0;
      }
    }

    if (GUINotifyShow)
    {
      if (cur_tick <= GUINotifyTimeUntil)
        DrawNotifyInfo(280, MakePixel565(255, 255, 255), MakePixel565(255, 255, 255), MakePixel565(0, 0, 0), GUINotifyInfo);
      else
        GUINotifyShow = 0;
    }

    SwapFramebuffers();
    frame_counter += 1;

    if (BAT_Voltage >= 2000 && BAT_Voltage <= 3400 && !BAT_IsCharging && !BAT_IsFull)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 96;
  PeriphClkInitStruct.PLL3.PLL3P = 4;
  PeriphClkInitStruct.PLL3.PLL3Q = 8;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hdma2d.Init.Mode = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_YCBCR;
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
  #define HAL_SD_Init(x) HAL_OK
  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 1;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */
  #undef HAL_SD_Init
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
  * Enable MDMA controller clock
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

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

  /*Configure GPIO pins : ENC1_SW_Pin ENC1_A_Pin ENC1_B_Pin ENC2_SW_Pin
                           ENC2_A_Pin ENC2_B_Pin */
  GPIO_InitStruct.Pin = ENC1_SW_Pin|ENC1_A_Pin|ENC1_B_Pin|ENC2_SW_Pin
                          |ENC2_A_Pin|ENC2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ENC1_SW_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ENC1_SW_EXTI_IRQn);

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
  longjmp(SysErrorJmp, 1);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
