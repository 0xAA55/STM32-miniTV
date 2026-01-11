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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
MDMA_HandleTypeDef hmdma_jpeg_outfifo_ne;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

/* USER CODE BEGIN PV */
const int FramebufferWidth = 320;
const int FramebufferHeight = 240;
const int GUITextAreaWidth = FramebufferWidth - 3;
int USB_SDCardReady;
LCD hlcd;
Pixel565 Framebuffer1[240][320];
Pixel565 Framebuffer2[240][320];
uint8_t FILE_buffer[320 * 240 * 2]; // Same size as the framebuffer
Pixel565 (*CurDrawFramebuffer)[320] = Framebuffer1;
Phat_t phat;
avi_reader avir;
avi_stream_reader avi_meta_stream;
avi_stream_reader avi_video_stream;
avi_stream_reader avi_audio_stream;
i2saudio_t i2saudio;
int FsMounted;
int CurVolume = 100;
int GUICurMenu;
int GUICurMenuLevel;
int GUIMenuAnim;
int GUIMenuReady;
int GUICurFileIndex;
int GUIFirstFileIndex;
int GUILastFileIndex;
WChar_t GUIFileName[MAX_FILE_NAMELEN];
uint8_t GUIFileType;
Phat_DirInfo_t GUICurDir;
Phat_FileInfo_t CurFileStream1;
Phat_FileInfo_t CurFileStream2;
Phat_FileInfo_t CurFileStream3;
volatile int GUIIsUsingFile;
int GUITextFileSize;
int GUITextFileTextSize;
int GUITextFileReadPos;
int GUITextFileMaxReadPosS;
int GUITextFileMaxReadPosM;
int GUITextFileMaxReadPosL;
uint64_t AVIStartPlayTime;
uint64_t AVIPausePlayTime;
int AVIPaused;
int GUINotifyShow;
uint32_t GUINotifyTimeUntil;
char GUINotifyInfo[256];
int GUIVolumeShow;
uint32_t GUIVolumeShowTimeUntil;
char FormatBuf[256];
volatile int BAT_IsCharging;
volatile int BAT_IsFull;
volatile int Enc1;
volatile int Enc2;
volatile uint32_t BAT_ADC_VAL;
volatile int BAT_ADC_Sampling;
volatile int MainBtnClick;
volatile int SecondBtnClick;
volatile int BAT_Voltage;
volatile int HWJPEG_is_running;
volatile uint8_t* HWJPEG_src_pointer;
volatile uint8_t* HWJPEG_dst_pointer;
volatile size_t HWJPEG_src_size;
volatile uint32_t TickHigh;
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
#ifndef DEBUG
__attribute__((section(".itcm_code"))) void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd);
__attribute__((section(".itcm_code"))) void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd);
__attribute__((section(".itcm_code"))) void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd);
__attribute__((section(".itcm_code"))) void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd);
__attribute__((section(".itcm_code"))) void BSP_SD_ReadCpltCallback(void);
__attribute__((section(".itcm_code"))) void BSP_SD_WriteCpltCallback(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
__attribute__((section(".itcm_code")))
void HAL_IncTick(void)
{
  uint32_t old_tick = uwTick;
  uwTick += (uint32_t)uwTickFreq;
  if (uwTick < old_tick) TickHigh ++;
}
__attribute__((section(".itcm_code")))
uint64_t HAL_GetTick64()
{
  uint64_t ret;
  __disable_irq();
  ret = ((uint64_t)TickHigh << 32) | uwTick;
  __enable_irq();
  return ret;
}
__attribute__((section(".itcm_code")))
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  __attribute__((section(".dtcm_bss"))) static int adc_read1;
  __attribute__((section(".dtcm_bss"))) static int adc_read2;
  __attribute__((section(".dtcm_bss"))) static int adc_read3;
  __attribute__((section(".dtcm_bss"))) static int adc_read4;
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
  SCB_CleanDCache_by_Addr((uint32_t*)CurDrawFramebuffer, sizeof Framebuffer1);
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
int IsEnc1Click()
{
  int ret = MainBtnClick;
  MainBtnClick = 0;
  return ret;
}
int IsEnc2Click()
{
  int ret = SecondBtnClick;
  SecondBtnClick = 0;
  return ret;
}
int GetEnc1Delta()
{
  __attribute__((section(".dtcm_bss"))) static int last_enc1;
  int enc1_val = Enc1 / 2;
  int ret = enc1_val - last_enc1;
  last_enc1 = enc1_val;
  return ret;
}
int GetEnc2Delta()
{
  __attribute__((section(".dtcm_bss"))) static int last_enc2;
  int enc2_val = Enc2 / 2;
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
    if (IsEnc1Click()) Suicide();
  }
}
void ShowNotifyV(uint32_t duration, const char *format, va_list ap)
{
  char *from;
  size_t size;
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
  GUINotifyTimeUntil = HAL_GetTick() + duration;
  GUINotifyShow = 1;
}
void ShowNotify(uint32_t duration, const char *format, ...)
{
  va_list ap;
  va_start(ap, format);
  ShowNotifyV(duration, format, ap);
  va_end(ap);
}
void ShowVolume(uint32_t duration)
{
  GUIVolumeShowTimeUntil = HAL_GetTick() + duration;
  GUIVolumeShow = 1;
}
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
    }
  }
  return ret;
}
void OnMainMenu(int cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
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
    CurVolume += enc2_delta * 5;
    if (CurVolume < 0) CurVolume = 0;
    if (CurVolume > 100) CurVolume = 100;
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
      GUICurFileIndex = 0;
      GUIFirstFileIndex = 0;
      GUIIsUsingFile = 0;
    }
  }
}
static void PrepareTextFile()
{
  PhatState res;
  FileSize_t filesize;
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
  if (filesize > (sizeof FILE_buffer) - 2)
  {
    Phat_CloseFile(&CurFileStream1);
    ShowNotify(1000, "文件太大了。");
    return;
  }
  res = Phat_ReadFile(&CurFileStream1, FILE_buffer, filesize, NULL);
  if (res != PhatState_OK && res != PhatState_EndOfFile)
  {
    Phat_CloseFile(&CurFileStream1);
    ShowNotify(1000, "读取文件失败。");
    return;
  }
  FILE_buffer[filesize] = 0;
  FILE_buffer[filesize + 1] = 0;
  if (*(uint16_t*)FILE_buffer == 0xFFFE)
  {
    uint32_t max_height;
    uint16_t *ptr = (uint16_t*)FILE_buffer;
    while(*ptr)
    {
      *ptr = BSwap16(*ptr);
      ptr ++;
    }
    UseSmallFont();
    GetTextSizeW((uint16_t*)FILE_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosS = max_height / 12;
    UseMediumFont();
    GetTextSizeW((uint16_t*)FILE_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosM = max_height / 14;
    UseLargeFont();
    GetTextSizeW((uint16_t*)FILE_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosL = max_height / 17;
  }
  else
  {
    uint32_t max_height;
    UseSmallFont();
    GetTextSize((char*)FILE_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosS = max_height / 12;
    UseMediumFont();
    GetTextSize((char*)FILE_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
    GUITextFileMaxReadPosM = max_height / 14;
    UseLargeFont();
    GetTextSize((char*)FILE_buffer, GUITextAreaWidth, INT_MAX, NULL, &max_height);
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
static void QuitVideoFile()
{
  GUIIsUsingFile = 0;
  Phat_CloseFile(&CurFileStream1);
  Phat_CloseFile(&CurFileStream2);
  Phat_CloseFile(&CurFileStream3);
  HAL_JPEG_DeInit(&hjpeg);
  HWJPEG_is_running = 0;
}
void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{
  UNUSED(hjpeg);
  HWJPEG_is_running = 0;
}
void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg)
{
  UNUSED(hjpeg);
  HAL_JPEG_DeInit(hjpeg);
  HWJPEG_is_running = 0;
  ShowNotify(200, "JPEG 解码错误");
}
void HAL_JPEG_InfoReadyCallback(JPEG_HandleTypeDef *hjpeg, JPEG_ConfTypeDef *pInfo)
{
  UNUSED(hjpeg);
  if (pInfo->ImageWidth > FramebufferWidth)
  {
    QuitVideoFile();
    ShowNotify(1000, "视频像素宽度 %u 大于 %u，无法播放。", pInfo->ImageWidth, FramebufferWidth);
  }
  if (pInfo->ImageHeight > FramebufferHeight)
  {
    QuitVideoFile();
    ShowNotify(1000, "视频像素高度 %u 大于 %u，无法播放。", pInfo->ImageHeight, FramebufferHeight);
  }
}
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData)
{
  uint32_t decoded = HWJPEG_src_pointer - FILE_buffer;
  uint32_t in_size = HWJPEG_src_size - decoded;
  if (in_size > JPEG_CHUNK_SIZE_IN) in_size = JPEG_CHUNK_SIZE_IN;
  HWJPEG_src_pointer += NbDecodedData;
  HAL_JPEG_ConfigInputBuffer(hjpeg, HWJPEG_src_pointer, in_size);
}
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
  HWJPEG_dst_pointer += OutDataLength;
  HAL_JPEG_ConfigOutputBuffer(hjpeg, HWJPEG_dst_pointer, JPEG_CHUNK_SIZE_OUT);
}
void JPEG_Wait_Decode()
{
  while(HWJPEG_is_running) __WFI();
}
void JPEG_Decode_DMA(void *decode_to)
{
  JPEG_Wait_Decode();
  uint32_t in_size = HWJPEG_src_size;
  if (in_size > JPEG_CHUNK_SIZE_IN) in_size = JPEG_CHUNK_SIZE_IN;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK) goto FailExit;
  if (HAL_JPEG_EnableHeaderParsing(&hjpeg) != HAL_OK) goto FailExit;
  HWJPEG_src_pointer = FILE_buffer;
  HWJPEG_dst_pointer = (uint8_t *)decode_to;
  HWJPEG_is_running = 1;
  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)HWJPEG_dst_pointer, sizeof Framebuffer1);
  if (HAL_JPEG_Decode_DMA(&hjpeg, HWJPEG_src_pointer, in_size, HWJPEG_dst_pointer, JPEG_CHUNK_SIZE_OUT) != HAL_OK) goto FailExit;
  HWJPEG_src_pointer += in_size;
  return;
FailExit:
  HWJPEG_is_running = 0;
  QuitVideoFile();
  return;
}
void AVIPause()
{
  if (!AVIPaused)
  {
    AVIPaused = 1;
    AVIPausePlayTime = HAL_GetTick64();
  }
}
void AVIResume()
{
  if (AVIPaused)
  {
    uint64_t paused_duration = HAL_GetTick64() - AVIPausePlayTime;
    AVIStartPlayTime += paused_duration;
    AVIPaused = 0;
  }
}
uint64_t AVIGetTime()
{
  return AVIPaused ? AVIPausePlayTime - AVIStartPlayTime : HAL_GetTick64() - AVIStartPlayTime;
}
static fssize_t AVIStreamRead(void *buffer, size_t len, void *userdata)
{
  size_t bytes_read;
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;

  Phat_ReadFile(stream, buffer, len, &bytes_read);
  return bytes_read;
}
static fssize_t AVIStreamTell(void *userdata)
{
  FileSize_t position;
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;
  Phat_GetFilePointer(stream, &position);
  return position;
}
static fssize_t AVIStreamSeek(fsize_t offset, void *userdata)
{
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;
  Phat_SeekFile(stream, offset);
  return AVIStreamTell(userdata);
}
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
  if (length > sizeof FILE_buffer) return;
  while(HWJPEG_is_running) __WFI();

  Phat_ReadFile(stream, FILE_buffer, length, &bytes_read);
  if (length == bytes_read)
  {
    HWJPEG_src_size = length;
    LCD_WaitToIdle(&hlcd);
    JPEG_Decode_DMA(Framebuffer2);
  }
}
static void OnAudio(fsize_t offset, fsize_t length, void *userdata)
{
  Phat_FileInfo_p stream = (Phat_FileInfo_p)userdata;
  //TODO
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
  if (avi_audio_format->wFormatTag != 1 && avi_audio_format->wFormatTag != 0xFFFE) goto FailExit;
  if (avi_audio_format->wBitsPerSample != 16) goto FailExit;
  if (avi_audio_format->nChannels != 1 && avi_audio_format->nChannels != 2) goto FailExit;
  if (avi_audio_format->nBlockAlign != 2 * avi_audio_format->nChannels) goto FailExit;
  i2saudio_init(&i2saudio, &hi2s2, CurVolume, avi_audio_format->nSamplesPerSec);
  AVIStartPlayTime = HAL_GetTick64();
  GUIIsUsingFile = 1;
  return;
FailExit:
  QuitVideoFile();
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
void OnFileListGUI(int cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
{
  PhatState res;
  if (!FsMounted)
  {
    if ((res = Phat_Init(&phat)) != PhatState_OK)
    {
      if (res == PhatState_DriverError)
        strcpy(FormatBuf, "未检测到SD卡。请插入SD卡");
      else
        snprintf(FormatBuf, sizeof FormatBuf, "初始化SD卡失败(%s)，请尝试更换SD卡", Phat_StateToString(res));
      Phat_DeInit(&phat);
      DrawStandByScreen();
      DrawTextOpaque(40, 110, 240, 80, FormatBuf, MakePixel565(255, 255, 255), MakePixel565(0, 0, 0));
    }
    else if ((res = Phat_Mount(&phat, 0, 0)) != PhatState_OK)
    {
      Phat_DeInit(&phat);
      snprintf(FormatBuf, sizeof FormatBuf, "无法挂载SD卡(%s)，请尝试更换SD卡", Phat_StateToString(res));
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
        QuitFileList();
        break;
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
        QuitFileList();
        break;
      }
      filetype = GetCurFileType();
      DrawFileIcon(0, y, filetype);
      DrawTextW(17, y, 280, 20, GUICurDir.LFN_name, MakePixel565(255, 255, 255));
      if (file_index == GUICurFileIndex)
      {
        InvertRect(17, y + 1, 280, 18, 1);
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
        res = Phat_ChDir(&GUICurDir, GUIFileName);
        GUIFirstFileIndex = 0;
        GUICurFileIndex = 0;
        UpdateLastFileIndex();
        break;
      case 1: //Text file
        PrepareTextFile();
        break;
      case 2: // Video file
        PrepareVideoFile();
        break;
      case 3: // Bug file
        ShowNotify(500, "无法打开文件");
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
          QuitFileList();
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
    CurVolume += enc2_delta * 5;
    if (CurVolume < 0) CurVolume = 0;
    if (CurVolume > 100) CurVolume = 100;
    ShowVolume(200);
  }
  DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
}
void OnUsingTextFileGUI(int cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
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
  if (*(uint16_t*)FILE_buffer == 0xFEFF)
    DrawTextW(0, y, GUITextAreaWidth, FramebufferHeight - y, (uint16_t*)FILE_buffer, MakePixel565(255, 255, 255));
  else
    DrawText(0, y, GUITextAreaWidth, FramebufferHeight - y, (char*)FILE_buffer, MakePixel565(255, 255, 255));
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
void OnUsingVideoFileGUI(int cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
{
  uint64_t time = AVIGetTime();
  int have_video = (avi_video_stream.r != 0);
  int have_audio = (avi_audio_stream.r != 0);
  if (!have_video && !have_audio)
  {
    QuitVideoFile();
  }
  CurDrawFramebuffer = Framebuffer2;
  if (have_video)
  {
    fsize_t target_frame = avi_video_get_frame_number_by_time(&avi_video_stream, time);
    avi_video_seek_to_frame_index(&avi_video_stream, target_frame, 1);
  }

  if (have_audio)
  {

  }

  if ((have_video && avi_video_stream.is_no_more_packets) && (have_audio && avi_audio_stream.is_no_more_packets))
  {
    QuitVideoFile();
  }
  if (enc2_click)
  {
    QuitVideoFile();
  }
  if (enc2_delta)
  {
    CurVolume += enc2_delta * 5;
    if (CurVolume < 0) CurVolume = 0;
    if (CurVolume > 100) CurVolume = 100;
    ShowVolume(200);
  }
  DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
}
void OnUsingFileGUI(int cur_tick, int delta_tick, int enc1_delta, int enc1_click, int enc2_delta, int enc2_click)
{
  //TODO
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
      ClearScreen(MakePixel565(0, 0, 0));
      if (enc2_click)
      {
        GUIIsUsingFile = 0;
      }
      DrawBattery(GetPowerPercentage(), BAT_IsCharging, BAT_IsFull);
      break;
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
  MX_USB_DEVICE_Init();
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
  HAL_GPIO_WritePin(PWCTRL_GPIO_Port, PWCTRL_Pin, GPIO_PIN_SET);
  WaitForPresent();
  HAL_GPIO_WritePin(LCD_PWCTRL_GPIO_Port, LCD_PWCTRL_Pin, GPIO_PIN_SET);
  UseLargeFont();
  UpdatePowerRead();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int frame_counter = 0;
  uint32_t last_tick = HAL_GetTick();
  uint32_t delta_tick = 0;
  while (1)
  {
    uint32_t cur_tick = HAL_GetTick();
    int enc1_click = IsEnc1Click();
    int enc2_click = IsEnc2Click();
    int enc1_delta = GetEnc1Delta();
    int enc2_delta = GetEnc2Delta();

    UpdatePowerRead();

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
            ClearScreen(MakePixel565(0, 0, 0));
            if (enc2_click) GUICurMenuLevel = 0;
            break;
          case 2: // Option
            ClearScreen(MakePixel565(0, 0, 0));
            if (enc2_click) GUICurMenuLevel = 0;
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

    if (GUIVolumeShow)
    {
      if (cur_tick <= GUIVolumeShowTimeUntil)
        DrawVolume(CurVolume);
      else
        GUIVolumeShow = 0;
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
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 1;
  /* USER CODE BEGIN SDMMC1_Init 2 */
  // TODO:
  // Everytime you use STM32CubeMX to generate codes, be sure to remove the `HAL_SD_Init()` call above there.
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
