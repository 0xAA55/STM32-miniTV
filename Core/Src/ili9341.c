/*
 * ili9341.c
 *
 *  Created on: Jun 16, 2025
 *      Author: 0xaa55
 */
#include"ili9341.h"
#include <string.h>

#define CHECK_STATE(hlcd) while (hlcd->is_dma_active) __WFI()

void LCD_On_DMA_TX(LCD *hlcd);
void LCD_On_DMA_RX(LCD *hlcd);

extern LCD hlcd;

LCD_Pin LCD_MakePin(GPIO_TypeDef *gpio, uint32_t pin_bit)
{
  LCD_Pin ret =
  {
      gpio,
      pin_bit
  };
  return ret;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == hlcd.hspi)
  {
    LCD_On_DMA_TX(&hlcd);
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == hlcd.hspi)
  {
    LCD_On_DMA_RX(&hlcd);
  }
}

static void Pin_Low(LCD_Pin *pin)
{
  pin->gpio->BSRR = pin->pin_bit;
}

static void Pin_High(LCD_Pin *pin)
{
  pin->gpio->BSRR = pin->pin_bit << 16;
}

HAL_StatusTypeDef LCD_Init
(
  LCD *hlcd,
  SPI_HandleTypeDef *hspi,
  DMA_HandleTypeDef *hdma_spi_tx,
  DMA_HandleTypeDef *hdma_spi_rx,
  const LCD_GPIO *gpio,
  LCD_Color_Mode color_mode,
  LCD_Orient orient
)
{
  hlcd->hspi = hspi;
  hlcd->hdma_spi_tx = hdma_spi_tx;
  hlcd->hdma_spi_rx = hdma_spi_rx;
  hlcd->pin_rst = gpio->pin_rst;
  hlcd->pin_cs = gpio->pin_cs;
  hlcd->pin_dc = gpio->pin_dc;
  hlcd->pin_sck = gpio->pin_sck;
  hlcd->pin_mosi = gpio->pin_mosi;
  hlcd->pin_miso = gpio->pin_miso;
  hlcd->color_mode = color_mode;
  hlcd->orient = orient;
  hlcd->is_dma_active = 0;

  return HAL_OK;
}

static HAL_StatusTypeDef SPI_8bit(SPI_HandleTypeDef *hspi)
{
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  HAL_SPI_Init(hspi);
  return HAL_OK;
}

static HAL_StatusTypeDef SPI_16bit(SPI_HandleTypeDef *hspi)
{
  hspi->Init.DataSize = SPI_DATASIZE_16BIT;
  HAL_SPI_Init(hspi);
  return HAL_OK;
}

static HAL_StatusTypeDef DMA_8bit(DMA_HandleTypeDef *hdma)
{
  hdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  HAL_DMA_Init(hdma);
  return HAL_OK;
}

static HAL_StatusTypeDef DMA_16bit(DMA_HandleTypeDef *hdma)
{
  hdma->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  HAL_DMA_Init(hdma);
  return HAL_OK;
}

static HAL_StatusTypeDef LCD_WriteCommand(LCD *hlcd, uint8_t cmd)
{
  HAL_StatusTypeDef ret = HAL_OK;
  Pin_Low(&hlcd->pin_dc);
  ret = HAL_SPI_Transmit(hlcd->hspi, &cmd, 1, LCD_TIMEOUT);
  Pin_High(&hlcd->pin_dc);
  return ret;
}

static HAL_StatusTypeDef LCD_WriteReg8Multi(LCD *hlcd, uint8_t reg, uint8_t *val, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  ret = HAL_SPI_Transmit(hlcd->hspi, val, count, LCD_TIMEOUT);
  if (ret != HAL_OK) return ret;
  return ret;
}

static HAL_StatusTypeDef LCD_WriteReg8Multi_DMA(LCD *hlcd, uint8_t reg, uint8_t *val, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  ret = HAL_SPI_Transmit_DMA(hlcd->hspi, val, count);
  if (ret != HAL_OK) return ret;
  hlcd->is_dma_active = 1;
  return ret;
}

static HAL_StatusTypeDef LCD_WriteReg8(LCD *hlcd, uint8_t reg, uint8_t val)
{
  return LCD_WriteReg8Multi(hlcd, reg, &val, 1);
}

static HAL_StatusTypeDef LCD_WriteReg16Multi(LCD *hlcd, uint8_t reg, uint16_t *val, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) goto ErrRet;
  ret = SPI_16bit(hlcd->hspi);
  if (ret != HAL_OK) goto ErrRet;
  ret = HAL_SPI_Transmit(hlcd->hspi, (void *)val, count, LCD_TIMEOUT);
  if (ret != HAL_OK) goto ErrRet;

ErrRet:
  ret = SPI_8bit(hlcd->hspi);
  if (ret != HAL_OK) return ret;
  return ret;
}

static HAL_StatusTypeDef LCD_WriteReg16Multi_DMA(LCD *hlcd, uint8_t reg, uint16_t *val, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) goto ErrRet;
  ret = SPI_16bit(hlcd->hspi);
  if (ret != HAL_OK) goto ErrRet;
  ret = DMA_16bit(hlcd->hdma_spi_tx);
  if (ret != HAL_OK) goto ErrRet;
  ret = HAL_SPI_Transmit_DMA(hlcd->hspi, (void *)val, count);
  if (ret != HAL_OK) goto ErrRet;
  hlcd->is_dma_active = 1;
  return ret;

ErrRet:
  DMA_8bit(hlcd->hdma_spi_tx);
  SPI_8bit(hlcd->hspi);
  return ret;
}

static HAL_StatusTypeDef LCD_ReadReg8Multi(LCD *hlcd, uint8_t reg, uint8_t *val, size_t count)
{
  uint8_t z = 0;
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  ret = HAL_SPI_Transmit(hlcd->hspi, &z, 1, LCD_TIMEOUT);
  if (ret != HAL_OK) return ret;
  ret = HAL_SPI_Receive(hlcd->hspi, val, count, LCD_TIMEOUT);
  if (ret != HAL_OK) return ret;
  return ret;
}

static HAL_StatusTypeDef LCD_ReadReg8Multi_DMA(LCD *hlcd, uint8_t reg, uint8_t *val, size_t count)
{
  uint8_t z = 0;
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  ret = HAL_SPI_Transmit(hlcd->hspi, &z, 1, LCD_TIMEOUT);
  if (ret != HAL_OK) return ret;
  ret = HAL_SPI_Receive_DMA(hlcd->hspi, val, count);
  if (ret != HAL_OK) return ret;
  hlcd->is_dma_active = 1;
  return ret;
}

static HAL_StatusTypeDef LCD_ReadReg8(LCD *hlcd, uint8_t reg, uint8_t *val)
{
  return LCD_ReadReg8Multi(hlcd, reg, val, 1);
}

static HAL_StatusTypeDef LCD_ReadReg16Multi(LCD *hlcd, uint8_t reg, uint16_t *val, size_t count)
{
  uint8_t z = 0;
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) goto ErrRet;
  ret = HAL_SPI_Transmit(hlcd->hspi, &z, 1, LCD_TIMEOUT);
  if (ret != HAL_OK) goto ErrRet;
  ret = SPI_16bit(hlcd->hspi);
  if (ret != HAL_OK) goto ErrRet;
  ret = HAL_SPI_Receive(hlcd->hspi, (void *)val, count, LCD_TIMEOUT);
  if (ret != HAL_OK) goto ErrRet;

ErrRet:
  ret = SPI_8bit(hlcd->hspi);
  if (ret != HAL_OK) return ret;
  return ret;
}

static HAL_StatusTypeDef LCD_ReadReg16Multi_DMA(LCD *hlcd, uint8_t reg, uint16_t *val, size_t count)
{
  uint8_t z = 0;
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) goto ErrRet;
  ret = HAL_SPI_Transmit(hlcd->hspi, &z, 1, LCD_TIMEOUT);
  if (ret != HAL_OK) goto ErrRet;
  ret = SPI_16bit(hlcd->hspi);
  if (ret != HAL_OK) goto ErrRet;
  ret = DMA_16bit(hlcd->hdma_spi_rx);
  if (ret != HAL_OK) goto ErrRet;
  ret = HAL_SPI_Receive_DMA(hlcd->hspi, (void *)val, count);
  if (ret != HAL_OK) goto ErrRet;
  hlcd->is_dma_active = 1;
  return ret;

ErrRet:
  DMA_8bit(hlcd->hdma_spi_rx);
  SPI_8bit(hlcd->hspi);
  return ret;
}

HAL_StatusTypeDef LCD_Config(LCD *hlcd)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = SPI_8bit(hlcd->hspi);
  if (ret != HAL_OK) goto ErrRet;

  Pin_Low(&hlcd->pin_cs);
  Pin_Low(&hlcd->pin_rst);
  HAL_Delay(10);
  Pin_High(&hlcd->pin_rst);
  HAL_Delay(50);
  ret = LCD_WriteCommand(hlcd, 0x11);
  if (ret != HAL_OK) goto ErrRet;
  HAL_Delay(150);
  ret = LCD_WriteCommand(hlcd, 0x29);
  if (ret != HAL_OK) goto ErrRet;
  Pin_High(&hlcd->pin_cs);

  ret = LCD_SetOrient(hlcd, hlcd->orient);
  if (ret != HAL_OK) goto ErrRet;
  ret = LCD_SetColorMode(hlcd, hlcd->color_mode);
  if (ret != HAL_OK) goto ErrRet;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

HAL_StatusTypeDef LCD_SetOrient(LCD *hlcd, LCD_Orient orient)
{
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t o;
  int xres;
  int yres;

  switch (orient)
  {
    default:
      return HAL_ERROR;
    case LCD_Landscape:
      o = 0x28;
      xres = 320;
      yres = 240;
      break;
    case LCD_Portrait:
      o = 0x48;
      xres = 240;
      yres = 320;
      break;
    case LCD_InvPortrait:
      o = 0x68;
      xres = 240;
      yres = 320;
      break;
    case LCD_InvLandscape:
      o = 0x88;
      xres = 320;
      yres = 240;
      break;
  }
  Pin_Low(&hlcd->pin_cs);
  ret = LCD_WriteReg8(hlcd, 0x36, o);
  Pin_High(&hlcd->pin_cs);
  if (ret == HAL_OK)
  {
    hlcd->xres = xres;
    hlcd->yres = yres;
    hlcd->orient = orient;
  }
  ret = LCD_SetWindow(hlcd, 0, 0, xres - 1, yres - 1);
  return ret;
}

HAL_StatusTypeDef LCD_SetColorMode(LCD *hlcd, LCD_Color_Mode color_mode)
{
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  switch(color_mode)
  {
    default:
      return HAL_ERROR;
    case LCD_RGB565:
      Pin_Low(&hlcd->pin_cs);
      ret = LCD_WriteReg8(hlcd, 0x3A, 0x55);
      Pin_High(&hlcd->pin_cs);
      break;
    case LCD_RGB666:
      Pin_Low(&hlcd->pin_cs);
      ret = LCD_WriteReg8(hlcd, 0x3A, 0x66);
      Pin_High(&hlcd->pin_cs);
      break;
  }
  if (ret == HAL_OK) hlcd->color_mode = color_mode;
  return ret;
}

HAL_StatusTypeDef LCD_VScroll(LCD *hlcd, int top, int lines, int dest_y)
{
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  uint16_t scrolls[] = {top, lines, hlcd->yres - top - lines};

  Pin_Low(&hlcd->pin_cs);

  ret = LCD_WriteReg16Multi(hlcd, 0x33, scrolls, 3);
  if (ret != HAL_OK) goto ErrRet;

  ret = LCD_WriteReg16Multi(hlcd, 0x37, (uint16_t *)&dest_y, 1);
  if (ret != HAL_OK) goto ErrRet;

ErrRet:
  ret = SPI_8bit(hlcd->hspi);
  Pin_High(&hlcd->pin_cs);
  return ret;
}

HAL_StatusTypeDef LCD_SetWindow(LCD *hlcd, int x, int y, int r, int b)
{
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  uint16_t horz[] = {x, r};
  uint16_t vert[] = {y, b};

  Pin_Low(&hlcd->pin_cs);

  ret = LCD_WriteReg16Multi(hlcd, 0x2A, horz, 2);
  if (ret != HAL_OK) goto ErrRet;
  ret = LCD_WriteReg16Multi(hlcd, 0x2B, vert, 2);
  if (ret != HAL_OK) goto ErrRet;

  hlcd->cur_window_x = x;
  hlcd->cur_window_y = y;
  hlcd->cur_window_r = r;
  hlcd->cur_window_b = b;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

HAL_StatusTypeDef LCD_SetBrightness(LCD *hlcd, uint8_t brightness)
{
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  Pin_Low(&hlcd->pin_cs);

  ret = LCD_WriteReg8(hlcd, 0x51, brightness);
  if (ret != HAL_OK) goto ErrRet;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

HAL_StatusTypeDef LCD_GetBrightness(LCD *hlcd, uint8_t *brightness)
{
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  Pin_Low(&hlcd->pin_cs);

  ret = LCD_ReadReg8(hlcd, 0x52, brightness);
  if (ret != HAL_OK) goto ErrRet;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

HAL_StatusTypeDef LCD_ReadGRAM(LCD *hlcd, void *pixels, size_t count)
{
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  Pin_Low(&hlcd->pin_cs);

  switch(hlcd->color_mode)
  {
    default:
      ret = HAL_ERROR;
      goto ErrRet;
    case LCD_RGB565:
      ret = LCD_ReadReg16Multi(hlcd, 0x2E, pixels, count);
      break;
    case LCD_RGB666:
      ret = LCD_ReadReg8Multi(hlcd, 0x2E, pixels, count * 3);
      break;
  }

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

HAL_StatusTypeDef LCD_WriteGRAM(LCD *hlcd, void *pixels, size_t count)
{
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  Pin_Low(&hlcd->pin_cs);

  switch(hlcd->color_mode)
  {
    default:
      ret = HAL_ERROR;
      goto ErrRet;
    case LCD_RGB565:
      ret = LCD_WriteReg16Multi(hlcd, 0x2C, pixels, count);
      break;
    case LCD_RGB666:
      ret = LCD_WriteReg8Multi(hlcd, 0x2C, pixels, count * 3);
      break;
  }

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

HAL_StatusTypeDef LCD_ReadGRAM_DMA(LCD *hlcd, void *pixels, size_t count)
{
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  Pin_Low(&hlcd->pin_cs);

  switch(hlcd->color_mode)
  {
    default:
      ret = HAL_ERROR;
      goto ErrRet;
    case LCD_RGB565:
      ret = LCD_ReadReg16Multi_DMA(hlcd, 0x2E, pixels, count);
      break;
    case LCD_RGB666:
      ret = LCD_ReadReg8Multi_DMA(hlcd, 0x2E, pixels, count * 3);
      break;
  }
  if (ret == HAL_OK) return ret;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

HAL_StatusTypeDef LCD_WriteGRAM_DMA(LCD *hlcd, void *pixels, size_t count)
{
  DMA_Stream_TypeDef *DMA = hlcd->hdma_spi_tx->Instance;
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  Pin_Low(&hlcd->pin_cs);

  switch(hlcd->color_mode)
  {
    default:
      ret = HAL_ERROR;
      goto ErrRet;
    case LCD_RGB565:
      ret = LCD_WriteReg16Multi_DMA(hlcd, 0x2E, pixels, count);
      break;
    case LCD_RGB666:
      ret = LCD_WriteReg8Multi_DMA(hlcd, 0x2E, pixels, count * 3);
      break;
  }
  if (ret == HAL_OK) return ret;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

HAL_StatusTypeDef LCD_FillGRAMColor(LCD *hlcd, uint8_t R, uint8_t G, uint8_t B, size_t count)
{
  CHECK_STATE(hlcd);
  HAL_StatusTypeDef ret = HAL_OK;
  Pin_Low(&hlcd->pin_cs);
  Pixel565 pixel565;
  Pixel666 pixel666;

  switch(hlcd->color_mode)
  {
    default:
      ret = HAL_ERROR;
      goto ErrRet;
    case LCD_RGB565:
      ret = LCD_WriteCommand(hlcd, 0x2C);
      if (ret != HAL_OK) goto ErrRet;
      ret = SPI_16bit(hlcd->hspi);
      if (ret != HAL_OK) goto ErrRet;

      pixel565 = MakePixel565(R, G, B);

      for (size_t i = 0; i < count; i++)
      {
        ret = HAL_SPI_Transmit(hlcd->hspi, (void *)&pixel565, 1, LCD_TIMEOUT);
        if (ret != HAL_OK) goto ErrRet;
      }
      break;
    case LCD_RGB666:
      ret = LCD_WriteCommand(hlcd, 0x2C);
      if (ret != HAL_OK) goto ErrRet;

      pixel666 = MakePixel666(R, G, B);

      for (size_t i = 0; i < count; i++)
      {
        ret = HAL_SPI_Transmit(hlcd->hspi, (void *)&pixel666, 3, LCD_TIMEOUT);
        if (ret != HAL_OK) goto ErrRet;
      }
      break;
  }

ErrRet:
  ret = SPI_8bit(hlcd->hspi);
  Pin_High(&hlcd->pin_cs);
  return ret;
}

size_t LCD_ReadPixels(LCD *hlcd, int x, int y, int w, int h, void *pixels)
{
  HAL_StatusTypeDef ret = HAL_OK;
  int dx = 0, dy = 0, dw = w, dh = h;
  if (x >= hlcd->xres || y >= hlcd->yres || w <= 0 || h <= 0) return 0;
  if (x < 0)
  {
    dx = -x;
    x = 0;
  }
  if (y < 0)
  {
    dy = -y;
    y = 0;
  }
  if (x + w > hlcd->xres) w = hlcd->xres - x;
  if (y + h > hlcd->yres) h = hlcd->yres - y;
  int r = x + w - 1;
  int b = y + h - 1;
  int cwx = hlcd->cur_window_x;
  int cwy = hlcd->cur_window_y;
  int cwr = hlcd->cur_window_r;
  int cwb = hlcd->cur_window_b;
  int db = dy + h - 1;
  uint8_t *row_ptr = pixels;
  size_t pitch;
  size_t pix_size;
  switch(hlcd->color_mode)
  {
  case LCD_RGB565:
    pix_size = 2;
    break;
  case LCD_RGB666:
    pix_size = 3;
    break;
  default:
    return 0;
  }
  ret = LCD_SetWindow(hlcd, x, y, r, b);
  if (ret != HAL_OK) return 0;
  pitch = dw * pix_size;
  size_t num_read = 0;
  for (int iy = 0; iy < dh; iy ++)
  {
    if (iy < dy || iy > db)
    {
      memset(row_ptr, 0, pitch);
    }
    else
    {
      if (dx) memset(row_ptr, 0, dx * pix_size);
      ret = LCD_ReadGRAM_DMA(hlcd, &row_ptr[dx * pix_size], w);
      if (ret != HAL_OK) return num_read;
      row_ptr += pitch;
      num_read += w;
      if (w < dw) memset(&row_ptr[(dx + w) * pix_size], 0, (dw - w) * pix_size);
    }
  }
  ret = LCD_SetWindow(hlcd, cwx, cwy, cwr, cwb);
  if (ret != HAL_OK) return 0;
  return num_read;
}

size_t LCD_WritePixels(LCD *hlcd, int x, int y, int w, int h, void *pixels)
{
  HAL_StatusTypeDef ret = HAL_OK;
  int dx = 0, dy = 0, dw = w;
  if (x >= hlcd->xres || y >= hlcd->yres || w <= 0 || h <= 0) return 0;
  if (x < 0)
  {
    dx = -x;
    x = 0;
  }
  if (y < 0)
  {
    dy = -y;
    y = 0;
  }
  if (x + w > hlcd->xres) w = hlcd->xres - x;
  if (y + h > hlcd->yres) h = hlcd->yres - y;
  int r = x + w - 1;
  int b = y + h - 1;
  int cwx = hlcd->cur_window_x;
  int cwy = hlcd->cur_window_y;
  int cwr = hlcd->cur_window_r;
  int cwb = hlcd->cur_window_b;
  int db = dy + h - 1;
  uint8_t *row_ptr = pixels;
  size_t pitch;
  size_t pix_size;
  switch(hlcd->color_mode)
  {
  case LCD_RGB565:
    pix_size = 2;
    break;
  case LCD_RGB666:
    pix_size = 3;
    break;
  default:
    return 0;
  }
  ret = LCD_SetWindow(hlcd, x, y, r, b);
  if (ret != HAL_OK) return 0;
  pitch = dw * pix_size;
  size_t num_write = 0;
  for (int iy = dy; iy <= db; iy ++)
  {
    ret = LCD_WriteGRAM_DMA(hlcd, &row_ptr[dx * pix_size], w);
    if (ret != HAL_OK) return num_write;
    row_ptr += pitch;
    num_write += w;
  }
  ret = LCD_SetWindow(hlcd, cwx, cwy, cwr, cwb);
  if (ret != HAL_OK) return 0;
  return num_write;
}

HAL_StatusTypeDef LCD_FillRect(LCD *hlcd, int x, int y, int w, int h, uint8_t R, uint8_t G, uint8_t B)
{
  HAL_StatusTypeDef ret = HAL_OK;
  if (x >= hlcd->xres || y >= hlcd->yres || w <= 0 || h <= 0) return HAL_ERROR;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x + w > hlcd->xres) w = hlcd->xres - x;
  if (y + h > hlcd->yres) h = hlcd->yres - y;
  int r = x + w - 1;
  int b = y + h - 1;
  int cwx = hlcd->cur_window_x;
  int cwy = hlcd->cur_window_y;
  int cwr = hlcd->cur_window_r;
  int cwb = hlcd->cur_window_b;
  ret = LCD_SetWindow(hlcd, x, y, r, b);
  if (ret != HAL_OK) return ret;
  ret = LCD_FillGRAMColor(hlcd, R, G, B, w * h);
  if (ret != HAL_OK) return ret;
  ret = LCD_SetWindow(hlcd, cwx, cwy, cwr, cwb);
  if (ret != HAL_OK) return ret;
  return ret;
}

HAL_StatusTypeDef LCD_FillRect2(LCD *hlcd, int x, int y, int r, int b, uint8_t R, uint8_t G, uint8_t B)
{
  HAL_StatusTypeDef ret = HAL_OK;
  if (x >= hlcd->xres || y >= hlcd->yres || r < x || b < y) return HAL_ERROR;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (r >= hlcd->xres) r = hlcd->xres - x;
  if (b > hlcd->yres) b = hlcd->yres - y;
  int w = r + 1 - x;
  int h = b + 1 - y;
  int cwx = hlcd->cur_window_x;
  int cwy = hlcd->cur_window_y;
  int cwr = hlcd->cur_window_r;
  int cwb = hlcd->cur_window_b;
  ret = LCD_SetWindow(hlcd, x, y, r, b);
  if (ret != HAL_OK) return ret;
  ret = LCD_FillGRAMColor(hlcd, R, G, B, w * h);
  if (ret != HAL_OK) return ret;
  ret = LCD_SetWindow(hlcd, cwx, cwy, cwr, cwb);
  if (ret != HAL_OK) return ret;
  return ret;
}

void LCD_On_DMA_TX(LCD *hlcd)
{
  DMA_8bit(hlcd->hdma_spi_tx);
  SPI_8bit(hlcd->hspi);
  Pin_High(&hlcd->pin_cs);
  hlcd->is_dma_active = 0;
}

void LCD_On_DMA_RX(LCD *hlcd)
{
  DMA_8bit(hlcd->hdma_spi_rx);
  SPI_8bit(hlcd->hspi);
  Pin_High(&hlcd->pin_cs);
  hlcd->is_dma_active = 0;
}

Pixel565 MakePixel565(uint8_t R, uint8_t G, uint8_t B)
{
  return
      ((Pixel565)R >> 3) |
      (((Pixel565)G >> 2) << 5) |
      (((Pixel565)B >> 3) << 11);
}

Pixel666 MakePixel666(uint8_t R, uint8_t G, uint8_t B)
{
  Pixel666 ret = {R, G, B};
  return ret;
}
