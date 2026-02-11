/*
 * ili9341.c
 *
 *  Created on: Jun 16, 2025
 *      Author: 0xaa55
 */
#include"ili9341.h"
#include <stdlib.h>
#include <string.h>

LCD_FUNC void LCD_On_DMA_TX(LCD *hlcd);
LCD_FUNC void LCD_On_DMA_RX(LCD *hlcd);

__attribute__((optimize("O3")))
LCD_FUNC LCD_Pin LCD_MakePin(GPIO_TypeDef *gpio, uint32_t pin_bit)
{
  LCD_Pin ret =
  {
      gpio,
      pin_bit
  };
  return ret;
}

__attribute__((optimize("O3")))
LCD_STATIC_FUNC void Pin_Low(LCD_Pin *pin)
{
  pin->gpio->BSRR = pin->pin_bit << 16;
  // HAL_GPIO_WritePin(pin->gpio, pin->pin_bit, GPIO_PIN_RESET);
}

__attribute__((optimize("O3")))
LCD_STATIC_FUNC void Pin_High(LCD_Pin *pin)
{
  pin->gpio->BSRR = pin->pin_bit;
  // HAL_GPIO_WritePin(pin->gpio, pin->pin_bit, GPIO_PIN_SET);
}

__attribute__((optimize("O3")))
LCD_FUNC int LCD_Enter16BitMode(LCD *hlcd)
{
  HAL_StatusTypeDef ret = HAL_OK;
  if (!hlcd->is_16bit_mode)
  {
    hlcd->hspi->Init.DataSize = SPI_DATASIZE_16BIT;
    hlcd->hspi->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hlcd->hspi->hdmatx->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hlcd->hspi->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hlcd->hspi->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    ret = HAL_SPI_Init(hlcd->hspi); if (ret != HAL_OK) return ret;
    ret = HAL_DMA_Init(hlcd->hspi->hdmatx); if (ret != HAL_OK) return ret;
    ret = HAL_DMA_Init(hlcd->hspi->hdmarx); if (ret != HAL_OK) return ret;
    if (ret != HAL_OK) return ret;
    hlcd->is_16bit_mode = 1;
  }
  return ret;
}

__attribute__((optimize("O3")))
LCD_FUNC int LCD_Ensure8BitMode(LCD *hlcd)
{
  HAL_StatusTypeDef ret = HAL_OK;
  if (hlcd->is_16bit_mode)
  {
    hlcd->hspi->Init.DataSize = SPI_DATASIZE_8BIT;
    hlcd->hspi->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hlcd->hspi->hdmatx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hlcd->hspi->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hlcd->hspi->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    ret = HAL_SPI_Init(hlcd->hspi); if (ret != HAL_OK) return ret;
    ret = HAL_DMA_Init(hlcd->hspi->hdmatx); if (ret != HAL_OK) return ret;
    ret = HAL_DMA_Init(hlcd->hspi->hdmarx); if (ret != HAL_OK) return ret;
    hlcd->is_16bit_mode = 0;
  }
  return ret;
}

LCD_FUNC int LCD_IsBusy(LCD *hlcd)
{
  return hlcd->is_dma_active != 0;
}

LCD_FUNC void LCD_WaitToIdle(LCD *hlcd)
{
  while (LCD_IsBusy(hlcd))
    __WFI();
}

LCD_FUNC HAL_StatusTypeDef LCD_Init
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
  memset(hlcd, 0, sizeof *hlcd);
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

  return HAL_OK;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_Transmit(LCD *hlcd, void *data, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  while (count >= 0xFFFF)
  {
    ret = HAL_SPI_Transmit(hlcd->hspi, data, 0xFFFF, LCD_TIMEOUT);
    if (ret != HAL_OK) return ret;
    count -= 0xFFFF;
    data += 0xFFFF;
  }
  if (count)
  {
    ret = HAL_SPI_Transmit(hlcd->hspi, data, count, LCD_TIMEOUT);
    if (ret != HAL_OK) return ret;
  }
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_Receive(LCD *hlcd, void *data, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  while (count >= 0xFFFF)
  {
    ret = HAL_SPI_Receive(hlcd->hspi, data, 0xFFFF, LCD_TIMEOUT);
    if (ret != HAL_OK) return ret;
    count -= 0xFFFF;
    data += 0xFFFF;
  }
  if (count)
  {
    ret = HAL_SPI_Receive(hlcd->hspi, data, count, LCD_TIMEOUT);
    if (ret != HAL_OK) return ret;
  }
  return ret;
}

LCD_STATIC_FUNC size_t LCD_min_size(size_t size1, size_t size2)
{
  if (size1 <= size2)
    return size1;
  else
    return size2;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_Transmit_DMA(LCD *hlcd, void *data, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  size_t dma_size = LCD_min_size(0xFFFF, count);
  hlcd->is_dma_active = 1;
  hlcd->dma_pointer = data + dma_size * (1 << hlcd->is_16bit_mode);
  hlcd->dma_transfers_to_go = count - dma_size;
  ret = HAL_SPI_Transmit_DMA(hlcd->hspi, data, dma_size);
  if (ret != HAL_OK) hlcd->is_dma_active = 0;
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_Receive_DMA(LCD *hlcd, void *data, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  size_t dma_size = LCD_min_size(0xFFFF, count);
  hlcd->is_dma_active = 1;
  hlcd->dma_pointer = data + dma_size * (1 << hlcd->is_16bit_mode);
  hlcd->dma_transfers_to_go = count - dma_size;
  ret = HAL_SPI_Receive_DMA(hlcd->hspi, data, dma_size);
  if (ret != HAL_OK) hlcd->is_dma_active = 0;
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_WriteCommand(LCD *hlcd, uint8_t cmd)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_Ensure8BitMode(hlcd);
  if (ret != HAL_OK) return ret;
  Pin_Low(&hlcd->pin_dc);
  ret = HAL_SPI_Transmit(hlcd->hspi, &cmd, 1, LCD_TIMEOUT);
  Pin_High(&hlcd->pin_dc);
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_WriteReg8Multi(LCD *hlcd, uint8_t reg, uint8_t *data, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  ret = LCD_Transmit(hlcd, data, count);
  if (ret != HAL_OK) return ret;
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_WriteReg8Multi_DMA(LCD *hlcd, uint8_t reg, uint8_t *data, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  ret =  LCD_Transmit_DMA(hlcd, data, count);
  if (ret != HAL_OK) return ret;
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_WriteReg16Multi_DMA(LCD *hlcd, uint8_t reg, uint16_t *data, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  LCD_Enter16BitMode(hlcd);
  ret =  LCD_Transmit_DMA(hlcd, data, count);
  if (ret != HAL_OK) return ret;
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_WriteReg8(LCD *hlcd, uint8_t reg, uint8_t data)
{
  return LCD_WriteReg8Multi(hlcd, reg, &data, 1);
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_WriteReg16Multi(LCD *hlcd, uint8_t reg, uint16_t *data, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  LCD_Enter16BitMode(hlcd);
  ret = LCD_Transmit(hlcd, data, count);
  if (ret != HAL_OK) return ret;
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_ReadReg8Multi(LCD *hlcd, uint8_t reg, uint8_t *data, size_t count)
{
  uint8_t z = 0;
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  ret = HAL_SPI_Transmit(hlcd->hspi, &z, 1, LCD_TIMEOUT);
  if (ret != HAL_OK) return ret;
  ret = LCD_Receive(hlcd, data, count);
  if (ret != HAL_OK) return ret;
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_ReadReg8Multi_DMA(LCD *hlcd, uint8_t reg, uint8_t *data, size_t count)
{
  uint8_t z = 0;
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  ret = HAL_SPI_Transmit(hlcd->hspi, &z, 1, LCD_TIMEOUT);
  if (ret != HAL_OK) return ret;
  ret =  LCD_Receive_DMA(hlcd, data, count);
  if (ret != HAL_OK) return ret;
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_ReadReg8(LCD *hlcd, uint8_t reg, uint8_t *data)
{
  return LCD_ReadReg8Multi(hlcd, reg, data, 1);
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_ReadReg16Multi(LCD *hlcd, uint8_t reg, uint16_t *data, size_t count)
{
  uint8_t z = 0;
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  ret = HAL_SPI_Transmit(hlcd->hspi, &z, 1, LCD_TIMEOUT);
  if (ret != HAL_OK) return ret;
  LCD_Enter16BitMode(hlcd);
  ret = LCD_Receive(hlcd, data, count);
  if (ret != HAL_OK) return ret;
  return ret;
}

LCD_STATIC_FUNC HAL_StatusTypeDef LCD_ReadReg16Multi_DMA(LCD *hlcd, uint8_t reg, uint16_t *data, size_t count)
{
  uint8_t z = 0;
  HAL_StatusTypeDef ret = HAL_OK;
  ret = LCD_WriteCommand(hlcd, reg);
  if (ret != HAL_OK) return ret;
  ret = HAL_SPI_Transmit(hlcd->hspi, &z, 1, LCD_TIMEOUT);
  if (ret != HAL_OK) return ret;
  LCD_Enter16BitMode(hlcd);
  ret =  LCD_Receive_DMA(hlcd, data, count);
  if (ret != HAL_OK) return ret;
  return ret;
}

LCD_FUNC LCD_FUNC HAL_StatusTypeDef LCD_Config(LCD *hlcd)
{
  HAL_StatusTypeDef ret = HAL_OK;
  LCD_WaitToIdle(hlcd);
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

  ret = LCD_SetColorMode(hlcd, hlcd->color_mode);
  if (ret != HAL_OK) goto ErrRet;
  ret = LCD_SetOrient(hlcd, hlcd->orient);
  if (ret != HAL_OK) goto ErrRet;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

LCD_FUNC HAL_StatusTypeDef LCD_SWReset(LCD *hlcd)
{
  HAL_StatusTypeDef ret = HAL_OK;
  LCD_WaitToIdle(hlcd);
  Pin_Low(&hlcd->pin_cs);
  ret = LCD_WriteCommand(hlcd, 0x01);
  Pin_High(&hlcd->pin_cs);
  HAL_Delay(5);
  return ret;
}

LCD_FUNC HAL_StatusTypeDef LCD_GetDispID(LCD *hlcd, uint8_t *manufacturer_id, uint8_t *driver_version_id, uint8_t *driver_id)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[3];
  LCD_WaitToIdle(hlcd);
  Pin_Low(&hlcd->pin_cs);
  ret = LCD_ReadReg8Multi(hlcd, 0x04, buf, sizeof buf);
  Pin_High(&hlcd->pin_cs);
  if (ret != HAL_OK) return ret;
  *manufacturer_id = buf[0];
  *driver_version_id = buf[1];
  *driver_id = buf[2];
  return ret;
}

LCD_FUNC HAL_StatusTypeDef LCD_GetScanLine(LCD *hlcd, uint16_t *scanline)
{
  HAL_StatusTypeDef ret = HAL_OK;
  LCD_WaitToIdle(hlcd);
  Pin_Low(&hlcd->pin_cs);
  ret = LCD_ReadReg16Multi(hlcd, 0x45, scanline, 1);
  Pin_High(&hlcd->pin_cs);
  if (ret != HAL_OK) return ret;
  *scanline &= 1023;
  return ret;
}

LCD_FUNC HAL_StatusTypeDef LCD_SetOrient(LCD *hlcd, LCD_Orient orient)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t o;
  int xres;
  int yres;

  LCD_WaitToIdle(hlcd);

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

LCD_FUNC HAL_StatusTypeDef LCD_SetColorMode(LCD *hlcd, LCD_Color_Mode color_mode)
{
  HAL_StatusTypeDef ret = HAL_OK;
  LCD_WaitToIdle(hlcd);
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

LCD_FUNC HAL_StatusTypeDef LCD_VScroll(LCD *hlcd, int top, int lines, int dest_y)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint16_t scrolls[] = {top, lines, hlcd->yres - top - lines};
  LCD_WaitToIdle(hlcd);

  Pin_Low(&hlcd->pin_cs);

  ret = LCD_WriteReg16Multi(hlcd, 0x33, scrolls, 3);
  if (ret != HAL_OK) goto ErrRet;

  ret = LCD_WriteReg16Multi(hlcd, 0x37, (uint16_t *)&dest_y, 1);
  if (ret != HAL_OK) goto ErrRet;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

LCD_FUNC HAL_StatusTypeDef LCD_SetWindow(LCD *hlcd, int x, int y, int r, int b)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint16_t horz[] = {x, r};
  uint16_t vert[] = {y, b};
  LCD_WaitToIdle(hlcd);

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

LCD_FUNC HAL_StatusTypeDef LCD_SetBrightness(LCD *hlcd, uint8_t brightness)
{
  HAL_StatusTypeDef ret = HAL_OK;
  LCD_WaitToIdle(hlcd);
  Pin_Low(&hlcd->pin_cs);

  ret = LCD_WriteReg8(hlcd, 0x51, brightness);
  if (ret != HAL_OK) goto ErrRet;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

LCD_FUNC HAL_StatusTypeDef LCD_GetBrightness(LCD *hlcd, uint8_t *brightness)
{
  HAL_StatusTypeDef ret = HAL_OK;
  LCD_WaitToIdle(hlcd);
  Pin_Low(&hlcd->pin_cs);

  ret = LCD_ReadReg8(hlcd, 0x52, brightness);
  if (ret != HAL_OK) goto ErrRet;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

LCD_FUNC HAL_StatusTypeDef LCD_ReadGRAM(LCD *hlcd, void *pixels, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  LCD_WaitToIdle(hlcd);
  Pin_Low(&hlcd->pin_cs);

  switch(hlcd->color_mode)
  {
    default:
      ret = HAL_ERROR;
      goto ErrRet;
    case LCD_RGB565:
      ret = LCD_ReadReg8Multi(hlcd, 0x2E, pixels, count * 2);
      break;
    case LCD_RGB666:
      ret = LCD_ReadReg8Multi(hlcd, 0x2E, pixels, count * 3);
      break;
  }

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

LCD_FUNC HAL_StatusTypeDef LCD_WriteGRAM(LCD *hlcd, void *pixels, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  LCD_WaitToIdle(hlcd);
  Pin_Low(&hlcd->pin_cs);

  switch(hlcd->color_mode)
  {
    default:
      ret = HAL_ERROR;
      goto ErrRet;
    case LCD_RGB565:
      ret = LCD_WriteReg8Multi(hlcd, 0x2C, pixels, count * 2);
      break;
    case LCD_RGB666:
      ret = LCD_WriteReg8Multi(hlcd, 0x2C, pixels, count * 3);
      break;
  }

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

LCD_FUNC HAL_StatusTypeDef LCD_ReadGRAM_DMA(LCD *hlcd, void *pixels, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  LCD_WaitToIdle(hlcd);
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

LCD_FUNC HAL_StatusTypeDef LCD_WriteGRAM_DMA(LCD *hlcd, void *pixels, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  LCD_WaitToIdle(hlcd);
  Pin_Low(&hlcd->pin_cs);

  switch(hlcd->color_mode)
  {
    default:
      ret = HAL_ERROR;
      goto ErrRet;
    case LCD_RGB565:
      ret = LCD_WriteReg16Multi_DMA(hlcd, 0x2C, pixels, count);
      break;
    case LCD_RGB666:
      ret = LCD_WriteReg8Multi_DMA(hlcd, 0x2C, pixels, count * 3);
      break;
  }
  if (ret == HAL_OK) return ret;

ErrRet:
  Pin_High(&hlcd->pin_cs);
  return ret;
}

LCD_FUNC HAL_StatusTypeDef LCD_FillGRAMColor(LCD *hlcd, uint8_t R, uint8_t G, uint8_t B, size_t count)
{
  HAL_StatusTypeDef ret = HAL_OK;
  Pixel565 pixel565;
  Pixel666 pixel666;
  LCD_WaitToIdle(hlcd);
  Pin_Low(&hlcd->pin_cs);

  switch(hlcd->color_mode)
  {
    default:
      ret = HAL_ERROR;
      goto ErrRet;
    case LCD_RGB565:
      ret = LCD_WriteCommand(hlcd, 0x2C);
      if (ret != HAL_OK) goto ErrRet;

      pixel565 = MakePixel565(R, G, B);

      LCD_Enter16BitMode(hlcd);
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
  Pin_High(&hlcd->pin_cs);
  return ret;
}

LCD_FUNC size_t LCD_ReadPixels(LCD *hlcd, int x, int y, int w, int h, void *pixels)
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

LCD_FUNC size_t LCD_WritePixels(LCD *hlcd, int x, int y, int w, int h, void *pixels)
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

LCD_FUNC HAL_StatusTypeDef LCD_FillRect(LCD *hlcd, int x, int y, int w, int h, uint8_t R, uint8_t G, uint8_t B)
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

LCD_FUNC HAL_StatusTypeDef LCD_FillRect2(LCD *hlcd, int x, int y, int r, int b, uint8_t R, uint8_t G, uint8_t B)
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

LCD_FUNC void LCD_On_DMA_TX(LCD *hlcd)
{
  if (hlcd->dma_transfers_to_go)
  {
    LCD_Transmit_DMA(hlcd, hlcd->dma_pointer, hlcd->dma_transfers_to_go);
  }
  else
  {
    Pin_High(&hlcd->pin_cs);
    hlcd->is_dma_active = 0;
  }
}

LCD_FUNC void LCD_On_DMA_RX(LCD *hlcd)
{
  if (hlcd->dma_transfers_to_go)
  {
    LCD_Receive_DMA(hlcd, hlcd->dma_pointer, hlcd->dma_transfers_to_go);
  }
  else
  {
    Pin_High(&hlcd->pin_cs);
    hlcd->is_dma_active = 0;
  }
}

__attribute__((optimize("O3")))
LCD_FUNC Pixel565 MakePixel565(uint8_t R, uint8_t G, uint8_t B)
{
  return
    ((Pixel565)B >> 3) |
    (((Pixel565)G >> 2) << 5) |
    (((Pixel565)R >> 3) << 11);
}

__attribute__((optimize("O3")))
LCD_FUNC Pixel666 MakePixel666(uint8_t R, uint8_t G, uint8_t B)
{
  Pixel666 ret = {R, G, B};
  return ret;
}
