/*
 * ili9341.h
 *
 *  Created on: Jun 16, 2025
 *      Author: 0xaa55
 */

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#include "stm32h7xx_hal.h"
#include <inttypes.h>
#include <stddef.h>

#ifndef LCD_TIMEOUT
#define LCD_TIMEOUT 100
#endif

#ifndef LCD_FUNC
#define LCD_FUNC
#endif

typedef enum LCD_Color_Mode_e
{
  LCD_RGB565 = 0x55,
  LCD_RGB666 = 0x66,
}LCD_Color_Mode;

typedef enum LCD_Orient_e
{
  LCD_Landscape = 0,
  LCD_Portrait = 1,
  LCD_InvPortrait = 2,
  LCD_InvLandscape = 3,
}LCD_Orient;

typedef uint16_t Pixel565;
typedef struct Pixel666_s
{
  uint8_t R;
  uint8_t G;
  uint8_t B;
}Pixel666;

typedef struct LCD_Pin_s
{
  GPIO_TypeDef *gpio;
  uint32_t pin_bit;
}LCD_Pin;

LCD_FUNC LCD_Pin LCD_MakePin(GPIO_TypeDef *gpio, uint32_t pin_bit);

typedef struct LCD_s
{
  SPI_HandleTypeDef *hspi;
  DMA_HandleTypeDef *hdma_spi_tx;
  DMA_HandleTypeDef *hdma_spi_rx;
  LCD_Pin pin_rst;
  LCD_Pin pin_cs;
  LCD_Pin pin_dc;
  LCD_Pin pin_sck;
  LCD_Pin pin_mosi;
  LCD_Pin pin_miso;
  LCD_Color_Mode color_mode;
  LCD_Orient orient;
  int xres;
  int yres;
  int cur_window_x;
  int cur_window_y;
  int cur_window_r;
  int cur_window_b;
  int is_16bit_mode;
  volatile int is_dma_active;
  uint8_t *dma_pointer;
  size_t dma_transfers_to_go;
}LCD;

typedef struct LCD_GPIO_s
{
  LCD_Pin pin_rst;
  LCD_Pin pin_cs;
  LCD_Pin pin_dc;
  LCD_Pin pin_sck;
  LCD_Pin pin_mosi;
  LCD_Pin pin_miso;
}LCD_GPIO;

HAL_StatusTypeDef LCD_Init
(
  LCD *hlcd,
  SPI_HandleTypeDef *hspi,
  DMA_HandleTypeDef *hdma_spi_tx,
  DMA_HandleTypeDef *hdma_spi_rx,
  const LCD_GPIO *gpio,
  LCD_Color_Mode color_mode,
  LCD_Orient orient
);

LCD_FUNC HAL_StatusTypeDef LCD_Config(LCD *hlcd);
LCD_FUNC void LCD_WaitToIdle(LCD *hlcd);
LCD_FUNC HAL_StatusTypeDef LCD_SWReset(LCD *hlcd);
LCD_FUNC HAL_StatusTypeDef LCD_GetDispID(LCD *hlcd, uint8_t *manufacturer_id, uint8_t *driver_version_id, uint8_t *driver_id);
LCD_FUNC HAL_StatusTypeDef LCD_GetScanLine(LCD *hlcd, uint16_t *scanline);
LCD_FUNC HAL_StatusTypeDef LCD_SetOrient(LCD *hlcd, LCD_Orient orient);
LCD_FUNC HAL_StatusTypeDef LCD_SetColorMode(LCD *hlcd, LCD_Color_Mode color_mode);
LCD_FUNC HAL_StatusTypeDef LCD_VScroll(LCD *hlcd, int top, int lines, int dest_y);
LCD_FUNC HAL_StatusTypeDef LCD_SetBrightness(LCD *hlcd, uint8_t brightness);
LCD_FUNC HAL_StatusTypeDef LCD_GetBrightness(LCD *hlcd, uint8_t *brightness);
LCD_FUNC HAL_StatusTypeDef LCD_SetWindow(LCD *hlcd, int x, int y, int r, int b);
LCD_FUNC HAL_StatusTypeDef LCD_ReadGRAM(LCD *hlcd, void *pixels, size_t count);
LCD_FUNC HAL_StatusTypeDef LCD_WriteGRAM(LCD *hlcd, void *pixels, size_t count);
LCD_FUNC HAL_StatusTypeDef LCD_ReadGRAM_DMA(LCD *hlcd, void *pixels, size_t count);
LCD_FUNC HAL_StatusTypeDef LCD_WriteGRAM_DMA(LCD *hlcd, void *pixels, size_t count);
LCD_FUNC HAL_StatusTypeDef LCD_FillGRAMColor(LCD *hlcd, uint8_t R, uint8_t G, uint8_t B, size_t count);

LCD_FUNC size_t LCD_ReadPixels(LCD *hlcd, int x, int y, int w, int h, void *pixels);
LCD_FUNC size_t LCD_WritePixels(LCD *hlcd, int x, int y, int w, int h, void *pixels);
LCD_FUNC HAL_StatusTypeDef LCD_FillRect(LCD *hlcd, int x, int y, int w, int h, uint8_t R, uint8_t G, uint8_t B);
LCD_FUNC HAL_StatusTypeDef LCD_FillRect2(LCD *hlcd, int x, int y, int r, int b, uint8_t R, uint8_t G, uint8_t B);

LCD_FUNC Pixel565 MakePixel565(uint8_t R, uint8_t G, uint8_t B);
LCD_FUNC Pixel666 MakePixel666(uint8_t R, uint8_t G, uint8_t B);

LCD_FUNC void LCD_On_DMA_TX(LCD *hlcd);
LCD_FUNC void LCD_On_DMA_RX(LCD *hlcd);

#endif /* INC_ILI9341_H_ */
