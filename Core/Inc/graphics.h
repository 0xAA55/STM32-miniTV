/*
 * graphics.h
 *
 *  Created on: 2025年12月8日
 *      Author: 0xaa55
 */

#ifndef INC_GRAPHICS_H_
#define INC_GRAPHICS_H_

#include <stdint.h>
#include <stdlib.h>

typedef uint16_t Pixel565;

#define FixedInterpolate(a, b, s, fix) ((a) + ((b) - (a)) * (s) / (fix))

extern Pixel565 MakePixel565(uint8_t R, uint8_t G, uint8_t B);
extern Pixel565 (*CurDrawFramebuffer)[320];

void Graphics_Init();

/// * A period is 1536
/// * brightness is from 0 to 511
/// * colorness from 0 to 256
Pixel565 ColorFromPhase(uint32_t phase, uint32_t brightness, uint32_t colorness);
Pixel565 ColorFromPhaseSimple(uint32_t phase);
void DrawText(int x, int y, const char* text, Pixel565 TextColor);
void DrawTextOpaque(int x, int y, const char* text, Pixel565 TextColor, Pixel565 BgColor);

typedef struct hls
{
  int8_t y_offset;
  int8_t x_offset;
  int8_t x_length;
}HorzLine;

// Max scaling is 512
void DrawHorzLines(int x_center, int y_center, const HorzLine *lines, size_t count, Pixel565 color, int scaling);
void FillRect(int x, int y, int w, int h, Pixel565 color);

void DrawTFCardButton(int x_center, int y_center, Pixel565 c1, Pixel565 c2, int scaling);
void DrawOptionButton(int x_center, int y_center, Pixel565 c1, Pixel565 c2, int scaling);
void DrawShutdownButton(int x_center, int y_center, Pixel565 color, int scaling);

#endif /* INC_GRAPHICS_H_ */
