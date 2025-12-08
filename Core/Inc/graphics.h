/*
 * graphics.h
 *
 *  Created on: 2025年12月8日
 *      Author: 0xaa55
 */

#ifndef INC_GRAPHICS_H_
#define INC_GRAPHICS_H_

#include <stdint.h>

typedef uint16_t Pixel565;

extern Pixel565 MakePixel565(uint8_t R, uint8_t G, uint8_t B);
extern Pixel565 Framebuffer[240][320];

void DrawText(int x, int y, const char* text, Pixel565 TextColor);
void DrawTextOpaque(int x, int y, const char* text, Pixel565 TextColor, Pixel565 BgColor);


#endif /* INC_GRAPHICS_H_ */
