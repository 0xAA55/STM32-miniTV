#ifndef _FLASH_MAP_
#define _FLASH_MAP_ 1

#include <stdint.h>
#include <stdlib.h>

typedef struct FlashMap_s
{
	uint32_t Signature;
	uint32_t BatteryBMP_Width;
	uint32_t BatteryBMP_Height;
	const uint8_t *BatteryBMP;
	const uint16_t *FontChar;
	size_t NumFontChars;
	const uint8_t *Font12pxWidthTable;
	const uint32_t *Font12pxXTable;
	const uint8_t *Font12pxBitmap;
	const size_t Font12pxBitmapPitch;
	const uint8_t *Font14pxWidthTable;
	const uint32_t *Font14pxXTable;
	const uint8_t *Font14pxBitmap;
	const size_t Font14pxBitmapPitch;
	const uint8_t *Font17pxWidthTable;
	const uint32_t *Font17pxXTable;
	const uint8_t *Font17pxBitmap;
	const size_t Font17pxBitmapPitch;
} FlashMap_t;

#define FLASH_MAP ((const FlashMap_t *)0x90000000)

#endif
