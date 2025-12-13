#include "flashmap.h"

extern const uint8_t BatteryBMP[];
extern const uint16_t FontChars[];
extern const uint8_t Font12pxWidthTable[];
extern const uint32_t Font12pxXTable[];
extern const uint32_t Font12pxBitmap[];
extern const uint8_t Font14pxWidthTable[];
extern const uint32_t Font14pxXTable[];
extern const uint32_t Font14pxBitmap[];
extern const uint8_t Font17pxWidthTable[];
extern const uint32_t Font17pxXTable[];
extern const uint32_t Font17pxBitmap[];

const uint32_t BatteryBMP_Width = 16;
const uint32_t BatteryBMP_Height = 290;
const size_t NumFontChars = 7318;
const size_t Font12pxBitmapPitch = 10908;
const size_t Font14pxBitmapPitch = 12668;
const size_t Font17pxBitmapPitch = 14544;

__attribute__((section(".flash_map")))
const FlashMap_t FlashMap =
{
	0xAA55,
	BatteryBMP_Width,
	BatteryBMP_Height,
	BatteryBMP,
	FontChars,
	NumFontChars,
	Font12pxWidthTable,
	Font12pxXTable,
	(const uint8_t*)&Font12pxBitmap,
	Font12pxBitmapPitch,
	Font14pxWidthTable,
	Font14pxXTable,
	(const uint8_t*)&Font14pxBitmap,
	Font14pxBitmapPitch,
	Font17pxWidthTable,
	Font17pxXTable,
	(const uint8_t*)&Font17pxBitmap,
	Font17pxBitmapPitch,
};

size_t DummyEntry()
{
	size_t ptr = 0x90000000 +
	(size_t)&FlashMap +
	(size_t)&BatteryBMP +
	(size_t)&FontChars +
	(size_t)&Font12pxWidthTable +
	(size_t)&Font12pxXTable +
	(size_t)&Font12pxBitmap +
	(size_t)&Font14pxWidthTable +
	(size_t)&Font14pxXTable +
	(size_t)&Font14pxBitmap +
	(size_t)&Font17pxWidthTable +
	(size_t)&Font17pxXTable +
	(size_t)&Font17pxBitmap;
	return ptr;
}
