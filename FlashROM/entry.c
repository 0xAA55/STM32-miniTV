#include "flashmap.h"

extern const uint8_t BatteryBMP[];
extern const uint8_t FolderBMP[];
extern const uint8_t USBMSCBMP[];
extern const uint16_t FontCodeTable[];
extern const uint8_t Font12pxWidthTable[];
extern const uint32_t Font12pxXTable[];
extern const uint32_t Font12pxBitmap[];
extern const uint8_t Font14pxWidthTable[];
extern const uint32_t Font14pxXTable[];
extern const uint32_t Font14pxBitmap[];
extern const uint8_t Font17pxWidthTable[];
extern const uint32_t Font17pxXTable[];
extern const uint32_t Font17pxBitmap[];
extern const uint16_t CP936Pairs[][2];

const uint32_t BatteryBMP_Width = 16;
const uint32_t BatteryBMP_Height = 290;
const uint32_t FolderBMP_Width = 68;
const uint32_t FolderBMP_Height = 17;
const size_t NumFontChars = 7318;
const size_t Font12pxBitmapPitch = 10908;
const size_t Font14pxBitmapPitch = 12668;
const size_t Font17pxBitmapPitch = 14544;
const uint32_t USBMSCBMP_Width = 320;
const uint32_t USBMSCBMP_Height = 240;
const uint32_t NumCP936Pairs = 21791;

size_t CP936_to_Unicode(char **cp936_char, uint16_t *utf_16, uint16_t char_for_fail);

__attribute__((section(".flash_map")))
const FlashMap_t FlashMap =
{
	0xAA55,
	BatteryBMP_Width,
	BatteryBMP_Height,
	BatteryBMP,
	FolderBMP_Width,
	FolderBMP_Height,
	FolderBMP,
	FontCodeTable,
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
	USBMSCBMP_Width,
	USBMSCBMP_Height,
	USBMSCBMP,
	FLASH_VERSION,
	CP936Pairs,
	NumCP936Pairs,
	CP936_to_Unicode,
};

const FlashMap_t *GetFlashMap()
{
	return &FlashMap;
}
