#ifndef _FLASH_MAP_
#define _FLASH_MAP_ 1

#include <stdint.h>
#include <stdlib.h>

#define FLASH_VERSION 0x00010100

typedef struct FlashMap_s
{
	const uint32_t Signature;
	const uint32_t BatteryBMP_Width;
	const uint32_t BatteryBMP_Height;
	const uint8_t *BatteryBMP;
	const uint32_t FolderBMP_Width;
	const uint32_t FolderBMP_Height;
	const uint8_t *FolderBMP;
	const uint16_t *FontCodeTable;
	const size_t NumFontChars;
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
	const uint32_t USBMSCBMP_Width;
	const uint32_t USBMSCBMP_Height;
	const uint8_t *USBMSCBMP;
	const uint32_t Version;
	const uint32_t NumCP936Pairs;
	const uint16_t (*CP936PairsGBKToUnicode)[2];
	const uint16_t (*CP936PairsUnicodeToGBK)[2];
	size_t (*CP936_to_Unicode)(char **pp_cp936_char, uint16_t *utf_16, uint16_t char_for_fail);
	size_t (*Unicode_to_CP936)(uint32_t unicode, char **pp_cp936_char, uint16_t char_for_fail);
} FlashMap_t;

#define FLASH_MAP ((const FlashMap_t *)0x90000000)

#endif
