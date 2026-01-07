/*
 * utf16.h
 *
 *  Created on: 2025年12月16日
 *      Author: 0xaa55
 */

#ifndef INC_UTF16_H_
#define INC_UTF16_H_

#include <stdint.h>
#include <stddef.h>

typedef struct UTF16Parser_s
{
  const uint16_t *ptr;
  size_t rem;
}UTF16Parser;

size_t strlenW(const uint16_t *utf16);
uint16_t *strcpyW(uint16_t *destination, const uint16_t *source);
uint16_t *strncpyW(uint16_t *destination, const uint16_t *source, size_t maxlen);
int strcmpW(const uint16_t *str1, const uint16_t *str2);
uint16_t *strcatW(uint16_t *destination, const uint16_t *source);
UTF16Parser utf16_start_parse(const uint16_t *utf16);
uint32_t utf16_to_utf32(UTF16Parser *p, uint32_t bad_char_code);
void utf16_end_parse(UTF16Parser *p);
uint16_t *utf32_to_utf16(uint32_t code, uint16_t *utf16);

#endif /* INC_UTF16_H_ */
