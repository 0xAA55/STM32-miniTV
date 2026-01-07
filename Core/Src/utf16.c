/*
 * utf16.c
 *
 *  Created on: 2025年12月16日
 *      Author: 0xaa55
 */

#include "utf16.h"
#include <string.h>

#define UNUSED(x) ((void)(x))

size_t strlenW(const uint16_t *utf16)
{
  size_t ret = 0;
  while(*utf16++) ret++;
  return ret;
}

uint16_t * strcpyW(uint16_t *destination, const uint16_t *source)
{
  size_t len = strlenW(source) + 1;
  memcpy(destination, source, len * 2);
  return destination;
}

uint16_t * strncpyW(uint16_t *destination, const uint16_t *source, size_t maxlen)
{
  size_t len = strlenW(source) + 1;
  if (len > maxlen) len = maxlen;
  memcpy(destination, source, len * 2);
  destination[len - 1] = 0;
  return destination;
}

int strcmpW(const uint16_t *str1, const uint16_t *str2)
{
  for(size_t i = 0;; i++)
  {
    int ch1 = str1[i];
    int ch2 = str2[i];
    int diff = ch1 - ch2;
    if (diff == 0)
    {
      if (ch1 == 0) break; else continue;
    }
    if (diff > 0) return 1;
    if (diff < 0) return -1;
  }
  return 0;
}

uint16_t *strcatW(uint16_t *destination, const uint16_t *source)
{
  uint16_t *ret = destination;
  while(*destination) destination++;
  while(*source) *destination++ = *source++;
  return ret;
}

UTF16Parser utf16_start_parse(const uint16_t *utf16)
{
  UTF16Parser ret =
  {
    utf16,
    strlenW(utf16),
  };
  return ret;
}

static uint32_t process_surrogate(uint16_t c1, uint16_t c2)
{
  uint32_t h = c1 & 0x3FF;
  uint32_t l = c2 & 0x3FF;
  return ((h << 10) | l) + 0x10000;
}

uint32_t utf16_to_utf32(UTF16Parser *p, uint32_t bad_char_code)
{
  const uint16_t *utf16s = p->ptr;
  if (!p->rem) return 0;
  switch (utf16s[0] & 0xFC00)
  {
    default:
      p->ptr ++;
      p->rem --;
      return utf16s[0];
    case 0xD800:
      switch (utf16s[1] & 0xFC00)
      {
        default:
          return bad_char_code;
        case 0xDC00:
          if (p->rem < 2)
          {
            p->ptr ++;
            p->rem --;
            return bad_char_code;
          }
          p->ptr += 2;
          p->rem -= 2;
          return process_surrogate(utf16s[0], utf16s[1]);
      }
    case 0xDC00:
      switch (utf16s[1] & 0xFC00)
      {
        default:
          return bad_char_code;
        case 0xD800:
          if (p->rem < 2)
          {
            p->ptr ++;
            p->rem --;
            return bad_char_code;
          }
          p->ptr += 2;
          p->rem -= 2;
          return process_surrogate(utf16s[1], utf16s[0]);
      }
  }
}

void utf16_end_parse(UTF16Parser *p)
{
  UNUSED(p);
}

uint16_t *utf32_to_utf16(uint32_t code, uint16_t *utf16)
{
  if (code < 0x10000)
    *utf16++ = code;
  else
  {
    code -= 0x10000;
    *utf16++ = 0xD800 | ((code >> 10) & 0x3FF);
    *utf16++ = 0xDC00 | (code & 0x3FF);
  }
  return utf16;
}

