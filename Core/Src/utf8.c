/*
 * utf8.c
 *
 *  Created on: 2025年12月8日
 *      Author: 0xaa55
 */

#include "utf8.h"
#include <string.h>

#define UNUSED(x) ((void)(x))

UTF8Parser utf8_start_parse(const char *utf8)
{
  UTF8Parser ret =
  {
    utf8,
    strlen(utf8),
  };
  return ret;
}

uint32_t utf8_to_utf32(UTF8Parser *p, uint32_t bad_char_code)
{
  char *utf8s = p->ptr;
  if((utf8s[0] & 0xFE) == 0xFC) //1111110x
  {
    if (p->rem >= 6)
    {
      p->ptr += 6;
      p->rem -= 6;
      return
        (((uint32_t)(utf8s[0]) & 0x01) << 30)|
        (((uint32_t)(utf8s[1]) & 0x3F) << 24)|
        (((uint32_t)(utf8s[2]) & 0x3F) << 18)|
        (((uint32_t)(utf8s[3]) & 0x3F) << 12)|
        (((uint32_t)(utf8s[4]) & 0x3F) << 6)|
        (((uint32_t)(utf8s[5]) & 0x3F) << 0);
    }
    else
    {
      p->ptr += 1;
      p->rem -= 1;
      return bad_char_code;
    }
  }
  else if((utf8s[0] & 0xFC) == 0xF8) //111110xx
  {
    if (p->rem >= 5)
    {
      p->ptr += 5;
      p->rem -= 5;
      return
        (((uint32_t)(utf8s[0]) & 0x03) << 24)|
        (((uint32_t)(utf8s[1]) & 0x3F) << 18)|
        (((uint32_t)(utf8s[2]) & 0x3F) << 12)|
        (((uint32_t)(utf8s[3]) & 0x3F) << 6)|
        (((uint32_t)(utf8s[4]) & 0x3F) << 0);
    }
    else
    {
      p->ptr += 1;
      p->rem -= 1;
      return bad_char_code;
    }
  }
  else if((utf8s[0] & 0xF8) == 0xF0) //11110xxx
  {
    if (p->rem >= 4)
    {
      p->ptr += 4;
      p->rem -= 4;
      return
        (((uint32_t)(utf8s[0]) & 0x07) << 18)|
        (((uint32_t)(utf8s[1]) & 0x3F) << 12)|
        (((uint32_t)(utf8s[2]) & 0x3F) << 6)|
        (((uint32_t)(utf8s[3]) & 0x3F) << 0);
    }
    else
    {
      p->ptr += 1;
      p->rem -= 1;
      return bad_char_code;
    }
  }
  else if((utf8s[0] & 0xF0) == 0xE0) //1110xxxx
  {
    if (p->rem >= 3)
    {
      p->ptr += 3;
      p->rem -= 3;
      return
        (((uint32_t)(utf8s[0]) & 0x0F) << 12)|
        (((uint32_t)(utf8s[1]) & 0x3F) << 6)|
        (((uint32_t)(utf8s[2]) & 0x3F) << 0);
    }
    else
    {
      p->ptr += 1;
      p->rem -= 1;
      return bad_char_code;
    }
  }
  else if((utf8s[0] & 0xE0) == 0xC0) //110xxxxx
  {
    if (p->rem >= 2)
    {
      p->ptr += 2;
      p->rem -= 2;
      return
        (((uint32_t)(utf8s[0]) & 0x1F) << 6)|
        (((uint32_t)(utf8s[1]) & 0x3F) << 0);
    }
    else
    {
      p->ptr += 1;
      p->rem -= 1;
      return bad_char_code;
    }
  }
  else if((utf8s[0] & 0xC0) == 0x80) //10xxxxxx
  {
    p->ptr += 1;
    p->rem -= 1;
    return bad_char_code;
  }
  else if((utf8s[0] & 0x80) == 0x00) //0xxxxxxx
  {
    p->ptr += 1;
    p->rem -= 1;
    return utf8s[0] & 0x7F;
  }
  else
  {
    p->ptr += 1;
    p->rem -= 1;
    return bad_char_code;
  }
}

void utf8_end_parse(UTF8Parser *p)
{
  UNUSED(p);
}

void utf32_to_utf8(uint32_t code, char *utf8)
{
  if(code >= 0x4000000)
  {
    *utf8++ = (char)(0xFC | ((code >> 30) & 0x01));
    *utf8++ = (char)(0x80 | ((code >> 24) & 0x3F));
    *utf8++ = (char)(0x80 | ((code >> 18) & 0x3F));
    *utf8++ = (char)(0x80 | ((code >> 12) & 0x3F));
    *utf8++ = (char)(0x80 | ((code >> 6) & 0x3F));
    *utf8++ = (char)(0x80 | (code & 0x3F));
  }
  else if(code >= 0x200000)
  {
    *utf8++ = (char)(0xF8 | ((code >> 24) & 0x03));
    *utf8++ = (char)(0x80 | ((code >> 18) & 0x3F));
    *utf8++ = (char)(0x80 | ((code >> 12) & 0x3F));
    *utf8++ = (char)(0x80 | ((code >> 6) & 0x3F));
    *utf8++ = (char)(0x80 | (code & 0x3F));
  }
  else if(code >= 0x10000)
  {
    *utf8++ = (char)(0xF0 | ((code >> 18) & 0x07));
    *utf8++ = (char)(0x80 | ((code >> 12) & 0x3F));
    *utf8++ = (char)(0x80 | ((code >> 6) & 0x3F));
    *utf8++ = (char)(0x80 | (code & 0x3F));
  }
  else if(code >= 0x0800)
  {
    *utf8++ = (char)(0xE0 | ((code >> 12) & 0x0F));
    *utf8++ = (char)(0x80 | ((code >> 6) & 0x3F));
    *utf8++ = (char)(0x80 | (code & 0x3F));
  }
  else if(code >= 0x0080)
  {
    *utf8++ = (char)(0xC0 | ((code >> 6) & 0x1F));
    *utf8++ = (char)(0x80 | (code & 0x3F));
  }
  else
  {
    *utf8++ = (char)(code);
  }
}

