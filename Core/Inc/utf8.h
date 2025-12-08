/*
 * utf8.h
 *
 *  Created on: 2025年12月8日
 *      Author: 0xaa55
 */

#ifndef INC_UTF8_H_
#define INC_UTF8_H_

#include <stdint.h>
#include <stddef.h>

typedef struct UTF8Parser_s
{
  char *ptr;
  size_t rem;
}UTF8Parser;

UTF8Parser utf8_start_parse(const char *utf8);
uint32_t utf8_to_utf32(UTF8Parser *p, uint32_t bad_char_code);
void utf8_end_parse(UTF8Parser *p);
void utf32_to_utf8(uint32_t code, char *utf8);

#endif /* INC_UTF8_H_ */
