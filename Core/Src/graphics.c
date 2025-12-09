/*
 * graphics.c
 *
 *  Created on: 2025年12月8日
 *      Author: 0xaa55
 */

#include "graphics.h"
#include "utf8.h"
#include "font.h"

const int space_size = 6;
const int font_height = 12;
const int tab_size = space_size * 4;
const size_t num_font_codes = (sizeof font_code_table) / (sizeof font_code_table[0]);
const uint32_t min_code = font_code_table[0];
const uint32_t max_code = font_code_table[num_font_codes - 1];
const size_t font_bmp_pitch = sizeof font_bitmap / font_height;
uint16_t font_x_table[(sizeof font_width_table) / (sizeof font_width_table[0])];

typedef void(*fn_on_draw)(void *userdata, int x, int y, size_t char_index);
typedef int ssize_t;

static ssize_t GetCharIndex(uint32_t unicode)
{
  ssize_t max_index = (ssize_t)num_font_codes - 1;
  ssize_t min_index = 0;
  if (unicode < min_code || unicode > max_code) return -1;
  ssize_t ci = (size_t)(num_font_codes) >> 1;
  for (;;)
  {
    uint32_t cur_code = font_code_table[ci];
    if (cur_code == unicode) return ci;
    if (cur_code < unicode)
      min_index = ci;
    else
      max_index = ci;
    if (max_index - 1 == min_index) return -1;
    ci = (max_index + min_index) >> 1;
  }
}

static size_t GetCharIndexMust(uint32_t unicode)
{
  ssize_t ci = GetCharIndex(unicode);
  if (ci < 0) ci = GetCharIndex('?');
  if (ci < 0) ci = 0;
  return (size_t) ci;
}

static int SampleFont(int x, int y)
{
  return (font_bitmap[y * font_bmp_pitch + x / 8] & (0x80 >> (x & 7))) != 0;
}

static void Compose(int x, int y, int r, int b, const char* text, fn_on_draw on_draw, void *userdata)
{
  int cur_x = x, cur_y = y;
  UTF8Parser parser = utf8_start_parse(text);
  uint32_t code;
  size_t char_index;
  int char_width;
  for(;;)
  {
    code = utf8_to_utf32(&parser, '?');
    if (!code) break;
    if (code == '\n')
    {
      cur_x = x;
      cur_y += font_height;
      continue;
    }
    if (code == '\t')
    {
      cur_x += ((cur_x - x) / tab_size + 1) * tab_size;
      continue;
    }
    if (code == ' ')
    {
      cur_x += space_size;
      continue;
    }
    char_index = GetCharIndexMust(code);
    char_width = font_width_table[char_index];
    if (cur_x + char_width > r)
    {
      cur_x = x;
      cur_y += font_height;
    }
    if (cur_y + font_height - 1 > b) break;
    on_draw(userdata, cur_x, cur_y, char_index);
    cur_x += char_width;
  }
  utf8_end_parse(&parser);
}

void Graphics_Init()
{
  uint16_t x = 0;
  for (size_t i = 0; i < num_font_codes; i++)
  {
    font_x_table[i] = x;
    x += font_width_table[i];
  }
}

Pixel565 ColorFromPhase(uint32_t phase, uint32_t brightness)
{
  uint32_t phase_value = phase % 1536;
  uint32_t r, g, b;
  if (phase_value <= 255)
  {
    r = 255;
    g = phase_value;
    b = 0;
  }
  else if (phase_value <= 511)
  {
    r = 511 - phase_value;
    g = 255;
    b = 0;
  }
  else if (phase_value <= 767)
  {
    r = 0;
    g = 255;
    b = phase_value - 512;
  }
  else if (phase_value <= 1023)
  {
    r = 0;
    g = 1023 - phase_value;
    b = 255;
  }
  else if (phase_value <= 1279)
  {
    r = phase_value - 1024;
    g = 0;
    b = 255;
  }
  else
  {
    r = 255;
    g = 0;
    b = 1535 - phase_value;
  }
  if (brightness > 256)
  {
    if (brightness > 511) brightness = 511;
    int gain = brightness - 256;
    int dim = 256 - gain;
    r = r * dim / 256 + gain;
    g = g * dim / 256 + gain;
    b = b * dim / 256 + gain;
  }
  else
  {
    r = r * brightness / 256;
    g = g * brightness / 256;
    b = b * brightness / 256;
  }
  return MakePixel565(r, g, b);
}

void DrawText(int x, int y, const char* text, Pixel565 text_color)
{
  typedef struct dts
  {
    Pixel565 text_color;
  } DrawData;
  DrawData dt;
  void on_draw(void *userdata, int x, int y, size_t char_index)
  {
    DrawData *dt = (DrawData*)userdata;
    int char_width = font_width_table[char_index];
    int char_x = font_x_table[char_index];
    for (int iy = 0; iy < font_height; iy ++)
    {
      for (int ix = 0; ix < char_width; ix++)
      {
        int dx = (x + ix) % 320;
        int dy = (y + iy) % 240;
        if (SampleFont(char_x + ix, iy))
          Framebuffer[dy][dx] = dt->text_color;
      }
    }
  }
  dt.text_color = text_color;
  Compose(x, y, 319, 239, text, on_draw, &dt);
}

void DrawTextOpaque(int x, int y, const char* text, Pixel565 text_color, Pixel565 bg_color)
{
  typedef struct dts
  {
    Pixel565 text_color;
    Pixel565 bg_color;
  } DrawData;
  DrawData dt;
  void on_draw(void *userdata, int x, int y, size_t char_index)
  {
    DrawData *dt = (DrawData*)userdata;
    int char_width = font_width_table[char_index];
    int char_x = font_x_table[char_index];
    for (int iy = 0; iy < font_height; iy ++)
    {
      for (int ix = 0; ix < char_width; ix++)
      {
        int dx = (x + ix) % 320;
        int dy = (y + iy) % 240;
        if (SampleFont(char_x + ix, iy))
          Framebuffer[dy][dx] = dt->text_color;
        else
          Framebuffer[dy][dx] = dt->bg_color;
      }
    }
  }
  dt.text_color = text_color;
  dt.bg_color = bg_color;
  Compose(x, y, 319, 239, text, on_draw, &dt);
}

