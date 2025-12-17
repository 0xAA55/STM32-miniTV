/*
 * graphics.c
 *
 *  Created on: 2025年12月8日
 *      Author: 0xaa55
 */

#include "graphics.h"
#include "utf8.h"
#include "utf16.h"
#include "font.h"
#include "../../FlashROM/flashmap.h"

typedef void(*fn_on_draw)(void *userdata, int x, int y, size_t char_index);
typedef int ssize_t;

font_t CurrentFont = DefaultFont;
int WordWrap = 1;

typedef struct dtts
{
  Pixel565 text_color;
} DrawDataTransparent;

typedef struct dtos
{
  Pixel565 text_color;
  Pixel565 bg_color;
} DrawDataOpaque;

typedef struct dtgs
{
  uint32_t w;
  uint32_t h;
} DrawDataGetSize;

const HorzLine circle_60[] =
{
  {-60 ,-4 , 9 }, {-59 ,-12 , 25 }, {-58 ,-16 , 33 }, {-57 ,-19 , 39 },
  {-56 ,-22 , 45 }, {-55 ,-24 , 49 }, {-54 ,-26 , 53 }, {-53 ,-28 , 57 },
  {-52 ,-30 , 61 }, {-51 ,-32 , 65 }, {-50 ,-33 , 67 }, {-49 ,-35 , 71 },
  {-48 ,-36 , 73 }, {-47 ,-37 , 75 }, {-46 ,-38 , 77 }, {-45 ,-39 , 79 },
  {-44 ,-41 , 83 }, {-43 ,-42 , 85 }, {-42 ,-43 , 87 }, {-41 ,-44 , 89 },
  {-40 ,-44 , 89 }, {-39 ,-45 , 91 }, {-38 ,-46 , 93 }, {-37 ,-47 , 95 },
  {-36 ,-48 , 97 }, {-35 ,-49 , 99 }, {-34 ,-49 , 99 }, {-33 ,-50 , 101 },
  {-32 ,-51 , 103 }, {-31 ,-51 , 103 }, {-30 ,-52 , 105 }, {-29 ,-52 , 105 },
  {-28 ,-53 , 107 }, {-27 ,-53 , 107 }, {-26 ,-54 , 109 }, {-25 ,-54 , 109 },
  {-24 ,-55 , 111 }, {-23 ,-55 , 111 }, {-22 ,-56 , 113 }, {-21 ,-56 , 113 },
  {-20 ,-56 , 113 }, {-19 ,-57 , 115 }, {-18 ,-57 , 115 }, {-17 ,-57 , 115 },
  {-16 ,-58 , 117 }, {-15 ,-58 , 117 }, {-14 ,-58 , 117 }, {-13 ,-59 , 119 },
  {-12 ,-59 , 119 }, {-11 ,-59 , 119 }, {-10 ,-59 , 119 }, {-9 ,-59 , 119 },
  {-8 ,-59 , 119 }, {-7 ,-59 , 119 }, {-6 ,-59 , 119 }, {-5 ,-60 , 121 },
  {-4 ,-60 , 121 }, {-3 ,-60 , 121 }, {-2 ,-60 , 121 }, {-1 ,-60 , 121 },
  { 0 ,-60 , 121 }, { 1 ,-60 , 121 }, { 2 ,-60 , 121 }, { 3 ,-60 , 121 },
  { 4 ,-60 , 121 }, { 5 ,-60 , 121 }, { 6 ,-59 , 119 }, { 7 ,-59 , 119 },
  { 8 ,-59 , 119 }, { 9 ,-59 , 119 }, { 10 ,-59 , 119 }, { 11 ,-59 , 119 },
  { 12 ,-59 , 119 }, { 13 ,-59 , 119 }, { 14 ,-58 , 117 }, { 15 ,-58 , 117 },
  { 16 ,-58 , 117 }, { 17 ,-57 , 115 }, { 18 ,-57 , 115 }, { 19 ,-57 , 115 },
  { 20 ,-56 , 113 }, { 21 ,-56 , 113 }, { 22 ,-56 , 113 }, { 23 ,-55 , 111 },
  { 24 ,-55 , 111 }, { 25 ,-54 , 109 }, { 26 ,-54 , 109 }, { 27 ,-53 , 107 },
  { 28 ,-53 , 107 }, { 29 ,-52 , 105 }, { 30 ,-52 , 105 }, { 31 ,-51 , 103 },
  { 32 ,-51 , 103 }, { 33 ,-50 , 101 }, { 34 ,-49 , 99 }, { 35 ,-49 , 99 },
  { 36 ,-48 , 97 }, { 37 ,-47 , 95 }, { 38 ,-46 , 93 }, { 39 ,-45 , 91 },
  { 40 ,-44 , 89 }, { 41 ,-44 , 89 }, { 42 ,-43 , 87 }, { 43 ,-42 , 85 },
  { 44 ,-41 , 83 }, { 45 ,-39 , 79 }, { 46 ,-38 , 77 }, { 47 ,-37 , 75 },
  { 48 ,-36 , 73 }, { 49 ,-35 , 71 }, { 50 ,-33 , 67 }, { 51 ,-32 , 65 },
  { 52 ,-30 , 61 }, { 53 ,-28 , 57 }, { 54 ,-26 , 53 }, { 55 ,-24 , 49 },
  { 56 ,-22 , 45 }, { 57 ,-19 , 39 }, { 58 ,-16 , 33 }, { 59 ,-12 , 25 },
  { 60 ,-4 , 9 },
};

static HorzLine shutdown_50[] =
{
  {-60 ,-5 , 10 }, {-59 ,-5 , 10 }, {-58 ,-5 , 10 }, {-57 ,-5 , 10 },
  {-56 ,-5 , 10 }, {-55 ,-5 , 10 }, {-54 ,-5 , 10 }, {-53 ,-5 , 10 },
  {-52 ,-5 , 10 }, {-51 ,-5 , 10 }, {-50 ,-5 , 10 }, {-49 ,-5 , 10 },
  {-48 ,-5 , 10 }, {-47 ,-15 , 1 }, {-47 ,-5 , 10 }, {-47 , 14 , 2 },
  {-46 ,-17 , 5 }, {-46 ,-5 , 10 }, {-46 , 11 , 8 }, {-45 ,-19 , 8 },
  {-45 ,-5 , 10 }, {-45 , 11 , 10 }, {-44 ,-21 , 10 }, {-44 ,-5 , 10 },
  {-44 , 11 , 12 }, {-43 ,-23 , 13 }, {-43 ,-5 , 10 }, {-43 , 10 , 15 },
  {-42 ,-25 , 15 }, {-42 ,-5 , 10 }, {-42 , 10 , 17 }, {-41 ,-26 , 16 },
  {-41 ,-5 , 10 }, {-41 , 11 , 18 }, {-40 ,-29 , 18 }, {-40 ,-5 , 10 },
  {-40 , 11 , 19 }, {-39 ,-31 , 20 }, {-39 ,-5 , 10 }, {-39 , 12 , 19 },
  {-38 ,-32 , 19 }, {-38 ,-5 , 10 }, {-38 , 14 , 18 }, {-37 ,-33 , 18 },
  {-37 ,-5 , 10 }, {-37 , 16 , 17 }, {-36 ,-34 , 17 }, {-36 ,-5 , 10 },
  {-36 , 18 , 16 }, {-35 ,-34 , 15 }, {-35 ,-5 , 10 }, {-35 , 20 , 15 },
  {-34 ,-35 , 14 }, {-34 ,-5 , 10 }, {-34 , 22 , 14 }, {-33 ,-36 , 13 },
  {-33 ,-5 , 10 }, {-33 , 23 , 14 }, {-32 ,-37 , 13 }, {-32 ,-5 , 10 },
  {-32 , 24 , 14 }, {-31 ,-38 , 13 }, {-31 ,-5 , 10 }, {-31 , 25 , 14 },
  {-30 ,-39 , 13 }, {-30 ,-5 , 10 }, {-30 , 26 , 14 }, {-29 ,-40 , 13 },
  {-29 ,-5 , 10 }, {-29 , 27 , 14 }, {-28 ,-41 , 14 }, {-28 ,-5 , 10 },
  {-28 , 28 , 14 }, {-27 ,-41 , 13 }, {-27 ,-5 , 10 }, {-27 , 29 , 13 },
  {-26 ,-42 , 13 }, {-26 ,-5 , 10 }, {-26 , 30 , 13 }, {-25 ,-42 , 12 },
  {-25 ,-5 , 10 }, {-25 , 31 , 12 }, {-24 ,-43 , 12 }, {-24 ,-5 , 10 },
  {-24 , 32 , 12 }, {-23 ,-43 , 11 }, {-23 ,-5 , 10 }, {-23 , 33 , 12 },
  {-22 ,-43 , 10 }, {-22 ,-5 , 10 }, {-22 , 34 , 11 }, {-21 ,-44 , 10 },
  {-21 ,-5 , 10 }, {-21 , 35 , 10 }, {-20 ,-44 , 10 }, {-20 ,-5 , 10 },
  {-20 , 35 , 11 }, {-19 ,-45 , 10 }, {-19 ,-5 , 10 }, {-19 , 36 , 10 },
  {-18 ,-45 , 10 }, {-18 ,-5 , 10 }, {-18 , 36 , 11 }, {-17 ,-46 , 10 },
  {-17 ,-5 , 10 }, {-17 , 37 , 10 }, {-16 ,-46 , 10 }, {-16 ,-5 , 10 },
  {-16 , 37 , 10 }, {-15 ,-47 , 11 }, {-15 ,-5 , 10 }, {-15 , 38 , 10 },
  {-14 ,-47 , 10 }, {-14 ,-5 , 10 }, {-14 , 38 , 10 }, {-13 ,-48 , 11 },
  {-13 ,-5 , 10 }, {-13 , 39 , 10 }, {-12 ,-48 , 10 }, {-12 ,-5 , 10 },
  {-12 , 39 , 10 }, {-11 ,-48 , 10 }, {-11 ,-5 , 10 }, {-11 , 39 , 10 },
  {-10 ,-48 , 10 }, {-10 ,-5 , 10 }, {-10 , 39 , 10 }, {-9 ,-48 , 10 },
  {-9 ,-5 , 10 }, {-9 , 39 , 10 }, {-8 ,-49 , 10 }, {-8 ,-5 , 10 },
  {-8 , 40 , 10 }, {-7 ,-49 , 10 }, {-7 ,-5 , 10 }, {-7 , 40 , 10 },
  {-6 ,-49 , 10 }, {-6 ,-5 , 10 }, {-6 , 40 , 10 }, {-5 ,-49 , 10 },
  {-5 ,-5 , 10 }, {-5 , 40 , 10 }, {-4 ,-49 , 10 }, {-4 ,-5 , 10 },
  {-4 , 40 , 10 }, {-3 ,-49 , 10 }, {-3 ,-5 , 10 }, {-3 , 40 , 10 },
  {-2 ,-49 , 10 }, {-2 ,-5 , 10 }, {-2 , 40 , 10 }, {-1 ,-49 , 10 },
  {-1 ,-5 , 10 }, {-1 , 40 , 10 }, { 0 ,-50 , 10 }, { 0 ,-5 , 10 },
  { 0 , 40 , 10 }, { 1 ,-49 , 10 }, { 1 ,-4 , 9 }, { 1 , 40 , 10 },
  { 2 ,-49 , 10 }, { 2 ,-4 , 9 }, { 2 , 40 , 10 }, { 3 ,-49 , 10 },
  { 3 ,-3 , 7 }, { 3 , 40 , 10 }, { 4 ,-49 , 10 }, { 4 ,-2 , 5 },
  { 4 , 40 , 10 }, { 5 ,-49 , 10 }, { 5 , 40 , 10 }, { 6 ,-49 , 10 },
  { 6 , 40 , 10 }, { 7 ,-49 , 10 }, { 7 , 40 , 10 }, { 8 ,-49 , 10 },
  { 8 , 40 , 10 }, { 9 ,-49 , 10 }, { 9 , 40 , 10 }, { 10 ,-48 , 10 },
  { 10 , 39 , 10 }, { 11 ,-48 , 10 }, { 11 , 39 , 10 }, { 12 ,-48 , 10 },
  { 12 , 39 , 10 }, { 13 ,-47 , 10 }, { 13 , 38 , 10 }, { 14 ,-47 , 10 },
  { 14 , 38 , 10 }, { 15 ,-47 , 10 }, { 15 , 38 , 10 }, { 16 ,-46 , 10 },
  { 16 , 37 , 10 }, { 17 ,-46 , 10 }, { 17 , 37 , 10 }, { 18 ,-46 , 11 },
  { 18 , 36 , 11 }, { 19 ,-45 , 10 }, { 19 , 36 , 10 }, { 20 ,-45 , 11 },
  { 20 , 35 , 11 }, { 21 ,-44 , 10 }, { 21 , 35 , 10 }, { 22 ,-44 , 11 },
  { 22 , 34 , 11 }, { 23 ,-43 , 11 }, { 23 , 33 , 11 }, { 24 ,-43 , 12 },
  { 24 , 32 , 12 }, { 25 ,-42 , 11 }, { 25 , 32 , 11 }, { 26 ,-42 , 12 },
  { 26 , 31 , 12 }, { 27 ,-41 , 12 }, { 27 , 30 , 12 }, { 28 ,-41 , 13 },
  { 28 , 29 , 13 }, { 29 ,-40 , 13 }, { 29 , 28 , 13 }, { 30 ,-39 , 13 },
  { 30 , 27 , 13 }, { 31 ,-38 , 13 }, { 31 , 26 , 13 }, { 32 ,-38 , 15 },
  { 32 , 24 , 15 }, { 33 ,-37 , 15 }, { 33 , 23 , 15 }, { 34 ,-36 , 15 },
  { 34 , 22 , 15 }, { 35 ,-35 , 16 }, { 35 , 20 , 16 }, { 36 ,-34 , 17 },
  { 36 , 18 , 17 }, { 37 ,-33 , 18 }, { 37 , 16 , 18 }, { 38 ,-32 , 20 },
  { 38 , 13 , 20 }, { 39 ,-30 , 21 }, { 39 , 10 , 21 }, { 40 ,-29 , 59 },
  { 41 ,-28 , 57 }, { 42 ,-26 , 53 }, { 43 ,-24 , 49 }, { 44 ,-22 , 45 },
  { 45 ,-20 , 41 }, { 46 ,-18 , 37 }, { 47 ,-15 , 31 }, { 48 ,-12 , 25 },
  { 49 ,-9 , 19 },
};

static HorzLine gear_45[] =
{
  {-45 ,-8 , 9 }, {-44 ,-12 , 13 }, {-43 ,-15 , 16 }, {-43 , 13 , 3 },
  {-42 ,-14 , 15 }, {-42 , 13 , 5 }, {-41 ,-14 , 15 }, {-41 , 13 , 7 },
  {-40 ,-14 , 15 }, {-40 , 12 , 10 }, {-39 ,-13 , 14 }, {-39 , 12 , 12 },
  {-38 ,-13 , 14 }, {-38 , 12 , 14 }, {-37 ,-13 , 14 }, {-37 , 11 , 16 },
  {-36 ,-27 , 2 }, {-36 ,-12 , 13 }, {-36 , 11 , 16 }, {-35 ,-29 , 4 },
  {-35 ,-12 , 19 }, {-35 , 11 , 15 }, {-34 ,-30 , 6 }, {-34 ,-12 , 38 },
  {-33 ,-31 , 8 }, {-33 ,-14 , 39 }, {-32 ,-32 , 10 }, {-32 ,-16 , 40 },
  {-31 ,-33 , 11 }, {-31 ,-17 , 41 }, {-30 ,-34 , 13 }, {-30 ,-19 , 42 },
  {-29 ,-35 , 57 }, {-28 ,-35 , 58 }, {-27 ,-36 , 60 }, {-26 ,-36 , 61 },
  {-26 , 35 , 3 }, {-25 ,-36 , 62 }, {-25 , 33 , 6 }, {-24 ,-35 , 62 },
  {-24 , 32 , 7 }, {-23 ,-33 , 61 }, {-23 , 30 , 10 }, {-22 ,-32 , 72 },
  {-21 ,-30 , 71 }, {-20 ,-29 , 70 }, {-19 ,-30 , 27 }, {-19 , 3 , 39 },
  {-18 ,-30 , 23 }, {-18 , 7 , 35 }, {-17 ,-31 , 22 }, {-17 , 10 , 33 },
  {-16 ,-31 , 21 }, {-16 , 11 , 32 }, {-15 ,-32 , 20 }, {-15 , 13 , 31 },
  {-14 ,-43 , 4 }, {-14 ,-32 , 19 }, {-14 , 14 , 30 }, {-13 ,-43 , 7 },
  {-13 ,-33 , 19 }, {-13 , 15 , 29 }, {-12 ,-43 , 28 }, {-12 , 16 , 25 },
  {-11 ,-44 , 29 }, {-11 , 16 , 22 }, {-10 ,-44 , 28 }, {-10 , 17 , 18 },
  {-9 ,-44 , 27 }, {-9 , 18 , 17 }, {-8 ,-44 , 27 }, {-8 , 18 , 17 },
  {-7 ,-45 , 27 }, {-7 , 19 , 16 }, {-6 ,-45 , 27 }, {-6 , 19 , 17 },
  {-5 ,-45 , 27 }, {-5 , 19 , 17 }, {-4 ,-45 , 27 }, {-4 , 19 , 17 },
  {-3 ,-45 , 26 }, {-3 , 20 , 16 }, {-2 ,-45 , 26 }, {-2 , 20 , 16 },
  {-1 ,-45 , 26 }, {-1 , 20 , 26 }, { 0 ,-45 , 26 }, { 0 , 20 , 26 },
  { 1 ,-35 , 16 }, { 1 , 20 , 26 }, { 2 ,-35 , 16 }, { 2 , 20 , 26 },
  { 3 ,-35 , 16 }, { 3 , 20 , 26 }, { 4 ,-35 , 17 }, { 4 , 19 , 27 },
  { 5 ,-35 , 17 }, { 5 , 19 , 27 }, { 6 ,-35 , 17 }, { 6 , 19 , 27 },
  { 7 ,-34 , 16 }, { 7 , 19 , 27 }, { 8 ,-34 , 17 }, { 8 , 18 , 27 },
  { 9 ,-34 , 17 }, { 9 , 18 , 27 }, { 10 ,-34 , 18 }, { 10 , 17 , 28 },
  { 11 ,-37 , 22 }, { 11 , 16 , 29 }, { 12 ,-40 , 25 }, { 12 , 16 , 28 },
  { 13 ,-43 , 29 }, { 13 , 15 , 19 }, { 13 , 37 , 7 }, { 14 ,-43 , 30 },
  { 14 , 14 , 19 }, { 14 , 40 , 4 }, { 15 ,-43 , 31 }, { 15 , 13 , 20 },
  { 16 ,-42 , 32 }, { 16 , 11 , 21 }, { 17 ,-42 , 33 }, { 17 , 9 , 23 },
  { 18 ,-41 , 34 }, { 18 , 7 , 24 }, { 19 ,-41 , 38 }, { 19 , 3 , 28 },
  { 20 ,-40 , 70 }, { 21 ,-40 , 71 }, { 22 ,-39 , 72 }, { 23 ,-39 , 10 },
  { 23 ,-27 , 61 }, { 24 ,-38 , 7 }, { 24 ,-26 , 62 }, { 25 ,-38 , 6 },
  { 25 ,-25 , 62 }, { 26 ,-37 , 3 }, { 26 ,-24 , 61 }, { 27 ,-23 , 60 },
  { 28 ,-22 , 58 }, { 29 ,-21 , 57 }, { 30 ,-22 , 42 }, { 30 , 22 , 13 },
  { 31 ,-23 , 41 }, { 31 , 22 , 12 }, { 32 ,-23 , 39 }, { 32 , 23 , 10 },
  { 33 ,-24 , 38 }, { 33 , 24 , 8 }, { 34 ,-25 , 37 }, { 34 , 25 , 6 },
  { 35 ,-25 , 15 }, { 35 ,-7 , 20 }, { 35 , 25 , 5 }, { 36 ,-26 , 16 },
  { 36 ,-1 , 14 }, { 36 , 26 , 2 }, { 37 ,-26 , 15 }, { 37 ,-1 , 14 },
  { 38 ,-25 , 14 }, { 38 ,-1 , 15 }, { 39 ,-23 , 12 }, { 39 ,-1 , 15 },
  { 40 ,-21 , 9 }, { 40 ,-1 , 15 }, { 41 ,-20 , 8 }, { 41 ,-1 , 16 },
  { 42 ,-18 , 6 }, { 42 ,-1 , 16 }, { 43 ,-16 , 3 }, { 43 ,-1 , 16 },
  { 44 ,-1 , 13 }, { 45 ,-1 , 9 },
};

static HorzLine tf_card_35[] =
{
  {-37 ,-26 , 53 }, {-36 ,-27 , 55 }, {-35 ,-27 , 55 }, {-34 ,-27 , 55 },
  {-33 ,-27 , 55 }, {-32 ,-27 , 5 }, {-32 , 23 , 5 }, {-31 ,-27 , 5 },
  {-31 , 23 , 5 }, {-30 ,-27 , 5 }, {-30 ,-2 , 3 }, {-30 , 8 , 3 },
  {-30 , 23 , 5 }, {-29 ,-27 , 5 }, {-29 ,-2 , 3 }, {-29 , 8 , 3 },
  {-29 , 23 , 5 }, {-28 ,-27 , 5 }, {-28 ,-17 , 3 }, {-28 ,-12 , 3 },
  {-28 ,-7 , 3 }, {-28 ,-2 , 3 }, {-28 , 3 , 3 }, {-28 , 8 , 3 },
  {-28 , 13 , 3 }, {-28 , 18 , 3 }, {-28 , 23 , 5 }, {-27 ,-27 , 5 },
  {-27 ,-17 , 3 }, {-27 ,-12 , 3 }, {-27 ,-7 , 3 }, {-27 ,-2 , 3 },
  {-27 , 3 , 3 }, {-27 , 8 , 3 }, {-27 , 13 , 3 }, {-27 , 18 , 3 },
  {-27 , 23 , 5 }, {-26 ,-27 , 5 }, {-26 ,-17 , 3 }, {-26 ,-12 , 3 },
  {-26 ,-7 , 3 }, {-26 ,-2 , 3 }, {-26 , 3 , 3 }, {-26 , 8 , 3 },
  {-26 , 13 , 3 }, {-26 , 18 , 3 }, {-26 , 23 , 5 }, {-25 ,-27 , 5 },
  {-25 ,-17 , 3 }, {-25 ,-12 , 3 }, {-25 ,-7 , 3 }, {-25 ,-2 , 3 },
  {-25 , 3 , 3 }, {-25 , 8 , 3 }, {-25 , 13 , 3 }, {-25 , 18 , 3 },
  {-25 , 23 , 5 }, {-24 ,-27 , 5 }, {-24 ,-17 , 3 }, {-24 ,-12 , 3 },
  {-24 ,-7 , 3 }, {-24 ,-2 , 3 }, {-24 , 3 , 3 }, {-24 , 8 , 3 },
  {-24 , 13 , 3 }, {-24 , 18 , 3 }, {-24 , 23 , 5 }, {-23 ,-27 , 5 },
  {-23 ,-17 , 3 }, {-23 ,-12 , 3 }, {-23 ,-7 , 3 }, {-23 ,-2 , 3 },
  {-23 , 3 , 3 }, {-23 , 8 , 3 }, {-23 , 13 , 3 }, {-23 , 18 , 3 },
  {-23 , 23 , 5 }, {-22 ,-27 , 5 }, {-22 ,-17 , 3 }, {-22 ,-12 , 3 },
  {-22 ,-7 , 3 }, {-22 ,-2 , 3 }, {-22 , 3 , 3 }, {-22 , 8 , 3 },
  {-22 , 13 , 3 }, {-22 , 18 , 3 }, {-22 , 23 , 5 }, {-21 ,-27 , 5 },
  {-21 ,-17 , 3 }, {-21 ,-12 , 3 }, {-21 ,-7 , 3 }, {-21 ,-2 , 3 },
  {-21 , 3 , 3 }, {-21 , 8 , 3 }, {-21 , 13 , 3 }, {-21 , 18 , 3 },
  {-21 , 23 , 5 }, {-20 ,-27 , 5 }, {-20 ,-17 , 3 }, {-20 ,-12 , 3 },
  {-20 ,-7 , 3 }, {-20 ,-2 , 3 }, {-20 , 3 , 3 }, {-20 , 8 , 3 },
  {-20 , 13 , 3 }, {-20 , 18 , 3 }, {-20 , 23 , 5 }, {-19 ,-27 , 5 },
  {-19 ,-17 , 3 }, {-19 ,-12 , 3 }, {-19 ,-7 , 3 }, {-19 ,-2 , 3 },
  {-19 , 3 , 3 }, {-19 , 8 , 3 }, {-19 , 13 , 3 }, {-19 , 18 , 3 },
  {-19 , 23 , 5 }, {-18 ,-27 , 5 }, {-18 ,-17 , 3 }, {-18 ,-12 , 3 },
  {-18 ,-7 , 3 }, {-18 ,-2 , 3 }, {-18 , 3 , 3 }, {-18 , 8 , 3 },
  {-18 , 13 , 3 }, {-18 , 18 , 3 }, {-18 , 23 , 5 }, {-17 ,-27 , 5 },
  {-17 ,-17 , 3 }, {-17 ,-12 , 3 }, {-17 ,-7 , 3 }, {-17 ,-2 , 3 },
  {-17 , 3 , 3 }, {-17 , 8 , 3 }, {-17 , 13 , 3 }, {-17 , 18 , 3 },
  {-17 , 23 , 5 }, {-16 ,-27 , 5 }, {-16 ,-17 , 3 }, {-16 ,-12 , 3 },
  {-16 ,-7 , 3 }, {-16 ,-2 , 3 }, {-16 , 3 , 3 }, {-16 , 8 , 3 },
  {-16 , 13 , 3 }, {-16 , 18 , 3 }, {-16 , 23 , 5 }, {-15 ,-28 , 6 },
  {-15 ,-17 , 3 }, {-15 ,-12 , 3 }, {-15 ,-7 , 3 }, {-15 ,-2 , 3 },
  {-15 , 3 , 3 }, {-15 , 8 , 3 }, {-15 , 13 , 3 }, {-15 , 18 , 3 },
  {-15 , 23 , 5 }, {-14 ,-29 , 7 }, {-14 , 23 , 5 }, {-13 ,-30 , 7 },
  {-13 , 23 , 5 }, {-12 ,-31 , 7 }, {-12 , 23 , 5 }, {-11 ,-32 , 7 },
  {-11 , 23 , 5 }, {-10 ,-33 , 7 }, {-10 , 23 , 5 }, {-9 ,-34 , 7 },
  {-9 , 23 , 5 }, {-8 ,-35 , 7 }, {-8 , 23 , 5 }, {-7 ,-36 , 7 },
  {-7 , 23 , 5 }, {-6 ,-37 , 7 }, {-6 , 23 , 5 }, {-5 ,-37 , 6 },
  {-5 , 23 , 5 }, {-4 ,-37 , 5 }, {-4 , 23 , 5 }, {-3 ,-37 , 5 },
  {-3 , 23 , 5 }, {-2 ,-37 , 5 }, {-2 , 23 , 5 }, {-1 ,-37 , 5 },
  {-1 , 23 , 5 }, { 0 ,-37 , 5 }, { 0 , 23 , 5 }, { 1 ,-37 , 5 },
  { 1 , 23 , 5 }, { 2 ,-37 , 5 }, { 2 , 23 , 5 }, { 3 ,-37 , 14 },
  { 3 , 23 , 5 }, { 4 ,-37 , 15 }, { 4 , 23 , 5 }, { 5 ,-37 , 15 },
  { 5 , 23 , 5 }, { 6 ,-37 , 15 }, { 6 , 23 , 5 }, { 7 ,-36 , 14 },
  { 7 , 23 , 5 }, { 8 ,-27 , 5 }, { 8 , 23 , 5 }, { 9 ,-27 , 5 },
  { 9 , 23 , 5 }, { 10 ,-27 , 5 }, { 10 , 23 , 5 }, { 11 ,-27 , 5 },
  { 11 , 23 , 5 }, { 12 ,-27 , 5 }, { 12 , 23 , 5 }, { 13 ,-27 , 5 },
  { 13 , 23 , 5 }, { 14 ,-27 , 5 }, { 14 , 23 , 5 }, { 15 ,-28 , 6 },
  { 15 , 23 , 5 }, { 16 ,-29 , 7 }, { 16 , 23 , 5 }, { 17 ,-30 , 7 },
  { 17 , 23 , 5 }, { 18 ,-31 , 7 }, { 18 , 23 , 5 }, { 19 ,-32 , 7 },
  { 19 , 23 , 5 }, { 20 ,-33 , 7 }, { 20 , 23 , 5 }, { 21 ,-34 , 7 },
  { 21 , 23 , 5 }, { 22 ,-35 , 7 }, { 22 , 23 , 5 }, { 23 ,-36 , 7 },
  { 23 , 23 , 5 }, { 24 ,-37 , 7 }, { 24 , 23 , 5 }, { 25 ,-37 , 6 },
  { 25 , 23 , 5 }, { 26 ,-37 , 5 }, { 26 , 23 , 5 }, { 27 ,-37 , 5 },
  { 27 , 23 , 5 }, { 28 ,-37 , 5 }, { 28 , 23 , 5 }, { 29 ,-37 , 5 },
  { 29 , 23 , 5 }, { 30 ,-37 , 5 }, { 30 , 23 , 5 }, { 31 ,-37 , 5 },
  { 31 , 23 , 5 }, { 32 ,-37 , 5 }, { 32 , 23 , 5 }, { 33 ,-37 , 5 },
  { 33 , 23 , 5 }, { 34 ,-37 , 5 }, { 34 , 23 , 5 }, { 35 ,-37 , 5 },
  { 35 , 23 , 5 }, { 36 ,-37 , 5 }, { 36 , 23 , 5 }, { 37 ,-37 , 5 },
  { 37 , 23 , 5 }, { 38 ,-37 , 65 }, { 39 ,-37 , 65 }, { 40 ,-37 , 65 },
  { 41 ,-37 , 65 }, { 42 ,-36 , 63 },
};

static HorzLine usb_conn_30[] =
{
  {-26 , 9 , 11 }, {-25 , 9 , 11 }, {-24 , 9 , 11 }, {-23 , 9 , 2 },
  {-23 , 13 , 3 }, {-23 , 18 , 2 }, {-22 , 9 , 2 }, {-22 , 13 , 3 },
  {-22 , 18 , 2 }, {-21 , 9 , 2 }, {-21 , 13 , 3 }, {-21 , 18 , 2 },
  {-20 , 9 , 11 }, {-19 , 9 , 11 }, {-18 , 9 , 11 }, {-15 , 7 , 15 },
  {-14 , 7 , 15 }, {-13 , 7 , 15 }, {-12 , 7 , 15 }, {-11 ,-16 , 11 },
  {-11 , 7 , 15 }, {-10 ,-18 , 15 }, {-10 , 7 , 15 }, {-9 ,-19 , 17 },
  {-9 , 7 , 15 }, {-8 ,-20 , 19 }, {-8 , 7 , 15 }, {-7 ,-21 , 6 },
  {-7 ,-6 , 5 }, {-7 , 7 , 15 }, {-6 ,-21 , 4 }, {-6 ,-5 , 5 },
  {-6 , 7 , 15 }, {-5 ,-22 , 5 }, {-5 ,-4 , 4 }, {-5 , 7 , 15 },
  {-4 ,-22 , 4 }, {-4 ,-4 , 4 }, {-4 , 7 , 15 }, {-3 ,-22 , 4 },
  {-3 ,-4 , 4 }, {-3 , 7 , 15 }, {-2 ,-4 , 4 }, {-2 , 8 , 13 },
  {-1 ,-4 , 4 }, {-1 , 8 , 13 }, { 0 ,-22 , 4 }, { 0 ,-4 , 4 },
  { 0 , 9 , 11 }, { 1 ,-22 , 4 }, { 1 ,-4 , 4 }, { 1 , 10 , 9 },
  { 2 ,-22 , 4 }, { 2 ,-4 , 4 }, { 2 , 11 , 7 }, { 3 ,-22 , 4 },
  { 3 ,-4 , 4 }, { 3 , 11 , 7 }, { 4 ,-24 , 8 }, { 4 ,-4 , 4 },
  { 4 , 11 , 7 }, { 5 ,-24 , 8 }, { 5 ,-4 , 4 }, { 5 , 11 , 7 },
  { 6 ,-24 , 8 }, { 6 ,-4 , 4 }, { 6 , 11 , 7 }, { 7 ,-24 , 8 },
  { 7 ,-4 , 4 }, { 8 ,-24 , 8 }, { 8 ,-4 , 4 }, { 9 ,-24 , 8 },
  { 9 ,-4 , 4 }, { 9 , 12 , 5 }, { 10 ,-24 , 8 }, { 10 ,-4 , 4 },
  { 10 , 12 , 5 }, { 11 ,-24 , 8 }, { 11 ,-4 , 4 }, { 11 , 12 , 5 },
  { 12 ,-24 , 8 }, { 12 ,-4 , 4 }, { 12 , 12 , 5 }, { 13 ,-24 , 8 },
  { 13 ,-4 , 5 }, { 13 , 12 , 5 }, { 14 ,-24 , 8 }, { 14 ,-4 , 6 },
  { 14 , 11 , 5 }, { 15 ,-24 , 8 }, { 15 ,-3 , 7 }, { 15 , 9 , 7 },
  { 16 ,-2 , 17 }, { 17 ,-1 , 15 }, { 18 ,-23 , 6 }, { 18 , 1 , 11 },
  { 19 ,-23 , 6 }, { 19 , 4 , 5 }, { 20 ,-23 , 6 }, { 21 ,-23 , 6 },
  { 22 ,-23 , 6 },
};

Pixel565 ColorFromPhaseSimple(uint32_t phase)
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
  return MakePixel565(r, g, b);
}

Pixel565 ColorFromPhase(uint32_t phase, uint32_t brightness, uint32_t colorness)
{
  uint32_t phase_value = phase % 1536;
  uint32_t r, g, b;
  uint32_t grey;
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
  grey = (r + g + b) / 3;
  if (colorness > 256) colorness = 256;
  r = FixedInterpolate(grey, r, colorness, 256);
  g = FixedInterpolate(grey, g, colorness, 256);
  b = FixedInterpolate(grey, b, colorness, 256);
  return MakePixel565(r, g, b);
}

static ssize_t GetCharIndex(uint32_t unicode)
{
  ssize_t max_index = (ssize_t)CurrentFont.num_codes - 1;
  ssize_t min_index = 0;
  if (unicode < CurrentFont.min_code || unicode > CurrentFont.max_code) return -1;
  if (unicode == CurrentFont.min_code) return min_index;
  if (unicode == CurrentFont.max_code) return max_index;
  ssize_t ci = (size_t)(CurrentFont.num_codes) >> 1;
  for (;;)
  {
    uint32_t cur_code = CurrentFont.code_table[ci];
    if (cur_code == unicode) return ci;
    if (cur_code < unicode)
      min_index = ci;
    else
      max_index = ci;
    if (max_index - 1 == min_index) break;
    ci = (max_index + min_index) >> 1;
  }
  return -1;
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
  return (CurrentFont.bitmap[y * CurrentFont.bitmap_pitch + x / 8] & (0x80 >> (x & 7))) != 0;
}

typedef struct cs
{
  int x, y, r, b;
  int cur_x;
  int cur_y;
  fn_on_draw on_draw;
  void *userdata;
}ComposeStatus_t;

static ComposeStatus_t CreateCompose(int x, int y, int r, int b, fn_on_draw on_draw, void *userdata)
{
  ComposeStatus_t ret =
  {
    x, y, r, b,
    x, y,
    on_draw,
    userdata
  };
  return ret;
}

static int ComposeCode(ComposeStatus_t *cs, uint32_t code)
{
  size_t char_index;
  int char_width;
  if (!code) return 0;
  if (code == '\n')
  {
    cs->cur_x = cs->x;
    cs->cur_y += CurrentFont.bitmap_height;
    return 1;
  }
  if (code == '\t')
  {
    cs->cur_x += ((cs->cur_x - cs->x) / CurrentFont.tab_size + 1) * CurrentFont.tab_size;
    return 1;
  }
  char_index = GetCharIndexMust(code);
  char_width = CurrentFont.width_table[char_index];
  if (WordWrap && cs->cur_x + char_width > cs->r)
  {
      cs->cur_x = cs->x;
      cs->cur_y += CurrentFont.bitmap_height;
  }
  if (cs->cur_y + CurrentFont.bitmap_height - 1 > cs->b) return 0;
  cs->on_draw(cs->userdata, cs->cur_x, cs->cur_y, char_index);
  cs->cur_x += char_width;
  return 1;
}

static void Compose(int x, int y, int w, int h, const char* text, fn_on_draw on_draw, void *userdata)
{
  UTF8Parser parser = utf8_start_parse(text);
  ComposeStatus_t cs = CreateCompose(x, y, x + w, y + h, on_draw, userdata);
  while(1)
  {
    if (!ComposeCode(&cs, utf8_to_utf32(&parser, '?'))) break;
  }
  utf8_end_parse(&parser);
}

static void ComposeW(int x, int y, int w, int h, const uint16_t* text, fn_on_draw on_draw, void *userdata)
{
  UTF16Parser parser = utf16_start_parse(text);
  ComposeStatus_t cs = CreateCompose(x, y, x + w, y + h, on_draw, userdata);
  while(1)
  {
    if (!ComposeCode(&cs, utf16_to_utf32(&parser, '?'))) break;
  }
  utf16_end_parse(&parser);
}

static void ComposeU(int x, int y, int w, int h, const uint32_t* text, fn_on_draw on_draw, void *userdata)
{
  ComposeStatus_t cs = CreateCompose(x, y, x + w, y + h, on_draw, userdata);
  while(1)
  {
    if (!ComposeCode(&cs, *text++)) break;
  }
}

void Graphics_Init()
{
  UseDefaultFont();
}

static void FontChanged()
{
  int space_char_index = GetCharIndex(' ');
  CurrentFont.space_size = CurrentFont.width_table[space_char_index];
  CurrentFont.tab_size = CurrentFont.space_size * 4;
}

static void ChangeFontFailed()
{
  UseDefaultFont();
  FillRect(0, 0, FramebufferWidth, FramebufferHeight, MakePixel565(255, 255, 255));
  DrawTextOpaque(40, 100, FramebufferWidth, FramebufferHeight, "READ FLASH ROM FAILED", MakePixel565(0, 0, 0), MakePixel565(255, 255, 255));
  DrawTextOpaque(40, 114, FramebufferWidth, FramebufferHeight, "PRESS BUTTON TO TURN OFF THE POWER", MakePixel565(0, 0, 0), MakePixel565(255, 255, 255));
  DrawTextOpaque(40, 128, FramebufferWidth, FramebufferHeight, "THEN TRY TO TURN ON AGAIN", MakePixel565(0, 0, 0), MakePixel565(255, 255, 255));
  SwapFramebuffers();
  OnException();
}

void UseDefaultFont()
{
  CurrentFont = DefaultFont;
  FontChanged();
}

void UseSmallFont()
{
  if (FLASH_MAP->Signature != 0xAA55) ChangeFontFailed();
  font_t SmallFont =
  {
      FLASH_MAP->NumFontChars,
      FLASH_MAP->Font12pxBitmap,
      FLASH_MAP->Font12pxBitmapPitch,
      12,
      FLASH_MAP->Font12pxWidthTable,
      FLASH_MAP->Font12pxXTable,
      FLASH_MAP->FontCodeTable,
      FLASH_MAP->FontCodeTable[0],
      FLASH_MAP->FontCodeTable[FLASH_MAP->NumFontChars - 1],
      6,
      24,
  };
  CurrentFont = SmallFont;
  FontChanged();
}

void UseMediumFont()
{
  if (FLASH_MAP->Signature != 0xAA55) ChangeFontFailed();
  font_t MediumFont =
  {
      FLASH_MAP->NumFontChars,
      FLASH_MAP->Font14pxBitmap,
      FLASH_MAP->Font14pxBitmapPitch,
      14,
      FLASH_MAP->Font14pxWidthTable,
      FLASH_MAP->Font14pxXTable,
      FLASH_MAP->FontCodeTable,
      FLASH_MAP->FontCodeTable[0],
      FLASH_MAP->FontCodeTable[FLASH_MAP->NumFontChars - 1],
      7,
      28,
  };
  CurrentFont = MediumFont;
  FontChanged();
}

void UseLargeFont()
{
  if (FLASH_MAP->Signature != 0xAA55) ChangeFontFailed();
  font_t LargeFont =
  {
      FLASH_MAP->NumFontChars,
      FLASH_MAP->Font17pxBitmap,
      FLASH_MAP->Font17pxBitmapPitch,
      17,
      FLASH_MAP->Font17pxWidthTable,
      FLASH_MAP->Font17pxXTable,
      FLASH_MAP->FontCodeTable,
      FLASH_MAP->FontCodeTable[0],
      FLASH_MAP->FontCodeTable[FLASH_MAP->NumFontChars - 1],
      7,
      28,
  };
  CurrentFont = LargeFont;
  FontChanged();
}

void SetWordWrap(int wrap)
{
  WordWrap = wrap;
}

static void on_draw_transparent(void *userdata, int x, int y, size_t char_index)
{
  DrawDataTransparent *dt = (DrawDataTransparent*)userdata;
  uint32_t font_height = CurrentFont.bitmap_height;
  int char_width = CurrentFont.width_table[char_index];
  int char_x = CurrentFont.x_table[char_index];
  for (uint32_t iy = 0; iy < font_height; iy ++)
  {
    int dy = y + iy;
    if (dy < 0 || dy >= FramebufferHeight) continue;
    for (int ix = 0; ix < char_width; ix ++)
    {
      int dx = x + ix;
      if (dx < 0 || dx >= FramebufferWidth) continue;
      if (SampleFont(char_x + ix, iy))
        CurDrawFramebuffer[dy][dx] = dt->text_color;
    }
  }
}

static void on_draw_opaque(void *userdata, int x, int y, size_t char_index)
{
  DrawDataOpaque *dt = (DrawDataOpaque*)userdata;
  uint32_t font_height = CurrentFont.bitmap_height;
  int char_width = CurrentFont.width_table[char_index];
  int char_x = CurrentFont.x_table[char_index];
  for (uint32_t iy = 0; iy < font_height; iy ++)
  {
    int dy = y + iy;
    if (dy < 0 || dy >= FramebufferHeight) continue;
    for (int ix = 0; ix < char_width; ix ++)
    {
      int dx = x + ix;
      if (dx < 0 || dx >= FramebufferWidth) continue;
      if (SampleFont(char_x + ix, iy))
        CurDrawFramebuffer[dy][dx] = dt->text_color;
      else
        CurDrawFramebuffer[dy][dx] = dt->bg_color;
    }
  }
}

static void on_draw_get_size(void *userdata, int x, int y, size_t char_index)
{
  DrawDataGetSize *dt = (DrawDataGetSize*)userdata;
  uint32_t font_height = CurrentFont.bitmap_height;
  int char_width = CurrentFont.width_table[char_index];
  int w = x + char_width;
  int h = y + font_height;
  if (w > dt->w) dt->w = w;
  if (h > dt->h) dt->h = h;
}

void DrawText(int x, int y, int w, int h, const char* text, Pixel565 text_color)
{
  DrawDataTransparent dt;
  dt.text_color = text_color;
  Compose(x, y, w, h, text, on_draw_transparent, &dt);
}

void DrawTextOpaque(int x, int y, int w, int h, const char* text, Pixel565 text_color, Pixel565 bg_color)
{
  DrawDataOpaque dt;
  dt.text_color = text_color;
  dt.bg_color = bg_color;
  Compose(x, y, w, h, text, on_draw_opaque, &dt);
}

void GetTextSize(const char* text, int w, int h, uint32_t *width, uint32_t *height)
{
  DrawDataGetSize dt = {0};
  Compose(0, 0, w, h, text, on_draw_get_size, &dt);
  *width = dt.w;
  *height = dt.h;
}

void DrawTextW(int x, int y, int w, int h, const uint16_t* text, Pixel565 text_color)
{
  DrawDataTransparent dt;
  dt.text_color = text_color;
  ComposeW(x, y, w, h, text, on_draw_transparent, &dt);
}

void DrawTextOpaqueW(int x, int y, int w, int h, const uint16_t* text, Pixel565 text_color, Pixel565 bg_color)
{
  DrawDataOpaque dt;
  dt.text_color = text_color;
  dt.bg_color = bg_color;
  ComposeW(x, y, w, h, text, on_draw_opaque, &dt);
}

void GetTextSizeW(const uint16_t* text, int w, int h, uint32_t *width, uint32_t *height)
{
  DrawDataGetSize dt = {0};
  ComposeW(0, 0, w, h, text, on_draw_get_size, &dt);
  *width = dt.w;
  *height = dt.h;
}

void DrawTextU(int x, int y, int w, int h, const uint32_t* text, Pixel565 text_color)
{
  DrawDataTransparent dt;
  dt.text_color = text_color;
  ComposeU(x, y, w, h, text, on_draw_transparent, &dt);
}

void DrawTextOpaqueU(int x, int y, int w, int h, const uint32_t* text, Pixel565 text_color, Pixel565 bg_color)
{
  DrawDataOpaque dt;
  dt.text_color = text_color;
  dt.bg_color = bg_color;
  ComposeU(x, y, w, h, text, on_draw_opaque, &dt);
}

void GetTextSizeU(const uint32_t* text, int w, int h, uint32_t *width, uint32_t *height)
{
  DrawDataGetSize dt = {0};
  ComposeU(0, 0, w, h, text, on_draw_get_size, &dt);
  *width = dt.w;
  *height = dt.h;
}

void FillRect(int x, int y, int w, int h, Pixel565 color)
{
  int r = x + w;
  int b = y + h;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (r < x || b < y) return;
  if (r > FramebufferWidth) r = FramebufferWidth;
  if (b > FramebufferHeight) b = FramebufferHeight;
  for (int iy = y; iy < b; iy ++)
  {
    for (int ix = x; ix < r; ix ++)
    {
      CurDrawFramebuffer[iy][ix] = color;
    }
  }
}

void DrawRect(int x, int y, int w, int h, Pixel565 color)
{
  FillRect(x, y, w, 1, color);
  FillRect(x, y + 1, x, h, color);
  FillRect(x + w - 1, y + 1, x + w - 1, h, color);
  FillRect(x + 1, y + h - 1, x + w - 2, 1, color);
}

void InvertRect(int x, int y, int w, int h, int fill)
{
  if (fill)
  {
    int r = x + w;
    int b = y + h;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (r < 0 || b < 0) return;
    if (r > FramebufferWidth) r = FramebufferWidth;
    if (b > FramebufferHeight) b = FramebufferHeight;
    for (int iy = y; iy < b; iy ++)
    {
      for (int ix = x; ix < r; ix ++)
      {
        CurDrawFramebuffer[iy][ix] = ~CurDrawFramebuffer[iy][ix];
      }
    }
  }
  else
  {
    InvertRect(x, y, w, 1, 1);
    InvertRect(x, y + 1, x, h, 1);
    InvertRect(x + w - 1, y + 1, x + w - 1, h, 1);
    InvertRect(x + 1, y + h - 1, x + w - 2, 1, 1);
  }
}

void ClearScreen(Pixel565 color)
{
  FillRect(0, 0, FramebufferWidth, FramebufferHeight, color);
}

static int BitBltCheck(int *dx, int *dy, int *w, int *h, const SrcPicture* src, int *src_x, int *src_y)
{
  if (*src_x < 0)
  {
    *dx += *src_x;
    *w -= *src_x;
    *src_x = 0;
  }
  if (*src_y < 0)
  {
    *dy += *src_y;
    *h -= *src_y;
    *src_y = 0;
  }
  if (*dx < 0)
  {
    *src_x -= *dx;
    *w += *dx;
    *dx = 0;
  }
  if (*dy < 0)
  {
    *src_y -= *dy;
    *h += *dy;
    *dy = 0;
  }
  if (*dx >= FramebufferWidth || *dy >= FramebufferHeight) return 0;
  if (*src_x >= src->width || *src_y >= src->height) return 0;
  if (*dx + *w > FramebufferWidth) *w = FramebufferWidth - *dx;
  if (*dy + *h > FramebufferHeight) *h = FramebufferHeight - *dy;
  if (*w <= 0 || *h <= 0) return 0;
  if (*w > src->width) *w = src->width;
  if (*h > src->height) *h = src->height;
  return 1;
}

void BitBlt565(int dx, int dy, int w, int h, const SrcPicture* src, int src_x, int src_y)
{
  if (!BitBltCheck(&dx, &dy, &w, &h, src, &src_x, &src_y)) return;
  for (int y = 0; y < h; y++)
  {
    int sy = src_y + y;
    int ty = dy + y;
    const Pixel565 *row_ptr = (const Pixel565*)((uint8_t*)src->bitmap + src->pitch * sy);
    for (int x = 0; x < w; x++)
    {
      int sx = src_x + x;
      int tx = dx + x;
      CurDrawFramebuffer[ty][tx] = row_ptr[sx];
    }
  }
}

void TransparentBlt565(int dx, int dy, int w, int h, const SrcPicture* src, int src_x, int src_y, Pixel565 key)
{
  if (!BitBltCheck(&dx, &dy, &w, &h, src, &src_x, &src_y)) return;
  for (int y = 0; y < h; y++)
  {
    int sy = src_y + y;
    int ty = dy + y;
    const Pixel565 *row_ptr = (const Pixel565*)((uint8_t*)src->bitmap + src->pitch * sy);
    for (int x = 0; x < w; x++)
    {
      int sx = src_x + x;
      int tx = dx + x;
      if (row_ptr[sx] != key) CurDrawFramebuffer[ty][tx] = row_ptr[sx];
    }
  }
}

void DrawHorzLines(int x_center, int y_center, const HorzLine *lines, size_t count, Pixel565 color, int scaling)
{
  if (scaling <= 0) return;
  for(size_t i = 0; i < count; i++)
  {
    int y = y_center + lines[i].y_offset * scaling / 512;
    int x = x_center + lines[i].x_offset * scaling / 512;
    int l = lines[i].x_length * scaling / 512;
    if (y < 0 || y >= FramebufferHeight) continue;
    if (x < 0)
    {
      l += x;
      x = 0;
    }
    if (l <= 0) continue;
    if (x + l >= FramebufferWidth)
    {
      l = (FramebufferWidth - 1) - x;
      if (l <= 0) continue;
    }
    for (int ix = 0; ix < l; ix ++)
      CurDrawFramebuffer[y][ix + x] = color;
  }
}

void DrawTFCardButton(int x_center, int y_center, Pixel565 c1, Pixel565 c2, int scaling)
{
  DrawHorzLines(x_center, y_center, circle_60, (sizeof circle_60) / (sizeof circle_60[0]), c1, scaling);
  DrawHorzLines(x_center, y_center, tf_card_35, (sizeof tf_card_35) / (sizeof tf_card_35[0]), c2, scaling);
}

void DrawUSBConnButton(int x_center, int y_center, Pixel565 c1, Pixel565 c2, int scaling)
{
  DrawHorzLines(x_center, y_center, circle_60, (sizeof circle_60) / (sizeof circle_60[0]), c1, scaling);
  DrawHorzLines(x_center, y_center, usb_conn_30, (sizeof usb_conn_30) / (sizeof usb_conn_30[0]), c2, scaling);
}

void DrawOptionButton(int x_center, int y_center, Pixel565 c1, Pixel565 c2, int scaling)
{
  DrawHorzLines(x_center, y_center, circle_60, (sizeof circle_60) / (sizeof circle_60[0]), c1, scaling);
  DrawHorzLines(x_center, y_center, gear_45, (sizeof gear_45) / (sizeof gear_45[0]), c2, scaling);
}

void DrawShutdownButton(int x_center, int y_center, Pixel565 c1, Pixel565 c2, int scaling)
{
  DrawHorzLines(x_center, y_center, circle_60, (sizeof circle_60) / (sizeof circle_60[0]), c1, scaling);
  DrawHorzLines(x_center, y_center, shutdown_50, (sizeof shutdown_50) / (sizeof shutdown_50[0]), c2, scaling);
}

void DrawStandByScreen()
{
  const Pixel565 colors1[] =
  {
    MakePixel565(255, 255, 255),
    MakePixel565(255, 255, 0),
    MakePixel565(0, 255, 255),
    MakePixel565(0, 255, 0),
    MakePixel565(255, 0, 255),
    MakePixel565(255, 0, 0),
    MakePixel565(0, 0, 255),
  };
  const Pixel565 colors2[] =
  {
    MakePixel565(0, 0, 255),
    MakePixel565(0, 0, 0),
    MakePixel565(255, 0, 255),
    MakePixel565(0, 0, 0),
    MakePixel565(0, 255, 255),
    MakePixel565(0, 0, 0),
    MakePixel565(255, 255, 255),
  };
  FillRect(0, 210, 320, 30, MakePixel565(0, 0, 0));
  for (int i = 0; i < 7; i++)
  {
    int x1 = i * 320 / 7;
    int x2 = (i + 1) * 320 / 7;
    int brightness = i * 255 / 6;
    Pixel565 color3 = MakePixel565(brightness, brightness, brightness);
    FillRect(x1, 0, x2 - x1, 180, colors1[i]);
    FillRect(x1, 180, x2 - x1, 10, colors2[i]);
    FillRect(x1, 200, x2 - x1, 10, color3);
  }
  for (int x = 0; x < 320; x++)
  {
    int brightness = x * 256 / 320;
    Pixel565 color = MakePixel565(brightness, brightness, brightness);
    for (int y = 190; y < 200; y++)
    {
      CurDrawFramebuffer[y][x] = color;
    }
  }
}

void DrawBattery(int percentage, int is_charging, int is_full)
{
  const int BatLevels = 14;
  const SrcPicture BatteryPicture =
  {
    (const Pixel565*)FLASH_MAP->BatteryBMP,
    ((FLASH_MAP->BatteryBMP_Width * 2 - 1) / 4 + 1) * 4,
    FLASH_MAP->BatteryBMP_Width,
    FLASH_MAP->BatteryBMP_Height,
  };
  int BatteryHeight = FLASH_MAP->BatteryBMP_Height / 29;
  int BatImageIndex = percentage * 14 / 100;
  if (BatImageIndex < 0 || BatImageIndex > BatLevels)
    BatImageIndex = 28;
  else
  {
    if (BatImageIndex == BatLevels) BatImageIndex = 13;
    BatImageIndex = 13 - BatImageIndex;
    if (is_charging || is_full) BatImageIndex += BatLevels;
  }
  BitBlt565(FramebufferWidth - FLASH_MAP->BatteryBMP_Width, 0, FLASH_MAP->BatteryBMP_Width, BatteryHeight, &BatteryPicture, 0, BatImageIndex * BatteryHeight);
}

void DrawFolderOrFile(int x, int y, int is_folder)
{
  const SrcPicture FolderFilePicture =
  {
    (const Pixel565*)FLASH_MAP->FolderBMP,
    ((FLASH_MAP->FolderBMP_Width * 2 - 1) / 4 + 1) * 4,
    FLASH_MAP->FolderBMP_Width,
    FLASH_MAP->FolderBMP_Height,
  };
  if (is_folder)
    TransparentBlt565(x, y, FLASH_MAP->FolderBMP_Width / 2, FLASH_MAP->FolderBMP_Height, &FolderFilePicture, 0, 0, MakePixel565(128, 128, 128));
  else
    TransparentBlt565(x, y, FLASH_MAP->FolderBMP_Width / 2, FLASH_MAP->FolderBMP_Height, &FolderFilePicture, FLASH_MAP->FolderBMP_Width / 2, 0, MakePixel565(128, 128, 128));
}

