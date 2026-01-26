/*
 * srt.h
 *
 *  Created on: 2026年1月22日
 *      Author: 0xaa55
 */

#ifndef INC_SRT_H_
#define INC_SRT_H_

#include <stdint.h>
#include <stddef.h>

#ifndef SRT_MAX_SLOTS
#define SRT_MAX_SLOTS 5000
#endif

typedef int(*srt_seek_file_fp)(uint32_t offset, void *userdata);
typedef int(*srt_tell_file_fp)(uint32_t *offset, void *userdata);
typedef size_t(*srt_read_file_fp)(void *buffer, size_t size, void *userdata);

typedef struct srt_slot_s
{
  uint16_t index;
  uint16_t length;
  uint32_t time_start_ms;
  uint32_t time_end_ms;
  uint32_t offset_in_file;
}srt_slot_t, *srt_slot_p;

typedef struct srt_s
{
  void *userdata;
  srt_seek_file_fp seek_file;
  srt_tell_file_fp tell_file;
  srt_read_file_fp read_file;
  uint32_t num_cached_slots;
  uint32_t cur_parse_offset;
  srt_slot_t cached_slots[SRT_MAX_SLOTS];
}srt_t, *srt_p;

void srt_init
(
  srt_p psrt,
  srt_seek_file_fp seek_file,
  srt_tell_file_fp tell_file,
  srt_read_file_fp read_file,
  void *userdata
);

srt_slot_p srt_get_subtitle(srt_p psrt, uint32_t time_ms);
srt_slot_p srt_get_subtitle_by_index(srt_p psrt, uint16_t index);




#endif /* INC_SRT_H_ */
