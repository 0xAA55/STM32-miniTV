/*
 * srt.c
 *
 *  Created on: 2026年1月22日
 *      Author: 0xaa55
 */


#include "srt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void srt_init
(
  srt_p psrt,
  srt_seek_file_fp seek_file,
  srt_tell_file_fp tell_file,
  srt_read_file_fp read_file,
  void *userdata
)
{
  memset(psrt, 0, sizeof *psrt);
  psrt->userdata = userdata;
  psrt->seek_file = seek_file;
  psrt->tell_file = tell_file;
  psrt->read_file = read_file;
}

static size_t srt_read_line(srt_p psrt, char * buffer, size_t buffer_size)
{
  uint32_t offset;
  size_t nb_read;
  size_t line_len;
  if (!psrt->tell_file(&offset, psrt->userdata)) return 0;
  nb_read = psrt->read_file(buffer, buffer_size, psrt->userdata);
  if (!nb_read || nb_read > buffer_size) return 0;
  for(line_len = 0;; line_len++)
  {
    if (line_len == buffer_size)
    {
      // Whole buffer filled, without finding '\n'
      if (line_len)
      {
        // Replace the last char to NUL, seek to the char, next read from this char.
        line_len -= 1;
        buffer[line_len] = 0;
        psrt->seek_file(offset + line_len, psrt->userdata);
      }
      else buffer[0] = 0;
      break;
    }
    if (line_len == nb_read)
    {
      // Read to EOF, not exceed the buffer size
      buffer[line_len] = 0;
      psrt->seek_file(offset + line_len, psrt->userdata);
      break;
    }
    if (buffer[line_len] == '\n')
    {
      // Find the '\n', Next read skip the '\n'
      buffer[line_len] = 0;
      psrt->seek_file(offset + line_len + 1, psrt->userdata);
      break;
    }
  }
  return line_len;
}

static size_t srt_kill_CR(char * buffer, size_t line_len)
{
  if (line_len && buffer[line_len - 1] == '\r')
  {
    line_len -= 1;
    buffer[line_len] = 0;
  }
  return line_len;
}

static uint32_t srt_parse_index_strict(char * buffer, size_t line_len)
{
  uint32_t index = 0;
  for (size_t i = 0; i < line_len; i++)
  {
    if (buffer[i] < '0' || buffer[i] > '9') return 0;
    index = index * 10 + (buffer[i] - '0');
  }
  return index;
}

static int srt_parse_time(char * buffer, uint32_t *stime, uint32_t *etime)
{
  unsigned sh, sm, ss, sms;
  unsigned eh, em, es, ems;
  if (sscanf(buffer, "%u:%u:%u,%u --> %u:%u:%u,%u",
    &sh, &sm, &ss, &sms,
    &eh, &em, &es, &ems) != 8)
    return 0;
  if (sm >= 60 || em >= 60 || ss >= 60 || es >= 60 || sms >= 1000 || ems >= 1000) return 0;
  *stime = (((sh * 60) + sm) * 60 + ss) * 1000 + sms;
  *etime = (((eh * 60) + em) * 60 + es) * 1000 + ems;
  return 1;
}

static size_t srt_get_array_index_by_start_time(srt_p psrt, uint32_t time_ms)
{
  size_t min_index;
  size_t max_index;
  if (!psrt->num_cached_slots) return 0;
  min_index = 0;
  max_index = psrt->num_cached_slots - 1;
  if (psrt->cached_slots[min_index].time_start_ms >= time_ms) return min_index;
  if (psrt->cached_slots[max_index].time_start_ms <= time_ms) return max_index;
  for(;;)
  {
    size_t cur_index = (max_index + min_index) >> 1;
    srt_slot_p cur_slot = &psrt->cached_slots[cur_index];
    if (cur_slot->time_start_ms == time_ms ||
        (cur_slot->time_start_ms < time_ms && cur_slot[1].time_start_ms > time_ms)) return cur_index;
    if (cur_slot->time_start_ms < time_ms)
      min_index = cur_index;
    else
      max_index = cur_index;
    if (max_index - 1 == min_index) break;
  }
  return min_index;
}

static size_t srt_get_array_index_by_slot_index(srt_p psrt, uint16_t slot_index)
{
  size_t min_index;
  size_t max_index;
  if (!psrt->num_cached_slots) return 0;
  min_index = 0;
  max_index = psrt->num_cached_slots - 1;
  if (psrt->cached_slots[min_index].index >= slot_index) return min_index;
  if (psrt->cached_slots[max_index].index <= slot_index) return max_index;
  for(;;)
  {
    size_t cur_index = (max_index + min_index) >> 1;
    srt_slot_p cur_slot = &psrt->cached_slots[cur_index];
    if (cur_slot->index == slot_index ||
        (cur_slot->index < slot_index && cur_slot[1].index > slot_index)) return cur_index;
    if (cur_slot->index < slot_index)
      min_index = cur_index;
    else
      max_index = cur_index;
    if (max_index - 1 == min_index) break;
  }
  return min_index;
}

static srt_slot_p srt_insert_slot(srt_p psrt, srt_slot_p to_insert)
{
  size_t index;
  srt_slot_p cached;
  if (!psrt->num_cached_slots)
  {
    psrt->cached_slots[0] = *to_insert;
    psrt->num_cached_slots = 1;
    return &psrt->cached_slots[0];
  }

  index = srt_get_array_index_by_slot_index(psrt, to_insert->index);
  cached = &psrt->cached_slots[index];
  if (cached->index != to_insert->index)
  {
    if (to_insert->index < cached->index)
    {
      size_t to_copy;
      if (psrt->num_cached_slots < SRT_MAX_SLOTS)
        to_copy = psrt->num_cached_slots++;
      else
        to_copy = psrt->num_cached_slots - 1;
      if (to_copy) memmove(&psrt->cached_slots[index + 1], cached, to_copy * sizeof *cached);
    }
    else
    {
      if (psrt->num_cached_slots < SRT_MAX_SLOTS)
        cached = &psrt->cached_slots[psrt->num_cached_slots++];
      else
      {
        memmove(psrt->cached_slots, &psrt->cached_slots[1], (psrt->num_cached_slots - 1) * sizeof *cached);
        cached = &psrt->cached_slots[psrt->num_cached_slots - 1];
      }
    }
  }
  *cached = *to_insert;
  return cached;
}

static srt_slot_p srt_parse_next_subtitle(srt_p psrt)
{
  size_t line_len;
  char buf[256];
  srt_slot_t new_slot = {0};
  uint32_t index;
  uint32_t end_offset;

  if (!psrt->seek_file(psrt->cur_parse_offset, psrt->userdata)) return NULL;

  line_len = srt_read_line(psrt, buf, sizeof buf);
  if (!line_len) return NULL;

  line_len = srt_kill_CR(buf, line_len);
  if (!line_len) return NULL;

  index = srt_parse_index_strict(buf, line_len);
  if (!index || index >= 65536) return NULL;

  line_len = srt_read_line(psrt, buf, sizeof buf);
  if (!line_len) return NULL;

  new_slot.index = (uint16_t)index;
  if (!srt_parse_time(buf, &new_slot.time_start_ms, &new_slot.time_end_ms)) return NULL;
  if (!psrt->tell_file(&new_slot.offset_in_file, psrt->userdata)) return NULL;

  for(;;)
  {
    line_len = srt_read_line(psrt, buf, sizeof buf);
    if (!line_len) return NULL;
    line_len = srt_kill_CR(buf, line_len);
    if (!line_len) break;
  }

  if (!psrt->tell_file(&end_offset, psrt->userdata)) return NULL;
  if (end_offset > new_slot.offset_in_file)
  {
    new_slot.length = end_offset - new_slot.offset_in_file;
    psrt->cur_parse_offset = end_offset;
    return srt_insert_slot(psrt, &new_slot);
  }
  else
  {
    return NULL;
  }
}

srt_slot_p srt_get_subtitle(srt_p psrt, uint32_t time_ms)
{
  size_t index;
  srt_slot_p first_slot;
  srt_slot_p last_slot;
  int loaded = 0;
  if (!psrt->num_cached_slots)
  {
    if (!srt_parse_next_subtitle(psrt))
      return NULL;
    loaded = 1;
  }
  first_slot = &psrt->cached_slots[0];
  if (first_slot->time_start_ms > time_ms)
  {
    if (first_slot->index > 1)
    {
      if (!psrt->seek_file(0, psrt->userdata))
        return NULL;
      psrt->num_cached_slots = 0;
      if (!srt_parse_next_subtitle(psrt))
        return NULL;
      loaded = 1;
      if (first_slot->time_start_ms > time_ms) return NULL;
    }
    else
    {
      return NULL;
    }
  }
  for(;;)
  {
    last_slot = &psrt->cached_slots[psrt->num_cached_slots - 1];
    if (last_slot->time_start_ms >= time_ms)
    {
      if (!loaded) break;
      if (last_slot->time_end_ms >= time_ms)
        return NULL;
      else
        return last_slot;
    }
    if (!srt_parse_next_subtitle(psrt)) return NULL;
    loaded = 1;
  }

  index = srt_get_array_index_by_start_time(psrt, time_ms);
  return &psrt->cached_slots[index];
}

srt_slot_p srt_get_subtitle_by_index(srt_p psrt, uint16_t index)
{
  size_t cur_index;
  srt_slot_p cur_slot;
  if (!psrt->num_cached_slots)
  {
    if (!srt_parse_next_subtitle(psrt)) return NULL;
  }
  cur_index = srt_get_array_index_by_slot_index(psrt, index);
  cur_slot = &psrt->cached_slots[cur_index];

  if (cur_slot->index == index) return cur_slot;
  for(;;)
  {
    cur_slot = srt_parse_next_subtitle(psrt);
    if (!cur_slot) break;
    if (cur_slot->index == index) break;
  }
  return cur_slot;
}
