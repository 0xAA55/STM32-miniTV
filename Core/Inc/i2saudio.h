/*
 * i2saudio.h
 *
 *  Created on: 2026年1月10日
 *      Author: 0xaa55
 */

#ifndef INC_I2SAUDIO_H_
#define INC_I2SAUDIO_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "stm32h7xx_hal.h"

#ifndef AUDIO_BUFFER_DURATION_MS
#define AUDIO_BUFFER_DURATION_MS 100
#endif

#ifndef AUDIO_FREQ_MAX
#define AUDIO_FREQ_MAX 50000
#endif

#define AUDIO_BUFFER_HALFSIZE (((AUDIO_FREQ_MAX) * (AUDIO_BUFFER_DURATION_MS) / 1000) * 2)
#define AUDIO_BUFFER_SIZE ((AUDIO_BUFFER_HALFSIZE) * 2)

typedef struct
{
  I2S_HandleTypeDef *hi2s;
  int volume;
  int16_t audio_buffer[AUDIO_BUFFER_HALFSIZE];
  int16_t audio_buffer_half[AUDIO_BUFFER_HALFSIZE];
  volatile int16_t* cur_write_audio_buffer;
  int16_t audio_prepare_buffer[AUDIO_BUFFER_HALFSIZE];
  volatile uint32_t audio_prepare_buffer_samples;
  volatile int audio_is_playing;
  volatile int audio_is_prepped;
} i2saudio_t, *i2saudio_p;

int i2saudio_init(i2saudio_p i2sa, I2S_HandleTypeDef *hi2s, int volume, uint32_t sample_rate);
int i2saudio_need_data(i2saudio_p i2sa);
int i2saudio_feed_samples_to_play(i2saudio_p i2sa, uint16_t *samples, size_t count, int blocking, size_t* written);
void i2saudio_set_volume(i2saudio_p i2sa, int volume);
int i2saudio_pause(i2saudio_p i2sa);
int i2saudio_resume(i2saudio_p i2sa);
int i2saudio_stop(i2saudio_p i2sa);
int i2saudio_stop_and_change_sample_rate(i2saudio_p i2sa, uint32_t sample_rate);

void i2saudio_tx_half_cplt_callback(i2saudio_p i2sa);
void i2saudio_tx_full_cplt_callback(i2saudio_p i2sa);

#endif /* INC_I2SAUDIO_H_ */
