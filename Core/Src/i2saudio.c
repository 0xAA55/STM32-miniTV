/*
 * i2saudio.c
 *
 *  Created on: 2026年1月10日
 *      Author: 0xaa55
 */

#include "i2saudio.h"

void Error_Handler(void);

static void i2saudio_prep_audio_buffer(i2saudio_p i2sa)
{
  for(size_t i = 0; i < i2sa->audio_prepare_buffer_samples; i++) i2sa->cur_write_audio_buffer[i] = (int16_t)(i2sa->audio_prepare_buffer[i] * i2sa->volume / 100);
  for(size_t i = i2sa->audio_prepare_buffer_samples; i < AUDIO_BUFFER_HALFSIZE; i++) i2sa->cur_write_audio_buffer[i] = 0;
  i2sa->audio_prepare_buffer_samples = 0;
  SCB_CleanDCache_by_Addr((uint32_t*)i2sa->cur_write_audio_buffer, AUDIO_BUFFER_HALFSIZE * 2);
}

void i2saudio_tx_half_cplt_callback(i2saudio_p i2sa)
{
  i2sa->cur_write_audio_buffer = i2sa->audio_buffer;
  i2sa->audio_is_prepped = 0;
}

void i2saudio_tx_full_cplt_callback(i2saudio_p i2sa)
{
  i2sa->cur_write_audio_buffer = i2sa->audio_buffer_half;
  i2sa->audio_is_prepped = 0;
}

void i2saudio_set_volume(i2saudio_p i2sa, int volume)
{
  if (i2sa) i2sa->volume = volume;
}

int i2saudio_init(i2saudio_p i2sa, I2S_HandleTypeDef *hi2s, int volume, uint32_t sample_rate)
{
  if (!i2sa || ! hi2s || !sample_rate) return 0;

  memset(i2sa, 0, sizeof *i2sa);
  i2sa->hi2s = hi2s;
  i2sa->volume = volume;
  i2sa->cur_write_audio_buffer = i2sa->audio_buffer;

  hi2s->Instance = SPI2;
  hi2s->Init.Mode = I2S_MODE_MASTER_TX;
  hi2s->Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s->Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s->Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s->Init.AudioFreq = sample_rate;
  hi2s->Init.CPOL = I2S_CPOL_LOW;
  hi2s->Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s->Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s->Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s->Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_ENABLE;
  return HAL_I2S_Init(hi2s) == HAL_OK;
}

int i2saudio_stop_and_change_sample_rate(i2saudio_p i2sa, uint32_t sample_rate)
{
  if (!i2sa) return 0;
  if (!i2saudio_stop(i2sa)) return 0;
  i2sa->hi2s->Init.AudioFreq = sample_rate;
  return HAL_I2S_Init(i2sa->hi2s) == HAL_OK;
}

int i2saudio_pause(i2saudio_p i2sa)
{
  if (!i2sa) return 0;
  if (HAL_I2S_DMAPause(i2sa->hi2s) == HAL_OK)
  {
    i2sa->audio_is_playing = 0;
    return 1;
  }
  return 0;
}
int i2saudio_resume(i2saudio_p i2sa)
{
  if (!i2sa) return 0;
  if (HAL_I2S_DMAResume(i2sa->hi2s) == HAL_OK)
  {
    i2sa->audio_is_playing = 1;
    return 1;
  }
  return 0;
}
int i2saudio_stop(i2saudio_p i2sa)
{
  if (!i2sa) return 0;
  if (HAL_I2S_DMAStop(i2sa->hi2s) == HAL_OK)
  {
    i2sa->audio_is_playing = 0;
    i2sa->audio_is_prepped = 0;
    i2sa->cur_write_audio_buffer = i2sa->audio_buffer;
    return 1;
  }
  return 0;
}

int i2saudio_need_data(i2saudio_p i2sa)
{
  if (!i2sa) return 0;
  return !i2sa->audio_is_prepped;
}

int i2saudio_feed_samples_to_play(i2saudio_p i2sa, uint16_t *samples, size_t count, size_t* written)
{
  if (!i2sa || !written) return 0;
  *written = 0;
  do
  {
    if (i2sa->audio_prepare_buffer_samples < AUDIO_BUFFER_HALFSIZE)
    {
      size_t vacant = AUDIO_BUFFER_HALFSIZE - i2sa->audio_prepare_buffer_samples;
      while (vacant && count)
      {
        i2sa->audio_prepare_buffer[i2sa->audio_prepare_buffer_samples++] = *samples++;
        count --;
        vacant --;
        (*written) ++;
      }
    }
    else
    {
      if (!i2sa->audio_is_playing)
      {
        i2saudio_prep_audio_buffer(i2sa);
        if (i2sa->cur_write_audio_buffer == i2sa->audio_buffer)
        {
          i2sa->cur_write_audio_buffer = i2sa->audio_buffer_half;
        }
        else
        {
          i2sa->audio_is_prepped = 1;
          i2sa->audio_is_playing = 1;
          if (HAL_I2S_Transmit_DMA(i2sa->hi2s, (uint16_t*)i2sa->audio_buffer, AUDIO_BUFFER_SIZE) != HAL_OK)
          {
            i2sa->audio_is_playing = 0;
            return 0;
          }
        }
      }
      else
      {
        if (!i2sa->audio_is_prepped)
        {
          i2saudio_prep_audio_buffer(i2sa);
          i2sa->audio_is_prepped = 1;
        }
        else
        {
          break;
        }
      }
    }
  } while (count);
  return 1;
}
