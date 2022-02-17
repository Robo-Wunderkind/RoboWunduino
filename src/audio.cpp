#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "audio.h"
#include "robowunderkind.h"

uint8_t system_loudness_level = 100;

void init_i2s(void)
{
  i2s_config_t i2s_config = {
       .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
       .sample_rate =  I2S_SAMPLE_RATE,
       .bits_per_sample = I2S_SAMPLE_BITS,
       .channel_format = I2S_FORMAT,
       .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S_MSB,
       .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
       .dma_buf_count = 2, 
       .dma_buf_len = 64
  };
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);  // Install and start i2s driver.
  i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);         // Initializes Pin 25 DAC.
}

int i2s_dac_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    for (int i = 0; i < len; i++) 
    {
      if(system_loudness_level > 75)
      {
        d_buff[j++] = 0;
        d_buff[j++] = s_buff[i];
      }
      else if(system_loudness_level > 50 && system_loudness_level <= 75)
      {
        d_buff[j++] = 0;
        d_buff[j++] = s_buff[i] >> 1;
      }
      else if(system_loudness_level > 25 && system_loudness_level <= 50)
      {
        d_buff[j++] = 0;
        d_buff[j++] = s_buff[i] >> 2;
      }
      else if(system_loudness_level > 0 && system_loudness_level <= 25)
      {
        d_buff[j++] = 0;
        d_buff[j++] = s_buff[i] >> 3;
      }
      else 
      {
        d_buff[j++] = 0;
        d_buff[j++] = 0x80;
      }
    }
    return (len * 2);
}

bool playback_audioclip(const uint8_t* audiosample_to_play, int tot_size)
{
  int offset = 0;
  int i2s_read_len = I2S_READ_LEN;
  uint8_t audiosample_found = 0;
  size_t bytes_written;

  if (tot_size > sizeof(uint8_t))
  {
    digitalWrite(GPIO_AUDIO_EN, HIGH);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    uint8_t* i2s_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    i2s_set_clk(I2S_NUM, I2S_SAMPLE_RATE, I2S_SAMPLE_BITS, I2S_CHANNEL_NUM);
    i2s_start(I2S_NUM);
    vTaskDelay(20 / portTICK_PERIOD_MS);

    while ((offset < tot_size))
    {
      int i2s_wr_len = 0;
      int play_len = ((tot_size - offset) > (4 * 1024)) ? (4 * 1024) : (tot_size - offset);
      i2s_wr_len = i2s_dac_data_scale(i2s_write_buff, (uint8_t*)(audiosample_to_play + offset), play_len);
      i2s_write(I2S_NUM, (const char*) i2s_write_buff, i2s_wr_len, &bytes_written, portMAX_DELAY);
      offset += play_len;
    }

    i2s_stop(I2S_NUM);
    i2s_set_clk(I2S_NUM, I2S_SAMPLE_RATE, I2S_SAMPLE_BITS, I2S_CHANNEL_NUM);
    free(i2s_write_buff);
    digitalWrite(GPIO_AUDIO_EN, LOW);
    i2s_zero_dma_buffer(I2S_NUM);
    return true;
  }
  return false;
}
