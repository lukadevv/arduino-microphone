#include "driver/i2s.h"
#include "Arduino.h"

#define I2S_MIC_PORT     I2S_NUM_1
#define I2S_MIC_BCLK     GPIO_NUM_14
#define I2S_MIC_WS       GPIO_NUM_15
#define I2S_MIC_SD       GPIO_NUM_32

#define I2S_AMP_PORT     I2S_NUM_0
#define I2S_AMP_BCLK     GPIO_NUM_27
#define I2S_AMP_WS       GPIO_NUM_26
#define I2S_AMP_DIN      GPIO_NUM_25

#define SAMPLE_RATE      16000
#define SAMPLE_BITS      I2S_BITS_PER_SAMPLE_16BIT
#define CHANNEL_FMT      I2S_CHANNEL_FMT_ONLY_LEFT
#define BUF_COUNT        8
#define BUF_LEN          64
#define AUDIO_BUFFER_SIZE 512

void setupI2SMic() {
  i2s_config_t mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = SAMPLE_BITS,
    .channel_format = CHANNEL_FMT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = BUF_COUNT,
    .dma_buf_len = BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t mic_pins = {
    .bck_io_num = I2S_MIC_BCLK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };
  i2s_driver_install(I2S_MIC_PORT, &mic_config, 0, NULL);
  i2s_set_pin(I2S_MIC_PORT, &mic_pins);
  i2s_start(I2S_MIC_PORT);
}

void setupI2SAmp() {
  i2s_config_t amp_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = SAMPLE_BITS,
    .channel_format = CHANNEL_FMT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = BUF_COUNT,
    .dma_buf_len = BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t amp_pins = {
    .bck_io_num = I2S_AMP_BCLK,
    .ws_io_num = I2S_AMP_WS,
    .data_out_num = I2S_AMP_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_AMP_PORT, &amp_config, 0, NULL);
  i2s_set_pin(I2S_AMP_PORT, &amp_pins);
  i2s_start(I2S_AMP_PORT);
}

void setup() {
  Serial.begin(921600);
  Serial.println("Starting megaphone...");
  setupI2SMic();
  setupI2SAmp();
}

void loop() {
  uint8_t micBuffer[AUDIO_BUFFER_SIZE] = {0};
  int16_t* samples = (int16_t*)micBuffer;
  size_t bytesRead = 0;

  esp_err_t err = i2s_read(I2S_MIC_PORT, micBuffer, AUDIO_BUFFER_SIZE, &bytesRead, portMAX_DELAY);
  if (err == ESP_OK && bytesRead > 0) {
    size_t sampleCount = bytesRead / sizeof(int16_t);
    float gain = 18.0f; 

    for (size_t i = 0; i < sampleCount; i++) {
      int32_t amplified = (int32_t)(samples[i] * gain);

      if (amplified > 32767) amplified = 32767;
      else if (amplified < -32768) amplified = -32768;

      samples[i] = (int16_t)amplified;
    }

    size_t bytesWritten = 0;
    i2s_write(I2S_AMP_PORT, micBuffer, bytesRead, &bytesWritten, portMAX_DELAY);
  }

  delay(1); 
}
