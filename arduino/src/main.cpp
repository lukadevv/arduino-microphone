#include <WiFi.h>
#include <WebSocketsClient.h>
#include "driver/i2s.h"
#include "config/wifi.h"

#define WEBSOCKET_SERVER "192.168.0.22"
#define WEBSOCKET_PORT   8765

// MIC (INMP441)
#define I2S_MIC_PORT     I2S_NUM_1
#define I2S_MIC_BCLK     GPIO_NUM_14
#define I2S_MIC_WS       GPIO_NUM_15
#define I2S_MIC_SD       GPIO_NUM_32

#define SAMPLE_RATE      8000
#define SAMPLE_BITS      I2S_BITS_PER_SAMPLE_32BIT
#define MIC_CHANNEL_FMT  I2S_CHANNEL_FMT_ONLY_LEFT
#define MIC_BUF_COUNT    16
#define MIC_BUF_LEN      64
#define AUDIO_BUFFER_SIZE 1024


// AMP
#define I2S_AMP_PORT     I2S_NUM_0
#define I2S_AMP_BCLK     26
#define I2S_AMP_WS       27
#define I2S_AMP_DIN      21

#define AMP_CHANNEL_FMT  I2S_CHANNEL_FMT_ONLY_LEFT
#define AMP_BUF_COUNT    8
#define AMP_BUF_LEN      64

WebSocketsClient webSocket;

uint8_t playBuffer[AUDIO_BUFFER_SIZE] = {0};
volatile bool newAudioReceived = false;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("Connected to websocket");
      break;
    case WStype_DISCONNECTED:
      Serial.println("Disconnected from Websocket");
      break;
    case WStype_ERROR:
      Serial.println("WebSocket Error");
      break;
    case WStype_BIN:
      if (length <= AUDIO_BUFFER_SIZE) {
        memcpy(playBuffer, payload, length);
        newAudioReceived = true;
      }
      break;
    default:
      break;
  }
}

void setupWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());
}

void setupI2SMic() {
  i2s_config_t mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = SAMPLE_BITS,
    .channel_format = MIC_CHANNEL_FMT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = MIC_BUF_COUNT,
    .dma_buf_len = MIC_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t mic_pin_config = {
    .bck_io_num = I2S_MIC_BCLK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };

  i2s_driver_install(I2S_MIC_PORT, &mic_config, 0, NULL);
  i2s_set_pin(I2S_MIC_PORT, &mic_pin_config);
  i2s_zero_dma_buffer(I2S_MIC_PORT);
  i2s_start(I2S_MIC_PORT);
}

void setupI2SAmp() {
  i2s_config_t amp_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = SAMPLE_BITS,
    .channel_format = AMP_CHANNEL_FMT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = AMP_BUF_COUNT,
    .dma_buf_len = AMP_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t amp_pin_config = {
    .bck_io_num = I2S_AMP_BCLK,
    .ws_io_num = I2S_AMP_WS,
    .data_out_num = I2S_AMP_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_AMP_PORT, &amp_config, 0, NULL);
  i2s_set_pin(I2S_AMP_PORT, &amp_pin_config);
  i2s_zero_dma_buffer(I2S_AMP_PORT);
  i2s_start(I2S_AMP_PORT);
}

void setup() {
  Serial.begin(921600);
  delay(1000);
  Serial.println("Setup....");
  setupWiFi();

  webSocket.begin(WEBSOCKET_SERVER, WEBSOCKET_PORT, "/");
  webSocket.onEvent(webSocketEvent);

  setupI2SMic();
  setupI2SAmp();
}

void loop() {
  webSocket.loop();

  uint8_t micBuffer[AUDIO_BUFFER_SIZE] = {0};
  size_t bytesRead = 0;
  esp_err_t err = i2s_read(I2S_MIC_PORT, (void*)micBuffer, AUDIO_BUFFER_SIZE, &bytesRead, portMAX_DELAY);
  
  if (err == ESP_OK && bytesRead > 0) {
    webSocket.sendBIN(micBuffer, bytesRead);
  }

  if (newAudioReceived) {
    size_t bytesWritten = 0;
    err = i2s_write(I2S_AMP_PORT, (const void*)playBuffer, AUDIO_BUFFER_SIZE, &bytesWritten, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("Error writing audio in I2S (amp): %d\n", err);
    }
    newAudioReceived = false;
  }

  delay(10);
}
