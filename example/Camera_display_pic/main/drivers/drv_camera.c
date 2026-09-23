#include "drv_camera.h"

#include <stdbool.h>

#include "app_config.h"
#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "camera";
static bool s_initialized;

esp_err_t drv_camera_init(void) {
  gpio_config_t power_config = {
      .pin_bit_mask = 1ULL << BOARD_CAMERA_POWER,
      .mode = GPIO_MODE_OUTPUT,
  };
  esp_err_t error = gpio_config(&power_config);
  if (error != ESP_OK) {
    return error;
  }

  gpio_set_level(BOARD_CAMERA_POWER, 0);
  vTaskDelay(pdMS_TO_TICKS(300));

  camera_config_t config = {
      .pin_pwdn = -1,
      .pin_reset = -1,
      .pin_xclk = BOARD_CAMERA_XCLK,
      .pin_sccb_sda = BOARD_CAMERA_SDA,
      .pin_sccb_scl = BOARD_CAMERA_SCL,
      .pin_d7 = BOARD_CAMERA_D7,
      .pin_d6 = BOARD_CAMERA_D6,
      .pin_d5 = BOARD_CAMERA_D5,
      .pin_d4 = BOARD_CAMERA_D4,
      .pin_d3 = BOARD_CAMERA_D3,
      .pin_d2 = BOARD_CAMERA_D2,
      .pin_d1 = BOARD_CAMERA_D1,
      .pin_d0 = BOARD_CAMERA_D0,
      .pin_vsync = BOARD_CAMERA_VSYNC,
      .pin_href = BOARD_CAMERA_HREF,
      .pin_pclk = BOARD_CAMERA_PCLK,
      .xclk_freq_hz = 20000000,
      .ledc_timer = LEDC_TIMER_0,
      .ledc_channel = LEDC_CHANNEL_0,
      .pixel_format = PIXFORMAT_RGB565,
      .frame_size = FRAMESIZE_SVGA,
      .jpeg_quality = 12,
      .fb_count = 1,
      .fb_location = CAMERA_FB_IN_PSRAM,
      .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
  };

  error = esp_camera_init(&config);
  if (error != ESP_OK) {
    gpio_set_level(BOARD_CAMERA_POWER, 1);
    ESP_LOGE(TAG, "initialization failed: %s", esp_err_to_name(error));
    return error;
  }
  s_initialized = true;

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    drv_camera_deinit();
    return ESP_FAIL;
  }
  sensor->set_hmirror(sensor, APP_CAMERA_HMIRROR);
  sensor->set_vflip(sensor, APP_CAMERA_VFLIP);
  ESP_LOGI(TAG, "initialized sensor PID=0x%04x", sensor->id.PID);
  return ESP_OK;
}

camera_fb_t *drv_camera_capture(void) {
  if (!s_initialized) {
    return NULL;
  }

  for (int i = 0; i < APP_CAMERA_WARMUP_FRAMES; ++i) {
    camera_fb_t *warmup = esp_camera_fb_get();
    if (warmup == NULL) {
      ESP_LOGE(TAG, "warmup frame %d failed", i + 1);
      return NULL;
    }
    esp_camera_fb_return(warmup);
  }

  camera_fb_t *frame = esp_camera_fb_get();
  if (frame == NULL) {
    ESP_LOGE(TAG, "capture failed");
    return NULL;
  }
  ESP_LOGI(TAG, "captured %ux%u RGB565 frame, %u bytes", (unsigned)frame->width,
           (unsigned)frame->height, (unsigned)frame->len);
  return frame;
}

void drv_camera_return(camera_fb_t *frame) {
  if (frame != NULL) {
    esp_camera_fb_return(frame);
  }
}

esp_err_t drv_camera_deinit(void) {
  esp_err_t error = ESP_OK;
  if (s_initialized) {
    error = esp_camera_deinit();
    s_initialized = false;
  }
  gpio_set_level(BOARD_CAMERA_POWER, 1);
  vTaskDelay(pdMS_TO_TICKS(50));
  return error;
}
