#include "app_controller.h"

#include "drv_camera.h"
#include "drv_epd.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "image_processor.h"

static const char *TAG = "app";

esp_err_t app_controller_run(void) {
  ESP_LOGI(TAG, "camera-to-EPD starting");

  uint8_t *epd_frame = heap_caps_malloc(DRV_EPD_FRAME_BYTES,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  ESP_RETURN_ON_FALSE(epd_frame != NULL, ESP_ERR_NO_MEM, TAG,
                      "cannot allocate %u-byte EPD frame",
                      (unsigned)DRV_EPD_FRAME_BYTES);

  esp_err_t result = drv_camera_init();
  if (result != ESP_OK) {
    goto cleanup;
  }

  camera_fb_t *camera_frame = drv_camera_capture();
  if (camera_frame == NULL) {
    result = ESP_FAIL;
    goto camera_cleanup;
  }

  result =
      image_processor_convert(camera_frame, epd_frame, DRV_EPD_FRAME_BYTES);
  drv_camera_return(camera_frame);

camera_cleanup: {
  esp_err_t deinit_result = drv_camera_deinit();
  if (result == ESP_OK) {
    result = deinit_result;
  }
}
  if (result != ESP_OK) {
    goto cleanup;
  }

  result = drv_epd_init();
  if (result != ESP_OK) {
    goto cleanup;
  }
  ESP_LOGI(TAG, "refreshing e-paper display");
  result = drv_epd_display(epd_frame, DRV_EPD_FRAME_BYTES);
  if (result == ESP_OK) {
    result = drv_epd_sleep();
  }
  if (result == ESP_OK) {
    ESP_LOGI(TAG, "capture and display completed");
  }

cleanup:
  heap_caps_free(epd_frame);
  return result;
}
