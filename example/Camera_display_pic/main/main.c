#include "app_controller.h"
#include "esp_err.h"
#include "esp_log.h"

void app_main(void) {
  esp_err_t error = app_controller_run();
  if (error != ESP_OK) {
    ESP_LOGE("main", "camera-to-EPD failed: %s", esp_err_to_name(error));
  }
}
