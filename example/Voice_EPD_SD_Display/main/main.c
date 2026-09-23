#include "app_controller.h"
#include "esp_err.h"

void app_main(void)
{
    ESP_ERROR_CHECK(app_controller_start());
}
