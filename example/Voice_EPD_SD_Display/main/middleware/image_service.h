#pragma once

#include "app_event.h"
#include "esp_err.h"

esp_err_t image_service_init(void);
esp_err_t image_service_show_initial(void);
esp_err_t image_service_navigate(app_event_type_t event);
