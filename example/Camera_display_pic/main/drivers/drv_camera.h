#pragma once

#include "esp_camera.h"
#include "esp_err.h"

esp_err_t drv_camera_init(void);
camera_fb_t *drv_camera_capture(void);
void drv_camera_return(camera_fb_t *frame);
esp_err_t drv_camera_deinit(void);
