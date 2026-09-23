#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_camera.h"
#include "esp_err.h"

esp_err_t image_processor_convert(const camera_fb_t *camera_frame,
                                  uint8_t *epd_frame, size_t epd_frame_size);
