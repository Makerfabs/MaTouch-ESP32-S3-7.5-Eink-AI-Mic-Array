#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define DRV_EPD_WIDTH 800
#define DRV_EPD_HEIGHT 480
#define DRV_EPD_FRAME_BYTES ((DRV_EPD_WIDTH * DRV_EPD_HEIGHT) / 4)

typedef enum {
  DRV_EPD_COLOR_WHITE = 0,
  DRV_EPD_COLOR_YELLOW = 1,
  DRV_EPD_COLOR_RED = 2,
  DRV_EPD_COLOR_BLACK = 3,
} drv_epd_color_t;

esp_err_t drv_epd_init(void);
esp_err_t drv_epd_display(const uint8_t *frame, size_t length);
esp_err_t drv_epd_sleep(void);
