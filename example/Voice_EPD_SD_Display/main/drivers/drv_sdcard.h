#pragma once

#include "esp_err.h"

esp_err_t drv_sdcard_mount(void);
void drv_sdcard_unmount(void);
