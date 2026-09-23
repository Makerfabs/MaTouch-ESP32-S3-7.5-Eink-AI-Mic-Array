#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t bmp_decoder_decode(const char *path, uint8_t *frame,
                             size_t frame_size);
