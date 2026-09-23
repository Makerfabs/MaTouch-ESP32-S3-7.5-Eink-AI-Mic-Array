#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t drv_audio_codec_init(void);
esp_err_t drv_audio_codec_read(int16_t *samples, size_t sample_count,
                               size_t *samples_read, uint32_t timeout_ms);
