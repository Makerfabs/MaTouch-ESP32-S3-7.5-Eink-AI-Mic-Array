#include "bmp_decoder.h"

#include <stdio.h>
#include <stdlib.h>

#include "drv_epd.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bmp";

static uint16_t read_le16(const uint8_t *value)
{
    return value[0] | ((uint16_t)value[1] << 8);
}

static uint32_t read_le32(const uint8_t *value)
{
    return value[0] | ((uint32_t)value[1] << 8) |
           ((uint32_t)value[2] << 16) | ((uint32_t)value[3] << 24);
}

static uint8_t nearest_color(uint8_t red, uint8_t green, uint8_t blue)
{
    static const struct {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } palette[] = {
        {255, 255, 255},
        {255, 255, 0},
        {255, 0, 0},
        {0, 0, 0},
    };

    uint32_t best_distance = UINT32_MAX;
    uint8_t best = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        int dr = red - palette[i].red;
        int dg = green - palette[i].green;
        int db = blue - palette[i].blue;
        uint32_t distance = dr * dr + dg * dg + db * db;
        if (distance < best_distance) {
            best_distance = distance;
            best = i;
        }
    }
    return best;
}

esp_err_t bmp_decoder_decode(const char *path, uint8_t *frame,
                             size_t frame_size)
{
    ESP_RETURN_ON_FALSE(path != NULL && frame != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "null argument");
    ESP_RETURN_ON_FALSE(frame_size == DRV_EPD_FRAME_BYTES,
                        ESP_ERR_INVALID_SIZE, TAG, "invalid frame size");

    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        ESP_LOGE(TAG, "cannot open %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t result = ESP_FAIL;
    uint8_t header[54];
    if (fread(header, 1, sizeof(header), file) != sizeof(header)) {
        ESP_LOGE(TAG, "%s has a truncated header", path);
        goto cleanup;
    }

    const uint32_t pixel_offset = read_le32(header + 10);
    const uint32_t dib_size = read_le32(header + 14);
    const int32_t width = (int32_t)read_le32(header + 18);
    const int32_t signed_height = (int32_t)read_le32(header + 22);
    const uint16_t planes = read_le16(header + 26);
    const uint16_t bits_per_pixel = read_le16(header + 28);
    const uint32_t compression = read_le32(header + 30);

    if (header[0] != 'B' || header[1] != 'M' || dib_size < 40 ||
        width != DRV_EPD_WIDTH ||
        (signed_height != DRV_EPD_HEIGHT &&
         signed_height != -DRV_EPD_HEIGHT) ||
        planes != 1 || bits_per_pixel != 24 || compression != 0) {
        ESP_LOGE(TAG,
                 "unsupported BMP %s: %ldx%ld, %u bpp, compression %lu",
                 path, (long)width, (long)signed_height, bits_per_pixel,
                 (unsigned long)compression);
        result = ESP_ERR_NOT_SUPPORTED;
        goto cleanup;
    }

    const size_t row_size = ((DRV_EPD_WIDTH * 3U + 3U) / 4U) * 4U;
    uint8_t *row = malloc(row_size);
    if (row == NULL) {
        result = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    for (int file_row = 0; file_row < DRV_EPD_HEIGHT; ++file_row) {
        if (fseek(file, pixel_offset + (long)file_row * row_size, SEEK_SET) != 0 ||
            fread(row, 1, row_size, file) != row_size) {
            ESP_LOGE(TAG, "%s is truncated at row %d", path, file_row);
            result = ESP_ERR_INVALID_SIZE;
            free(row);
            goto cleanup;
        }

        const int y = signed_height > 0
                          ? DRV_EPD_HEIGHT - 1 - file_row
                          : file_row;
        uint8_t *output = frame + y * (DRV_EPD_WIDTH / 4);
        for (int x = 0; x < DRV_EPD_WIDTH; x += 4) {
            uint8_t packed = 0;
            for (int pixel = 0; pixel < 4; ++pixel) {
                const uint8_t *bgr = row + (x + pixel) * 3;
                packed |= nearest_color(bgr[2], bgr[1], bgr[0])
                          << (6 - pixel * 2);
            }
            output[x / 4] = packed;
        }
    }
    free(row);
    result = ESP_OK;
    ESP_LOGI(TAG, "decoded %s", path);

cleanup:
    fclose(file);
    return result;
}
