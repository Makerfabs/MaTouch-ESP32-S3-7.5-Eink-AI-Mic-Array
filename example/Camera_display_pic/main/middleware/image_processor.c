#include "image_processor.h"

#include <string.h>

#include "drv_epd.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "image_processor";

#define ERROR_CHANNELS 3
#define ERROR_ROW_PIXELS (DRV_EPD_WIDTH + 2)
#define ERROR_ROW_VALUES (ERROR_ROW_PIXELS * ERROR_CHANNELS)
#define COLOR_MAX 255
#define ERROR_SCALE 16
#define ERROR_LIMIT 255
#define ERROR_RIGHT_WEIGHT 7
#define ERROR_DOWN_LEFT_WEIGHT 3
#define ERROR_DOWN_WEIGHT 5
#define ERROR_DOWN_RIGHT_WEIGHT 1
#define ERROR_WEIGHT_SUM                                                        \
  (ERROR_RIGHT_WEIGHT + ERROR_DOWN_LEFT_WEIGHT + ERROR_DOWN_WEIGHT +           \
   ERROR_DOWN_RIGHT_WEIGHT)
typedef struct {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} rgb_color_t;

static const rgb_color_t s_palette[] = {
    [DRV_EPD_COLOR_WHITE] = {COLOR_MAX, COLOR_MAX, COLOR_MAX},
    [DRV_EPD_COLOR_YELLOW] = {COLOR_MAX, COLOR_MAX, 0},
    [DRV_EPD_COLOR_RED] = {COLOR_MAX, 0, 0},
    [DRV_EPD_COLOR_BLACK] = {0, 0, 0},
};

static uint8_t nearest_epd_color(int red, int green, int blue) {
  uint32_t best_distance = UINT32_MAX;
  uint8_t best_color = DRV_EPD_COLOR_WHITE;
  for (uint8_t color = 0; color < 4; ++color) {
    const int dr = red - s_palette[color].red;
    const int dg = green - s_palette[color].green;
    const int db = blue - s_palette[color].blue;
    uint32_t distance = dr * dr + dg * dg + db * db;
    if (distance < best_distance) {
      best_distance = distance;
      best_color = color;
    }
  }
  return best_color;
}

static int clamp_channel(int value) {
  if (value < 0) {
    return 0;
  }
  if (value > COLOR_MAX) {
    return COLOR_MAX;
  }
  return value;
}

static int clamp_error(int value) {
  if (value < -ERROR_LIMIT) {
    return -ERROR_LIMIT;
  }
  if (value > ERROR_LIMIT) {
    return ERROR_LIMIT;
  }
  return value;
}

static void read_rgb565(const uint8_t *source, int *red, int *green,
                        int *blue) {
  const uint16_t value = ((uint16_t)source[0] << 8) | source[1];
  *red = (((value >> 11) & 0x1f) * 255 + 15) / 31;
  *green = (((value >> 5) & 0x3f) * 255 + 31) / 63;
  *blue = ((value & 0x1f) * 255 + 15) / 31;
}

static void pack_pixel(uint8_t *frame, size_t x, size_t y, uint8_t color) {
  uint8_t *packed = frame + y * (DRV_EPD_WIDTH / 4) + x / 4;
  const uint8_t shift = 6 - (x % 4) * 2;
  *packed = (*packed & ~(0x03U << shift)) | (color << shift);
}

static void add_error(int16_t *target, const int error[ERROR_CHANNELS],
                      int weight) {
  for (size_t channel = 0; channel < ERROR_CHANNELS; ++channel) {
    target[channel] += error[channel] * weight;
  }
}

esp_err_t image_processor_convert(const camera_fb_t *camera_frame,
                                  uint8_t *epd_frame, size_t epd_frame_size) {
  ESP_RETURN_ON_FALSE(camera_frame != NULL && epd_frame != NULL,
                      ESP_ERR_INVALID_ARG, TAG, "null buffer");
  ESP_RETURN_ON_FALSE(camera_frame->format == PIXFORMAT_RGB565,
                      ESP_ERR_NOT_SUPPORTED, TAG, "frame is not RGB565");
  ESP_RETURN_ON_FALSE(
      camera_frame->width >= DRV_EPD_WIDTH &&
          camera_frame->height >= DRV_EPD_HEIGHT,
      ESP_ERR_INVALID_SIZE, TAG, "frame %ux%u is smaller than the panel",
      (unsigned)camera_frame->width, (unsigned)camera_frame->height);
  ESP_RETURN_ON_FALSE(epd_frame_size == DRV_EPD_FRAME_BYTES,
                      ESP_ERR_INVALID_SIZE, TAG, "invalid EPD frame size");

  const size_t required_bytes =
      (size_t)camera_frame->width * camera_frame->height * 2;
  ESP_RETURN_ON_FALSE(camera_frame->len >= required_bytes, ESP_ERR_INVALID_SIZE,
                      TAG, "truncated camera frame");

  const size_t crop_x = (camera_frame->width - DRV_EPD_WIDTH) / 2;
  const size_t crop_y = (camera_frame->height - DRV_EPD_HEIGHT) / 2;

  int16_t *error_rows = heap_caps_calloc(ERROR_ROW_VALUES * 2,
                                         sizeof(*error_rows), MALLOC_CAP_8BIT);
  ESP_RETURN_ON_FALSE(error_rows != NULL, ESP_ERR_NO_MEM, TAG,
                      "cannot allocate dithering buffers");
  int16_t *current_error = error_rows;
  int16_t *next_error = error_rows + ERROR_ROW_VALUES;
  size_t color_counts[4] = {0};

  memset(epd_frame, 0, epd_frame_size);
  for (size_t y = 0; y < DRV_EPD_HEIGHT; ++y) {
    const uint8_t *source =
        camera_frame->buf + ((y + crop_y) * camera_frame->width + crop_x) * 2;
    for (int x = 0; x < DRV_EPD_WIDTH; ++x) {
      const size_t error_index = (x + 1) * ERROR_CHANNELS;
      int channels[ERROR_CHANNELS];
      read_rgb565(source + x * 2, &channels[0], &channels[1], &channels[2]);
      for (size_t channel = 0; channel < ERROR_CHANNELS; ++channel) {
        channels[channel] =
            clamp_channel(channels[channel] +
                          current_error[error_index + channel] / ERROR_SCALE);
      }

      const uint8_t color =
          nearest_epd_color(channels[0], channels[1], channels[2]);
      pack_pixel(epd_frame, x, y, color);
      ++color_counts[color];

      const int quantization_error[ERROR_CHANNELS] = {
          clamp_error(channels[0] - s_palette[color].red),
          clamp_error(channels[1] - s_palette[color].green),
          clamp_error(channels[2] - s_palette[color].blue),
      };

      add_error(current_error + error_index + ERROR_CHANNELS,
                quantization_error,
                ERROR_RIGHT_WEIGHT);
      add_error(next_error + error_index - ERROR_CHANNELS, quantization_error,
                ERROR_DOWN_LEFT_WEIGHT);
      add_error(next_error + error_index, quantization_error,
                ERROR_DOWN_WEIGHT);
      add_error(next_error + error_index + ERROR_CHANNELS, quantization_error,
                ERROR_DOWN_RIGHT_WEIGHT);
    }

    int16_t *finished_error = current_error;
    current_error = next_error;
    next_error = finished_error;
    memset(next_error, 0, ERROR_ROW_VALUES * sizeof(*next_error));
  }

  heap_caps_free(error_rows);

  ESP_LOGI(TAG,
           "reference Floyd-Steinberg %ux%u -> 800x480: diffusion=%u%% "
           "white=%u yellow=%u red=%u black=%u",
           (unsigned)camera_frame->width, (unsigned)camera_frame->height,
           ERROR_WEIGHT_SUM * 100 / ERROR_SCALE,
           (unsigned)color_counts[DRV_EPD_COLOR_WHITE],
           (unsigned)color_counts[DRV_EPD_COLOR_YELLOW],
           (unsigned)color_counts[DRV_EPD_COLOR_RED],
           (unsigned)color_counts[DRV_EPD_COLOR_BLACK]);
  return ESP_OK;
}
