#include "image_service.h"

#include <ctype.h>
#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "app_config.h"
#include "bmp_decoder.h"
#include "drv_epd.h"
#include "drv_sdcard.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

typedef struct {
    char *names[APP_MAX_IMAGES];
    size_t count;
    size_t current;
    uint8_t *frame;
} image_service_context_t;

static const char *TAG = "image_service";
static image_service_context_t s_images;

static bool has_bmp_extension(const char *name)
{
    const char *dot = strrchr(name, '.');
    return dot != NULL && strlen(dot) == 4 &&
           tolower((unsigned char)dot[1]) == 'b' &&
           tolower((unsigned char)dot[2]) == 'm' &&
           tolower((unsigned char)dot[3]) == 'p';
}

static int compare_names(const void *left, const void *right)
{
    const char *const *a = left;
    const char *const *b = right;
    return strcasecmp(*a, *b);
}

static const char *current_name(void)
{
    return s_images.count == 0 ? NULL : s_images.names[s_images.current];
}

static void move_index(int direction)
{
    if (direction > 0) {
        s_images.current = (s_images.current + 1) % s_images.count;
    } else {
        s_images.current =
            (s_images.current + s_images.count - 1) % s_images.count;
    }
}

static esp_err_t scan_catalog(void)
{
    DIR *directory = opendir(APP_IMAGE_DIRECTORY);
    if (directory == NULL) {
        ESP_LOGE(TAG, "cannot open %s", APP_IMAGE_DIRECTORY);
        return ESP_ERR_NOT_FOUND;
    }

    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL &&
           s_images.count < APP_MAX_IMAGES) {
        if (!has_bmp_extension(entry->d_name)) {
            continue;
        }
        s_images.names[s_images.count] = strdup(entry->d_name);
        if (s_images.names[s_images.count] == NULL) {
            closedir(directory);
            return ESP_ERR_NO_MEM;
        }
        ++s_images.count;
    }
    closedir(directory);

    if (s_images.count == 0) {
        ESP_LOGW(TAG, "no BMP files found in %s", APP_IMAGE_DIRECTORY);
        return ESP_ERR_NOT_FOUND;
    }
    qsort(s_images.names, s_images.count, sizeof(s_images.names[0]),
          compare_names);
    ESP_LOGI(TAG, "found %u BMP files", (unsigned)s_images.count);
    return ESP_OK;
}

static esp_err_t display_current(void)
{
    const char *name = current_name();
    ESP_RETURN_ON_FALSE(name != NULL, ESP_ERR_NOT_FOUND, TAG,
                        "image catalog is empty");

    char path[512];
    int written = snprintf(path, sizeof(path), "%s/%s", APP_IMAGE_DIRECTORY,
                           name);
    ESP_RETURN_ON_FALSE(written > 0 && (size_t)written < sizeof(path),
                        ESP_ERR_INVALID_SIZE, TAG, "image path is too long");
    ESP_RETURN_ON_ERROR(
        bmp_decoder_decode(path, s_images.frame, DRV_EPD_FRAME_BYTES), TAG,
        "decode %s", name);
    ESP_RETURN_ON_ERROR(drv_epd_init(), TAG, "initialize panel");
    ESP_LOGI(TAG, "displaying %s", name);
    ESP_RETURN_ON_ERROR(
        drv_epd_display(s_images.frame, DRV_EPD_FRAME_BYTES), TAG,
        "display %s", name);
    return drv_epd_sleep();
}

esp_err_t image_service_init(void)
{
    if (s_images.frame != NULL) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(drv_sdcard_mount(), TAG, "mount SD card");
    ESP_RETURN_ON_ERROR(scan_catalog(), TAG, "scan image catalog");

    s_images.frame = heap_caps_malloc(
        DRV_EPD_FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_images.frame == NULL) {
        s_images.frame =
            heap_caps_malloc(DRV_EPD_FRAME_BYTES, MALLOC_CAP_8BIT);
    }
    ESP_RETURN_ON_FALSE(s_images.frame != NULL, ESP_ERR_NO_MEM, TAG,
                        "allocate image frame");
    return ESP_OK;
}

esp_err_t image_service_show_initial(void)
{
    ESP_RETURN_ON_FALSE(s_images.frame != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "image service is not initialized");
    for (size_t attempt = 0; attempt < s_images.count; ++attempt) {
        esp_err_t error = display_current();
        if (error == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "skipping %s: %s", current_name(),
                 esp_err_to_name(error));
        move_index(1);
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t image_service_navigate(app_event_type_t event)
{
    ESP_RETURN_ON_FALSE(event == APP_EVENT_IMAGE_PREVIOUS ||
                            event == APP_EVENT_IMAGE_NEXT,
                        ESP_ERR_INVALID_ARG, TAG, "invalid navigation event");
    if (s_images.count < 2) {
        ESP_LOGW(TAG, "navigation ignored: only one image is available");
        return ESP_OK;
    }

    const size_t original = s_images.current;
    const int direction = event == APP_EVENT_IMAGE_PREVIOUS ? -1 : 1;
    for (size_t attempt = 0; attempt < s_images.count - 1; ++attempt) {
        move_index(direction);
        esp_err_t error = display_current();
        if (error == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "cannot display %s: %s; trying another image",
                 current_name(), esp_err_to_name(error));
    }

    s_images.current = original;
    ESP_LOGE(TAG, "no other valid image; keeping %s", current_name());
    return ESP_ERR_NOT_FOUND;
}
