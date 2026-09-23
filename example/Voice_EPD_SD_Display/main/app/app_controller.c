#include "app_controller.h"

#include <stdbool.h>

#include "app_event.h"
#include "drv_audio_codec.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "image_service.h"
#include "speech_recognizer.h"

static const char *TAG = "app_controller";

esp_err_t app_controller_start(void)
{
    ESP_LOGI(TAG, "Voice EPD SD Display starting");

    esp_err_t error = image_service_init();
    if (error != ESP_OK) {
        return error;
    }
    error = image_service_show_initial();
    if (error != ESP_OK) {
        return error;
    }

    QueueHandle_t event_queue = xQueueCreate(1, sizeof(app_event_t));
    if (event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    error = drv_audio_codec_init();
    if (error != ESP_OK) {
        return error;
    }
    error = speech_recognizer_start(event_queue);
    if (error != ESP_OK) {
        return error;
    }

    ESP_LOGI(TAG, "ready for voice navigation");
    while (true) {
        app_event_t event;
        if (xQueueReceive(event_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ESP_LOGI(TAG, "voice command: %s",
                 event.type == APP_EVENT_IMAGE_PREVIOUS ? "previous" :
                                                          "next");
        error = image_service_navigate(event.type);
        if (error != ESP_OK) {
            ESP_LOGW(TAG, "navigation failed: %s", esp_err_to_name(error));
        }
    }
}
