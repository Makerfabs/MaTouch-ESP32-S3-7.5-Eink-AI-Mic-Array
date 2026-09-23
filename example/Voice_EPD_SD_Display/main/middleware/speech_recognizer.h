#pragma once

#include "app_event.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

esp_err_t speech_recognizer_start(QueueHandle_t event_queue);
