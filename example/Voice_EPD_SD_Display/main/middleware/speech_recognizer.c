#include "speech_recognizer.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "drv_audio_codec.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "esp_timer.h"
#include "esp_wn_models.h"
#include "freertos/task.h"
#include "model_path.h"

typedef struct {
    QueueHandle_t queue;
    srmodel_list_t *models;
    esp_afe_sr_iface_t *afe;
    esp_afe_sr_data_t *afe_data;
    esp_mn_iface_t *multinet;
    model_iface_data_t *multinet_data;
} speech_context_t;

static const char *TAG = "speech";
static speech_context_t s_speech;

typedef struct {
    int16_t *samples;
    size_t capacity;
    size_t count;
} multinet_stream_t;

typedef struct {
    uint64_t sum_squares;
    size_t sample_count;
    uint32_t clipped_samples;
    uint32_t peak;
} audio_level_t;

static void audio_level_add_sample(audio_level_t *level, int16_t input)
{
    const int32_t sample = input;
    const uint32_t magnitude = (uint32_t)(sample < 0 ? -sample : sample);
    level->sum_squares += (uint64_t)(sample * sample);
    ++level->sample_count;
    if (magnitude > level->peak) {
        level->peak = magnitude;
    }
    if (magnitude >= 32760U) {
        ++level->clipped_samples;
    }
}

static void audio_level_add(audio_level_t *level, const int16_t *samples,
                            size_t sample_count)
{
    for (size_t index = 0; index < sample_count; ++index) {
        audio_level_add_sample(level, samples[index]);
    }
}

static double audio_level_rms(const audio_level_t *level)
{
    return level->sample_count > 0
               ? sqrt((double)level->sum_squares /
                      (double)level->sample_count)
               : 0.0;
}

static void log_audio_level(const audio_level_t *level, const char *result)
{
    ESP_LOGI(TAG,
             "MultiNet input: result=%s, rms=%.1f, peak=%u, clipped=%u, "
             "samples=%u",
             result, audio_level_rms(level), (unsigned)level->peak,
             (unsigned)level->clipped_samples,
             (unsigned)level->sample_count);
}

static void audio_feed_task(void *argument)
{
    speech_context_t *speech = argument;
    const int chunk = speech->afe->get_feed_chunksize(speech->afe_data);
    const int channels = speech->afe->get_feed_channel_num(speech->afe_data);
    if (channels != APP_AUDIO_CHANNELS) {
        ESP_LOGE(TAG, "AFE channel mismatch: %d/%d", channels,
                 APP_AUDIO_CHANNELS);
        vTaskDelete(NULL);
        return;
    }
    const size_t raw_sample_count =
        (size_t)chunk * APP_AUDIO_CAPTURE_CHANNELS;
    int16_t *raw_samples = malloc(raw_sample_count * sizeof(*raw_samples));
    if (raw_samples == NULL) {
        ESP_LOGE(TAG, "cannot allocate AFE feed buffer");
        free(raw_samples);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG,
             "audio feed started: %d I2S frames x %d MICs -> %d AFE channels",
             chunk, APP_AUDIO_CAPTURE_CHANNELS, channels);
    while (true) {
        size_t samples_read = 0;
        esp_err_t error = drv_audio_codec_read(raw_samples, raw_sample_count,
                                               &samples_read, 1000);
        if (error != ESP_OK || samples_read != raw_sample_count) {
            ESP_LOGW(TAG, "I2S read failed: %s (%u/%u samples)",
                     esp_err_to_name(error), (unsigned)samples_read,
                     (unsigned)raw_sample_count);
            continue;
        }

        if (speech->afe->feed(speech->afe_data, raw_samples) < 0) {
            ESP_LOGW(TAG, "AFE feed rejected a frame");
        }
    }
}

static esp_mn_state_t feed_multinet(speech_context_t *speech,
                                    multinet_stream_t *stream,
                                    const int16_t *samples,
                                    size_t sample_count)
{
    esp_mn_state_t state = ESP_MN_STATE_DETECTING;
    while (sample_count > 0) {
        size_t copy_count = stream->capacity - stream->count;
        if (copy_count > sample_count) {
            copy_count = sample_count;
        }
        memcpy(stream->samples + stream->count, samples,
               copy_count * sizeof(*samples));
        stream->count += copy_count;
        samples += copy_count;
        sample_count -= copy_count;

        if (stream->count == stream->capacity) {
            state = speech->multinet->detect(speech->multinet_data,
                                              stream->samples);
            stream->count = 0;
            if (state != ESP_MN_STATE_DETECTING) {
                break;
            }
        }
    }
    return state;
}

static bool handle_detected_command(speech_context_t *speech,
                                    int64_t *last_command_us)
{
    esp_mn_results_t *detected =
        speech->multinet->get_results(speech->multinet_data);
    if (detected == NULL || detected->num == 0) {
        return false;
    }

    const float probability = detected->prob[0];
    const int64_t now_us = esp_timer_get_time();
    if (*last_command_us != 0 &&
        now_us - *last_command_us < APP_COMMAND_DEBOUNCE_MS * 1000LL) {
        return false;
    }

    app_event_t event;
    if (detected->command_id[0] == APP_COMMAND_PREVIOUS_ID) {
        event.type = APP_EVENT_IMAGE_PREVIOUS;
    } else if (detected->command_id[0] == APP_COMMAND_NEXT_ID) {
        event.type = APP_EVENT_IMAGE_NEXT;
    } else {
        ESP_LOGW(TAG, "ignoring command id %d", detected->command_id[0]);
        return false;
    }

    *last_command_us = now_us;
    xQueueOverwrite(speech->queue, &event);
    ESP_LOGI(TAG, "recognized %s (id=%d, probability=%.3f)",
             event.type == APP_EVENT_IMAGE_PREVIOUS ? "previous" : "next",
             detected->command_id[0], probability);
    return true;
}

static void recognition_task(void *argument)
{
    speech_context_t *speech = argument;
    const int afe_chunk =
        speech->afe->get_fetch_chunksize(speech->afe_data);
    const int mn_chunk =
        speech->multinet->get_samp_chunksize(speech->multinet_data);
    if (afe_chunk != mn_chunk) {
        ESP_LOGE(TAG, "AFE/MultiNet chunk mismatch: %d/%d", afe_chunk,
                 mn_chunk);
        vTaskDelete(NULL);
        return;
    }

    multinet_stream_t stream = {
        .samples = malloc((size_t)mn_chunk * sizeof(int16_t)),
        .capacity = (size_t)mn_chunk,
        .count = 0,
    };
    if (stream.samples == NULL) {
        ESP_LOGE(TAG, "cannot allocate MultiNet frame buffer");
        vTaskDelete(NULL);
        return;
    }

    const int sample_rate = speech->afe->get_samp_rate(speech->afe_data);
    const int trailing_frames =
        (APP_VAD_TRAILING_AUDIO_MS * sample_rate / 1000 + afe_chunk - 1) /
        afe_chunk;
    bool recognition_active = false;
    bool wait_for_silence = false;
    bool command_window_open = false;
    int silence_frames = 0;
    int64_t last_command_us = 0;
    int64_t command_window_deadline_us = 0;
    audio_level_t audio_level = {0};
    ESP_LOGI(TAG, "say wake word '%s' before %s / %s", APP_WAKE_WORD,
             APP_COMMAND_PREVIOUS_PHRASE, APP_COMMAND_NEXT_PHRASE);
    while (true) {
        afe_fetch_result_t *result = speech->afe->fetch(speech->afe_data);
        if (result == NULL || result->ret_value != ESP_OK) {
            ESP_LOGW(TAG, "AFE fetch failed");
            continue;
        }

        const bool voice_detected = result->vad_state == VAD_SPEECH;
        const int64_t now_us = esp_timer_get_time();
        if (result->wakeup_state == WAKENET_DETECTED) {
            command_window_open = true;
            command_window_deadline_us =
                now_us + APP_WAKE_COMMAND_WINDOW_MS * 1000LL;
            recognition_active = false;
            wait_for_silence = true;
            silence_frames = 0;
            stream.count = 0;
            speech->multinet->clean(speech->multinet_data);
            ESP_LOGI(TAG,
                     "wake word detected (index=%d); command window open for "
                     "%d ms",
                     result->wake_word_index, APP_WAKE_COMMAND_WINDOW_MS);
            continue;
        }

        if (!command_window_open) {
            continue;
        }
        if (now_us >= command_window_deadline_us) {
            command_window_open = false;
            recognition_active = false;
            wait_for_silence = false;
            stream.count = 0;
            speech->multinet->clean(speech->multinet_data);
            ESP_LOGI(TAG, "command window expired; wake word required");
            continue;
        }

        if (wait_for_silence) {
            if (!voice_detected) {
                wait_for_silence = false;
                ESP_LOGI(TAG, "VAD ready for next utterance");
            }
            continue;
        }

        if (!recognition_active && !voice_detected) {
            continue;
        }

        esp_mn_state_t state = ESP_MN_STATE_DETECTING;
        if (!recognition_active) {
            recognition_active = true;
            silence_frames = 0;
            stream.count = 0;
            memset(&audio_level, 0, sizeof(audio_level));
            speech->multinet->clean(speech->multinet_data);
            ESP_LOGI(TAG, "VAD speech start (cache=%d bytes)",
                     result->vad_cache_size);

            if (result->vad_cache != NULL && result->vad_cache_size > 0) {
                const size_t cache_samples =
                    (size_t)result->vad_cache_size / sizeof(int16_t);
                audio_level_add(&audio_level, result->vad_cache,
                                cache_samples);
                state = feed_multinet(
                    speech, &stream, result->vad_cache,
                    cache_samples);
            }
        }

        if (state == ESP_MN_STATE_DETECTING) {
            const size_t data_samples =
                (size_t)result->data_size / sizeof(int16_t);
            audio_level_add(&audio_level, result->data, data_samples);
            state = feed_multinet(speech, &stream, result->data,
                                  data_samples);
        }

        if (state == ESP_MN_STATE_DETECTED) {
            log_audio_level(&audio_level, "detected");
            if (handle_detected_command(speech, &last_command_us)) {
                command_window_open = false;
                ESP_LOGI(TAG, "command accepted; wake word required again");
            }
            speech->multinet->clean(speech->multinet_data);
            stream.count = 0;
            recognition_active = false;
            wait_for_silence = voice_detected;
            ESP_LOGI(TAG, "VAD utterance complete");
            continue;
        }
        if (state == ESP_MN_STATE_TIMEOUT) {
            log_audio_level(&audio_level, "timeout");
            speech->multinet->clean(speech->multinet_data);
            stream.count = 0;
            recognition_active = false;
            wait_for_silence = voice_detected;
            ESP_LOGI(TAG, "VAD utterance timed out");
            continue;
        }

        if (voice_detected) {
            silence_frames = 0;
        } else {
            ++silence_frames;
            if (silence_frames >= trailing_frames) {
                log_audio_level(&audio_level, "no-command");
                ESP_LOGI(TAG, "VAD speech end without command");
                speech->multinet->clean(speech->multinet_data);
                stream.count = 0;
                recognition_active = false;
            }
        }
    }
}

static esp_err_t init_models(speech_context_t *speech)
{
    speech->models = esp_srmodel_init("model");
    ESP_RETURN_ON_FALSE(speech->models != NULL, ESP_FAIL, TAG,
                        "cannot load model partition");

    char *model_name =
        esp_srmodel_filter(speech->models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    ESP_RETURN_ON_FALSE(model_name != NULL, ESP_ERR_NOT_FOUND, TAG,
                        "Chinese MultiNet model not found");
    speech->multinet = esp_mn_handle_from_name(model_name);
    ESP_RETURN_ON_FALSE(speech->multinet != NULL, ESP_FAIL, TAG,
                        "cannot get MultiNet interface");
    speech->multinet_data =
        speech->multinet->create(model_name, APP_COMMAND_TIMEOUT_MS);
    ESP_RETURN_ON_FALSE(speech->multinet_data != NULL, ESP_ERR_NO_MEM, TAG,
                        "cannot create MultiNet");

    ESP_RETURN_ON_ERROR(
        esp_mn_commands_alloc(speech->multinet, speech->multinet_data), TAG,
        "allocate command list");
    ESP_RETURN_ON_ERROR(
        esp_mn_commands_add(APP_COMMAND_PREVIOUS_ID,
                            APP_COMMAND_PREVIOUS_PHRASE), TAG,
        "add previous command");
    ESP_RETURN_ON_ERROR(
        esp_mn_commands_add(APP_COMMAND_NEXT_ID,
                            APP_COMMAND_NEXT_PHRASE), TAG,
        "add next command");
    ESP_RETURN_ON_FALSE(esp_mn_commands_update() == NULL,
                        ESP_ERR_INVALID_STATE, TAG,
                        "one or more commands are invalid");
    speech->multinet->print_active_speech_commands(speech->multinet_data);

    char *wake_model =
        esp_srmodel_filter(speech->models, ESP_WN_PREFIX, "wn9s_hiesp");
    ESP_RETURN_ON_FALSE(wake_model != NULL, ESP_ERR_NOT_FOUND, TAG,
                        "wake word model not found");

    afe_config_t *afe_config =
        afe_config_init("NM", speech->models, AFE_TYPE_SR,
                        AFE_MODE_HIGH_PERF);
    ESP_RETURN_ON_FALSE(afe_config != NULL, ESP_ERR_NO_MEM, TAG,
                        "cannot create AFE config");
    afe_config->aec_init = false;
    afe_config->wakenet_init = true;
    afe_config->wakenet_model_name = wake_model;
    afe_config->agc_init = true;
    afe_config->agc_mode = AFE_AGC_MODE_WEBRTC;
    afe_config->agc_compression_gain_db = 9;
    afe_config->agc_target_level_dbfs = 3;
    afe_config->ns_init = false;
    afe_config->afe_ns_mode = AFE_NS_MODE_WEBRTC;
    afe_config->vad_init = true;
    afe_config->vad_mode = VAD_MODE_1;
    afe_config->vad_min_speech_ms = APP_VAD_MIN_SPEECH_MS;
    afe_config->vad_min_noise_ms = APP_VAD_MIN_NOISE_MS;
    afe_config->vad_delay_ms = APP_VAD_DELAY_MS;
    speech->afe = esp_afe_handle_from_config(afe_config);
    if (speech->afe != NULL) {
        speech->afe_data = speech->afe->create_from_config(afe_config);
    }
    afe_config_free(afe_config);
    ESP_RETURN_ON_FALSE(speech->afe != NULL && speech->afe_data != NULL,
                        ESP_ERR_NO_MEM, TAG, "cannot create AFE");
    speech->afe->print_pipeline(speech->afe_data);
    ESP_LOGI(TAG, "wake word enabled: %s (%s)", APP_WAKE_WORD, wake_model);
    return ESP_OK;
}

esp_err_t speech_recognizer_start(QueueHandle_t event_queue)
{
    ESP_RETURN_ON_FALSE(event_queue != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "event queue is null");
    ESP_RETURN_ON_FALSE(s_speech.queue == NULL, ESP_ERR_INVALID_STATE, TAG,
                        "speech is already running");

    s_speech.queue = event_queue;
    ESP_RETURN_ON_ERROR(init_models(&s_speech), TAG, "initialize ESP-SR");

    BaseType_t created = xTaskCreatePinnedToCore(
        audio_feed_task, "audio_feed", 8192, &s_speech, 6, NULL, 0);
    ESP_RETURN_ON_FALSE(created == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "create audio feed task");
    created = xTaskCreatePinnedToCore(recognition_task, "speech_recognize",
                                     8192, &s_speech, 5, NULL, 1);
    ESP_RETURN_ON_FALSE(created == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "create recognition task");
    ESP_LOGI(TAG, "ESP-SR started; wake word required");
    return ESP_OK;
}
