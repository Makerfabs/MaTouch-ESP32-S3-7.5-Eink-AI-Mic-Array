#include "drv_audio_codec.h"

#include <stdbool.h>

#include "app_config.h"
#include "board.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "es7210.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define AUDIO_I2C_PORT I2C_NUM_0
#define AUDIO_I2S_PORT I2S_NUM_0
#define AUDIO_MCLK_MULTIPLE I2S_MCLK_MULTIPLE_256

static const char *TAG = "audio_codec";
static i2s_chan_handle_t s_rx_channel;
static es7210_dev_handle_t s_codec;

static esp_err_t init_i2c(void)
{
    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BOARD_I2C_SDA,
        .scl_io_num = BOARD_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(AUDIO_I2C_PORT, &config), TAG,
                        "configure I2C");
    return i2c_driver_install(AUDIO_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t init_i2s(void)
{
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(AUDIO_I2S_PORT, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = 8;
    channel_config.dma_frame_num = 256;
    ESP_RETURN_ON_ERROR(
        i2s_new_channel(&channel_config, NULL, &s_rx_channel), TAG,
        "create I2S RX channel");

    const i2s_std_config_t std_config = {
        .clk_cfg = {
            .sample_rate_hz = APP_AUDIO_SAMPLE_RATE_HZ,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = AUDIO_MCLK_MULTIPLE,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BOARD_I2S_MCLK,
            .bclk = BOARD_I2S_BCLK,
            .ws = BOARD_I2S_LRCK,
            .dout = I2S_GPIO_UNUSED,
            .din = BOARD_I2S_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    return i2s_channel_init_std_mode(s_rx_channel, &std_config);
}

static esp_err_t init_es7210(void)
{
    const es7210_i2c_config_t i2c_config = {
        .i2c_port = AUDIO_I2C_PORT,
        .i2c_addr = ES7210_ADDRRES_00,
    };
    ESP_RETURN_ON_ERROR(es7210_new_codec(&i2c_config, &s_codec), TAG,
                        "create ES7210 handle");

    const es7210_codec_config_t codec_config = {
        .sample_rate_hz = APP_AUDIO_SAMPLE_RATE_HZ,
        .mclk_ratio = AUDIO_MCLK_MULTIPLE,
        .i2s_format = ES7210_I2S_FMT_I2S,
        .bit_width = ES7210_I2S_BITS_16B,
        .mic_bias = ES7210_MIC_BIAS_2V87,
        .mic_gain = ES7210_MIC_GAIN_33DB,
        .flags.tdm_enable = false,
    };
    ESP_RETURN_ON_ERROR(es7210_config_codec(s_codec, &codec_config), TAG,
                        "configure ES7210");
    return es7210_config_volume(s_codec, 0);
}

esp_err_t drv_audio_codec_init(void)
{
    if (s_rx_channel != NULL) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(init_i2s(), TAG, "initialize I2S");
    ESP_RETURN_ON_ERROR(init_i2c(), TAG, "initialize I2C");
    ESP_RETURN_ON_ERROR(init_es7210(), TAG, "initialize codec");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_channel), TAG,
                        "enable I2S RX");
    ESP_LOGI(TAG, "ES7210 ready: 16 kHz, 16-bit, stereo I2S MIC1/2");
    return ESP_OK;
}

esp_err_t drv_audio_codec_read(int16_t *samples, size_t sample_count,
                               size_t *samples_read, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_rx_channel != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "audio codec is not initialized");
    ESP_RETURN_ON_FALSE(samples != NULL && samples_read != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid read buffer");

    const size_t requested_bytes = sample_count * sizeof(*samples);
    size_t total_bytes = 0;
    while (total_bytes < requested_bytes) {
        size_t bytes_read = 0;
        esp_err_t error = i2s_channel_read(
            s_rx_channel, (uint8_t *)samples + total_bytes,
            requested_bytes - total_bytes, &bytes_read,
            timeout_ms);
        if (error != ESP_OK) {
            *samples_read = total_bytes / sizeof(*samples);
            return error;
        }
        if (bytes_read == 0) {
            *samples_read = total_bytes / sizeof(*samples);
            return ESP_ERR_TIMEOUT;
        }
        total_bytes += bytes_read;
    }

    *samples_read = total_bytes / sizeof(*samples);
    return ESP_OK;
}
