#include "drv_epd.h"

#include <stdbool.h>
#include <string.h>

#include "board.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define EPD_SPI_HOST SPI2_HOST
#define EPD_SPI_CLOCK_HZ (10 * 1000 * 1000)
#define EPD_TRANSFER_CHUNK 4096
#define EPD_BUSY_TIMEOUT_MS 60000

static const char *TAG = "epd";
static spi_device_handle_t s_spi;
static uint8_t *s_tx_buffer;

static esp_err_t epd_write(bool data, const void *buffer, size_t length)
{
    gpio_set_level(BOARD_EPD_DC, data);
    spi_transaction_t transaction = {
        .length = length * 8,
    };
    if (length <= sizeof(transaction.tx_data)) {
        transaction.flags = SPI_TRANS_USE_TXDATA;
        memcpy(transaction.tx_data, buffer, length);
    } else {
        if (length > EPD_TRANSFER_CHUNK) {
            return ESP_ERR_INVALID_SIZE;
        }
        if (buffer != s_tx_buffer) {
            memcpy(s_tx_buffer, buffer, length);
        }
        transaction.tx_buffer = s_tx_buffer;
    }
    return spi_device_polling_transmit(s_spi, &transaction);
}

static esp_err_t epd_command(uint8_t command)
{
    return epd_write(false, &command, 1);
}

static esp_err_t epd_data(const void *data, size_t length)
{
    return epd_write(true, data, length);
}

static esp_err_t epd_command_data(uint8_t command, const uint8_t *data,
                                  size_t length)
{
    ESP_RETURN_ON_ERROR(epd_command(command), TAG, "command 0x%02x", command);
    return epd_data(data, length);
}

static esp_err_t epd_wait_ready(void)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(EPD_BUSY_TIMEOUT_MS);
    while (gpio_get_level(BOARD_EPD_BUSY) == 0) {
        if (xTaskGetTickCount() - start >= timeout) {
            ESP_LOGE(TAG, "BUSY timeout after %d ms", EPD_BUSY_TIMEOUT_MS);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_OK;
}

esp_err_t drv_epd_init(void)
{
    if (s_spi == NULL) {
        gpio_config_t output_config = {
            .pin_bit_mask = (1ULL << BOARD_EPD_RST) | (1ULL << BOARD_EPD_DC),
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&output_config), TAG,
                            "configure outputs");

        gpio_config_t busy_config = {
            .pin_bit_mask = 1ULL << BOARD_EPD_BUSY,
            .mode = GPIO_MODE_INPUT,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&busy_config), TAG, "configure BUSY");

        spi_bus_config_t bus_config = {
            .mosi_io_num = BOARD_EPD_MOSI,
            .miso_io_num = GPIO_NUM_NC,
            .sclk_io_num = BOARD_EPD_SCLK,
            .quadwp_io_num = GPIO_NUM_NC,
            .quadhd_io_num = GPIO_NUM_NC,
            .max_transfer_sz = EPD_TRANSFER_CHUNK,
        };
        ESP_RETURN_ON_ERROR(spi_bus_initialize(EPD_SPI_HOST, &bus_config,
                                               SPI_DMA_CH_AUTO),
                            TAG, "initialize SPI bus");

        spi_device_interface_config_t device_config = {
            .clock_speed_hz = EPD_SPI_CLOCK_HZ,
            .mode = 0,
            .spics_io_num = BOARD_EPD_CS,
            .queue_size = 1,
        };
        esp_err_t error =
            spi_bus_add_device(EPD_SPI_HOST, &device_config, &s_spi);
        if (error != ESP_OK) {
            spi_bus_free(EPD_SPI_HOST);
            return error;
        }

        s_tx_buffer = heap_caps_malloc(EPD_TRANSFER_CHUNK, MALLOC_CAP_DMA);
        if (s_tx_buffer == NULL) {
            spi_bus_remove_device(s_spi);
            spi_bus_free(EPD_SPI_HOST);
            s_spi = NULL;
            return ESP_ERR_NO_MEM;
        }
    }

    gpio_set_level(BOARD_EPD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(BOARD_EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(epd_wait_ready(), TAG, "reset wait");

    static const uint8_t d4d[] = {0x78};
    static const uint8_t d00[] = {0x2f, 0x29};
    static const uint8_t d01[] = {0x07, 0x00};
    static const uint8_t d06[] = {0x0d, 0x0b, 0x33, 0x0a, 0x20, 0x12, 0x16};
    static const uint8_t d30[] = {0x08};
    static const uint8_t d41[] = {0x00};
    static const uint8_t d50[] = {0x37};
    static const uint8_t d61[] = {0x03, 0x20, 0x01, 0xe0};
    static const uint8_t d65[] = {0x00, 0x00, 0x00, 0x00};
    static const uint8_t df0[] = {0x7d};
    static const uint8_t de3[] = {0x08, 0x00};
    static const uint8_t de0[] = {0x00};
    static const uint8_t dba[] = {0x5a};
    static const uint8_t de9[] = {0x01};

#define WRITE_REGISTER(command, value)                                       \
    ESP_RETURN_ON_ERROR(epd_command_data(command, value, sizeof(value)), TAG, \
                        "initialize register 0x%02x", command)
    WRITE_REGISTER(0x4d, d4d);
    WRITE_REGISTER(0x00, d00);
    WRITE_REGISTER(0x01, d01);
    WRITE_REGISTER(0x06, d06);
    WRITE_REGISTER(0x30, d30);
    WRITE_REGISTER(0x41, d41);
    WRITE_REGISTER(0x50, d50);
    WRITE_REGISTER(0x61, d61);
    WRITE_REGISTER(0x65, d65);
    WRITE_REGISTER(0xf0, df0);
    WRITE_REGISTER(0xe3, de3);
    WRITE_REGISTER(0xe0, de0);
    WRITE_REGISTER(0xba, dba);
    WRITE_REGISTER(0xe9, de9);
#undef WRITE_REGISTER

    ESP_LOGI(TAG, "initialized 800x480 panel");
    return ESP_OK;
}

esp_err_t drv_epd_display(const uint8_t *frame, size_t length)
{
    ESP_RETURN_ON_FALSE(s_spi != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "EPD is not initialized");
    ESP_RETURN_ON_FALSE(frame != NULL && length == DRV_EPD_FRAME_BYTES,
                        ESP_ERR_INVALID_ARG, TAG, "invalid frame length");

    static const uint8_t panel_color[] = {1, 2, 3, 0};
    ESP_RETURN_ON_ERROR(epd_command(0x10), TAG, "start frame transfer");

    for (size_t offset = 0; offset < length;) {
        size_t chunk = length - offset;
        if (chunk > EPD_TRANSFER_CHUNK) {
            chunk = EPD_TRANSFER_CHUNK;
        }
        for (size_t i = 0; i < chunk; ++i) {
            const uint8_t source = frame[offset + i];
            s_tx_buffer[i] = (panel_color[(source >> 6) & 3] << 6) |
                             (panel_color[(source >> 4) & 3] << 4) |
                             (panel_color[(source >> 2) & 3] << 2) |
                             panel_color[source & 3];
        }
        ESP_RETURN_ON_ERROR(epd_data(s_tx_buffer, chunk), TAG,
                            "transfer frame at byte %u", (unsigned)offset);
        offset += chunk;
    }

    static const uint8_t refresh_data = 0x00;
    ESP_RETURN_ON_ERROR(epd_command(0x04), TAG, "power on");
    ESP_RETURN_ON_ERROR(epd_wait_ready(), TAG, "power-on wait");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(epd_command_data(0x12, &refresh_data, 1), TAG,
                        "refresh");
    ESP_RETURN_ON_ERROR(epd_wait_ready(), TAG, "refresh wait");
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

esp_err_t drv_epd_sleep(void)
{
    static const uint8_t zero = 0x00;
    static const uint8_t key = 0xa5;
    ESP_RETURN_ON_ERROR(epd_command_data(0x02, &zero, 1), TAG, "power off");
    ESP_RETURN_ON_ERROR(epd_wait_ready(), TAG, "power-off wait");
    vTaskDelay(pdMS_TO_TICKS(50));
    return epd_command_data(0x07, &key, 1);
}
