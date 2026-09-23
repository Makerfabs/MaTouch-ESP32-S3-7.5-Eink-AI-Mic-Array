#include "drv_sdcard.h"

#include <stdio.h>

#include "app_config.h"
#include "board.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static const char *TAG = "drv_sdcard";
static sdmmc_card_t *s_card;

esp_err_t drv_sdcard_mount(void)
{
    if (s_card != NULL) {
        return ESP_OK;
    }

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.clk = BOARD_SD_CLK;
    slot.cmd = BOARD_SD_CMD;
    slot.d0 = BOARD_SD_D0;
    slot.d1 = GPIO_NUM_NC;
    slot.d2 = GPIO_NUM_NC;
    slot.d3 = GPIO_NUM_NC;
    slot.d4 = GPIO_NUM_NC;
    slot.d5 = GPIO_NUM_NC;
    slot.d6 = GPIO_NUM_NC;
    slot.d7 = GPIO_NUM_NC;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };
    esp_err_t error = esp_vfs_fat_sdmmc_mount(
        APP_STORAGE_MOUNT_POINT, &host, &slot, &mount_config, &s_card);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "mount failed: %s", esp_err_to_name(error));
        return error;
    }
    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

void drv_sdcard_unmount(void)
{
    if (s_card != NULL) {
        esp_vfs_fat_sdcard_unmount(APP_STORAGE_MOUNT_POINT, s_card);
        s_card = NULL;
    }
}
