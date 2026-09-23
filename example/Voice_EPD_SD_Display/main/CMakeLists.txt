idf_component_register(
    SRCS
        "main.c"
        "app/app_controller.c"
        "middleware/image_service.c"
        "middleware/bmp_decoder.c"
        "middleware/speech_recognizer.c"
        "drivers/drv_audio_codec.c"
        "drivers/drv_epd.c"
        "drivers/drv_sdcard.c"
    INCLUDE_DIRS
        "."
        "app"
        "middleware"
        "common"
        "drivers"
    REQUIRES driver esp_timer fatfs sdmmc espressif__es7210 espressif__esp-sr
)
