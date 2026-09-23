idf_component_register(
    SRCS
        "main.c"
        "app/app_controller.c"
        "drivers/drv_camera.c"
        "drivers/drv_epd.c"
        "middleware/image_processor.c"
    INCLUDE_DIRS
        "."
        "app"
        "common"
        "drivers"
        "middleware"
    REQUIRES driver espressif__esp32-camera
)
