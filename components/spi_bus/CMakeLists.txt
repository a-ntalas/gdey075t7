idf_component_register(
    SRCS "drv_spi.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi esp_driver_gpio
)