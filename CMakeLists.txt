idf_component_register(
    SRCS "esp_mp65.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi esp_driver_gpio
    PRIV_REQUIRES esp_timer
)
