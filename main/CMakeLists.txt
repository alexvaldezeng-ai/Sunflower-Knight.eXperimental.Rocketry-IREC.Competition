idf_component_register(SRCS "payload.c"
                    INCLUDE_DIRS "."
    REQUIRES
        driver
        log
        fatfs
        esp_adc
        esp_driver_gpio
        esp_driver_spi
        esp_driver_mcpwm
        esp_driver_uart
        esp_driver_ledc
        esp_timer)
