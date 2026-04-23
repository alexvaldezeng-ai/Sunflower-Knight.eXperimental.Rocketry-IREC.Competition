idf_component_register(SRCS "payload.c"
                    INCLUDE_DIRS "."
    REQUIRES 
        log
        fatfs
        esp_adc
        esp_driver_i2c
        esp_driver_mcpwm
        esp_driver_uart
        esp_driver_ledc)
