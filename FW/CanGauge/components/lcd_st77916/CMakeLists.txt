idf_component_register(SRCS "ST77916.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_lcd esp_timer driver esp_driver_ledc esp_driver_spi
                                esp_driver_gpio freertos heap log)