idf_component_register(SRCS "ina219_driver.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c esp_driver_gpio freertos)
