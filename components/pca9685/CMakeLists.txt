idf_component_register(SRCS "pca9685_driver.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c freertos)
