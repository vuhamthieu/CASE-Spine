idf_component_register(SRCS "vl53l0x_triple_test.c"
                       INCLUDE_DIRS "."
                       REQUIRES vl53l0x_driver bmi160_driver esp_driver_gpio esp_driver_i2c freertos)

