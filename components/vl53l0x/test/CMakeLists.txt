idf_component_register(SRCS "test_vl53l0x.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity vl53l0x esp_driver_gpio esp_driver_i2c freertos)

