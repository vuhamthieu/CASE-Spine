idf_component_register(SRCS "../tests/bmi160_spi_test.c" 
                       INCLUDE_DIRS "."
                       REQUIRES vl53l0x bmi160 ina219 pca9685 kinematics uart_bridge esp_driver_gpio esp_driver_i2c esp_driver_spi freertos)
