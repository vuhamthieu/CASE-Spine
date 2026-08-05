idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES vl53l0x bmi160 ina219 pca9685 kinematics uart_bridge esp_driver_gpio esp_driver_i2c freertos)
