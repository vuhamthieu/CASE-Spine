#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L0X_DEFAULT_ADDR 0x29
#define VL53L0X_MODEL_ID 0xEE

typedef struct {
    const char *name;
    gpio_num_t xshut_gpio;
    uint8_t address;
    i2c_master_dev_handle_t dev;
    uint8_t stop_variable;
} vl53l0x_t;

void vl53l0x_set_i2c_mutex(SemaphoreHandle_t mutex);
esp_err_t vl53l0x_add_i2c_device(i2c_master_bus_handle_t bus,
                                 uint8_t address,
                                 i2c_master_dev_handle_t *out_dev);
esp_err_t vl53l0x_set_address(i2c_master_dev_handle_t default_dev, uint8_t new_addr);
esp_err_t vl53l0x_init(vl53l0x_t *sensor);
esp_err_t vl53l0x_read_range_mm(vl53l0x_t *sensor, uint16_t *range_mm);

#ifdef __cplusplus
}
#endif
