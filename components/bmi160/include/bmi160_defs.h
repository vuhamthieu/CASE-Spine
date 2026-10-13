#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BMI160 register map
 */
#define BMI160_REG_CHIP_ID 0x00
#define BMI160_REG_ERR 0x02
#define BMI160_REG_PMU_STATUS 0x03
#define BMI160_REG_GYRO_DATA 0x0C
#define BMI160_REG_ACCEL_DATA 0x12
#define BMI160_REG_STATUS 0x1B
#define BMI160_REG_TEMP_DATA 0x1F
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_ACC_RANGE 0x41
#define BMI160_REG_GYR_CONF 0x42
#define BMI160_REG_GYR_RANGE 0x43
#define BMI160_REG_CMD 0x7E
#define BMI160_REG_SPI_IF 0x7F

/*
 * BMI160 command register values
 */
#define BMI160_CMD_SOFT_RESET 0xB6
#define BMI160_CMD_ACCEL_NORMAL 0x11
#define BMI160_CMD_GYRO_NORMAL 0x15

/*
 * I2C addresses
 */
#define BMI160_I2C_ADDR_GND 0x68
#define BMI160_I2C_ADDR_VDD 0x69
#define BMI160_DEFAULT_ADDR BMI160_I2C_ADDR_GND

/*
 * SPI transport helpers
 */
#define BMI160_SPI_READ_FLAG 0x80
#define BMI160_SPI_MAX_TRANSFER_BYTES 32

/*
 * Chip identification
 */
#define BMI160_CHIP_ID 0xD1

/*
 * Measurement ranges
 */
typedef enum {
    BMI160_ACCEL_RANGE_2G = 0x03,
    BMI160_ACCEL_RANGE_4G = 0x05,
    BMI160_ACCEL_RANGE_8G = 0x08,
    BMI160_ACCEL_RANGE_16G = 0x0C,
} bmi160_accel_range_t;

typedef enum {
    BMI160_GYRO_RANGE_2000DPS = 0x00,
    BMI160_GYRO_RANGE_1000DPS = 0x01,
    BMI160_GYRO_RANGE_500DPS = 0x02,
    BMI160_GYRO_RANGE_250DPS = 0x03,
    BMI160_GYRO_RANGE_125DPS = 0x04,
} bmi160_gyro_range_t;

typedef struct {
    float x;
    float y;
    float z;
} bmi160_data_t;

typedef struct {
    const char *name;
    gpio_num_t int_gpio;
    i2c_master_dev_handle_t dev;
    spi_device_handle_t spi_dev;
    bmi160_accel_range_t accel_range;
    bmi160_gyro_range_t gyro_range;
} bmi160_t;

#ifdef __cplusplus
}
#endif
