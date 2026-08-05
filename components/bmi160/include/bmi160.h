#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BMI160 Register Addresses
 */
#define BMI160_REG_CHIP_ID 0x00
#define BMI160_REG_ERR 0x02
#define BMI160_REG_PMU_STATUS 0x03
#define BMI160_REG_GYRO_DATA                                                   \
  0x0C // GYR_X_LSB address (6 bytes: X, Y, Z LSB/MSB)
#define BMI160_REG_ACCEL_DATA                                                  \
  0x12 // ACC_X_LSB address (6 bytes: X, Y, Z LSB/MSB)
#define BMI160_REG_STATUS 0x1B
#define BMI160_REG_TEMP_DATA 0x1F // TEMP_LSB address (2 bytes)
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_ACC_RANGE 0x41
#define BMI160_REG_GYR_CONF 0x42
#define BMI160_REG_GYR_RANGE 0x43

#define BMI160_REG_CMD 0x7E

/*
 * BMI160 Command Register (0x7E) Values
 */
#define BMI160_CMD_SOFT_RESET 0xB6
#define BMI160_CMD_ACCEL_NORMAL 0x11
#define BMI160_CMD_GYRO_NORMAL 0x15

/*
 * I2C Addresses
 */
#define BMI160_I2C_ADDR_GND 0x68 // SDO pin tied to GND
#define BMI160_I2C_ADDR_VDD 0x69 // SDO pin tied to VDDIO
#define BMI160_DEFAULT_ADDR BMI160_I2C_ADDR_GND

/*
 * Chip identification
 */
#define BMI160_CHIP_ID                                                         \
  0xD1 // The CHIP_ID register (0x00) should always return 0xD1

/**
 * @brief Accelerometer measurement range settings
 */
typedef enum {
  BMI160_ACCEL_RANGE_2G = 0x03, // +/- 2g
  BMI160_ACCEL_RANGE_4G = 0x05, // +/- 4g
  BMI160_ACCEL_RANGE_8G = 0x08, // +/- 8g
  BMI160_ACCEL_RANGE_16G = 0x0C // +/- 16g
} bmi160_accel_range_t;

/**
 * @brief Gyroscope measurement range (Degrees Per Second) settings
 */
typedef enum {
  BMI160_GYRO_RANGE_2000DPS = 0x00, // +/- 2000 dps
  BMI160_GYRO_RANGE_1000DPS = 0x01, // +/- 1000 dps
  BMI160_GYRO_RANGE_500DPS = 0x02,  // +/- 500 dps
  BMI160_GYRO_RANGE_250DPS = 0x03,  // +/- 250 dps
  BMI160_GYRO_RANGE_125DPS = 0x04   // +/- 125 dps
} bmi160_gyro_range_t;

/**
 * @brief Structure representing a 3-axis vector in physical units
 */
typedef struct {
  float x;
  float y;
  float z;
} bmi160_data_t;

/**
 * @brief BMI160 Device configuration and state structure
 */
typedef struct {
  const char *name;            // Human-readable device name (for logging)
  gpio_num_t int_gpio;         // Interrupt GPIO pin (use GPIO_NUM_NC if unused)
  i2c_master_dev_handle_t dev; // ESP-IDF I2C device handle
  bmi160_accel_range_t
      accel_range;                // Currently active accelerometer range config
  bmi160_gyro_range_t gyro_range; // Currently active gyroscope range config
} bmi160_t;

/**
 * @brief Set the I2C mutex for bus synchronization.
 *
 * @param mutex FreeRTOS Semaphore handle to prevent concurrent bus access.
 */
void bmi160_set_i2c_mutex(SemaphoreHandle_t mutex);

/**
 * @brief Add the BMI160 device to the ESP-IDF I2C master bus.
 *
 * @param bus Handle of the I2C master bus.
 * @param address 7-bit I2C address of the BMI160 (typically 0x68 or 0x69).
 * @param out_dev Pointer to store the created device handle.
 * @return esp_err_t ESP_OK on success, appropriate error code on failure.
 */
esp_err_t bmi160_add_i2c_device(i2c_master_bus_handle_t bus, uint8_t address,
                                i2c_master_dev_handle_t *out_dev);

/**
 * @brief Initialize the BMI160 sensor (soft reset, configure power modes to
 * Normal, and load defaults).
 *
 * @param sensor Pointer to the bmi160_t struct containing driver settings.
 * @return esp_err_t ESP_OK on success, or error code on failure.
 */
esp_err_t bmi160_init(bmi160_t *sensor);

/**
 * @brief Configure the accelerometer measurement range.
 *
 * @param sensor Pointer to the bmi160_t struct.
 * @param range The desired range (2g, 4g, 8g, 16g).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmi160_set_accel_range(bmi160_t *sensor, bmi160_accel_range_t range);

/**
 * @brief Configure the gyroscope measurement range.
 *
 * @param sensor Pointer to the bmi160_t struct.
 * @param range The desired range (125dps, 250dps, 500dps, 1000dps, 2000dps).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmi160_set_gyro_range(bmi160_t *sensor, bmi160_gyro_range_t range);

/**
 * @brief Read the raw integer ADC values from the BMI160.
 *
 * @param sensor Pointer to the bmi160_t struct.
 * @param accel_raw Array of 3 elements to store the raw [x, y, z] accel
 * measurements.
 * @param gyro_raw Array of 3 elements to store the raw [x, y, z] gyro
 * measurements.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmi160_read_raw(bmi160_t *sensor, int16_t accel_raw[3],
                          int16_t gyro_raw[3]);

/**
 * @brief Read and scale the accelerometer and gyroscope measurements to
 * physical units.
 *
 * @param sensor Pointer to the bmi160_t struct.
 * @param accel Pointer to structure to store the acceleration in g (Earth's
 * gravity).
 * @param gyro Pointer to structure to store the angular velocity in degrees per
 * second (dps).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmi160_read_data(bmi160_t *sensor, bmi160_data_t *accel,
                           bmi160_data_t *gyro);

/**
 * @brief Read the internal temperature of the BMI160.
 *
 * @param sensor Pointer to the bmi160_t struct.
 * @param temp_c Pointer to store the temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmi160_read_temp(bmi160_t *sensor, float *temp_c);

#ifdef __cplusplus
}
#endif
