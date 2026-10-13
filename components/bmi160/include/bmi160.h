#pragma once

#include "bmi160_defs.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

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
 * @brief Add the BMI160 device to the ESP-IDF SPI master bus.
 *
 * @param host SPI host controller to attach the device to.
 * @param cs_gpio Chip-select GPIO pin.
 * @param out_dev Pointer to store the created device handle.
 * @return esp_err_t ESP_OK on success, appropriate error code on failure.
 */
esp_err_t bmi160_add_spi_device(spi_host_device_t host, gpio_num_t cs_gpio,
                                spi_device_handle_t *out_dev);

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
