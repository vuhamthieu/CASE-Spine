#include "bmi160.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/task.h"

#define I2C_FREQ_HZ 400000
#define I2C_TIMEOUT_MS 100

static const char *TAG = "bmi160";
static SemaphoreHandle_t s_i2c_mutex = NULL;

void bmi160_set_i2c_mutex(SemaphoreHandle_t mutex)
{
    s_i2c_mutex = mutex;
}

static esp_err_t lock_bus(void)
{
    if (s_i2c_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE ? ESP_OK
                                                                                : ESP_ERR_TIMEOUT;
}

static void unlock_bus(void)
{
    if (s_i2c_mutex != NULL) {
        xSemaphoreGive(s_i2c_mutex);
    }
}

static bool bmi160_uses_spi(const bmi160_t *sensor)
{
    return sensor != NULL && sensor->spi_dev != NULL;
}

static esp_err_t write_bytes(bmi160_t *sensor, uint8_t reg, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid sensor");
    ESP_RETURN_ON_ERROR(lock_bus(), TAG, "bus mutex timeout");

    esp_err_t ret = ESP_FAIL;
    if (bmi160_uses_spi(sensor)) {
        if (len > BMI160_SPI_MAX_TRANSFER_BYTES) {
            unlock_bus();
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t tx[BMI160_SPI_MAX_TRANSFER_BYTES + 1] = {0};
        spi_transaction_t t = {0};

        tx[0] = reg & (uint8_t)~BMI160_SPI_READ_FLAG;
        if (len > 0) {
            memcpy(&tx[1], data, len);
        }

        t.length = (len + 1) * 8;
        t.tx_buffer = tx;
        ret = spi_device_transmit(sensor->spi_dev, &t);
    } else {
        if (sensor->dev == NULL) {
            unlock_bus();
            return ESP_ERR_INVALID_ARG;
        }
        if (len > BMI160_SPI_MAX_TRANSFER_BYTES) {
            unlock_bus();
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t tx[BMI160_SPI_MAX_TRANSFER_BYTES + 1] = {0};
        tx[0] = reg;
        if (len > 0) {
            memcpy(&tx[1], data, len);
        }

        ret = i2c_master_transmit(sensor->dev, tx, len + 1, I2C_TIMEOUT_MS);
    }

    unlock_bus();
    return ret;
}

static esp_err_t read_bytes(bmi160_t *sensor, uint8_t reg, uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid sensor");
    ESP_RETURN_ON_ERROR(lock_bus(), TAG, "bus mutex timeout");

    esp_err_t ret = ESP_FAIL;
    if (bmi160_uses_spi(sensor)) {
        if (len > BMI160_SPI_MAX_TRANSFER_BYTES) {
            unlock_bus();
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t tx[BMI160_SPI_MAX_TRANSFER_BYTES + 1] = {0};
        uint8_t rx[BMI160_SPI_MAX_TRANSFER_BYTES + 1] = {0};
        spi_transaction_t t = {0};

        tx[0] = reg | BMI160_SPI_READ_FLAG;
        t.length = (len + 1) * 8;
        t.tx_buffer = tx;
        t.rx_buffer = rx;
        ret = spi_device_transmit(sensor->spi_dev, &t);
        if (ret == ESP_OK && len > 0) {
            memcpy(data, &rx[1], len);
        }
    } else {
        if (sensor->dev == NULL) {
            unlock_bus();
            return ESP_ERR_INVALID_ARG;
        }
        ret = i2c_master_transmit_receive(sensor->dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
    }

    unlock_bus();
    return ret;
}

static esp_err_t write_u8(bmi160_t *sensor, uint8_t reg, uint8_t value)
{
    return write_bytes(sensor, reg, &value, 1);
}

static esp_err_t read_u8(bmi160_t *sensor, uint8_t reg, uint8_t *value)
{
    return read_bytes(sensor, reg, value, 1);
}

esp_err_t bmi160_add_i2c_device(i2c_master_bus_handle_t bus,
                                uint8_t address,
                                i2c_master_dev_handle_t *out_dev)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    return i2c_master_bus_add_device(bus, &dev_cfg, out_dev);
}

esp_err_t bmi160_add_spi_device(spi_host_device_t host,
                                gpio_num_t cs_gpio,
                                spi_device_handle_t *out_dev)
{
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 5 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = cs_gpio,
        .queue_size = 1,
    };

    return spi_bus_add_device(host, &dev_cfg, out_dev);
}

esp_err_t bmi160_init(bmi160_t *sensor)
{
    uint8_t chip_id = 0;

    ESP_RETURN_ON_FALSE(sensor != NULL && (sensor->dev != NULL || sensor->spi_dev != NULL), ESP_ERR_INVALID_ARG, TAG, "invalid sensor configuration");

    ESP_LOGI(TAG, "Initializing %s BMI160 sensor...", sensor->name ? sensor->name : "unnamed");

    // 1. Perform a Soft Reset to get the chip into a clean default state
    ESP_RETURN_ON_ERROR(write_u8(sensor, BMI160_REG_CMD, BMI160_CMD_SOFT_RESET),
                        TAG, "%s soft reset failed", sensor->name);
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for reset to complete

    // 2. Enable SPI interface mode if using SPI.
    // According to the BMI160 datasheet, a dummy read of register 0x7F must be
    // performed before the chip ID/state registers are valid on SPI.
    if (bmi160_uses_spi(sensor)) {
        uint8_t dummy = 0;
        ESP_RETURN_ON_ERROR(read_bytes(sensor, BMI160_REG_SPI_IF, &dummy, 1),
                            TAG, "%s SPI mode enable read failed", sensor->name);
    }

    // 3. Read and verify Chip ID
    ESP_RETURN_ON_ERROR(read_u8(sensor, BMI160_REG_CHIP_ID, &chip_id),
                        TAG, "%s chip ID read failed", sensor->name);
    if (chip_id != BMI160_CHIP_ID) {
        ESP_LOGE(TAG, "%s invalid chip ID: 0x%02x (expected: 0x%02x)", sensor->name, chip_id, BMI160_CHIP_ID);
        return ESP_ERR_INVALID_VERSION;
    }
    ESP_LOGI(TAG, "%s detected BMI160 successfully (Chip ID: 0x%02x)", sensor->name, chip_id);

    // 4. Bring Accelerometer and Gyroscope from Suspend to Normal Power Mode
    ESP_RETURN_ON_ERROR(write_u8(sensor, BMI160_REG_CMD, BMI160_CMD_ACCEL_NORMAL),
                        TAG, "%s accel normal mode cmd failed", sensor->name);
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay for accel startup

    ESP_RETURN_ON_ERROR(write_u8(sensor, BMI160_REG_CMD, BMI160_CMD_GYRO_NORMAL),
                        TAG, "%s gyro normal mode cmd failed", sensor->name);
    vTaskDelay(pdMS_TO_TICKS(100)); // Gyro normal mode transition takes up to 80ms. Wait 100ms to be safe.

    // 5. Configure ranges: use configured settings or default if not specified
    if (sensor->accel_range == 0) {
        sensor->accel_range = BMI160_ACCEL_RANGE_2G;
    }
    if (sensor->gyro_range == 0) {
        sensor->gyro_range = BMI160_GYRO_RANGE_2000DPS;
    }

    ESP_RETURN_ON_ERROR(bmi160_set_accel_range(sensor, sensor->accel_range), TAG, "%s set accel range failed", sensor->name);
    ESP_RETURN_ON_ERROR(bmi160_set_gyro_range(sensor, sensor->gyro_range), TAG, "%s set gyro range failed", sensor->name);

    // 6. Configure Accelerometer and Gyroscope output data rate (ODR) to 100Hz
    // Register 0x40 (ACC_CONF): Bits 7:4 for bandwidth, 3:0 for ODR (0x08 = 100Hz, normal bandwidth = 0x20) => 0x28
    // Register 0x42 (GYR_CONF): Bits 7:4 for bandwidth, 3:0 for ODR (0x08 = 100Hz, normal bandwidth = 0x20) => 0x28
    ESP_RETURN_ON_ERROR(write_u8(sensor, BMI160_REG_ACC_CONF, 0x28), TAG, "%s accel ODR config failed", sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor, BMI160_REG_GYR_CONF, 0x28), TAG, "%s gyro ODR config failed", sensor->name);

    ESP_LOGI(TAG, "%s initialized successfully in normal mode", sensor->name);
    return ESP_OK;
}

esp_err_t bmi160_set_accel_range(bmi160_t *sensor, bmi160_accel_range_t range)
{
    ESP_RETURN_ON_FALSE(sensor != NULL && (sensor->dev != NULL || sensor->spi_dev != NULL), ESP_ERR_INVALID_ARG, TAG, "invalid sensor");
    ESP_RETURN_ON_ERROR(write_u8(sensor, BMI160_REG_ACC_RANGE, (uint8_t)range),
                        TAG, "%s write accel range failed", sensor->name);
    sensor->accel_range = range;
    return ESP_OK;
}

esp_err_t bmi160_set_gyro_range(bmi160_t *sensor, bmi160_gyro_range_t range)
{
    ESP_RETURN_ON_FALSE(sensor != NULL && (sensor->dev != NULL || sensor->spi_dev != NULL), ESP_ERR_INVALID_ARG, TAG, "invalid sensor");
    ESP_RETURN_ON_ERROR(write_u8(sensor, BMI160_REG_GYR_RANGE, (uint8_t)range),
                        TAG, "%s write gyro range failed", sensor->name);
    sensor->gyro_range = range;
    return ESP_OK;
}

esp_err_t bmi160_read_raw(bmi160_t *sensor, int16_t accel_raw[3], int16_t gyro_raw[3])
{
    uint8_t buf[12] = {0};
    ESP_RETURN_ON_FALSE(sensor != NULL && (sensor->dev != NULL || sensor->spi_dev != NULL), ESP_ERR_INVALID_ARG, TAG, "invalid sensor");

    // Read 12 sequential bytes starting from BMI160_REG_GYRO_DATA (0x0C) to read Gyro and Accel data simultaneously
    ESP_RETURN_ON_ERROR(read_bytes(sensor, BMI160_REG_GYRO_DATA, buf, sizeof(buf)),
                        TAG, "%s read raw data failed", sensor->name);

    // Parse gyroscope X, Y, Z
    gyro_raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
    gyro_raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
    gyro_raw[2] = (int16_t)((buf[5] << 8) | buf[4]);

    // Parse accelerometer X, Y, Z
    accel_raw[0] = (int16_t)((buf[7] << 8) | buf[6]);
    accel_raw[1] = (int16_t)((buf[9] << 8) | buf[8]);
    accel_raw[2] = (int16_t)((buf[11] << 8) | buf[10]);

    return ESP_OK;
}

esp_err_t bmi160_read_data(bmi160_t *sensor, bmi160_data_t *accel, bmi160_data_t *gyro)
{
    int16_t accel_raw[3] = {0};
    int16_t gyro_raw[3] = {0};

    ESP_RETURN_ON_ERROR(bmi160_read_raw(sensor, accel_raw, gyro_raw), TAG, "%s raw read failed", sensor->name);

    // Convert raw Accelerometer values to float (unit: g)
    // Resolution is 16-bit signed integer (-32768 to 32767)
    float accel_scale = 1.0f;
    switch (sensor->accel_range) {
        case BMI160_ACCEL_RANGE_2G:  accel_scale = 2.0f / 32768.0f; break;
        case BMI160_ACCEL_RANGE_4G:  accel_scale = 4.0f / 32768.0f; break;
        case BMI160_ACCEL_RANGE_8G:  accel_scale = 8.0f / 32768.0f; break;
        case BMI160_ACCEL_RANGE_16G: accel_scale = 16.0f / 32768.0f; break;
        default: accel_scale = 2.0f / 32768.0f; break;
    }

    accel->x = (float)accel_raw[0] * accel_scale;
    accel->y = (float)accel_raw[1] * accel_scale;
    accel->z = (float)accel_raw[2] * accel_scale;

    // Convert raw Gyroscope values to float (unit: degrees per second - dps)
    float gyro_scale = 1.0f;
    switch (sensor->gyro_range) {
        case BMI160_GYRO_RANGE_2000DPS: gyro_scale = 2000.0f / 32768.0f; break;
        case BMI160_GYRO_RANGE_1000DPS: gyro_scale = 1000.0f / 32768.0f; break;
        case BMI160_GYRO_RANGE_500DPS:  gyro_scale = 500.0f / 32768.0f; break;
        case BMI160_GYRO_RANGE_250DPS:  gyro_scale = 250.0f / 32768.0f; break;
        case BMI160_GYRO_RANGE_125DPS:  gyro_scale = 125.0f / 32768.0f; break;
        default: gyro_scale = 2000.0f / 32768.0f; break;
    }

    gyro->x = (float)gyro_raw[0] * gyro_scale;
    gyro->y = (float)gyro_raw[1] * gyro_scale;
    gyro->z = (float)gyro_raw[2] * gyro_scale;

    return ESP_OK;
}

esp_err_t bmi160_read_temp(bmi160_t *sensor, float *temp_c)
{
    uint8_t buf[2] = {0};
    ESP_RETURN_ON_FALSE(sensor != NULL && (sensor->dev != NULL || sensor->spi_dev != NULL) && temp_c != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid arguments");

    ESP_RETURN_ON_ERROR(read_bytes(sensor, BMI160_REG_TEMP_DATA, buf, sizeof(buf)),
                        TAG, "%s read temperature register failed", sensor->name);

    int16_t temp_raw = (int16_t)((buf[1] << 8) | buf[0]);

    // Value 0x8000 (-32768) indicates invalid / measurement error
    if (temp_raw == -32768) {
        ESP_LOGW(TAG, "%s temperature measurement invalid", sensor->name);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Formula: Temp_Celsius = (Raw_Value / 512.0) + 23.0
    *temp_c = ((float)temp_raw / 512.0f) + 23.0f;
    return ESP_OK;
}
