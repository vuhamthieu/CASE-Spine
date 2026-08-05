#include "vl53l0x.h"

#include <stdbool.h>
#include <stddef.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/task.h"

#define I2C_FREQ_HZ 400000
#define I2C_TIMEOUT_MS 100

#define REG_SYSRANGE_START 0x00
#define REG_SYSTEM_SEQUENCE_CONFIG 0x01
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR 0x0B
#define REG_RESULT_INTERRUPT_STATUS 0x13
#define REG_RESULT_RANGE_STATUS 0x14
#define REG_MSRC_CONFIG_CONTROL 0x60
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0xB0
#define REG_GPIO_HV_MUX_ACTIVE_HIGH 0x84
#define REG_I2C_SLAVE_DEVICE_ADDRESS 0x8A
#define REG_IDENTIFICATION_MODEL_ID 0xC0
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44

static const char *TAG = "vl53l0x";
static SemaphoreHandle_t s_i2c_mutex;

void vl53l0x_set_i2c_mutex(SemaphoreHandle_t mutex)
{
    s_i2c_mutex = mutex;
}

esp_err_t vl53l0x_add_i2c_device(i2c_master_bus_handle_t bus,
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

static esp_err_t lock_i2c(void)
{
    if (s_i2c_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

static esp_err_t write_bytes(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_ERROR(lock_i2c(), TAG, "I2C mutex timeout");
    esp_err_t ret = i2c_master_transmit(dev, data, len, I2C_TIMEOUT_MS);
    xSemaphoreGive(s_i2c_mutex);
    return ret;
}

static esp_err_t write_u8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return write_bytes(dev, data, sizeof(data));
}

static esp_err_t write_u16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t value)
{
    const uint8_t data[] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return write_bytes(dev, data, sizeof(data));
}

static esp_err_t read_bytes(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    ESP_RETURN_ON_ERROR(lock_i2c(), TAG, "I2C mutex timeout");
    esp_err_t ret = i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
    xSemaphoreGive(s_i2c_mutex);
    return ret;
}

static esp_err_t read_u8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *value)
{
    return read_bytes(dev, reg, value, 1);
}

static esp_err_t wait_for_reg_mask(i2c_master_dev_handle_t dev,
                                   uint8_t reg,
                                   uint8_t mask,
                                   bool set,
                                   uint32_t timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    uint8_t value = 0;

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        ESP_RETURN_ON_ERROR(read_u8(dev, reg, &value), TAG, "read 0x%02x failed", reg);

        if (set ? ((value & mask) != 0) : ((value & mask) == 0)) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t get_spad_info(i2c_master_dev_handle_t dev, uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp = 0;

    ESP_RETURN_ON_ERROR(write_u8(dev, 0x80, 0x01), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0xFF, 0x01), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0x00, 0x00), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0xFF, 0x06), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(read_u8(dev, 0x83, &tmp), TAG, "SPAD read failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0x83, tmp | 0x04), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0xFF, 0x07), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0x81, 0x01), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0x80, 0x01), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0x94, 0x6B), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0x83, 0x00), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(wait_for_reg_mask(dev, 0x83, 0xFF, true, 100), TAG, "SPAD query timeout");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0x83, 0x01), TAG, "SPAD prep failed");
    ESP_RETURN_ON_ERROR(read_u8(dev, 0x92, &tmp), TAG, "SPAD result read failed");

    *count = tmp & 0x7F;
    *type_is_aperture = ((tmp >> 7) & 0x01) != 0;

    ESP_RETURN_ON_ERROR(write_u8(dev, 0x81, 0x00), TAG, "SPAD cleanup failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0xFF, 0x06), TAG, "SPAD cleanup failed");
    ESP_RETURN_ON_ERROR(read_u8(dev, 0x83, &tmp), TAG, "SPAD cleanup read failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0x83, tmp & ~0x04), TAG, "SPAD cleanup failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0xFF, 0x01), TAG, "SPAD cleanup failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0x00, 0x01), TAG, "SPAD cleanup failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0xFF, 0x00), TAG, "SPAD cleanup failed");
    ESP_RETURN_ON_ERROR(write_u8(dev, 0x80, 0x00), TAG, "SPAD cleanup failed");

    return ESP_OK;
}

static esp_err_t load_default_tuning(i2c_master_dev_handle_t dev)
{
    /*
     * ST's recommended default tuning table, mirrored by the Pololu VL53L0X
     * driver. These opaque model registers configure VCSEL, phase, histogram,
     * DSS, and range-limit behavior before reference calibration.
     */
    static const uint8_t sequence[][2] = {
        {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00},
        {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},
        {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
        {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},
        {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
        {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
        {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
        {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
        {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
        {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
        {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00},
        {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
        {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00},
        {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
        {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
        {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
        {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE},
        {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
        {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01},
        {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
    };

    for (size_t i = 0; i < sizeof(sequence) / sizeof(sequence[0]); ++i) {
        ESP_RETURN_ON_ERROR(write_u8(dev, sequence[i][0], sequence[i][1]),
                            TAG,
                            "tuning write 0x%02x failed",
                            sequence[i][0]);
    }

    return ESP_OK;
}

static esp_err_t apply_ref_spads(i2c_master_dev_handle_t dev, uint8_t spad_count, bool aperture_spads)
{
    uint8_t ref_spad_map[6] = {0};
    uint8_t spads_enabled = 0;
    const uint8_t first_spad_to_enable = aperture_spads ? 12 : 0;

    ESP_RETURN_ON_ERROR(read_bytes(dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, sizeof(ref_spad_map)),
                        TAG,
                        "read SPAD map failed");

    for (uint8_t i = 0; i < 48; ++i) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[i / 8] &= (uint8_t)~(1U << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x01) {
            ++spads_enabled;
        }
    }

    uint8_t write_buf[7] = {REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                            ref_spad_map[0],
                            ref_spad_map[1],
                            ref_spad_map[2],
                            ref_spad_map[3],
                            ref_spad_map[4],
                            ref_spad_map[5]};
    return write_bytes(dev, write_buf, sizeof(write_buf));
}

static esp_err_t perform_single_ref_calibration(i2c_master_dev_handle_t dev, uint8_t vhv_init_byte)
{
    ESP_RETURN_ON_ERROR(write_u8(dev, REG_SYSRANGE_START, 0x01 | vhv_init_byte),
                        TAG,
                        "calibration start failed");
    ESP_RETURN_ON_ERROR(wait_for_reg_mask(dev, REG_RESULT_INTERRUPT_STATUS, 0x07, true, 500),
                        TAG,
                        "calibration timeout");
    ESP_RETURN_ON_ERROR(write_u8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01),
                        TAG,
                        "calibration interrupt clear failed");
    return write_u8(dev, REG_SYSRANGE_START, 0x00);
}

esp_err_t vl53l0x_set_address(i2c_master_dev_handle_t default_dev, uint8_t new_addr)
{
    /*
     * Register 0x8A is I2C_SLAVE_DEVICE_ADDRESS. The VL53L0X stores the 7-bit
     * address directly in bits [6:0]; do not shift it left and do not include
     * the R/W bit. ESP-IDF emits the bus address byte from this 7-bit value.
     */
    return write_u8(default_dev, REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
}

esp_err_t vl53l0x_init(vl53l0x_t *sensor)
{
    uint8_t model_id = 0;
    uint8_t tmp = 0;
    uint8_t spad_count = 0;
    bool aperture_spads = false;

    ESP_RETURN_ON_FALSE(sensor != NULL && sensor->dev != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid sensor");
    ESP_RETURN_ON_ERROR(read_u8(sensor->dev, REG_IDENTIFICATION_MODEL_ID, &model_id),
                        TAG,
                        "%s model ID read failed",
                        sensor->name);
    if (model_id != VL53L0X_MODEL_ID) {
        ESP_LOGW(TAG, "%s unexpected model ID 0x%02x; continuing", sensor->name, model_id);
    }

    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, 0x88, 0x00), TAG, "%s I2C mode setup failed", sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, 0x80, 0x01), TAG, "%s stop-variable setup failed", sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, 0xFF, 0x01), TAG, "%s stop-variable setup failed", sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, 0x00, 0x00), TAG, "%s stop-variable setup failed", sensor->name);
    ESP_RETURN_ON_ERROR(read_u8(sensor->dev, 0x91, &sensor->stop_variable),
                        TAG,
                        "%s stop-variable read failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, 0x00, 0x01), TAG, "%s stop-variable setup failed", sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, 0xFF, 0x00), TAG, "%s stop-variable setup failed", sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, 0x80, 0x00), TAG, "%s stop-variable setup failed", sensor->name);

    ESP_RETURN_ON_ERROR(read_u8(sensor->dev, REG_MSRC_CONFIG_CONTROL, &tmp), TAG, "%s MSRC read failed", sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_MSRC_CONFIG_CONTROL, tmp | 0x12),
                        TAG,
                        "%s MSRC write failed",
                        sensor->name);

    /* 0x44 is an 8.8 fixed-point count-rate limit. 0.25 MCPS = 0.25 * 128 = 32. */
    ESP_RETURN_ON_ERROR(write_u16(sensor->dev, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32),
                        TAG,
                        "%s signal-rate limit failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xFF),
                        TAG,
                        "%s sequence config failed",
                        sensor->name);

    ESP_RETURN_ON_ERROR(get_spad_info(sensor->dev, &spad_count, &aperture_spads),
                        TAG,
                        "%s SPAD info failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(apply_ref_spads(sensor->dev, spad_count, aperture_spads),
                        TAG,
                        "%s SPAD map apply failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(load_default_tuning(sensor->dev), TAG, "%s tuning failed", sensor->name);

    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04),
                        TAG,
                        "%s interrupt config failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(read_u8(sensor->dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, &tmp),
                        TAG,
                        "%s GPIO mux read failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, tmp & ~0x10),
                        TAG,
                        "%s GPIO mux write failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01),
                        TAG,
                        "%s interrupt clear failed",
                        sensor->name);

    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8),
                        TAG,
                        "%s pre-calibration sequence failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x01),
                        TAG,
                        "%s VHV calibration sequence failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(perform_single_ref_calibration(sensor->dev, 0x40),
                        TAG,
                        "%s VHV calibration failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x02),
                        TAG,
                        "%s phase calibration sequence failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(perform_single_ref_calibration(sensor->dev, 0x00),
                        TAG,
                        "%s phase calibration failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8),
                        TAG,
                        "%s final sequence config failed",
                        sensor->name);

    ESP_LOGI(TAG, "%s initialized at 0x%02x", sensor->name, sensor->address);
    return ESP_OK;
}

esp_err_t vl53l0x_read_range_mm(vl53l0x_t *sensor, uint16_t *range_mm)
{
    uint8_t block[12] = {0};

    ESP_RETURN_ON_FALSE(sensor != NULL && sensor->dev != NULL && range_mm != NULL,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid range read args");
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_SYSRANGE_START, 0x01),
                        TAG,
                        "%s range start failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(wait_for_reg_mask(sensor->dev, REG_SYSRANGE_START, 0x01, false, 100),
                        TAG,
                        "%s range start-bit timeout",
                        sensor->name);
    ESP_RETURN_ON_ERROR(wait_for_reg_mask(sensor->dev, REG_RESULT_INTERRUPT_STATUS, 0x07, true, 100),
                        TAG,
                        "%s range data-ready timeout",
                        sensor->name);

    /*
     * RESULT_RANGE_STATUS starts a 12-byte result block. The millimetric range
     * is big-endian at offsets 10 and 11, corresponding to registers 0x1E and
     * 0x1F in the VL53L0X result map.
     */
    ESP_RETURN_ON_ERROR(read_bytes(sensor->dev, REG_RESULT_RANGE_STATUS, block, sizeof(block)),
                        TAG,
                        "%s range block read failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(write_u8(sensor->dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01),
                        TAG,
                        "%s range interrupt clear failed",
                        sensor->name);

    *range_mm = ((uint16_t)block[10] << 8) | block[11];
    return ESP_OK;
}
