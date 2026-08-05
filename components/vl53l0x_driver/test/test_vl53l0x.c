#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "vl53l0x.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define I2C_FREQ_HZ 400000

/* Avoid GPIO12: it is an ESP32 strapping pin and can break flash boot/programming. */
#define XSHUT_LEFT_GPIO GPIO_NUM_25
#define XSHUT_CENTER_GPIO GPIO_NUM_13
#define XSHUT_RIGHT_GPIO GPIO_NUM_14

#define VL53L0X_LEFT_ADDR 0x30
#define VL53L0X_CENTER_ADDR 0x31
#define VL53L0X_RIGHT_ADDR 0x32

#define POLL_TASK_STACK_WORDS 4096
#define POLL_TASK_PRIORITY 5
#define POLL_INTERVAL_MS 100
#define TEST_RUNTIME_MS 30000
#define RANGE_VALID_MIN_MM 30
#define RANGE_VALID_MAX_MM 2000
#define RANGE_INVALID_STALE_LIMIT 3
#define RANGE_FILTER_PREV_WEIGHT 3
#define RANGE_FILTER_NEW_WEIGHT 1

typedef struct {
    uint16_t filtered_mm;
    uint8_t invalid_count;
    bool has_value;
} range_filter_t;

static const char *TAG = "test_vl53l0x";
static i2c_master_bus_handle_t s_i2c_bus;
static SemaphoreHandle_t s_i2c_mutex;

static vl53l0x_t s_sensors[] = {
    {.name = "LEFT", .xshut_gpio = XSHUT_LEFT_GPIO, .address = VL53L0X_LEFT_ADDR},
    {.name = "CENTER", .xshut_gpio = XSHUT_CENTER_GPIO, .address = VL53L0X_CENTER_ADDR},
    {.name = "RIGHT", .xshut_gpio = XSHUT_RIGHT_GPIO, .address = VL53L0X_RIGHT_ADDR},
};
static range_filter_t s_filters[sizeof(s_sensors) / sizeof(s_sensors[0])];

static bool is_valid_range_mm(uint16_t range_mm)
{
    return range_mm >= RANGE_VALID_MIN_MM && range_mm <= RANGE_VALID_MAX_MM;
}

static void update_range_filter(range_filter_t *filter, esp_err_t read_ret, uint16_t raw_mm)
{
    if (read_ret == ESP_OK && is_valid_range_mm(raw_mm)) {
        if (filter->has_value) {
            filter->filtered_mm =
                (uint16_t)(((uint32_t)filter->filtered_mm * RANGE_FILTER_PREV_WEIGHT +
                            (uint32_t)raw_mm * RANGE_FILTER_NEW_WEIGHT) /
                           (RANGE_FILTER_PREV_WEIGHT + RANGE_FILTER_NEW_WEIGHT));
        } else {
            filter->filtered_mm = raw_mm;
            filter->has_value = true;
        }
        filter->invalid_count = 0;
        return;
    }

    if (filter->invalid_count < UINT8_MAX) {
        ++filter->invalid_count;
    }
    if (filter->invalid_count >= RANGE_INVALID_STALE_LIMIT) {
        filter->has_value = false;
    }
}

static void xshut_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << XSHUT_LEFT_GPIO) | (1ULL << XSHUT_CENTER_GPIO) | (1ULL << XSHUT_RIGHT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    TEST_ESP_OK(gpio_config(&io_conf));
    gpio_set_level(XSHUT_LEFT_GPIO, 0);
    gpio_set_level(XSHUT_CENTER_GPIO, 0);
    gpio_set_level(XSHUT_RIGHT_GPIO, 0);
}

static void i2c_master_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    TEST_ESP_OK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
}

static esp_err_t address_and_init_sensor(vl53l0x_t *sensor)
{
    i2c_master_dev_handle_t default_dev = NULL;

    gpio_set_level(sensor->xshut_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(vl53l0x_add_i2c_device(s_i2c_bus, VL53L0X_DEFAULT_ADDR, &default_dev),
                        TAG,
                        "%s default address add failed",
                        sensor->name);
    ESP_RETURN_ON_ERROR(vl53l0x_set_address(default_dev, sensor->address),
                        TAG,
                        "%s address register write failed",
                        sensor->name);
    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(default_dev), TAG, "%s default handle remove failed", sensor->name);
    ESP_RETURN_ON_ERROR(vl53l0x_add_i2c_device(s_i2c_bus, sensor->address, &sensor->dev),
                        TAG,
                        "%s addressed device add failed",
                        sensor->name);
    return vl53l0x_init(sensor);
}

static void poll_task(void *arg)
{
    (void)arg;
    const size_t left = 0;
    const size_t center = 1;
    const size_t right = 2;

    while (true) {
        uint16_t left_mm = 0;
        uint16_t center_mm = 0;
        uint16_t right_mm = 0;

        esp_err_t left_ret = vl53l0x_read_range_mm(&s_sensors[0], &left_mm);
        esp_err_t center_ret = vl53l0x_read_range_mm(&s_sensors[1], &center_mm);
        esp_err_t right_ret = vl53l0x_read_range_mm(&s_sensors[2], &right_mm);

        update_range_filter(&s_filters[left], left_ret, left_mm);
        update_range_filter(&s_filters[center], center_ret, center_mm);
        update_range_filter(&s_filters[right], right_ret, right_mm);

        ESP_LOGI(TAG,
                 "[LEFT: %s%4dmm | CENTER: %s%4dmm | RIGHT: %s%4dmm] raw=[%u,%u,%u]",
                 s_filters[left].has_value ? "" : "---- ",
                 s_filters[left].has_value ? s_filters[left].filtered_mm : 0,
                 s_filters[center].has_value ? "" : "---- ",
                 s_filters[center].has_value ? s_filters[center].filtered_mm : 0,
                 s_filters[right].has_value ? "" : "---- ",
                 s_filters[right].has_value ? s_filters[right].filtered_mm : 0,
                 (unsigned int)left_mm,
                 (unsigned int)center_mm,
                 (unsigned int)right_mm);

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

TEST_CASE("triple VL53L0X XSHUT addressing and 100ms polling", "[vl53l0x][hardware]")
{
    s_i2c_mutex = xSemaphoreCreateMutex();
    TEST_ASSERT_NOT_NULL(s_i2c_mutex);
    vl53l0x_set_i2c_mutex(s_i2c_mutex);

    xshut_gpio_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_master_init();

    for (size_t i = 0; i < sizeof(s_sensors) / sizeof(s_sensors[0]); ++i) {
        TEST_ESP_OK(address_and_init_sensor(&s_sensors[i]));
    }

    BaseType_t task_ok = xTaskCreatePinnedToCore(poll_task,
                                                 "vl53_poll",
                                                 POLL_TASK_STACK_WORDS,
                                                 NULL,
                                                 POLL_TASK_PRIORITY,
                                                 NULL,
                                                 1);
    TEST_ASSERT_EQUAL(pdPASS, task_ok);

    vTaskDelay(pdMS_TO_TICKS(TEST_RUNTIME_MS));
}

void app_main(void)
{
    unity_run_menu();
}
