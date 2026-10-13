#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "bmi160.h"

static const char *TAG = "bmi160_i2c_test";

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define I2C_FREQ_HZ 100000

static i2c_master_bus_handle_t s_i2c_bus;
static SemaphoreHandle_t s_i2c_mutex;

static void i2c_master_init(void) {
  i2c_master_bus_config_t bus_cfg = {
      .i2c_port = I2C_PORT,
      .sda_io_num = I2C_SDA_GPIO,
      .scl_io_num = I2C_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
  ESP_LOGI(TAG, "I2C Master Bus initialized");
}

static void poll_task(void *arg) {
  bmi160_t imu = {.name = "SPINE_IMU",
                  .int_gpio = GPIO_NUM_NC,
                  .accel_range = BMI160_ACCEL_RANGE_4G,
                  .gyro_range = BMI160_GYRO_RANGE_1000DPS};

  ESP_ERROR_CHECK(
      bmi160_add_i2c_device(s_i2c_bus, BMI160_DEFAULT_ADDR, &imu.dev));

  if (bmi160_init(&imu) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize BMI160. Check wiring!");
    vTaskDelete(NULL);
  }

  bmi160_data_t accel;
  bmi160_data_t gyro;
  float temp_c;

  while (1) {
    if (bmi160_read_data(&imu, &accel, &gyro) == ESP_OK) {
      ESP_LOGI(TAG,
               "Accel (g): X=%0.2f, Y=%0.2f, Z=%0.2f | Gyro (dps): X=%0.2f, "
               "Y=%0.2f, Z=%0.2f",
               accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
    } else {
      ESP_LOGW(TAG, "Failed to read IMU data");
    }

    if (bmi160_read_temp(&imu, &temp_c) == ESP_OK) {
      ESP_LOGI(TAG, "Temperature: %0.2f C", temp_c);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Starting BMI160 Test Application");

  s_i2c_mutex = xSemaphoreCreateMutex();
  bmi160_set_i2c_mutex(s_i2c_mutex);

  i2c_master_init();

  xTaskCreate(poll_task, "bmi160_poll", 4096, NULL, 5, NULL);
}
