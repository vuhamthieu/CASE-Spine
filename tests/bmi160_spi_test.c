#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "bmi160.h"

static const char *TAG = "bmi160_spi_test";

#define BMI160_SPI_HOST SPI2_HOST
#define BMI160_SPI_MOSI GPIO_NUM_23
#define BMI160_SPI_MISO GPIO_NUM_19
#define BMI160_SPI_SCLK GPIO_NUM_18
#define BMI160_SPI_CS GPIO_NUM_5

static spi_device_handle_t s_spi_dev;
static SemaphoreHandle_t s_bus_mutex;

static void spi_master_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BMI160_SPI_MOSI,
        .miso_io_num = BMI160_SPI_MISO,
        .sclk_io_num = BMI160_SPI_SCLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 32,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(BMI160_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(bmi160_add_spi_device(BMI160_SPI_HOST, BMI160_SPI_CS, &s_spi_dev));
    ESP_LOGI(TAG, "SPI bus initialized");
}

static void poll_task(void *arg)
{
    bmi160_t imu = {
        .name = "SPINE_IMU",
        .int_gpio = GPIO_NUM_NC,
        .spi_dev = s_spi_dev,
        .accel_range = BMI160_ACCEL_RANGE_4G,
        .gyro_range = BMI160_GYRO_RANGE_1000DPS,
    };

    if (bmi160_init(&imu) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMI160 over SPI");
        vTaskDelete(NULL);
    }

    bmi160_data_t accel;
    bmi160_data_t gyro;
    float temp_c;

    while (1) {
        if (bmi160_read_data(&imu, &accel, &gyro) == ESP_OK) {
            ESP_LOGI(TAG,
                     "Accel (g): X=%0.2f, Y=%0.2f, Z=%0.2f | Gyro (dps): X=%0.2f, Y=%0.2f, Z=%0.2f",
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

void app_main(void)
{
    ESP_LOGI(TAG, "Starting BMI160 SPI Test Application");

    s_bus_mutex = xSemaphoreCreateMutex();
    bmi160_set_i2c_mutex(s_bus_mutex);

    spi_master_init();
    xTaskCreate(poll_task, "bmi160_spi_poll", 4096, NULL, 5, NULL);
}
