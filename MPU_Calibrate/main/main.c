#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000

static const char *TAG = "MPU6050_EXAMPLE";

void app_main(void) {
    // Cấu hình I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    // Khởi tạo MPU6050
    mpu6050_dev_t mpu_dev = {
        .i2c_port = I2C_NUM_0,
        .mpu_addr = 0x68
    };
    ESP_ERROR_CHECK(mpu6050_init(&mpu_dev));

    // Đọc dữ liệu liên tục
    mpu6050_data_t data;
    while (1) {
        if (mpu6050_read_data(&mpu_dev, &data) == ESP_OK) {
            ESP_LOGI(TAG, "Accel: X=%.2fg, Y=%.2fg, Z=%.2fg", data.accel_x_g, data.accel_y_g, data.accel_z_g);
            ESP_LOGI(TAG, "Gyro: X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s", data.gyro_x_degs, data.gyro_y_degs, data.gyro_z_degs);
            ESP_LOGI(TAG, "Roll=%.2f°, Pitch=%.2f°", data.roll_deg, data.pitch_deg);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}