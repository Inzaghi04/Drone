#include "mpu6050.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MPU6050";

// Ghi giá trị vào register
static esp_err_t mpu6050_register_write(mpu6050_dev_t *dev, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->mpu_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Lỗi ghi register 0x%02x: %s", reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

// Đọc nhiều byte từ register
static esp_err_t mpu6050_register_read(mpu6050_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->mpu_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->mpu_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Lỗi đọc register 0x%02x: %s", reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

// Khởi tạo MPU6050
esp_err_t mpu6050_init(mpu6050_dev_t *dev) {
    // Thoát khỏi chế độ sleep
    esp_err_t ret = mpu6050_register_write(dev, 0x6B, 0x00);
    if (ret != ESP_OK) return ret;

    // Cấu hình bộ lọc (DLPF)
    ret = mpu6050_register_write(dev, 0x1A, 0x05);
    if (ret != ESP_OK) return ret;

    // Cấu hình gia tốc kế (±8g)
    ret = mpu6050_register_write(dev, 0x1C, 0x10);
    if (ret != ESP_OK) return ret;

    // Cấu hình con quay (±500°/s)
    ret = mpu6050_register_write(dev, 0x1B, 0x08);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

// Đọc dữ liệu từ MPU6050
esp_err_t mpu6050_read_data(mpu6050_dev_t *dev, mpu6050_data_t *data) {
    uint8_t accel_data[6];
    esp_err_t ret = mpu6050_register_read(dev, 0x3B, accel_data, sizeof(accel_data));
    if (ret != ESP_OK) return ret;

    // Chuyển đổi dữ liệu gia tốc
    int16_t accel_x = (accel_data[0] << 8) | accel_data[1];
    int16_t accel_y = (accel_data[2] << 8) | accel_data[3];
    int16_t accel_z = (accel_data[4] << 8) | accel_data[5];
    data->accel_x_g = (float)accel_x / 4096.0f;  // 4096 LSB/g cho ±8g
    data->accel_y_g = (float)accel_y / 4096.0f;
    data->accel_z_g = (float)accel_z / 4096.0f;

    // Đọc dữ liệu con quay
    uint8_t gyro_data[6];
    ret = mpu6050_register_read(dev, 0x43, gyro_data, sizeof(gyro_data));
    if (ret != ESP_OK) return ret;

    // Chuyển đổi dữ liệu con quay
    int16_t gyro_x = (gyro_data[0] << 8) | gyro_data[1];
    int16_t gyro_y = (gyro_data[2] << 8) | gyro_data[3];
    int16_t gyro_z = (gyro_data[4] << 8) | gyro_data[5];
    data->gyro_x_degs = (float)gyro_x / 65.5f;  // 65.5 LSB/°/s cho ±500°/s
    data->gyro_y_degs = (float)gyro_y / 65.5f;
    data->gyro_z_degs = (float)gyro_z / 65.5f;

    // Tính góc Roll và Pitch từ gia tốc kế
    data->roll_deg = atan2f(data->accel_y_g, sqrtf(data->accel_x_g * data->accel_x_g + data->accel_z_g * data->accel_z_g)) * 180.0f / (float)M_PI;
    data->pitch_deg = -atan2f(data->accel_x_g, sqrtf(data->accel_y_g * data->accel_y_g + data->accel_z_g * data->accel_z_g)) * 180.0f / (float)M_PI;

    return ESP_OK;
}