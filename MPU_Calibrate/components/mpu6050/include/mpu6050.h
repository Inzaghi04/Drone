#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_port_t i2c_port;    // I2C port (e.g., I2C_NUM_0)
    uint8_t mpu_addr;       // Địa chỉ I2C của MPU6050 (0x68 hoặc 0x69)
} mpu6050_dev_t;

typedef struct {
    float accel_x_g;        // Gia tốc trục X (đơn vị g)
    float accel_y_g;        // Gia tốc trục Y (đơn vị g)
    float accel_z_g;        // Gia tốc trục Z (đơn vị g)
    float gyro_x_degs;      // Tốc độ góc trục X (đơn vị độ/giây)
    float gyro_y_degs;      // Tốc độ góc trục Y (đơn vị độ/giây)
    float gyro_z_degs;      // Tốc độ góc trục Z (đơn vị độ/giây)
    float roll_deg;         // Góc Roll (đơn vị độ)
    float pitch_deg;        // Góc Pitch (đơn vị độ)
} mpu6050_data_t;

// Khởi tạo MPU6050
esp_err_t mpu6050_init(mpu6050_dev_t *dev);

// Đọc dữ liệu từ MPU6050
esp_err_t mpu6050_read_data(mpu6050_dev_t *dev, mpu6050_data_t *data);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H