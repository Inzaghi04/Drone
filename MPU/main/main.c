#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include <math.h>

#define I2C_MASTER_SCL_IO 22    // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 21    // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0 // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency
#define MPU6050_SENSOR_ADDR 0x68 // Slave address of the MPU6050 sensor

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void i2c_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
void Gyro_signals(void) {
    uint8_t data[14];
    int16_t AccXLSB, AccYLSB, AccZLSB, GyroX, GyroY, GyroZ;
    i2c_write_bytes(MPU6050_SENSOR_ADDR, 0x1A, 0x05);
    i2c_write_bytes(MPU6050_SENSOR_ADDR, 0x1C, 0x10);
    i2c_write_bytes(MPU6050_SENSOR_ADDR, 0x3B, 0x00);
    i2c_read_bytes(MPU6050_SENSOR_ADDR, 0x3B, data, 6);

    //printf("Raw data: %02x %02x %02x %02x %02x %02x\n", data[0], data[1], data[2], data[3], data[4], data[5]);

    AccXLSB = (data[0] << 8) | data[1];
    AccYLSB = (data[2] << 8) | data[3];
    AccZLSB = (data[4] << 8) | data[5];

    //printf("AccXLSB: %d, AccYLSB: %d, AccZLSB: %d\n", AccXLSB, AccYLSB, AccZLSB);

    i2c_write_bytes(MPU6050_SENSOR_ADDR, 0x1B, 0x08);
    i2c_write_bytes(MPU6050_SENSOR_ADDR, 0x43, 0x00);
    i2c_read_bytes(MPU6050_SENSOR_ADDR, 0x43, data, 6);
    GyroX = (data[0] << 8) | data[1];
    GyroY = (data[2] << 8) | data[3];
    GyroZ = (data[4] << 8) | data[5];
    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;
    AccX = (float)AccXLSB / 4096;
    AccY = (float)AccYLSB / 4096;
    AccZ = (float)AccZLSB / 4096;
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180 / M_PI);
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180 / M_PI);
}
void app_main(void) {
    i2c_master_init();
    while (1) {
        Gyro_signals();
        printf("AceX: %f, AceY: %f, AceZ: %f\n", AccX, AccY, AccZ);
        printf("AngleRoll: %f, AnglePitch: %f\n", AngleRoll, AnglePitch);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}