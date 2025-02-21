#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <math.h>

#define I2C_MASTER_SCL_IO 22    // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 21    // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0 // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency
#define MPU6050_SENSOR_ADDR 0x68 // Slave address of the MPU6050 sensor

int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroX, GyroY, GyroZ;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccZCalibration,AccYCalibration,AccXCalibration;
int RateCalibrationNumber;

float calAccX, calAccY, calAccZ;

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
void gyro_signals(void) {
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

void calibrateAcc()
{
    //AccY calc
    printf("Place the quadcopter on its Right side with nose forward and press 'Space bar' and then 'Enter'.\n");
    while (getchar() != ' ') {}  // Wait for the user to press 'Enter'
    getchar();  // Clear the input buffer
    gyro_signals();
    printf("AccX: %.2f, AccY: %.2f, AccZ: %.2f\n", AccX, AccY, AccZ);
    AccYCalibration = AccY - 1;

    //Acc X calc
    printf("Place the quadcopter nose up and press 'Space bar' and then 'Enter'.\n");
    while (getchar() != ' ') {}  // Wait for the user to press 'Enter'
    getchar();  // Clear the input buffer
    gyro_signals(); // Record X axis values
    printf("AccX: %.2f, AccY: %.2f, AccZ: %.2f\n", AccX, AccY, AccZ);
    AccXCalibration = AccX - 1;
}

void calibrateGyro()
{
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 1000; RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    AccZCalibration +=  AccZ;

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  RateCalibrationRoll /= 1000;
  RateCalibrationPitch /= 1000;
  RateCalibrationYaw /= 1000;
  AccZCalibration /=1000;
  AccZCalibration = AccZCalibration-1 ;
}

void calibrateGyroSimple()
{
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    AccXCalibration +=  AccX;
    AccYCalibration +=  AccY;
    AccZCalibration +=  AccZ;

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  AccXCalibration /=2000;
  AccYCalibration /=2000;
  AccZCalibration /=2000;
  AccZCalibration = AccZCalibration-1 ;
}

void app_main(void) {
    i2c_master_init();
    i2c_write_bytes(MPU6050_SENSOR_ADDR, 0x6B, 0x00); // Wake up MPU6050
    vTaskDelay(100 / portTICK_PERIOD_MS);

    printf("Welcome to quadcopter IMU calibration\n");
    printf("Place the quadcopter flat on a surface and press 'Space bar' and then 'Enter' to begin Gyro and AccZ calibration.\n");
    while (getchar() != ' ') { // Wait for space bar
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    getchar(); // Consume the Enter key if present after space

    // Simple calibration
    calibrateGyroSimple();
    printf(" GyroX: %.2f, GyroY: %.2f, GyroZ: %.2f, AccX: %.2f, AccY: %.2f, AccZ: %.2f\n",
           RateRoll, RatePitch, RateYaw, AccX, AccY, AccZ);
    printf(" Gyro and AccZ calibration done!\n");

    // Gyro Calibrated Values
    printf("\n");
    printf("RateCalibrationRoll=%.2f;\n", RateCalibrationRoll);
    printf("RateCalibrationPitch=%.2f;\n", RateCalibrationPitch);
    printf("RateCalibrationYaw=%.2f;\n", RateCalibrationYaw);
    printf("AccXCalibration=%.2f;\n", AccXCalibration);
    printf("AccYCalibration=%.2f;\n", AccYCalibration);
    printf("AccZCalibration=%.2f;\n", AccZCalibration);

    printf("Press 'Space bar' and then 'Enter' to print Gyro and Acc continuously\n");
    while (getchar() != ' ') { // Wait for space bar
         vTaskDelay(pdMS_TO_TICKS(10));
    }
    getchar(); // Consume the Enter key if present after space


    while (1) {
        gyro_signals();

        RateRoll -= RateCalibrationRoll;
        RatePitch -= RateCalibrationPitch;
        RateYaw -= RateCalibrationYaw;

        AccX -= AccXCalibration;
        AccY -= AccYCalibration;
        AccZ -= AccZCalibration;

        //Print the accelerometer and gyroscope values
        printf("AccX: %.2f, AccY: %.2f, AccZ: %.2f", AccX, AccY, AccZ);
        printf(" | GyroX: %.2f, GyroY: %.2f, GyroZ: %.2f\n", RateRoll, RatePitch, RateYaw);

        vTaskDelay(pdMS_TO_TICKS(1)); // Delay for 1ms
    }
}