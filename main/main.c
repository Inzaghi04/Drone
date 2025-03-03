#include <string.h>
#include "esp_now.h"
#include "esp_timer.h" // Thêm dòng này
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_http_server.h"
#include <math.h>
static const char* TAG = "ESP_NOW_RECEIVER";

// Cấu trúc dữ liệu joystick nhận từ transmitter
typedef struct {
    uint8_t throttle;
    uint8_t yaw;
    uint8_t pitch;
    uint8_t roll;
    uint8_t AUX1;
} joystick_data_t;

joystick_data_t web_control_data = {0};
SemaphoreHandle_t data_mutex = NULL;

// Định nghĩa chân PWM cho ESC điều khiển động cơ
#define ESC_MOTOR_1 14
#define ESC_MOTOR_2 15
#define ESC_MOTOR_3 19
#define ESC_MOTOR_4 25

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE

#define LEDC_FREQ       50   
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT  // Giữ nguyên 10-bit
#define PWM_MIN_DUTY   (1023 * 5 / 100)    // 51 (~1000µs)
#define PWM_MAX_DUTY   (1023 * 10 / 100)   // 102 (~2000µs)

#define I2C_MASTER_SCL_IO 22    // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 21    // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0 // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency
#define MPU6050_SENSOR_ADDR 0x68 // Slave address of the MPU6050 sensor

volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch = - 1.32;
float RateCalibrationRoll = 0.01;
float RateCalibrationYaw = - 0.27;
float AccXCalibration = - 0.18;
float AccYCalibration = - 0.19;
float AccZCalibration = - 0.02;

// Dòng 57-59 (PID góc)
float PAngleRoll = 2.0; 
float PAnglePitch = 2.0;  // Thay PAnglePitch=PAngleRoll → gán trực tiếp

float IAngleRoll = 0.5; 
float IAnglePitch = 0.5;  // Thay IAnglePitch=IAngleRoll → gán trực tiếp

float DAngleRoll = 0.007; 
float DAnglePitch = 0.007; // Thay DAnglePitch=DAngleRoll → gán trực tiếp

float PRateRoll = 0.625;
float IRateRoll = 2.1;
float DRateRoll = 0.0088;


float PRatePitch = 0.625; // Thay PRatePitch=PRateRoll → gán trực tiếp


float IRatePitch = 2.1;    // Thay IRatePitch=IRateRoll → gán trực tiếp


float DRatePitch = 0.0088; // Thay DRatePitch=DRateRoll → gán trực tiếp

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

uint32_t LoopTimer;
float t = 0.004;      //time cycle

volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;


float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

uint32_t map_puslse_width(uint32_t input) {
    const uint32_t maxduty = (1 << LEDC_RESOLUTION) - 1;
    const uint32_t period_us = 1000000 / LEDC_FREQ;
    input = (input < 1000) ? 1000 : ((input > 2000) ? 2000 : input);
    return (input * maxduty) / period_us;
}

float getMotorSpeed(float intput) {
    float mapIp = 1000 + (intput * 1000 / 255);
    if (mapIp > 2000) mapIp = 2000;
    if (mapIp < 1000) mapIp = 1000;
    return mapIp;
}

void setMotorSpeed(ledc_channel_t channel, uint32_t input) {
    uint32_t duty = map_puslse_width(input);
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

void configureMotor(int gpio, ledc_channel_t channel) {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = gpio,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = PWM_MIN_DUTY,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
}

static esp_err_t control_post_handler(httpd_req_t *req) {
    // Thêm header CORS
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    char content[100];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        return ESP_FAIL;
    }

    sscanf(content, "{\"roll\":%hhu,\"pitch\":%hhu,\"yaw\":%hhu,\"throttle\":%hhu,\"AUX1\":%hhu}", 
           &web_control_data.roll, &web_control_data.pitch, 
           &web_control_data.yaw, &web_control_data.throttle, &web_control_data.AUX1);
    // setMotorSpeed(LEDC_CHANNEL_1, MotorInput1);
    // setMotorSpeed(LEDC_CHANNEL_2, MotorInput2);   
    // setMotorSpeed(LEDC_CHANNEL_3, MotorInput3);
    // setMotorSpeed(LEDC_CHANNEL_4, MotorInput4);
    // ESP_LOGI(TAG, "Web Control: Roll=%.2f, Pitch=%.2f, Yaw=%.2f, Throttle=%.2f, AUX1=%.2f",
    //      (double)web_control_data.roll, (double)web_control_data.pitch, 
    //      (double)web_control_data.yaw, (double)web_control_data.throttle, (double)web_control_data.AUX1);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// Handler cho yêu cầu OPTIONS đến /control (để hỗ trợ CORS preflight)
static esp_err_t control_options_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// Đăng ký URI cho POST và OPTIONS tại endpoint /control
httpd_uri_t control_post_uri = {
    .uri       = "/control",
    .method    = HTTP_POST,
    .handler   = control_post_handler,
    .user_ctx  = NULL
};

httpd_uri_t control_options_uri = {
    .uri       = "/control",
    .method    = HTTP_OPTIONS,
    .handler   = control_options_handler,
    .user_ctx  = NULL
};

void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &control_post_uri);
        httpd_register_uri_handler(server, &control_options_uri);
    }
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        esp_netif_ip_info_t* ip_info = (esp_netif_ip_info_t*) event_data;
        ESP_LOGI("NETWORK", "ESP32 IP Address: " IPSTR, IP2STR(&ip_info->ip));
    }
}

void init_wifi() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_any_id));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Van Vuong",
            .password = "n06111977"
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("NETWORK", "Connecting to WiFi...");
    ESP_ERROR_CHECK(esp_wifi_connect());
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState=KalmanState + (t*KalmanInput);
    KalmanUncertainty=KalmanUncertainty + (t*t*4*4); //here 4 is the vairnece of IMU i.e 4 deg/s
    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //std deviation of error is 3 deg
    KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0]=KalmanState; 
    Kalman1DOutput[1]=KalmanUncertainty;
}

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

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm +( I * (Error + PrevError) * (t/2));
  if (Iterm > 400)
  {
    Iterm = 400;
  }
  else if (Iterm < -400)
  {
  Iterm = -400;
  }
  float Dterm = D *( (Error - PrevError)/t);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
  {
    PIDOutput = 400;
  }
  else if (PIDOutput < -400)
  {
    PIDOutput = -400;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

// esp_err_t i2c_read_bytes_checked(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     esp_err_t ret;

//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg_addr, true);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
//     i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
//     i2c_master_stop(cmd);

//     ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);

//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
//     }
//     return ret;
// }
// bool read_MPU6050() {
//     uint8_t data[14];
//     const int max_retries = 3;

//     for (int i = 0; i < max_retries; i++) {
//         if (i2c_read_bytes_checked(MPU6050_SENSOR_ADDR, 0x3B, data, 14) == ESP_OK) {
//             // Parse dữ liệu thành công
//             AccX = ((data[0] << 8) | data[1]) / 4096.0 - AccXCalibration;
//             AccY = ((data[2] << 8) | data[3]) / 4096.0 - AccYCalibration;
//             AccZ = ((data[4] << 8) | data[5]) / 4096.0 - AccZCalibration;
//             return true;
//         }
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
//     ESP_LOGE(TAG, "MPU6050 read failed after %d retries", max_retries);
//     return false;
// }
void update_motor() {     
        uint8_t data[14];
        int16_t AccXLSB, AccYLSB, AccZLSB, GyroX, GyroY, GyroZ;
        i2c_write_bytes(MPU6050_SENSOR_ADDR, 0x1A, 0x05);
        i2c_write_bytes(MPU6050_SENSOR_ADDR, 0x1C, 0x10);
        i2c_write_bytes(MPU6050_SENSOR_ADDR, 0x3B, 0x00);
        i2c_read_bytes(MPU6050_SENSOR_ADDR, 0x3B, data, 6);
    
        AccXLSB = (data[0] << 8) | data[1];
        AccYLSB = (data[2] << 8) | data[3];
        AccZLSB = (data[4] << 8) | data[5];
    
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
    
        RateRoll -= RateCalibrationRoll;
        RatePitch -= RateCalibrationPitch;
        RateYaw -= RateCalibrationYaw;
    
        AccX = AccX - AccXCalibration ;
        AccY = AccY - AccYCalibration ;
        AccZ = AccZ - AccZCalibration;
        AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180 / M_PI);
        AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180 / M_PI);
        // ESP_LOGI("TAG", "AccX = %f\n",AccX);
        // ESP_LOGI("TAG", "AccY = %f\n",AccY);
        // ESP_LOGI("TAG", "AccZ = %f\n",AccZ);
        // ESP_LOGI("TAG", "AngleRoll = %f\n",AngleRoll);
        // ESP_LOGI("TAG", "AnglePitch = %f\n",AnglePitch);
        complementaryAngleRoll=0.991*(complementaryAngleRoll+RateRoll*t) + 0.009*AngleRoll;
        complementaryAnglePitch=0.991*(complementaryAnglePitch+RatePitch*t) + 0.009*AnglePitch;
        // Clamping complementary filter roll angle to ±20 degrees
        complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
        complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);

        DesiredAngleRoll = 0.1*(getMotorSpeed(web_control_data.roll) - 1500);
        DesiredAnglePitch = 0.1*(getMotorSpeed(web_control_data.pitch) -1500);
        InputThrottle = getMotorSpeed(web_control_data.throttle);
        DesiredRateYaw = 0.15*(getMotorSpeed(web_control_data.yaw) - 1500);

        // ESP_LOGI("TAG","DesiredAngleRoll %f\n",DesiredAngleRoll);
        // ESP_LOGI("TAG","DesiredAnglePitch %f\n",DesiredAnglePitch);
        // ESP_LOGI("TAG","InputThrottle %f\n",InputThrottle);
        // ESP_LOGI("TAG","DesireddRateYaw %f\n",DesiredRateYaw);
        ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
        PtermRoll = PAngleRoll * ErrorAngleRoll;
        ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
        ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
        DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
        PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
        PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
        DesiredRateRoll = PIDOutputRoll;
        PrevErrorAngleRoll = ErrorAngleRoll;
        PrevItermAngleRoll = ItermRoll;
    
        ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
        PtermPitch = PAnglePitch * ErrorAnglePitch;
        ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
        ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
        DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
        PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
        PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
        DesiredRatePitch = PIDOutputPitch;
        PrevErrorAnglePitch = ErrorAnglePitch;
        PrevItermAnglePitch = ItermPitch;
    
        // Compute errors
        ErrorRateRoll = DesiredRateRoll - RateRoll;
        ErrorRatePitch = DesiredRatePitch - RatePitch;
        ErrorRateYaw = DesiredRateYaw - RateYaw;
    
        // Roll Axis PID
        PtermRoll = PRateRoll * ErrorRateRoll;
        ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
        ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
        DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
        PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
        PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
    
        // Update output and previous values for Roll
        InputRoll = PIDOutputRoll;
        PrevErrorRateRoll = ErrorRateRoll;
        PrevItermRateRoll = ItermRoll;
    
        // Pitch Axis PID
        PtermPitch = PRatePitch * ErrorRatePitch;
        ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
        ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
        DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
        PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
        PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
    
        // Update output and previous values for Pitch
        InputPitch = PIDOutputPitch;
        PrevErrorRatePitch = ErrorRatePitch;
        PrevItermRatePitch = ItermPitch;
    
        // Yaw Axis PID
        PtermYaw = PRateYaw * ErrorRateYaw;
        ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
        ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);  // Clamp ItermYaw to [-400, 400]
        DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
        PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
        PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);  // Clamp PIDOutputYaw to [-400, 400]
    
    
        // Update output and previous values for Yaw
        InputYaw = PIDOutputYaw;
        PrevErrorRateYaw = ErrorRateYaw;
        PrevItermRateYaw = ItermYaw;
        if (InputThrottle > 1800)
        {
            InputThrottle = 1800;
        }
      
        MotorInput1 =  (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
        MotorInput2 =  (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
        MotorInput3 =  (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise
        MotorInput4 =  (InputThrottle + InputRoll - InputPitch + InputYaw); //front left - clockwise
   
        if (MotorInput1 > 2000)
        {
            MotorInput1 = 1999;
        }
        
        if (MotorInput2 > 2000)
        {
            MotorInput2 = 1999;
        }
    
        if (MotorInput3 > 2000)
        {
            MotorInput3 = 1999;
        }
    
        if (MotorInput4 > 2000)
        {
            MotorInput4 = 1999;
        }
    
    
        // int ThrottleIdle = 1150;
        // int ThrottleCutOff = 1000;
        if (MotorInput1 < ThrottleIdle)
        {
            MotorInput1 = ThrottleIdle;
        }
        if (MotorInput2 < ThrottleIdle)
        {
            MotorInput2 = ThrottleIdle;
        }
        if (MotorInput3 < ThrottleIdle)
        {
            MotorInput3 = ThrottleIdle;
        }
        if (MotorInput4 < ThrottleIdle)
        {
            MotorInput4 = ThrottleIdle;
        }
        // if (!read_MPU6050()) {
        //     MotorInput1 = ThrottleCutOff;
        //     MotorInput2 = ThrottleCutOff;
        //     MotorInput3 = ThrottleCutOff;
        //     MotorInput4 = ThrottleCutOff;
        //     return;
        // }
        if (web_control_data.AUX1 == 1 ) 
        {
        
            setMotorSpeed(LEDC_CHANNEL_0, MotorInput1);
            setMotorSpeed(LEDC_CHANNEL_1, MotorInput2);
            setMotorSpeed(LEDC_CHANNEL_2, MotorInput3);
            setMotorSpeed(LEDC_CHANNEL_3, MotorInput4);
            ESP_LOGI(TAG, "Simplified Motor Inputs:");
            ESP_LOGI(TAG, "M1=%.2f", MotorInput1);
            ESP_LOGI(TAG, "M2=%.2f", MotorInput2);
            ESP_LOGI(TAG, "M3=%.2f", MotorInput3);
            ESP_LOGI(TAG, "M4=%.2f", MotorInput4);
        }
        if (web_control_data.AUX1 == 0 ) {
            MotorInput1 = ThrottleCutOff;
            MotorInput2 = ThrottleCutOff;
            MotorInput3 = ThrottleCutOff;
            MotorInput4 = ThrottleCutOff;
    
            PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
            PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
            PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
            PrevItermAngleRoll=0; PrevItermAnglePitch=0;
            setMotorSpeed(LEDC_CHANNEL_0, MotorInput1 );
            setMotorSpeed(LEDC_CHANNEL_1, MotorInput2);
            setMotorSpeed(LEDC_CHANNEL_2, MotorInput3);
            setMotorSpeed(LEDC_CHANNEL_3, MotorInput4);
        }
        if (getMotorSpeed(web_control_data.throttle) < 1030) {
            MotorInput1 = ThrottleCutOff;
            MotorInput2 = ThrottleCutOff;
            MotorInput3 = ThrottleCutOff;
            MotorInput4 = ThrottleCutOff;
            PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
            PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
            PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
            PrevItermAngleRoll=0; PrevItermAnglePitch=0;
            setMotorSpeed(LEDC_CHANNEL_0, MotorInput1);
            setMotorSpeed(LEDC_CHANNEL_1, MotorInput2);
            setMotorSpeed(LEDC_CHANNEL_2, MotorInput3);
            setMotorSpeed(LEDC_CHANNEL_3, MotorInput4);
        }
}
void control_task(void *pvParam) {
    while(1) {
        int64_t start = esp_timer_get_time();
        update_motor(); 
        // int64_t duration = esp_timer_get_time() - start;
        // ESP_LOGI(TAG, "Execution time: %lldµs", duration);
        while(esp_timer_get_time() - start < 4000);
    }
}

void app_main(void)
{
    i2c_master_init();
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_netif_init();
    esp_event_loop_create_default();
    init_wifi();
    ESP_LOGI("NETWORK", "Waiting for IP address...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    start_webserver();
    ESP_LOGI(TAG, "Web Server started, ready to receive joystick data.");
    configureMotor(ESC_MOTOR_1, LEDC_CHANNEL_0);  // Motor 1: GPIO 25, Channel 0
    configureMotor(ESC_MOTOR_2, LEDC_CHANNEL_1);  // Motor 2: GPIO 19, Channel 1
    configureMotor(ESC_MOTOR_3, LEDC_CHANNEL_2);  // Motor 3: GPIO 21, Channel 2
    configureMotor(ESC_MOTOR_4, LEDC_CHANNEL_3);  // Motor 4: GPIO 22, Channel 3
    xTaskCreate(control_task, "control_loop", 4096, NULL, 5, NULL);
}
