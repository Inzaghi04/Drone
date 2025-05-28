#include <string.h>
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include <math.h>

static const char* TAG = "ESP32_DRONE";

// Cấu trúc dữ liệu từ web
typedef struct {
    float throttle;
    float yaw;
    float pitch;
    float roll;
    float AUX1;
} joystick_data_t;

static joystick_data_t web_control_data = {
    .throttle = 1000,  // Giá trị mặc định tương ứng PWM_MIN_US
    .roll = 1500,
    .pitch = 1500,
    .yaw = 1500,
    .AUX1 = 0
};
static SemaphoreHandle_t web_data_mutex; // Mutex để bảo vệ dữ liệu web

// Định nghĩa chân PWM cho ESC
#define ESC_MOTOR_1 19
#define ESC_MOTOR_2 15
#define ESC_MOTOR_3 14
#define ESC_MOTOR_4 25
#define LED 2

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_FREQ       50
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_MIN_US      1000
#define PWM_MAX_US      2000

#define PWM_MIN_DUTY   (1023 * 5 / 100)    // 51 (~1000µs)
#define PWM_MAX_DUTY   (1023 * 10 / 100)   // 102 (~2000µs)
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define MPU6050_ADDR                0x68

float x, y, z, t;
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float CurrentConsumed=0;
uint64_t LoopTimer;
float DesiredRateRoll, DesiredRatePitch,DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float Throttle, Yaw, Roll, Pitch;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};
float PRateRoll = 0.625;  float PRateYaw = 4.0;
float IRateRoll = 2.1;  float IRateYaw = 3.0;
float DRateRoll = 0.0088; float DRateYaw = 0;
float PRatePitch = 0.625;
float IRatePitch = 2.1; 
float DRatePitch = 0.0088 ;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll=2.0; float PAnglePitch = 2.0;
float IAngleRoll=0.5; float IAnglePitch = 0.5;
float DAngleRoll=0.007; float DAnglePitch = 0.007;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

float round_to_2_decimal_places(float num) {
    return roundf(num * 100) / 100;
}
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState = KalmanState + 0.02 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.02 * 0.02 * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState; 
    Kalman1DOutput[1] = KalmanUncertainty;
}
// Hàm ghi thanh ghi
esp_err_t mpu6050_register_write(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

// Hàm đọc thanh ghi
esp_err_t mpu6050_register_read(uint8_t reg, uint8_t *data, size_t len) {
    // Gửi địa chỉ thanh ghi trước
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &reg, 1, pdMS_TO_TICKS(1000)));
    // Đọc dữ liệu
    return i2c_master_receive(dev_handle, data, len, pdMS_TO_TICKS(1000));
}

void setMotorSpeed(ledc_channel_t channel, float input) {
    // Tính duty cycle cho 10-bit (0-1023)
 // Giới hạn giá trị input trong khoảng cho phép
    uint32_t duty = (uint32_t)(
        (input - 1000) * (PWM_MAX_DUTY - PWM_MIN_DUTY) / 1000 + PWM_MIN_DUTY
    );
   // ESP_LOGI(TAG, "Channel %d: mapped_input=%dµs, duty=%"PRIu32, channel, (int)mapped_input, duty);
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);

}


// Cấu hình LEDC PWM
static void configureMotor(int gpio, ledc_channel_t channel) {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = gpio,
        .speed_mode = LEDC_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = PWM_MIN_DUTY,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
}

// Xử lý yêu cầu HTTP POST từ web
static esp_err_t control_post_handler(httpd_req_t *req) {
    // Thêm header CORS
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        return ESP_FAIL;
    }
    
    // Đảm bảo kết thúc chuỗi đúng cách
    content[ret] = '\0';

    // Thử sử dụng nhiều định dạng phân tích khác nhau
    int parsed = 0;
    
    // Phân tích nếu số là chuỗi: "1500.00"
    parsed = sscanf(content, "{\"roll\":\"%f\",\"pitch\":\"%f\",\"yaw\":\"%f\",\"throttle\":\"%f\",\"AUX1\":\"%f\"}", 
                    &web_control_data.roll, &web_control_data.pitch, 
                    &web_control_data.yaw, &web_control_data.throttle, &web_control_data.AUX1);
    
    // Nếu không thành công, thử phân tích nếu số không phải chuỗi: 1500.00
    if (parsed != 5) {
        parsed = sscanf(content, "{\"roll\":%f,\"pitch\":%f,\"yaw\":%f,\"throttle\":%f,\"AUX1\":%f}", 
                        &web_control_data.roll, &web_control_data.pitch, 
                        &web_control_data.yaw, &web_control_data.throttle, &web_control_data.AUX1);
    }
    
    if (parsed == 5) {
        x = round_to_2_decimal_places(web_control_data.pitch);
        y = round_to_2_decimal_places(web_control_data.roll);
        z = round_to_2_decimal_places(web_control_data.yaw);
        t = round_to_2_decimal_places(web_control_data.throttle);
        
    } else {
        ESP_LOGE(TAG, "Failed to parse JSON data");
    }

    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// Xử lý yêu cầu OPTIONS cho CORS
static esp_err_t control_options_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// Đăng ký URI
httpd_uri_t control_post_uri = {.uri = "/control", .method = HTTP_POST, .handler = control_post_handler};
httpd_uri_t control_options_uri = {.uri = "/control", .method = HTTP_OPTIONS, .handler = control_options_handler};

// Khởi động web server
static void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &control_post_uri);
        httpd_register_uri_handler(server, &control_options_uri);
    }
}
// Xử lý sự kiện IP
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        esp_netif_ip_info_t* ip_info = (esp_netif_ip_info_t*)event_data;
        (void) ip_info;
    }
}

// Khởi tạo Wi-Fi
static void init_wifi(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Tăng tốc độ xử lý TCP/IP stack
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_set_hostname(netif, "ESP32_DRONE");


    // Cấu hình WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Van Vuong",    
            .password = "n06111977", 
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,  // Yêu cầu mã hóa WPA2 trở lên
            .scan_method = WIFI_FAST_SCAN,  // Quét mạng nhanh hơn
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL, // Kết nối AP mạnh nhất
            .pmf_cfg = {.capable = true, .required = false}, // Bật Protected Management Frames
        },
    };

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    esp_wifi_set_storage(WIFI_STORAGE_RAM);  // Lưu cấu hình Wi-Fi trong RAM thay vì NVS
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);  // Cố định băng thông 20MHz
    esp_wifi_config_11b_rate(ESP_IF_WIFI_STA, true);  // Bật tính năng HE nếu dùng Wi-Fi 6
    // Tăng tốc kết nối lại

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));  // Tắt Power Save mode
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi Initialization Complete. Connecting...");
    
    ESP_ERROR_CHECK(esp_wifi_connect());
  
}

void read_receiver(void) {
    xSemaphoreTake(web_data_mutex, portMAX_DELAY);
    xSemaphoreGive(web_data_mutex);
    Throttle = t;
    Yaw = z;
    Roll = y;
    Pitch = x;
}

void gyro_signals(void) {
    // Cấu hình MPU6050
    ESP_ERROR_CHECK(mpu6050_register_write(0x1A, 0x05));  // Low-pass filter
    ESP_ERROR_CHECK(mpu6050_register_write(0x1C, 0x10));  // Accelerometer +/- 8g
    ESP_ERROR_CHECK(mpu6050_register_write(0x1B, 0x8));  // Gyro +/- 500dps

    // Đọc dữ liệu accelerometer
    uint8_t acc_data[6];
    ESP_ERROR_CHECK(mpu6050_register_read(0x3B, acc_data, sizeof(acc_data)));
    // Xử lý dữ liệu accelerometer
    int16_t AccXLSB = (acc_data[0] << 8) | acc_data[1];
    int16_t AccYLSB = (acc_data[2] << 8) | acc_data[3];
    int16_t AccZLSB = (acc_data[4] << 8) | acc_data[5];
        
    
    // Đọc dữ liệu gyro
    uint8_t gyro_data[6];
    ESP_ERROR_CHECK(mpu6050_register_read(0x43, gyro_data, sizeof(gyro_data)));

    // Xử lý dữ liệu gyro
    int16_t GyroX = (gyro_data[0] << 8) | gyro_data[1];
    int16_t GyroY = (gyro_data[2] << 8) | gyro_data[3];
    int16_t GyroZ = (gyro_data[4] << 8) | gyro_data[5];

    // Chuyển đổi giá trị
    
    RateRoll = (float) GyroX / 65.5;
    RatePitch = (float) GyroY / 65.5;
    RateYaw = (float) GyroZ / 65.5;

    AccX = (float) AccXLSB / 4096.0 - 0.07;
    AccY = (float) AccYLSB / 4096.0 + 0.01;
    AccZ = (float) AccZLSB / 4096.0 - 0.04;

    // Tính toán góc
    AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
    AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
    // === Thông số PID ===
    const float dt = 0.02f;     // thời gian mẫu phần I (10ms)
    const float dt_d = 0.02f;   // thời gian mẫu phần D (20ms)
    const float error_threshold = 5.0f;
    const float Iterm_max = 400.0f;
    const float Iterm_min = -400.0f;
    const float output_max = 400.0f;
    const float output_min = -400.0f;
    const float anti_windup_gain = 0.1f;

    // === P-Term ===
    float Pterm = P * Error;

    // === I-Term với Conditional Integration ===
    float Iterm = PrevIterm + I * (Error + PrevError) * dt / 2.0f;

    // Clamping Iterm
    if (Iterm > Iterm_max) Iterm = Iterm_max;
    else if (Iterm < Iterm_min) Iterm = Iterm_min;

    // === D-Term (Derivative) ===
    float Dterm = D * (Error - PrevError) / dt_d;

    // === Tổng đầu ra PID chưa giới hạn ===
    float rawOutput = Pterm + Iterm + Dterm;

    // === Anti-windup với Back-calculation ===
    float PIDOutput = rawOutput;
    if (rawOutput > output_max) {
        PIDOutput = output_max;
        if (Error > 0) {
            float output_error = output_max - rawOutput;
            Iterm += anti_windup_gain * output_error;
        }
    } else if (rawOutput < output_min) {
        PIDOutput = output_min;
        if (Error < 0) {
            float output_error = output_min - rawOutput;
            Iterm += anti_windup_gain * output_error;
        }
    }

    // Clamping lại Iterm sau anti-windup
    if (Iterm > Iterm_max) Iterm = Iterm_max;
    else if (Iterm < Iterm_min) Iterm = Iterm_min;

    // === Cập nhật đầu ra ===
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}

void reset_pid(void) {
    PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;    
    PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
}
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Nếu NVS bị hỏng hoặc phiên bản mới, hãy xóa và khởi tạo lại
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    web_data_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    init_wifi();

    start_webserver();
        // Cấu hình bus I2C
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, // Kích hoạt pullup nội
    };

    // Khởi tạo bus I2C
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // Cấu hình thiết bị MPU6050 trên bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Thêm thiết bị vào bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    // Khởi động MPU6050
    ESP_ERROR_CHECK(mpu6050_register_write(0x6B, 0x00));
    vTaskDelay(pdMS_TO_TICKS(250));
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        vTaskDelay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
    esp_rom_gpio_pad_select_gpio(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED, 1);
    configureMotor(ESC_MOTOR_1, LEDC_CHANNEL_0);
    configureMotor(ESC_MOTOR_2, LEDC_CHANNEL_1);
    configureMotor(ESC_MOTOR_3, LEDC_CHANNEL_2);
    configureMotor(ESC_MOTOR_4, LEDC_CHANNEL_3);
    vTaskDelay(pdMS_TO_TICKS(5000));
    gpio_set_level(LED, 0);
    ESP_LOGI(TAG, "Motor PWM Configured");
    LoopTimer = esp_timer_get_time();
    while (1) {
        gyro_signals();
        RateRoll -= RateCalibrationRoll;
        RatePitch -= RateCalibrationPitch;
        RateYaw -= RateCalibrationYaw;
        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
        KalmanAngleRoll = Kalman1DOutput[0]; KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
        KalmanAnglePitch = Kalman1DOutput[0]; KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
        read_receiver();
        DesiredAngleRoll = 0.10 * (Roll - 1500);
        DesiredAnglePitch = 0.10 * (Pitch - 1500);
        InputThrottle = Throttle;
        DesiredRateYaw = 0.15 * (Yaw - 1500);
        ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
        ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
        pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);     
        DesiredRateRoll = PIDReturn[0]; 
        PrevErrorAngleRoll = PIDReturn[1];
        PrevItermAngleRoll = PIDReturn[2];
        pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
        DesiredRatePitch = PIDReturn[0]; 
        PrevErrorAnglePitch = PIDReturn[1];
        PrevItermAnglePitch = PIDReturn[2];
        ErrorRateRoll = DesiredRateRoll - RateRoll;
        ErrorRatePitch = DesiredRatePitch - RatePitch;
        ErrorRateYaw = DesiredRateYaw - RateYaw;
        pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
        InputRoll = PIDReturn[0];
        PrevErrorRateRoll = PIDReturn[1]; 
        PrevItermRateRoll = PIDReturn[2];
        pid_equation(ErrorRatePitch, PRatePitch,IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
        InputPitch = PIDReturn[0]; 
        PrevErrorRatePitch = PIDReturn[1]; 
        PrevItermRatePitch = PIDReturn[2];
        pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
        InputYaw = PIDReturn[0]; 
        PrevErrorRateYaw = PIDReturn[1]; 
        PrevItermRateYaw = PIDReturn[2];
        if (InputThrottle > 1800) InputThrottle = 1800;
        MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch + InputYaw);
        MotorInput4 = 1.024 * (InputThrottle - InputRoll + InputPitch - InputYaw);
        MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch + InputYaw);
        MotorInput2 = 1.024 * (InputThrottle + InputRoll - InputPitch - InputYaw);

        if (MotorInput1 > 2000) MotorInput1 = 1999;
        if (MotorInput2 > 2000) MotorInput2 = 1999; 
        if (MotorInput3 > 2000) MotorInput3 = 1999; 
        if (MotorInput4 > 2000) MotorInput4 = 1999;
        int ThrottleIdle = 1180;
        if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
        if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
        if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
        if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;
        int ThrottleCutOff = 1000;
        if (Throttle < 1050) {
            MotorInput1 = ThrottleCutOff; 
            MotorInput2 = ThrottleCutOff;
            MotorInput3 = ThrottleCutOff; 
            MotorInput4 = ThrottleCutOff;
            reset_pid();
        }
        if (web_control_data.AUX1 == 0) {
            MotorInput1 = ThrottleCutOff; 
            MotorInput2 = ThrottleCutOff;
            MotorInput3 = ThrottleCutOff; 
            MotorInput4 = ThrottleCutOff;
            reset_pid();
        } else {
            setMotorSpeed(LEDC_CHANNEL_0, MotorInput1);
            setMotorSpeed(LEDC_CHANNEL_1, MotorInput2);
            setMotorSpeed(LEDC_CHANNEL_2, MotorInput3);
            setMotorSpeed(LEDC_CHANNEL_3, MotorInput4);
            printf("AccX: %.2f, AccY: %.2f, AccZ: %.2f\n", AccX, AccY, AccZ);
            printf("AngleRoll: %.2f, AnglePitch: %.2f\n", AngleRoll, AnglePitch);
            printf("Throttle: %.2f, Yaw: %.2f, Roll: %.2f, Pitch: %.2f\n", Throttle, Yaw, Roll, Pitch);
            printf("RateRoll: %.2f, RatePitch: %.2f, RateYaw: %.2f\n", RateRoll, RatePitch, RateYaw);
            printf("InputThrottle: %.2f, InputRoll: %.2f, InputPitch: %.2f, InputYaw: %.2f\n", InputThrottle, InputRoll, InputPitch, InputYaw);
            printf("MotorInput1: %.2f, MotorInput2: %.2f, MotorInput3: %.2f, MotorInput4: %.2f\n", MotorInput1, MotorInput2, MotorInput3, MotorInput4);

        }
        while (esp_timer_get_time() - LoopTimer < 20000) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Chờ 1ms
        }
        LoopTimer = esp_timer_get_time();
    }
}
