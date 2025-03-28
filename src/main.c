#include <string.h>
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
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
    uint16_t throttle;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
    uint16_t AUX1;
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
#define ESC_MOTOR_1 14
#define ESC_MOTOR_2 19
#define ESC_MOTOR_3 25
#define ESC_MOTOR_4 15
#define LED 2

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_FREQ       50
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_MIN_US      1000
#define PWM_MAX_US      2000

#define PWM_MIN_DUTY   (1023 * 5 / 100)    // 51 (~1000µs)
#define PWM_MAX_DUTY   (1023 * 10 / 100)   // 102 (~2000µs)

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define MPU6050_ADDR 0x68

// Biến toàn cục cho PID và MPU6050
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
float elapsedTime, timePrev;
int gyro_error = 0, acc_error = 0;
float Gyr_rawX, Gyr_rawY, Gyro_angle_x, Gyro_angle_y, Gyro_raw_error_x, Gyro_raw_error_y;
float rad_to_deg = 180 / 3.141592654;
float Acc_rawX, Acc_rawY, Acc_rawZ, Acc_angleX, Acc_angleY, Acc_raw_error_x, Acc_raw_error_y;
float Total_angleX, Total_angleY;
float yaw_PID;

float roll_PID, roll_error, roll_previous_error, roll_pid_p, roll_pid_i, roll_pid_d;
float pitch_PID, pitch_error, pitch_previous_error, pitch_pid_p, pitch_pid_i, pitch_pid_d;
double roll_kp = 0.7, roll_ki = 0.006, roll_kd = 1.2;
double pitch_kp = 0.72, pitch_ki = 0.006, pitch_kd = 1.22;
double yaw_kp = 1.0, yaw_ki = 0.0, yaw_kd = 0.0; // Giá trị PID cho Yaw (chưa tinh chỉnh)
float yaw_error, yaw_previous_error, yaw_pid_p, yaw_pid_i, yaw_pid_d; // Các biến yaw PID

float roll_desired_angle = 0, pitch_desired_angle = 0, yaw_desired_angle = 0;

// Hàm ánh xạ PWM
void setMotorSpeed(ledc_channel_t channel, float input) {
    // Tính duty cycle cho 10-bit (0-1023)
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
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char content[100];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) return ESP_FAIL;

    joystick_data_t temp_data;
    sscanf(content, "{\"roll\":%hu,\"pitch\":%hu,\"yaw\":%hu,\"throttle\":%hu,\"AUX1\":%hu}",
           &temp_data.roll, &temp_data.pitch, &temp_data.yaw, &temp_data.throttle, &temp_data.AUX1);
   // ESP_LOGI(TAG, "Received data: roll=%d, pitch=%d, yaw=%d, throttle=%d, AUX1=%d",
      //       temp_data.roll, temp_data.pitch, temp_data.yaw, temp_data.throttle, temp_data.AUX1);

    xSemaphoreTake(web_data_mutex, portMAX_DELAY);
    web_control_data = temp_data;
    xSemaphoreGive(web_data_mutex);

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

    // Tăng tốc kết nối lại
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));  // Tắt Power Save mode
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi Initialization Complete. Connecting...");
    
    ESP_ERROR_CHECK(esp_wifi_connect());
}
// Khởi tạo I2C
static void i2c_master_init(void) {
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

// Ghi dữ liệu I2C
static esp_err_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Đọc dữ liệu I2C
static esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Cập nhật tốc độ động cơ
static void update_motor(joystick_data_t *control_data) { // Thêm tham số control_data
    int64_t time = esp_timer_get_time(); // Microseconds
    elapsedTime = (time - timePrev) / 1000000.0; // Seconds
    timePrev = time;

    uint8_t data[6];
    if (i2c_read_bytes(MPU6050_ADDR, 0x43, data, 4) == ESP_OK) {
        Gyr_rawX = ((int16_t)(data[0] << 8) | data[1]) / 32.8 - Gyro_raw_error_x;
        Gyr_rawY = ((int16_t)(data[2] << 8) | data[3]) / 32.8 - Gyro_raw_error_y;
        Gyro_angle_x = Gyr_rawX * elapsedTime;
        Gyro_angle_y = Gyr_rawY * elapsedTime;
    } else {
        ESP_LOGE(TAG, "Failed to read gyro data");
        return;
    }

    if (i2c_read_bytes(MPU6050_ADDR, 0x3B, data, 6) == ESP_OK) {
        Acc_rawX = ((int16_t)(data[0] << 8) | data[1]) / 4096.0;
        Acc_rawY = ((int16_t)(data[2] << 8) | data[3]) / 4096.0;
        Acc_rawZ = ((int16_t)(data[4] << 8) | data[5]) / 4096.0;
        Acc_angleX = (atan2(Acc_rawY, sqrt(Acc_rawX * Acc_rawX + Acc_rawZ * Acc_rawZ)) * rad_to_deg) - Acc_raw_error_x;
        Acc_angleY = (atan2(-Acc_rawX, sqrt(Acc_rawY * Acc_rawY + Acc_rawZ * Acc_rawZ)) * rad_to_deg) - Acc_raw_error_y;

        Total_angleX = 0.98 * (Total_angleX + Gyro_angle_x) + 0.02 * Acc_angleX;
        Total_angleY = 0.98 * (Total_angleY + Gyro_angle_y) + 0.02 * Acc_angleY;

    } else {
        ESP_LOGE(TAG, "Failed to read accel data");
        return;
    }
    //ESP_LOGI(TAG, "Gyr_rawX: %.2f, Gyr_rawY: %.2f, Acc_rawX: %.2f, Acc_rawY: %.2f, Acc_rawZ: %.2f", Gyr_rawX, Gyr_rawY, Acc_rawX, Acc_rawY, Acc_rawZ);

    roll_desired_angle = ((control_data->roll) - 1500) / 50.0; // -10 to 10 degrees
    pitch_desired_angle = ((control_data->pitch) - 1500) / 50.0;
    yaw_desired_angle = ((control_data->yaw) - 1500) / 5.0; 
    //ESP_LOGI(TAG, "Desired angles: roll=%.2f, pitch=%.2f", roll_desired_angle, pitch_desired_angle);
    roll_error = Total_angleY - roll_desired_angle;
    pitch_error = Total_angleX - pitch_desired_angle;
    //ESP_LOGI(TAG, "Roll error: %.2f, Pitch error: %.2f", roll_error, pitch_error);

    roll_pid_p = roll_kp * roll_error;
    pitch_pid_p = pitch_kp * pitch_error;

    if (-3 < roll_error && roll_error < 3) roll_pid_i += roll_ki * roll_error;
    if (-3 < pitch_error && pitch_error < 3) pitch_pid_i += pitch_ki * pitch_error;

    roll_pid_d = roll_kd * (roll_error - roll_previous_error) / elapsedTime;
    pitch_pid_d = pitch_kd * (pitch_error - pitch_previous_error) / elapsedTime;

    roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
    pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;

    roll_PID = (roll_PID < -400) ? -400 : (roll_PID > 400 ? 400 : roll_PID);
    pitch_PID = (pitch_PID < -400) ? -400 : (pitch_PID > 400 ? 400 : pitch_PID);
    yaw_PID = yaw_desired_angle; // Directly use desired yaw as control output.
    yaw_PID = (yaw_PID < -200) ? -200 : (yaw_PID > 200 ? 200 : yaw_PID); // Clamp Yaw PID - Adjust limits as needed

    //ESP_LOGI(TAG, "Roll PID: %.2f, Pitch PID: %.2f", roll_PID, pitch_PID);
    float throttle = control_data->throttle;
   // Sửa công thức thành:
    MotorInput1 = throttle + roll_PID + pitch_PID; //- yaw_PID; // phai truoc
    MotorInput2 = throttle + roll_PID - pitch_PID; //+ yaw_PID; // phai sau
    MotorInput3 = throttle - roll_PID - pitch_PID; //- yaw_PID; // trai sau
    MotorInput4 = throttle - roll_PID + pitch_PID; //+ yaw_PID; // trai truoc
    MotorInput1 = (MotorInput1 < 1025) ? PWM_MIN_US : (MotorInput1 > PWM_MAX_US ? PWM_MAX_US : MotorInput1);
    MotorInput2 = (MotorInput2 < 1025) ? PWM_MIN_US : (MotorInput2 > PWM_MAX_US ? PWM_MAX_US : MotorInput2);
    MotorInput3 = (MotorInput3 < 1025) ? PWM_MIN_US : (MotorInput3 > PWM_MAX_US ? PWM_MAX_US : MotorInput3);
    MotorInput4 = (MotorInput4 < 1025) ? PWM_MIN_US : (MotorInput4 > PWM_MAX_US ? PWM_MAX_US : MotorInput4);


    roll_previous_error = roll_error;
    pitch_previous_error = pitch_error;

    ESP_LOGI(TAG, "Control Data - Throttle: %d, Roll: %d, Pitch: %d, Yaw: %d, AUX1: %d", control_data->throttle, control_data->roll, control_data->pitch, control_data->yaw, control_data->AUX1);
    ESP_LOGI(TAG, "IMU Data - GyrX: %.2f, GyrY: %.2f, AccX: %.2f, AccY: %.2f, AccZ: %.2f", Gyr_rawX, Gyr_rawY, Acc_rawX, Acc_rawY, Acc_rawZ);
    ESP_LOGI(TAG, "Angles - TotalX: %.2f, TotalY: %.2f, DesiredRoll: %.2f, DesiredPitch: %.2f", Total_angleX, Total_angleY, roll_desired_angle, pitch_desired_angle);
    ESP_LOGI(TAG, "Errors - Roll: %.2f, Pitch: %.2f", roll_error, pitch_error);
    ESP_LOGI(TAG, "PID Terms - Roll_P: %.2f, Roll_I: %.2f, Roll_D: %.2f, Pitch_P: %.2f, Pitch_I: %.2f, Pitch_D: %.2f", roll_pid_p, roll_pid_i, roll_pid_d, pitch_pid_p, pitch_pid_i, pitch_pid_d);
    ESP_LOGI(TAG, "PID Outputs - Roll_PID: %.2f, Pitch_PID: %.2f", roll_PID, pitch_PID);
    ESP_LOGI(TAG, "Motor Inputs (Raw) - M1: %.2f, M2: %.2f, M3: %.2f, M4: %.2f", MotorInput1, MotorInput2, MotorInput3, MotorInput4);


    if (control_data->AUX1 == 0) {
        roll_PID = 0; // Zero out PID outputs when disarmed
        pitch_PID = 0;
        yaw_PID = 0;
        setMotorSpeed(LEDC_CHANNEL_0, PWM_MIN_US);
        setMotorSpeed(LEDC_CHANNEL_1, PWM_MIN_US);
        setMotorSpeed(LEDC_CHANNEL_2, PWM_MIN_US);
        setMotorSpeed(LEDC_CHANNEL_3, PWM_MIN_US);
        ESP_LOGI(TAG, "Motors Disarmed - PWM set to MIN_US");
    } else {
        setMotorSpeed(LEDC_CHANNEL_0, MotorInput1);
        setMotorSpeed(LEDC_CHANNEL_1, MotorInput2);
        setMotorSpeed(LEDC_CHANNEL_2, MotorInput3);
        setMotorSpeed(LEDC_CHANNEL_3, MotorInput4);
        ESP_LOGI(TAG, "Motors Armed - PWM set to calculated values");
    }
    ESP_LOGI(TAG, "Motor PWM Output - M1: %.2f, M2: %.2f, M3: %.2f, M4: %.2f", MotorInput1, MotorInput2, MotorInput3, MotorInput4);
    ESP_LOGI(TAG, "--------------------------------------------------");
}

// Task điều khiển
static void control_task(void *pvParam) {
    timePrev = esp_timer_get_time();
    joystick_data_t local_data;
    esp_task_wdt_add(NULL);

    vTaskDelay(pdMS_TO_TICKS(1000));

    web_control_data.throttle = PWM_MIN_US; // Đảm bảo throttle khởi đầu ở mức thấp nhất
    ESP_LOGI(TAG, "Initial Throttle set to PWM_MIN_US at task start");

    // Reset PID and angle variables at the start of the control task loop
    Total_angleX = 0;
    Total_angleY = 0;
    roll_pid_i = 0;
    pitch_pid_i = 0;
    roll_previous_error = 0;
    pitch_previous_error = 0;
    ESP_LOGI(TAG, "Control Task Started - PID and Angle variables reset");

    while (1) {
        // Reset PID and angle variables at the beginning of each loop iteration as well (for safety)
        Total_angleX = 0; // Reset angle each loop, consider if this is desired.
        Total_angleY = 0; // Reset angle each loop, consider if this is desired.
        roll_pid_i = 0;  // Reset integral term each loop, may reduce windup, but also PID performance.
        pitch_pid_i = 0; // Reset integral term each loop, may reduce windup, but also PID performance.
        roll_previous_error = 0; // Reset previous error each loop, might not be ideal for derivative term.
        pitch_previous_error = 0; // Reset previous error each loop, might not be ideal for derivative term.
        yaw_PID = 0;

        ESP_LOGI(TAG, "Control Loop Iteration - PID and Angle variables reset at loop start");


        xSemaphoreTake(web_data_mutex, portMAX_DELAY);
        local_data = web_control_data;
        xSemaphoreGive(web_data_mutex);

        if (local_data.AUX1 == 1) { // Kiểm tra AUX1 để arm/disarm
            update_motor(&local_data); // Truyền địa chỉ của local_data
        } else {
            Total_angleX = 0;
            Total_angleY = 0;
            roll_pid_i = 0;
            pitch_pid_i = 0;
            roll_previous_error = 0;
            pitch_previous_error = 0;
            yaw_PID = 0;
            // Disarm - Đặt tốc độ động cơ về MIN hoặc tắt hoàn toàn (nếu muốn tắt PWM)
            setMotorSpeed(LEDC_CHANNEL_0, PWM_MIN_US);
            setMotorSpeed(LEDC_CHANNEL_1, PWM_MIN_US);
            setMotorSpeed(LEDC_CHANNEL_2, PWM_MIN_US);
            setMotorSpeed(LEDC_CHANNEL_3, PWM_MIN_US);
            ESP_LOGI(TAG, "Motors Disarmed via AUX1=0 - PWM set to MIN_US"); // Thêm log khi disarm
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(40)); // Giảm delay để phản hồi nhanh hơn
        taskYIELD();//
    }
}


void app_main(void) {
    esp_err_t ret = nvs_flash_erase();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NVS partition erased");
    }
    web_data_mutex = xSemaphoreCreateMutex();

    // Initialize PID and angle variables in app_main
    Total_angleX = 0;
    Total_angleY = 0;
    roll_pid_i = 0;
    pitch_pid_i = 0;
    roll_previous_error = 0;
    pitch_previous_error = 0;
    ESP_LOGI(TAG, "App Main Started - PID and Angle variables initialized");


    ESP_ERROR_CHECK(nvs_flash_init());
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    i2c_master_init();
    init_wifi();

    start_webserver();

    ESP_ERROR_CHECK(i2c_write_byte(MPU6050_ADDR, 0x6B, 0x00)); // Reset MPU6050
    ESP_ERROR_CHECK(i2c_write_byte(MPU6050_ADDR, 0x1B, 0x10)); // Gyro 1000dps
    ESP_ERROR_CHECK(i2c_write_byte(MPU6050_ADDR, 0x1C, 0x10)); // Accel +/-8g
    ESP_LOGI(TAG, "MPU6050 Initialized");

    // Tính lỗi ban đầu cho gyro
    if (!gyro_error) {
        ESP_LOGI(TAG, "Starting Gyro Calibration");
        for (int i = 0; i < 200; i++) {
            uint8_t data[4];
            if (i2c_read_bytes(MPU6050_ADDR, 0x43, data, 4) == ESP_OK) {
                Gyr_rawX = ((int16_t)(data[0] << 8) | data[1]) / 32.8;
                Gyr_rawY = ((int16_t)(data[2] << 8) | data[3]) / 32.8;
                Gyro_raw_error_x += Gyr_rawX;
                Gyro_raw_error_y += Gyr_rawY;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        Gyro_raw_error_x /= 200;
        Gyro_raw_error_y /= 200;
        gyro_error = 1;
        ESP_LOGI(TAG, "Gyro Calibration Complete - ErrorX: %.2f, ErrorY: %.2f", Gyro_raw_error_x, Gyro_raw_error_y);
    }

    // Tính lỗi ban đầu cho accel
    if (!acc_error) {
        ESP_LOGI(TAG, "Starting Accel Calibration");
        for (int i = 0; i < 200; i++) {
            uint8_t data[6];
            if (i2c_read_bytes(MPU6050_ADDR, 0x3B, data, 6) == ESP_OK) {
                Acc_rawX = ((int16_t)(data[0] << 8) | data[1]) / 4096.0;
                Acc_rawY = ((int16_t)(data[2] << 8) | data[3]) / 4096.0;
                Acc_rawZ = ((int16_t)(data[4] << 8) | data[5]) / 4096.0;
                Acc_raw_error_x += atan2(Acc_rawY, sqrt(Acc_rawX * Acc_rawX + Acc_rawZ * Acc_rawZ)) * rad_to_deg;
                Acc_raw_error_y += atan2(-Acc_rawX, sqrt(Acc_rawY * Acc_rawY + Acc_rawZ * Acc_rawZ)) * rad_to_deg;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        Acc_raw_error_x /= 200;
        Acc_raw_error_y /= 200;
        acc_error = 1;
        ESP_LOGI(TAG, "Accel Calibration Complete - ErrorX: %.2f, ErrorY: %.2f", Acc_raw_error_x, Acc_raw_error_y);
    }
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
    

    xTaskCreate(control_task, "control_loop", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "Control Task Created");
    ESP_LOGI(TAG, "App Main Initialization Complete");
}