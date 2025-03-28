#include <inttypes.h>
#include <string.h>
#include "esp_now.h"
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
    uint16_t throttle;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
    uint16_t AUX1;
} joystick_data_t;

joystick_data_t web_control_data = {0};
// Định nghĩa chân PWM cho ESC điều khiển động cơ
#define ESC_MOTOR_1 14
#define ESC_MOTOR_2 19
#define ESC_MOTOR_3 25
#define ESC_MOTOR_4 15
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
// #define LEDC_CHANNEL_1  LEDC_CHANNEL_0
// #define LEDC_CHANNEL_2  LEDC_CHANNEL_1
// #define LEDC_CHANNEL_3  LEDC_CHANNEL_2
// #define LEDC_CHANNEL_4  LEDC_CHANNEL_3
#define LEDC_FREQ       50  // Sửa tần số từ 400Hz → 50Hz (chuẩn cho ESC)
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT  // Giữ nguyên 10-bit
#define PWM_MIN_DUTY   (1023 * 5 / 100)    // 51 (~1000µs)
#define PWM_MAX_DUTY   (1023 * 10 / 100)   // 102 (~2000µs)


void setMotorSpeed(ledc_channel_t channel, float input) {
    // Tính duty cycle cho 10-bit (0-1023)
    uint32_t duty = (uint32_t)(
        (input - 1000) * (PWM_MAX_DUTY - PWM_MIN_DUTY) / 1000 + PWM_MIN_DUTY
    );
   // ESP_LOGI(TAG, "Channel %d: mapped_input=%dµs, duty=%"PRIu32, channel, (int)mapped_input, duty);
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

    sscanf(content, "{\"roll\":%hu,\"pitch\":%hu,\"yaw\":%hu,\"throttle\":%hu,\"AUX1\":%hu}", 
           &web_control_data.roll, &web_control_data.pitch, 
           &web_control_data.yaw, &web_control_data.throttle, &web_control_data.AUX1);
   
    ESP_LOGI(TAG, "Web Control: Roll=%.2f, Pitch=%.2f, Yaw=%.2f, Throttle=%.2f, AUX1=%.2f",
         (double)web_control_data.roll, (double)web_control_data.pitch, 
         (double)web_control_data.yaw, (double)web_control_data.throttle, (double)web_control_data.AUX1);
    setMotorSpeed(LEDC_CHANNEL_0, web_control_data.throttle);
    setMotorSpeed(LEDC_CHANNEL_1, web_control_data.throttle);
    setMotorSpeed(LEDC_CHANNEL_2, web_control_data.throttle);
    setMotorSpeed(LEDC_CHANNEL_3, web_control_data.throttle);
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

// void update_motor() {
//     if (web_control_data.AUX1 == 1) {
//         int throttle_pwm = PWM_MIN_DUTY + (web_control_data.throttle * (PWM_MAX_DUTY - PWM_MIN_DUTY) / 255);
//         int motor1_pwm = throttle_pwm + web_control_data.pitch - web_control_data.roll - web_control_data.yaw;
//         int motor2_pwm = throttle_pwm + web_control_data.pitch + web_control_data.roll + web_control_data.yaw;
//         int motor3_pwm = throttle_pwm - web_control_data.pitch - web_control_data.roll + web_control_data.yaw;
//         int motor4_pwm = throttle_pwm - web_control_data.pitch + web_control_data.roll - web_control_data.yaw;

//         // Giới hạn giá trị PWM trong khoảng cho phép
//         motor1_pwm = fmax(PWM_MIN_DUTY, fmin(PWM_MAX_DUTY, motor1_pwm));        
//         motor2_pwm = fmax(PWM_MIN_DUTY, fmin(PWM_MAX_DUTY, motor2_pwm));
//         motor3_pwm = fmax(PWM_MIN_DUTY, fmin(PWM_MAX_DUTY, motor3_pwm));
//         motor4_pwm = fmax(PWM_MIN_DUTY, fmin(PWM_MAX_DUTY, motor4_pwm));

//          // Cập nhật PWM cho động cơ
//          setMotorSpeed(LEDC_CHANNEL_1, motor1_pwm);
//          setMotorSpeed(LEDC_CHANNEL_2, motor2_pwm);
//          setMotorSpeed(LEDC_CHANNEL_3, motor3_pwm);
//          setMotorSpeed(LEDC_CHANNEL_4, motor4_pwm);
//     } else {
//         // Tắt hết động cơ nếu AUX1 == 0
//         setMotorSpeed(LEDC_CHANNEL_1, PWM_MIN_DUTY);
//         setMotorSpeed(LEDC_CHANNEL_2, PWM_MIN_DUTY);
//         setMotorSpeed(LEDC_CHANNEL_3, PWM_MIN_DUTY);
//         setMotorSpeed(LEDC_CHANNEL_4, PWM_MIN_DUTY);
//     }
// }
void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_netif_init();
    esp_event_loop_create_default();
    init_wifi();
    ESP_LOGI("NETWORK", "Waiting for IP address...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    start_webserver();
    ESP_LOGI(TAG, "Web Server started, ready to receive joystick data.");
    configureMotor(ESC_MOTOR_1, LEDC_CHANNEL_0);
    configureMotor(ESC_MOTOR_2, LEDC_CHANNEL_1);
    configureMotor(ESC_MOTOR_3, LEDC_CHANNEL_2);
    configureMotor(ESC_MOTOR_4, LEDC_CHANNEL_3);

    ESP_LOGI(TAG, "Bắt đầu Calibrate ESC...");
    

    while (1) {
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
