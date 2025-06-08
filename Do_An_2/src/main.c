#include <stdio.h>
#include <string.h>
#include <math.h>
#include "driver/i2c.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define PCA9685_ADDR                0x40

#define SERVO_MIN_PULSE_WIDTH       560
#define SERVO_MAX_PULSE_WIDTH       2800
#define SERVO_FREQUENCY             50

#define UART_PORT_NUM               UART_NUM_2
#define UART_BAUD_RATE              115200
#define UART_RX_PIN                 16
#define UART_TX_PIN                 17
#define UART_BUF_SIZE               1024

#define SERVO1_CHANNEL  0
#define SERVO2_CHANNEL  3
#define SERVO3_CHANNEL  4
#define SERVO4_CHANNEL  7

#define L1 10.0 
#define L2 10.5 
#define L3 15.0  
#define L4 7.0   

static const char *TAG = "ROBOT_ARM";

typedef struct {
    double x, y, z;
} position_t;

esp_err_t pca9685_write(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return err;
}

// Khởi tạo PCA9685
void pca9685_init() {
    pca9685_write(0x00, 0x10); 
    uint8_t prescale_val = (uint8_t)(round(25000000.0 / (4096.0 * SERVO_FREQUENCY)) - 1);
    pca9685_write(0xFE, prescale_val);  
    pca9685_write(0x00, 0x20);  
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Gửi PWM đến kênh
void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t reg = 0x06 + 4 * channel;
    pca9685_write(reg, on & 0xFF);
    pca9685_write(reg + 1, on >> 8);
    pca9685_write(reg + 2, off & 0xFF);
    pca9685_write(reg + 3, off >> 8);
}

// Chuyển đổi góc thành độ rộng xung
void set_servo_angle(uint8_t channel, uint8_t angle) {
    uint16_t pulse_width = SERVO_MIN_PULSE_WIDTH +
        (angle * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH)) / 180;
    uint16_t pwm_val = (pulse_width * 4096) / 20000;
    pca9685_set_pwm(channel, 0, pwm_val);
}

// Chuyển động mượt cho cả 4 servo
void move_all_servos_smooth(float *from_angles, float *to_angles, int duration_ms) {
    const int steps = 100;
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / steps;
        float eased = 0.5 - 0.5 * cos(t * M_PI);
        set_servo_angle(SERVO1_CHANNEL, from_angles[0] + (to_angles[0] - from_angles[0]) * eased);
        set_servo_angle(SERVO2_CHANNEL, from_angles[1] + (to_angles[1] - from_angles[1]) * eased);
        set_servo_angle(SERVO3_CHANNEL, from_angles[2] + (to_angles[2] - from_angles[2]) * eased);
        set_servo_angle(SERVO4_CHANNEL, from_angles[3] + (to_angles[3] - from_angles[3]) * eased);
        vTaskDelay(pdMS_TO_TICKS(duration_ms / steps));
    }
}

// Hàm tính toán góc từ tọa độ
void calculate_angles(position_t pos, float *angle_1, float *angle_2, float *angle_3, float *angle_4) {
    double x = pos.x;
    double y = pos.y;
    double e = 0.1;
    double a, yc, yd1;

    // Tính angle_1 (góc giữa OB và trục hoành)
    double d = sqrt(x * x + y * y);
    if (d <= 1.0) {
        ESP_LOGE(TAG, "Điểm nằm trong hoặc trên đường tròn, không thể tạo tiếp tuyến.");
        *angle_1 = *angle_2 = *angle_3 = *angle_4 = 90; // Giữ nguyên góc nếu lỗi
        return;
    }

    double alpha = atan2(y, x);
    double beta = acos(1.0 / d);

    // Tính tọa độ hai tiếp điểm
    double theta1 = alpha + beta;
    double theta2 = alpha - beta;
    double bx1 = cos(theta1);
    double by1 = sin(theta1);
    double bx2 = cos(theta2);
    double by2 = sin(theta2);

    double cross1 = (-x) * by1 - (-y) * bx1;
    double bx, by;
    if (cross1 < 0) {
        bx = bx1;
        by = by1;
    } else {
        bx = bx2;
        by = by2;
    }

    *angle_1 = atan2(by, bx) * 180.0 / M_PI;
    if (*angle_1 < 0) *angle_1 += 360.0;

    // Tính toán góc cho servo 2, 3, 4
    x = sqrt(d*d - 0.5);
    y = pos.z + L4 - L1;
    for (int i = 0; i < (L2 * 100); i++) {
        a = i / 100.0;
        yc = sqrt(L2 * L2 - a * a);
        if (L3 * L3 - (x - a) * (x - a) < 0) {
            continue;
        }
        yd1 = yc - sqrt(L3 * L3 - (x - a) * (x - a));
        if (fabs(yd1 - y) < e) {
            break;
        }
    }

    // Tính angle_3
    double num_C = a * (x - a) + yc * (y - yc);
    double den_C = sqrt(a * a + yc * yc) * sqrt((x - a) * (x - a) + (y - yc) * (y - yc));
    *angle_3 = acos(num_C / den_C) * 180.0 / M_PI;
    *angle_3 = 180 - *angle_3;

    // Tính angle_4
    double num_D = L3 * (y - yc);
    double den_D = sqrt(L3 * L3) * sqrt((x - a) * (x - a) + (y - yc) * (y - yc));
    *angle_4 = acos(num_D / den_D) * 180.0 / M_PI;

    // Tính angle_2
    *angle_2 = 360 - *angle_3 - *angle_4;
    // Hiệu chỉnh góc
    *angle_1 = *angle_1 - 90;
    *angle_2 = *angle_2 -90;
    *angle_3 = 180 - *angle_3;
    ESP_LOGI(TAG, "Calculated angles: a1=%.2f, a2=%.2f, a3=%.2f, a4=%.2f", *angle_1, *angle_2, *angle_3, *angle_4);
}

// Hàm phân tích chuỗi UART
bool parse_uart_data(char *data, position_t *pos1, position_t *pos2) {
    char *sep = NULL;
    for (char *p = data + 1; *p != '\0'; p++) {
        if (*p == '-' && (p[-1] >= '0' && p[-1] <= '9') && (p[1] >= '0' || p[1] == '-')) {
            sep = p;
        }
    }

    if (sep == NULL) {
        ESP_LOGE(TAG, "Không tìm thấy dấu phân tách giữa 2 tọa độ: %s", data);
        return false;
    }

    // Tách chuỗi thành 2 phần
    *sep = '\0';  // kết thúc phần 1
    char *part1 = data;
    char *part2 = sep + 1;

    float x1, y1, z1, x2, y2, z2;
    if (sscanf(part1, "%f,%f,%f", &x1, &y1, &z1) == 3 &&
        sscanf(part2, "%f,%f,%f", &x2, &y2, &z2) == 3) {
        pos1->x = x1;
        pos1->y = y1;
        pos1->z = z1;
        pos2->x = x2;
        pos2->y = y2;
        pos2->z = z2;
        ESP_LOGI(TAG, "Parsed positions: Pos1(%.2f,%.2f,%.2f), Pos2(%.2f,%.2f,%.2f)",
                 x1, y1, z1, x2, y2, z2);
        return true;
    }

    ESP_LOGE(TAG, "Dữ liệu tọa độ không hợp lệ: %s - %s", part1, part2);
    return false;
}

// Task điều khiển robot arm
void robot_arm_task(void *param) {
    // Khởi tạo UART
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);

    // Đặt trạng thái IDLE ban đầu
    float current_angles[4] = {90, 90, 90, 90};
    set_servo_angle(SERVO1_CHANNEL, 90);
    set_servo_angle(SERVO2_CHANNEL, 90);
    set_servo_angle(SERVO3_CHANNEL, 90);
    set_servo_angle(SERVO4_CHANNEL, 90);
    ESP_LOGI(TAG, "Robot arm in IDLE state (90,90,90,90)");

    char uart_data[64];
    while (1) {
        // Đọc dữ liệu UART
        int len = uart_read_bytes(UART_PORT_NUM, (uint8_t *)uart_data, sizeof(uart_data) - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            uart_data[len] = '\0'; // Thêm ký tự kết thúc chuỗi
            position_t pos1, pos2;
            if (parse_uart_data(uart_data, &pos1, &pos2)) {
                float angles1[4], angles2[4], idle_angles[4] = {90, 90, 90, 90};

                // Tính góc cho vị trí 1
                calculate_angles(pos1, &angles1[0], &angles1[1], &angles1[2], &angles1[3]);
                // Di chuyển đến vị trí 1
                ESP_LOGI(TAG, "Moving to position 1...");
                move_all_servos_smooth(current_angles, angles1, 2000);
                memcpy(current_angles, angles1, sizeof(angles1));
                ESP_LOGI(TAG, "Reached position 1");

                // Chờ 5s
                vTaskDelay(pdMS_TO_TICKS(5000));

                // Tính góc cho vị trí 2
                calculate_angles(pos2, &angles2[0], &angles2[1], &angles2[2], &angles2[3]);
                // Di chuyển đến vị trí 2
                ESP_LOGI(TAG, "Moving to position 2...");
                move_all_servos_smooth(current_angles, angles2, 2000);
                memcpy(current_angles, angles2, sizeof(angles2));
                ESP_LOGI(TAG, "Reached position 2");

                // Chờ 5s
                vTaskDelay(pdMS_TO_TICKS(5000));

                // Quay về trạng thái IDLE
                ESP_LOGI(TAG, "Returning to IDLE state...");
                move_all_servos_smooth(current_angles, idle_angles, 2000);
                memcpy(current_angles, idle_angles, sizeof(idle_angles));
                ESP_LOGI(TAG, "Returned to IDLE state");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void app_main(void) {
    // Cấu hình I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                       I2C_MASTER_RX_BUF_DISABLE, 
                       I2C_MASTER_TX_BUF_DISABLE, 0);

    ESP_LOGI(TAG, "Khởi động PCA9685...");
    pca9685_init();

    // Tạo task điều khiển robot arm
    xTaskCreate(robot_arm_task, "robot_arm", 4096, NULL, 1, NULL);
}