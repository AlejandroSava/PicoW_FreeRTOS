#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lwip/tcp.h"
#include "pico/time.h"

// ---- PIN DEFINITIONS ----
#define TRIG_PIN 4
#define ECHO_PIN 5
#define SERVO_PIN 2

// ---- SERVO PARAMETERS ----
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define PWM_FREQ_HZ 50

// ---- WIFI CONFIG ----
#define WIFI_SSID     "AlexWifi"
#define WIFI_PASSWORD "Alex1234"
#define SERVER_IP     "192.168.0.104"
#define SERVER_PORT   1234

// ---- STRUCTURE TO HOLD ANGLE + DISTANCE ----
typedef struct {
    int angle;
    float distance;
} radar_data_t;

QueueHandle_t xRadarQueue;
static struct tcp_pcb *client_pcb = NULL;

// ---- ULTRASONIC SENSOR FUNCTION ----
float measure_distance_cm() {
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    absolute_time_t start_time = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 0) {
        if (absolute_time_diff_us(start_time, get_absolute_time()) > 30000)
            return -1; // Timeout
    }
    absolute_time_t echo_start = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 1);
    absolute_time_t echo_end = get_absolute_time();

    int64_t duration = absolute_time_diff_us(echo_start, echo_end);
    return duration / 58.0f;
}

// ---- SERVO CONTROL FUNCTION ----
uint16_t angle_to_level(uint wrap, int angle) {
    float us = SERVO_MIN_US + (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    float duty = us / (1000000.0f / PWM_FREQ_HZ);
    return (uint16_t)(wrap * duty);
}

// ---- TASK: SERVO + ULTRASONIC ----
void vRadarScanTask(void *pvParameters) {
    const uint wrap = 20000;
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    uint chan = pwm_gpio_to_channel(SERVO_PIN);

    while (1) {
        for (int angle = 0; angle <= 180; angle += 10) {
            // Move servo
            pwm_set_chan_level(slice, chan, angle_to_level(wrap, angle));
            vTaskDelay(pdMS_TO_TICKS(400));  // Allow servo to stabilize

            // Retry ultrasonic read up to 3 times
            float dist = -1.0f;
            for (int i = 0; i < 3; i++) {
                dist = measure_distance_cm();
                if (dist > 0) break;
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            radar_data_t data = {.angle = angle, .distance = dist};
            xQueueSend(xRadarQueue, &data, 0);
            printf("Angle: %d | Distance: %.2f cm\n", angle, dist);
        }
    }
}

// ---- TCP SEND CALLBACKS ----
err_t tcp_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    return ERR_OK;
}

err_t tcp_connected_cb(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        printf("Connection failed: %d\n", err);
        return err;
    }
    tcp_sent(tpcb, tcp_sent_cb);
    printf("TCP Connected.\n");
    return ERR_OK;
}

void connect_to_server() {
    ip_addr_t remote_ip;
    ip4addr_aton(SERVER_IP, &remote_ip);

    client_pcb = tcp_new();
    if (!client_pcb) {
        printf("Failed to create TCP PCB\n");
        return;
    }
    tcp_connect(client_pcb, &remote_ip, SERVER_PORT, tcp_connected_cb);
}

// ---- TASK: TCP SEND ----
void vTcpSenderTask(void *pvParameters) {
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
        printf("Wi-Fi init failed\n");
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Wi-Fi connection failed\n");
        vTaskDelete(NULL);
    }
    printf("Wi-Fi connected\n");
    connect_to_server();

    while (1) {
        radar_data_t data;
        if (xQueueReceive(xRadarQueue, &data, portMAX_DELAY)) {
            if (client_pcb) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Angle: %d, Distance: %.2f cm\n", data.angle, data.distance);
                err_t err = tcp_write(client_pcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
                if (err == ERR_OK) tcp_output(client_pcb);
                vTaskDelay(pdMS_TO_TICKS(200)); 
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---- MAIN ----
int main() {
    stdio_init_all();

    // Init GPIOs
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // PWM for servo
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);  // 1MHz
    pwm_config_set_wrap(&cfg, 20000);     // 20ms
    pwm_init(slice, &cfg, true);

    xRadarQueue = xQueueCreate(10, sizeof(radar_data_t));

    xTaskCreate(vRadarScanTask, "RadarScan", 1024, NULL, 1, NULL);
    xTaskCreate(vTcpSenderTask, "WiFiSender", 4096, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1);
}
