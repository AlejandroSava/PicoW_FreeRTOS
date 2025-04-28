#include <stdio.h>
#include <string.h>

// ---- Raspberry Pico Libraries ----
#include "pico/stdlib.h"         // Basic SDK functions (GPIO, UART, etc.)
#include "pico/cyw43_arch.h"      // Wi-Fi control for the Pico W
#include "hardware/pwm.h"         // PWM control for servo motor
#include "hardware/gpio.h"        // GPIO handling
#include "pico/multicore.h"       // Multicore (core0/core1) management

// ---- FreeRTOS Libraries ----
#include "FreeRTOS.h"             // FreeRTOS core
#include "task.h"                 // Task management
#include "queue.h"                // Queue management

// ---- Networking (TCP/IP) Libraries ----
#include "lwip/tcp.h"             // Lightweight TCP/IP stack
#include "pico/time.h"            // Time functions

// ---- Pin Definitions ----
#define TRIG_PIN 4                // GPIO pin connected to HC-SR04 trigger
#define ECHO_PIN 5                // GPIO pin connected to HC-SR04 echo
#define SERVO_PIN 2               // GPIO pin connected to SG90 servo signal

// ---- Servo PWM Parameters ----
#define SERVO_MIN_US 500          // Pulse width for 0 degrees
#define SERVO_MAX_US 2500         // Pulse width for 180 degrees
#define PWM_FREQ_HZ 50            // 50 Hz = 20ms period (standard for servos)

// ---- WiFi Configurations ----
#define WIFI_SSID     "AlexWifi"
#define WIFI_PASSWORD "Alex1234"
#define SERVER_IP     "192.168.0.104" // IP address of the PC/server
#define SERVER_PORT   1234            // TCP port number

// ---- Data Structure to Hold Angle + Distance ----
typedef struct {
    int angle;
    float distance;
} radar_data_t;

// ---- Global Variables ----
QueueHandle_t xRadarQueue;         // FreeRTOS Queue to send radar data
static struct tcp_pcb *client_pcb = NULL;  // TCP connection control block
static bool data_sent = true;       // Flag to know if last TCP packet was acknowledged

// ---- HC-SR04: Measure Distance ----
float measure_distance_cm() {
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);                  // 10 microseconds pulse
    gpio_put(TRIG_PIN, 0);

    absolute_time_t start_time = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 0) {
        if (absolute_time_diff_us(start_time, get_absolute_time()) > 30000)
            return -1;  // Timeout if no echo
    }

    absolute_time_t echo_start = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 1);
    absolute_time_t echo_end = get_absolute_time();

    int64_t duration = absolute_time_diff_us(echo_start, echo_end);
    return duration / 58.0f; // Convert pulse time to distance (in cm)
}

// ---- PWM: Convert Angle to PWM Pulse Width ----
uint16_t angle_to_level(uint wrap, int angle) {
    float us = SERVO_MIN_US + (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    float duty = us / (1000000.0f / PWM_FREQ_HZ);
    return (uint16_t)(wrap * duty); // Scale into PWM wrap
}

// ---- Task: Servo Move + Ultrasonic Scan ----
void vRadarScanTask(void *pvParameters) {
    const uint wrap = 20000; // 20ms period
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    uint chan = pwm_gpio_to_channel(SERVO_PIN);

    while (1) {
        for (int angle = 0; angle <= 180; angle += 10) {
            pwm_set_chan_level(slice, chan, angle_to_level(wrap, angle)); // Move servo
            vTaskDelay(pdMS_TO_TICKS(400));                               // Wait for servo to reach
            float dist = measure_distance_cm();                          // Measure distance
            vTaskDelay(pdMS_TO_TICKS(200));                               // Stabilization delay

            if (dist > 0 && dist < 400) {  // Filter invalid reads
                radar_data_t data = {.angle = angle, .distance = dist};
                xQueueSend(xRadarQueue, &data, 0); // Send data to queue
                printf("Angle: %d | Distance: %.2f cm\n", angle, dist);
            }
        }
    }
}

// ---- TCP Callback: Data Sent Acknowledgement ----
err_t tcp_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    data_sent = true; // Mark that previous data was sent
    return ERR_OK;
}

// ---- TCP Callback: Connection Established ----
err_t tcp_connected_cb(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        printf("Connection failed: %d\n", err);
        return err;
    }
    tcp_sent(tpcb, tcp_sent_cb); // Set sent callback
    printf("TCP Connected.\n");
    return ERR_OK;
}

// ---- TCP: Connect to Server ----
void connect_to_server() {
    ip_addr_t remote_ip;
    ip4addr_aton(SERVER_IP, &remote_ip);  // Convert IP string to IP object

    client_pcb = tcp_new();               // Create new TCP connection
    if (!client_pcb) {
        printf("Failed to create TCP PCB\n");
        return;
    }
    tcp_connect(client_pcb, &remote_ip, SERVER_PORT, tcp_connected_cb); // Connect to server
}

// ---- Task: Send Radar Data over TCP ----
void vTcpSenderTask(void *pvParameters) {
    // Initialize Wi-Fi hardware
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
        printf("Wi-Fi init failed\n");
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode(); // Set as Wi-Fi station
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Wi-Fi connection failed\n");
        vTaskDelete(NULL);
    }

    printf("Wi-Fi connected\n");
    connect_to_server();

    while (1) {
        radar_data_t data;
        if (xQueueReceive(xRadarQueue, &data, portMAX_DELAY)) {  // Wait for data
            if (client_pcb && data_sent) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Angle: %d, Distance: %.2f cm\n", data.angle, data.distance);
                err_t err = tcp_write(client_pcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY); // Write TCP data
                if (err == ERR_OK) {
                    tcp_output(client_pcb);   // Force data to be sent
                    data_sent = false;        // Wait until acknowledged
                } else {
                    printf("tcp_write error: %d\n", err);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(150)); // Small delay
    }
}

// ---- Core 1 Entry Function ----
void core1_entry() {
    xTaskCreate(vTcpSenderTask, "WiFiSender", 4096, NULL, 1, NULL); // Create TCP Sender task on core 1
    vTaskStartScheduler(); // Start FreeRTOS scheduler for core 1
}

// ---- MAIN FUNCTION ----
int main() {
    stdio_init_all();  // Initialize USB serial

    // Setup GPIOs
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // Setup PWM for Servo Motor
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f); // Set PWM clock
    pwm_config_set_wrap(&cfg, 20000);    // 20ms period
    pwm_init(slice, &cfg, true);         // Enable PWM

    // Create Queue
    xRadarQueue = xQueueCreate(10, sizeof(radar_data_t));

    // Create tasks and launch cores
    xTaskCreate(vRadarScanTask, "RadarScan", 1024, NULL, 1, NULL);  // Run sensor/servo on core 0
    multicore_launch_core1(core1_entry);                           // Run Wi-Fi TCP on core 1
    vTaskStartScheduler();                                         // Start FreeRTOS on core 0

    while (1); // Should never reach
}
