#include <stdio.h>
#include "pico/stdlib.h"         // Basic functions (GPIO, UART, etc.)
#include "hardware/pwm.h"        // PWM controller for RP2040
#include "hardware/adc.h"        // ADC controller (for potentiometer)
#include "FreeRTOS.h"            // FreeRTOS core
#include "task.h"                // Task management
#include "queue.h"               // Queue management

// Defined pins
#define SERVO_PIN 2              // PWM pin for SG90 servo
#define ADC_PIN 26               // ADC0 pin (GPIO 26)

// Servo parameters
#define SERVO_MIN_US 500         // Minimum pulse width (0°)
#define SERVO_MAX_US 2500        // Maximum pulse width (180°)
#define PWM_FREQ_HZ 50           // Frequency of 50 Hz (20 ms)

// Queue to send angles
QueueHandle_t xAngleQueue;

// Converts an angle to a PWM level (in ticks)
uint16_t angle_to_level(uint wrap, int angle) {
    float us = SERVO_MIN_US + (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);  // Calculate pulse width in microseconds
    float duty = us / (1000000.0f / PWM_FREQ_HZ);  // Convert to duty cycle
    return (uint16_t)(wrap * duty);  // Convert to PWM level
}

// Task that reads potentiometer and sends angle through the queue
void vReadPotTask(void *pvParameters) {
    while (1) {
        uint16_t raw = adc_read();                     // Read ADC (0–4095)
        int angle = (raw * 180) / 4095;                // Scale to 0–180 degrees
        xQueueSend(xAngleQueue, &angle, 0);            // Send to queue (non-blocking if full)
        vTaskDelay(pdMS_TO_TICKS(50));                 // Wait 50 ms
    }
}

// Task that receives angles and moves the servo
void vServoTask(void *pvParameters) {
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);     // Get PWM slice for the pin
    uint chan = pwm_gpio_to_channel(SERVO_PIN);        // Get PWM channel (A or B)
    const uint wrap = 20000;                           // 20ms → 50Hz at 1 MHz

    while (1) {
        int angle;
        if (xQueueReceive(xAngleQueue, &angle, portMAX_DELAY)) {
            uint16_t level = angle_to_level(wrap, angle);         // Convert angle to PWM level
            pwm_set_chan_level(slice, chan, level);               // Apply PWM signal
            printf("Received angle: %d°\n", angle);               // Print angle to serial
        }
    }
}

int main() {
    stdio_init_all();  // Initialize USB serial output

    // --- Configure PWM for servo ---
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);               // 125 MHz / 125 = 1 MHz
    pwm_config_set_wrap(&cfg, 20000);                  // 1 MHz → 20 ms period
    pwm_init(slice, &cfg, true);                       // Initialize and enable PWM

    // --- Configure ADC ---
    adc_init();                      // Initialize ADC peripheral
    adc_gpio_init(ADC_PIN);         // Set GPIO 26 as ADC input
    adc_select_input(0);            // Use ADC channel 0

    // Create a queue to hold several angle values
    xAngleQueue = xQueueCreate(5, sizeof(int));  // Queue with capacity for 5 angles

    // Create tasks
    xTaskCreate(vReadPotTask, "PotTask", 256, NULL, 1, NULL);
    xTaskCreate(vServoTask, "ServoTask", 512, NULL, 1, NULL);

    vTaskStartScheduler();  // Start FreeRTOS scheduler

    while (1);  // Should never be reached
}
