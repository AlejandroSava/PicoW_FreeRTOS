#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// Define pins
#define TRIG_PIN 2
#define ECHO_PIN 3

// Initialize GPIOs
void init_ultrasonic() {
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);  // Ensure TRIG starts LOW

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

// Function to measure distance in cm
float measure_distance_cm() {
    // Send 10us HIGH pulse on TRIG
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    // Wait for ECHO to go HIGH
    absolute_time_t start_time = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 0) {
        if (absolute_time_diff_us(start_time, get_absolute_time()) > 100000)
            return -1; // Timeout
    }
    absolute_time_t echo_start = get_absolute_time();

    // Wait for ECHO to go LOW
    while (gpio_get(ECHO_PIN) == 1);
    absolute_time_t echo_end = get_absolute_time();

    // Calculate pulse duration
    int64_t duration_us = absolute_time_diff_us(echo_start, echo_end);

    // Convert to cm (speed of sound ≈ 343 m/s)
    return duration_us / 58.0f;
}

// FreeRTOS task to read distance
void vUltrasonicTask(void *pvParameters) {
    while (1) {
        float distance = measure_distance_cm();

        if (distance > 0)
            printf("Distance: %.2f cm\n", distance);
        else
            printf("Timeout or Error Reading Distance\n");

        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms
    }
}

// Main function
int main() {
    stdio_init_all();       // Initialize USB serial
    init_ultrasonic();      // Configure TRIG and ECHO pins

    // Create task for ultrasonic sensor
    xTaskCreate(vUltrasonicTask, "Ultrasonic", 1024, NULL, 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    while (1); // Infinite loop (should not reach)
}
