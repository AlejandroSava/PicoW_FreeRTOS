#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"

#define SERVO_PIN 2  // Use any PWM-capable GPIO

// Adjust these values if your servo doesn’t fully sweep
#define SERVO_MIN_US 500   // Minimum pulse width in microseconds (~0°)
#define SERVO_MAX_US 2500  // Maximum pulse width in microseconds (~180°)
#define PWM_FREQ_HZ 50     // 50 Hz for SG90 = 20ms period

// Convert angle to PWM level (ticks in wrap cycle)
uint16_t angle_to_level(uint wrap, int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    float us = SERVO_MIN_US + (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    float duty = us / (1000000.0f / PWM_FREQ_HZ); // pulse_us / 20ms
    return (uint16_t)(wrap * duty);
}

void vServoTask(void *pvParameters) {
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    uint chan = pwm_gpio_to_channel(SERVO_PIN);
    const uint wrap = 20000; // 20 ms period at 1 MHz

    while (true) {
        for (int angle = 0; angle <= 180; angle += 1) {
            pwm_set_chan_level(slice, chan, angle_to_level(wrap, angle));
            printf("Angle: %d°\n", angle);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        for (int angle = 180; angle >= 0; angle -= 1) {
            pwm_set_chan_level(slice, chan, angle_to_level(wrap, angle));
            printf("Angle: %d°\n", angle);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

int main() {
    stdio_init_all();

    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);     // 125 MHz / 125 = 1 MHz
    pwm_config_set_wrap(&cfg, 20000);        // For 50 Hz (20ms)
    pwm_init(slice, &cfg, true);             // Enable PWM

    xTaskCreate(vServoTask, "servo", 1024, NULL, 1, NULL);
    vTaskStartScheduler();

    while (true);
}
