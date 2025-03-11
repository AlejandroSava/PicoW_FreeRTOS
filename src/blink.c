#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h" 
#include "FreeRTOS.h"
#include "task.h"

#define LED_PIN 2  // External LED on GPIO 2

void vBlinkTask(void *pvParameters) {
    // Initialize external LED (GPIO 2)
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (1) {
        // Blink External LED
        gpio_put(LED_PIN, 1);
        // Blink Onboard LED
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);  // Turn Onboard LED OFF
        vTaskDelay(pdMS_TO_TICKS(200));

        gpio_put(LED_PIN, 0);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);  // Turn Onboard LED ON
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

int main() {
    stdio_init_all();

    // Initialize WiFi driver (needed for CYW43 onboard LED)
    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }

    // Create FreeRTOS LED Task
    xTaskCreate(vBlinkTask, "Blink Task", 512, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1);  // Should never reach here
}
