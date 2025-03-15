#include <stdio.h>  // Standard C library for input/output
#include "pico/stdlib.h"  // Standard Pico SDK functions (GPIO, UART, etc.)
#include "pico/cyw43_arch.h"  // Wi-Fi driver for CYW43 chip (includes onboard LED control)
#include "FreeRTOS.h"  // FreeRTOS core library
#include "task.h"  // FreeRTOS task management functions

// FreeRTOS task to blink the onboard LED
void vBlinkTask(void *pvParameters) {
    while (1) {
        // Turn the onboard LED ON
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for 1 second
        
        // Turn the onboard LED OFF
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for 1 second
    }
}

int main() {
    stdio_init_all();  // Initialize standard input/output for debugging

    // Initialize the Wi-Fi driver (CYW43 needed for controlling onboard LED)
    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");  // Print error message if initialization fails
        return 1;  // Exit program
    }

    // Create a FreeRTOS task for LED blinking
    xTaskCreate(vBlinkTask, "Blink Task", 512, NULL, 1, NULL);
    // Task function, Name of the task, Stack size , Pointer to parameters passed to task, Task priority , Task handle
    
    // Start the FreeRTOS scheduler (manages all tasks)
    vTaskStartScheduler();

    while (1);  // Infinite loop (should never reach this point)
}
