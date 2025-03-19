#include <stdio.h>  // Standard C library for input/output
#include "pico/stdlib.h"  // Standard Pico SDK functions (GPIO, UART, etc.)
#include "pico/cyw43_arch.h"  // Wi-Fi driver for CYW43 chip (includes onboard LED control)
#include "FreeRTOS.h"  // FreeRTOS core library
#include "task.h"  // FreeRTOS task management functions

#define Semaphore_Green_1 13
#define Semaphore_Yellow_1 14
#define Semaphore_Red_1 15

#define Green_Time 8000
#define Yellow_Time 2000
#define Red_Time 10000

#define Semaphore_Green_2 18
#define Semaphore_Yellow_2 17
#define Semaphore_Red_2 16


void init_gpio() {
    gpio_init(Semaphore_Green_1);  // Initialize GPIO
    gpio_init(Semaphore_Yellow_1);  // Initialize GPIO
    gpio_init(Semaphore_Red_1);  // Initialize GPIO
    gpio_init(Semaphore_Green_2);  // Initialize GPIO
    gpio_init(Semaphore_Yellow_2);  // Initialize GPIO
    gpio_init(Semaphore_Red_2);  // Initialize GPIO

    gpio_set_dir(Semaphore_Green_1, GPIO_OUT); // Set as output
    gpio_set_dir(Semaphore_Yellow_1, GPIO_OUT); // Set as output
    gpio_set_dir(Semaphore_Red_1, GPIO_OUT); // Set as output
    gpio_set_dir(Semaphore_Green_2, GPIO_OUT); // Set as output
    gpio_set_dir(Semaphore_Yellow_2, GPIO_OUT); // Set as output
    gpio_set_dir(Semaphore_Red_2, GPIO_OUT); // Set as output

}

void vSemaphore_1(void *pvParameters) {
    

    while (1) {  // Infinite loop
        gpio_put(Semaphore_Green_1, 1);  // Turn LED ON
        gpio_put(Semaphore_Yellow_1, 0);  // Turn LED ON
        gpio_put(Semaphore_Red_1, 0);  // Turn LED ON
        vTaskDelay(pdMS_TO_TICKS(Green_Time));

        gpio_put(Semaphore_Green_1, 0);  // Turn LED ON
        gpio_put(Semaphore_Yellow_1, 1);  // Turn LED ON
        gpio_put(Semaphore_Red_1, 0);  // Turn LED ON
        vTaskDelay(pdMS_TO_TICKS(Yellow_Time));


        gpio_put(Semaphore_Green_1, 0);  // Turn LED ON
        gpio_put(Semaphore_Yellow_1, 0);  // Turn LED ON
        gpio_put(Semaphore_Red_1, 1);  // Turn LED ON
        vTaskDelay(pdMS_TO_TICKS(Red_Time));
        
    }
}

void vSemaphore_2(void *pvParameters) {
    

    while (1) {  // Infinite loop
        
        gpio_put(Semaphore_Green_2, 0);  // Turn LED ON
        gpio_put(Semaphore_Yellow_2, 0);  // Turn LED ON
        gpio_put(Semaphore_Red_2, 1);  // Turn LED ON
        vTaskDelay(pdMS_TO_TICKS(Red_Time));

        gpio_put(Semaphore_Green_2, 1);  // Turn LED ON
        gpio_put(Semaphore_Yellow_2, 0);  // Turn LED ON
        gpio_put(Semaphore_Red_2, 0);  // Turn LED ON
        vTaskDelay(pdMS_TO_TICKS(Green_Time));

        gpio_put(Semaphore_Green_2, 0);  // Turn LED ON
        gpio_put(Semaphore_Yellow_2, 1);  // Turn LED ON
        gpio_put(Semaphore_Red_2, 0);  // Turn LED ON
        vTaskDelay(pdMS_TO_TICKS(Yellow_Time));
        
    }
}




int main() {
    stdio_init_all();  // Initialize standard input/output for debugging

    init_gpio();
    // Create a FreeRTOS task for LED blinking
    xTaskCreate(vSemaphore_1, "Semaphore 1", 512, NULL, 1, NULL);
    xTaskCreate(vSemaphore_2, "Semaphore 1", 512, NULL, 2, NULL);
    // Task function, Name of the task, Stack size , Pointer to parameters passed to task, Task priority , Task handle
    
    // Start the FreeRTOS scheduler (manages all tasks)
    vTaskStartScheduler();

    while (1);  // Infinite loop (should never reach this point), Prevents main() from exiting after FreeRTOS starts.
}
