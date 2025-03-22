/*
    Practice 2. Semaphore Control
    I developed the control of two semaphores using multitasking 
    By: @Alejandro Salinas V.
*/

#include <stdio.h>  // Standard C library for input/output
#include "pico/stdlib.h"  // Standard Pico SDK functions (GPIO, UART, etc.)
#include "pico/cyw43_arch.h"  // Wi-Fi driver for CYW43 chip (includes onboard LED control)
#include "FreeRTOS.h"  // FreeRTOS core library
#include "task.h"  // FreeRTOS task management functions

// Defining GPIO semaphore 1
#define Semaphore_Green_1 13
#define Semaphore_Yellow_1 14
#define Semaphore_Red_1 15

// Defining GPIO semaphore 2
#define Semaphore_Green_2 18
#define Semaphore_Yellow_2 17
#define Semaphore_Red_2 16

// Defining delay timing for lights
#define Green_Time 8000
#define Yellow_Time 2000
#define Red_Time 10000


void init_gpio() {
    // initilize the GPIO
    gpio_init(Semaphore_Green_1);  
    gpio_init(Semaphore_Yellow_1);  
    gpio_init(Semaphore_Red_1);  
    gpio_init(Semaphore_Green_2);  
    gpio_init(Semaphore_Yellow_2); 
    gpio_init(Semaphore_Red_2);  

    // Set as output
    gpio_set_dir(Semaphore_Green_1, GPIO_OUT); 
    gpio_set_dir(Semaphore_Yellow_1, GPIO_OUT); 
    gpio_set_dir(Semaphore_Red_1, GPIO_OUT); 
    gpio_set_dir(Semaphore_Green_2, GPIO_OUT); 
    gpio_set_dir(Semaphore_Yellow_2, GPIO_OUT); 
    gpio_set_dir(Semaphore_Red_2, GPIO_OUT); 

}

void vSemaphore_1(void *pvParameters) {
    
    while (1) {  // Infinite loop
        printf("Semaphore 1: Green Light \n");
        gpio_put(Semaphore_Green_1, 1);  
        gpio_put(Semaphore_Yellow_1, 0);  
        gpio_put(Semaphore_Red_1, 0);  
        vTaskDelay(pdMS_TO_TICKS(Green_Time));

        printf("Semaphore 1: Yellow Light \n");
        gpio_put(Semaphore_Green_1, 0);  
        gpio_put(Semaphore_Yellow_1, 1);  
        gpio_put(Semaphore_Red_1, 0);  
        vTaskDelay(pdMS_TO_TICKS(Yellow_Time));

        printf("Semaphore 1: Red Light \n");
        gpio_put(Semaphore_Green_1, 0);  
        gpio_put(Semaphore_Yellow_1, 0);  
        gpio_put(Semaphore_Red_1, 1);  
        vTaskDelay(pdMS_TO_TICKS(Red_Time));
        
    }
}

void vSemaphore_2(void *pvParameters) {    

    while (1) {  // Infinite loop

        printf("Semaphore 2: Red Light \n");        
        gpio_put(Semaphore_Green_2, 0); 
        gpio_put(Semaphore_Yellow_2, 0);  
        gpio_put(Semaphore_Red_2, 1);  
        vTaskDelay(pdMS_TO_TICKS(Red_Time));

        printf("Semaphore 2: Green Light \n");
        gpio_put(Semaphore_Green_2, 1); 
        gpio_put(Semaphore_Yellow_2, 0);  
        gpio_put(Semaphore_Red_2, 0);  
        vTaskDelay(pdMS_TO_TICKS(Green_Time));

        printf("Semaphore 2: Yellow Light \n");
        gpio_put(Semaphore_Green_2, 0);  
        gpio_put(Semaphore_Yellow_2, 1);  
        gpio_put(Semaphore_Red_2, 0);   
        vTaskDelay(pdMS_TO_TICKS(Yellow_Time));
        
    }
}

int main() {
    stdio_init_all();  // Initialize standard input/output for debugging

    init_gpio();
    printf("Starting the Semaphores \n");
    
    xTaskCreate(vSemaphore_2, "Semaphore 1", 512, NULL, 2, NULL); //higher priority
    xTaskCreate(vSemaphore_1, "Semaphore 1", 512, NULL, 1, NULL); //lower priority
    // Task function, Name of the task, Stack size , Pointer to parameters passed to task, Task priority , Task handle
    
    // Start the FreeRTOS scheduler (manages all tasks)
    vTaskStartScheduler();

    while (1);  // Infinite loop (should never reach this point), Prevents main() from exiting after FreeRTOS starts.
}
