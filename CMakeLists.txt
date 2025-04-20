cmake_minimum_required(VERSION 3.13)

# Set the board used
set(PICO_BOARD pico_w)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_blink C CXX ASM) # <-- SET THE NAME OF THE PROJECT
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Set config files paths to be used
include(lwip_import.cmake) # lwIP config files
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/configs/FreeRTOS-Kernel" CACHE STRING "Local Config")
include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}") # FreeRTOS config files

# Initialize the SDK
pico_sdk_init()

# Initialize all other external libs & dependencies
include(FreeRTOS_Kernel_import.cmake)


# Find all .c files inside src/
file(GLOB SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

# Loop over each .c file to create a separate executable
foreach(SOURCE_FILE ${SOURCE_FILES})
    # Extract filename without extension (e.g., blink.c → blink)
    get_filename_component(FILE_NAME ${SOURCE_FILE} NAME_WE)

    # Create an individual executable for each .c file
    add_executable(${FILE_NAME} ${SOURCE_FILE})

    # Link dependencies (adjust as needed)
    target_link_libraries(${FILE_NAME}
        pico_stdlib
        hardware_pwm
        FreeRTOS-Kernel-Heap4
        hardware_i2c
        hardware_adc
        LWIP_PORT
        FREERTOS_PORT
        pico_lwip_nosys
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_iperf   
             
        
        #pico_cyw43_arch_lwip_poll
        #pico_wifi_freertos
        
    )

    # Enable USB stdio & disable UART
    pico_enable_stdio_usb(${FILE_NAME} 1)
    pico_enable_stdio_uart(${FILE_NAME} 0)

    # Generate UF2 and other necessary output formats
    pico_add_extra_outputs(${FILE_NAME})
endforeach()

