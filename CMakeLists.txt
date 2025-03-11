cmake_minimum_required(VERSION 3.13)

# Set the project name


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

# Adding the main file
add_executable(${PROJECT_NAME} 
    ./src/blink.c        # MAIN FILE TO COMPILE
)

# Pull in common dependencies
target_link_libraries(${PROJECT_NAME}
    pico_stdlib                                 # Core functionality       
    FreeRTOS-Kernel-Heap4                       # FreeRTOS kernel and dynamic heap
    hardware_i2c                                # Hardware I2C
    hardware_adc                                # Hardware ADC
    LWIP_PORT                                   # lwIP config files
    FREERTOS_PORT                               # FreeRTOS config files
    pico_lwip_nosys
    pico_cyw43_arch_lwip_threadsafe_background
)

# Enable print functionality on USB & disable on UART
pico_enable_stdio_usb(${PROJECT_NAME} 1) 
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

# Include necessary directories
target_include_directories(${PROJECT_NAME} 
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/configs/lwip
)
