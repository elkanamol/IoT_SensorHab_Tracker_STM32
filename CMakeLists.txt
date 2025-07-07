cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME nucleo_f756_uart_DMA_ATC_cmd_rc76)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Drivers/uAT/Src/uat_freertos.c
    Drivers/uAT/Src/uat_parser.c
    Drivers/uAT/Src/rc76xx_mqtt.c
    Drivers/BME280/Src/bme280_porting.c
    Drivers/BME280/Src/bme280.c
    # Core/Src/w25qxx_hal.c
    Core/Src/datalogger.c
    Core/Src/bme280_tasks.c
    Core/Src/mpu6050_task.c
    Core/Src/sensor_conversions.c    
    Drivers/w25qxx/Src/driver_w25qxx_interface.c
    Drivers/w25qxx/Src/driver_w25qxx.c
    Drivers/w25qxx/Src/driver_w25qxx_advance.c
    Drivers/w25qxx/Src/driver_w25qxx_basic.c
    # If your code still requires a HAL adapter for w25qxx, add it here, e.g.:
    # Drivers/w25qxx/Src/w25qxx_hal_adapter.c
    Drivers/MPU6050/Src/driver_mpu6050_interface.c
    Drivers/MPU6050/Src/driver_mpu6050.c
    Drivers/MPU6050/Src/driver_mpu6050_basic.c
    Drivers/lwgps/Src/lwgps.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
    Drivers/uAT/Inc
    Drivers/BME280/Inc
    Drivers/w25qxx/Inc
    Drivers/MPU6050/Inc
    Drivers/lwgps/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m  # Math library for sin, cos, sqrt, etc.
    # Add user defined libraries
)

# Linker flags for FPU and floating point printf
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    # -mfpu=fpv5-sp-d16
    # -mfloat-abi=hard
    -u _printf_float
    # -u _scanf_float
    -Wl,--gc-sections        # Remove unused sections
    -Wl,--print-memory-usage # Show memory usage after linking
)