# STM32F7 AT Command Parser with FreeRTOS

## Description

This project implements a lightweight, FreeRTOS-friendly AT command parser (`uAT`) for STM32F7 microcontrollers. It provides a robust interface for communicating with modems or other devices that use AT command sets. The implementation supports both DMA and interrupt-driven UART communication, offering efficient handling of asynchronous command responses with minimal CPU overhead.

The parser is designed to work seamlessly with FreeRTOS, utilizing stream buffers for receiving data and semaphores for thread synchronization. It provides a simple callback-based API for handling AT command responses, making it easy to integrate into embedded applications.

## Features

- FreeRTOS-compatible AT command parser
- Support for both DMA and interrupt-driven UART communication
- Asynchronous command handling with callbacks
- Synchronous command-response functionality with timeouts
- Thread-safe implementation using FreeRTOS primitives
- Configurable buffer sizes and command handler capacity
- Efficient line-based parsing with delimiter detection
- Minimal CPU overhead using DMA for data reception
- Support for command registration and unregistration at runtime
- Standardized error handling with detailed error codes

## Getting Started

### Prerequisites

- STM32F7 series microcontroller (tested on Nucleo-F756ZG)
- STM32CubeMX or STM32CubeIDE for project generation
- GCC ARM toolchain (arm-none-eabi-gcc)
- CMake 3.22 or higher
- Visual Studio Code with the following extensions:
 -- CMake Tools
 -- Cortex-Debug
 -- STM32 extension

### Installation

1. Clone this repository:

  ```bash
  git clone https://github.com/yourusername/stm32f7-at-command-parser.git
  cd stm32f7-at-command-parser
  ```

1. Configure your project with CMake:

  ```bash
  mkdir build
  cd build
  cmake ..
  ```

1. Build the project:

  ```bash
  cmake --build .
  ```

1. Flash the binary to your STM32F7 device using ST-Link utility or directly from VS Code with the Cortex-Debug extension.

## Usage

### Initializing the AT Command Parser

```c
#include "uat_freertos.h"

// In your main.c or initialization code:
extern UART_HandleTypeDef huart2;  // Your UART handle

// Initialize the parser
uAT_Result_t result = uAT_Init(&huart2);
if (result != UAT_OK) {
   // Handle initialization error
   printf("uAT parser initialization failed with error code: %d\n", result);
   Error_Handler();
}

// Create the parser task
xTaskCreate(uAT_Task,
           "uAT_Task",
           512,  // Stack depth in words
           NULL,
           tskIDLE_PRIORITY + 1,
           NULL);
```

### Registering Command Handlers

```c
// Define a handler function for network registration notifications
void creg_handler(const char *args) {
   printf("[%lu] >>> Network registration URC: %s", HAL_GetTick(), args);
}

// Define a handler for OK responses
void ok_handler(const char *args) {
   printf("[%lu] >>> Got OK response%s", HAL_GetTick(), args);
}

// Register the handlers
uAT_Result_t result;
result = uAT_RegisterCommand("+CREG", creg_handler);
if (result != UAT_OK) {
   printf("Failed to register +CREG handler: %d\n", result);
}

result = uAT_RegisterCommand("OK", ok_handler);
if (result != UAT_OK) {
   printf("Failed to register OK handler: %d\n", result);
}
```

### Sending AT Commands

```c
// Send a simple AT command (asynchronously)
uAT_Result_t result = uAT_SendCommand("AT");
if (result != UAT_OK) {
   printf("Failed to send AT command: %d\n", result);
}

// Send a query command
result = uAT_SendCommand("AT+CREG?");
if (result != UAT_OK) {
   printf("Failed to send AT+CREG? command: %d\n", result);
}
```

### Synchronous Command-Response

```c
char response_buffer[1024];
uAT_Result_t result;

// Send command and wait for "OK" response with 1 second timeout
result = uAT_SendReceive("ATI", "OK", response_buffer, sizeof(response_buffer), pdMS_TO_TICKS(1000));

if (result == UAT_OK) {
   printf("Command successful, response:\n%s\n", response_buffer);
} else {
   printf("Command failed with error code: %d\n", result);
}
```

### Example Application Task

```c
static void App_Task(void *pvParameters) {
   printf("Application task started\r\n");
   char myBuff[1024];
   uAT_Result_t result;
    
   for (;;) {
       // Periodically send AT commands
       printf("[%lu] sending 'AT' command\r\n", HAL_GetTick());
       result = uAT_SendCommand("AT");
       if (result != UAT_OK) {
           printf("Failed to send AT command: %d\r\n", result);
       }
       vTaskDelay(pdMS_TO_TICKS(1000));
        
       printf("[%lu] sending 'AT+CREG?' command\r\n", HAL_GetTick());
       result = uAT_SendCommand("AT+CREG?");
       if (result != UAT_OK) {
           printf("Failed to send AT+CREG? command: %d\r\n", result);
       }
       vTaskDelay(pdMS_TO_TICKS(1000));
        
       // Example of synchronous command-response
       result = uAT_SendReceive("ATI", "OK", myBuff, sizeof(myBuff), pdMS_TO_TICKS(1000));
       if (result != UAT_OK) {
           printf("Failed to uAT_SendReceive command: %d\r\n", result);
       } else {
           printf("[%lu] received: \n\n%s\r\n", HAL_GetTick(), myBuff);
       }
            
       vTaskDelay(pdMS_TO_TICKS(5000));
   }
}
```

## Testing Framework

This project includes a testing framework based on the CMSIS-DSP test suite approach. The testing framework allows for systematic validation of the AT command parser functionality.

### Setting Up Tests

```c
#include "jtest.h"
#include "all_tests.h"

void debug_init(void)
{
   uint32_t * SHCSR_ptr = (uint32_t *) 0xE000ED24; /* System Handler Control and State Register */
   *SHCSR_ptr |= 0x70000;             /* Enable UsageFault, BusFault, and MemManage fault*/
}

int main(void)
{
   debug_init();

   JTEST_INIT();               /* Initialize test framework. */

   JTEST_GROUP_CALL(all_tests); /* Run all tests. */

   JTEST_ACT_EXIT_FW();        /* Exit test framework.  */
   while (1);                   /* Never return. */
}
```

### Creating Test Cases

To create test cases for the AT command parser, define test functions that validate specific functionality:

```c
void test_at_command_registration(void)
{
   // Test setup
   UART_HandleTypeDef huart;
    
   // Initialize the parser
   uAT_Result_t result = uAT_Init(&huart);
   JTEST_ASSERT_EQUAL(result, UAT_OK);
    
   // Test command registration
   result = uAT_RegisterCommand("TEST", test_handler);
   JTEST_ASSERT_EQUAL(result, UAT_OK);
    
   // Test cleanup
   uAT_Reset();
}
```

### Running Tests

Tests can be run on the target hardware or in a simulated environment. The test framework provides detailed reporting of test results, making it easy to identify and fix issues.

## Development Environment

- **Operating System**: Cross-platform (Windows, Linux, macOS)
- **Programming Language**: C (C11)
- **Frameworks/Libraries**:
- FreeRTOS 10.x
- STM32 HAL Driver
- CMSIS
- **Hardware**:
- STM32F756ZG Nucleo board
- Any STM32F7 series microcontroller with UART and DMA support
- **Development Tools**:
- Visual Studio Code with CMake Tools
- GCC ARM Embedded Toolchain
- STM32CubeMX for initial project generation
- ST-Link utility for flashing

## Workspace Setup (For Developers)

1. Ensure you have the ARM GCC toolchain installed and in your PATH.

1. Install Visual Studio Code with the following extensions:

- CMake Tools
- Cortex-Debug
- C/C++ Extension Pack
- STM32 extension

1. Clone the repository:

  ```bash
  git clone https://github.com/yourusername/stm32f7-at-command-parser.git
  cd stm32f7-at-command-parser
  ```

1. Open the project folder in VS Code:

  ```bash
  code .
  ```

1. Configure the project with CMake:

- Press Ctrl+Shift+P and select "CMake: Configure"
- Select the GCC ARM toolchain

1. Build the project:

- Press Ctrl+Shift+P and select "CMake: Build"

1. Flash to your device using the Cortex-Debug extension:

- Configure your `launch.json` for your specific STM32F7 device
- Press F5 to start debugging

## Error Handling

The AT command parser uses a standardized error handling approach with the `uAT_Result_t` enum:

```c
typedef enum {
   UAT_OK = 0,             // Operation successful
   UAT_ERR_INVALID_ARG,    // Invalid argument provided
   UAT_ERR_BUSY,           // Resource is busy
   UAT_ERR_TIMEOUT,        // Operation timed out
   UAT_ERR_NOT_FOUND,      // Item not found
   UAT_ERR_SEND_FAIL,      // Failed to send data
   UAT_ERR_INIT_FAIL,      // Initialization failed
   UAT_ERR_INT,            // Internal error
   UAT_ERR_RESOURCE        // Resource allocation failed
} uAT_Result_t;
```

All API functions return one of these error codes, allowing for detailed error handling and reporting.

## Contributing

Contributions to improve the AT command parser are welcome. Please feel free to submit a Pull Request.

## License

MIT License

Copyright (c) 2023 Your Name or Organization

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
