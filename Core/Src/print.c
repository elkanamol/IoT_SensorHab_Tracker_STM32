// #include "cmsis_os.h"
#include "usart.h"
// #include "stm32f7xx_hal.h"

#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "print.h"

// #include "FreeRTOS.h"
// #include "semphr.h"

// this is thread safe impllementation of printf and scanf ussing UART3.
// this is works only if using with freeRTOS and UART3. 
// you need to call print_init() before using printf and scanf.

  // replace your uart handle with your uart handle.
  extern UART_HandleTypeDef huart3;

  QueueHandle_t xPrintQueue = NULL;     // Queue for printing messages
  TaskHandle_t xPrintTaskHandle = NULL; // Handle for the print task

#define PRINT_QUEUE_TIMEOUT_TICKS 10

  /**
   * @brief The print task that handles sending print messages to the UART.
   *
   * This task is responsible for dequeuing print messages from the `xPrintQueue` and
   * transmitting the message contents to the UART using `HAL_UART_Transmit()`. The
   * task will block indefinitely waiting for a message to be available in the queue.
   * After transmitting the message, the task will delay for 1 tick to allow other
   * tasks to run.
   *
   * This task is intended to be created by the `print_init()` function and should
   * not be started directly.
   */

  static void vStartPrintTask(void *pvParameters)
  {
      PrintMessage_t message;
      (void)pvParameters;
      for(;;) {
          if(xQueueReceive(xPrintQueue, &message, portMAX_DELAY) == pdTRUE) {
              HAL_UART_Transmit(&huart3, (uint8_t*)message.buffer, message.len, HAL_MAX_DELAY);
          }
          vTaskDelay(1);
      }
  }

  /**
   * @brief Initializes the print subsystem by creating a print queue and a print task.
   *
   * This function sets up the necessary infrastructure for the print subsystem to operate.
   * It creates a queue (`xPrintQueue`) to hold print messages, and a task (`vStartPrintTask`)
   * that will dequeue messages from the queue and transmit them to the UART.
   *
   * The `uxPriority` parameter specifies the priority of the print task. This should be
   * set to an appropriate value based on the requirements of the system.
   *
   * This function should be called during system initialization, before any print operations
   * are performed.
   *
   * @param uxPriority The priority of the print task.
   * @return pdPASS if the print subsystem was initialized successfully, pdFAIL otherwise.
   */
  BaseType_t xPrintInit(UBaseType_t uxPriority)
  {
      if (uxPriority > configMAX_PRIORITIES - 1)
      {
          printf("Error: Invalid priority\n");
          return pdFAIL;
      }
      BaseType_t xReturn = pdPASS;

      xPrintQueue = xQueueCreate(PRINT_QUEUE_SIZE, sizeof(PrintMessage_t));
      if (xPrintQueue == NULL)
      {
        //   printf("Error creating print queue\n");
          return pdFAIL;
      }

      BaseType_t xTaskRetVal = xTaskCreate(vStartPrintTask,
                                           "PrintTask",
                                           configMINIMAL_STACK_SIZE * 8,
                                           NULL,
                                           uxPriority,
                                           &xPrintTaskHandle);
      if (xTaskRetVal != pdPASS)
      {
          printf("Error creating print task\n");
          xReturn = pdFAIL;
      }
      if (xReturn == pdFAIL)
      {
          vQueueDelete(xPrintQueue);
          xPrintQueue = NULL;
      }
      configASSERT(xPrintQueue != NULL);
      return xReturn;
  }

    /**
   * @brief Deinitializes the print subsystem by destroying the print queue and print task.
   *
   * This function cleans up the resources used by the print subsystem. It deletes the print
   * queue (`xPrintQueue`) and the print task (`xPrintTaskHandle`). This function should be
   * called when the print subsystem is no longer needed, such as during system shutdown.
   */
  void print_deinit(void) {
      if (xPrintQueue != NULL) {
          vQueueDelete(xPrintQueue);
          xPrintQueue = NULL;
      }
      if (xPrintTaskHandle != NULL) {
          vTaskDelete(xPrintTaskHandle);
          xPrintTaskHandle = NULL;
      }
  }

  /**
   * @brief Prints an error message and enters an infinite loop.
   *
   * This function is used to print an error message to the console and then enter an infinite loop,
   * effectively halting the program execution. This is typically used for critical errors that
   * the program cannot recover from.
   *
   * @param msg The error message to be printed.
   */
  void print_error(const char * msg)
  {
    //   DEBUG_PRINT("ERROR: %s\n", msg);
    printf("ERROR: %s\n", msg);
      for(;;);
  }

  /**
   * @brief Writes a message to the print queue.
   *
   * This function copies the provided data buffer to a print message structure and sends it to the print queue.
   * The length of the message is limited to `MAX_PRINT_MSG_LEN` to ensure the message fits in the print queue.
   *
   * @param file Unused parameter, required by the signature of the `_write` function.
   * @param ptr Pointer to the data buffer to be printed.
   * @param len Length of the data buffer.
   * @return The number of bytes written to the print queue.
   */
  int _write(int __attribute__((unused)) file, char *ptr, int len)
  {
      PrintMessage_t message;

      if (len > MAX_PRINT_MSG_LEN)
          len = MAX_PRINT_MSG_LEN;

      memcpy(message.buffer, ptr, len);
      message.len = len;

      if (xQueueSend(xPrintQueue, &message, PRINT_QUEUE_TIMEOUT_TICKS) != pdPASS)
      {
          return 0;
      }
      return len;
  }

  /**
   * @brief Writes a single character to the print queue.
   *
   * This function takes a single character, creates a print message structure with the character,
   * and sends it to the print queue. It is used as a low-level output function for printing
   * characters to the console.
   *
   * @param ch The character to be printed.
   * @return The character that was printed.
   */
  int __io_putchar(int ch)
  {
      PrintMessage_t message;
      message.buffer[0] = ch;
      message.len = 1;

      xQueueSend(xPrintQueue, &message, PRINT_QUEUE_TIMEOUT_TICKS);
      return ch;
  }
