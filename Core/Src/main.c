/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "uat_freertos.h" // your parser header
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void App_Task(void *pvParameters);
/* USER CODE END PFP */

// /* Private user code ---------------------------------------------------------*/
// /* USER CODE BEGIN 0 */

/* creg_handler: called whenever a line beginning with "+CREG" is received */
void creg_handler(const char *args)
{
  // args might be e.g. ": 1" or ": 0,1"
  printf("[%lu] >>> Network registration URC: %s", HAL_GetTick(), args);
  if (strcmp(args, ": 1,1") == 0)
  {
    printf("[%lu] >>> Network EPS registration successful", HAL_GetTick());
  }
}

/* creg_handler: called whenever a line beginning with "+CREG" is received */
void cgreg_handler(const char *args)
{
  // args might be e.g. ": 1" or ": 0,1"
  printf("[%lu] >>> Network registration URC: %s", HAL_GetTick(), args);
  if (strcmp(args, ": 1") == 0)
  {
    printf("[%lu] >>> Network GPRS registration successful!!", HAL_GetTick());
  }
}

// This will be called whenever a line “OK\r\n” arrives.
void ok_handler(const char *args)
{
  // args points just past the “OK” in your buffer.
  // Since the default terminator is “\r\n”, args will typically
  // be just “\r\n” (or the empty string if you trimmed CRLF yourself).
  printf("[%lu] >>> Got OK response%s", HAL_GetTick(), args);
}
// This will be called whenever a line “OK\r\n” arrives.
void error_handler(const char *args)
{
  printf("[%lu] >>> Got OK response%s", HAL_GetTick(), args);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize uAT parser on huart3
  uAT_Result_t result = uAT_Init(&huart2);
  if (result != UAT_OK)
  {
    // handle init error (blink LED, etc.)
    printf("uAT parser initialization failed with error code: %d\n", result);
    Error_Handler();
  }
  printf("uAT parser initialized\n");

  result = uAT_RegisterURC("+CREG", creg_handler);
  if (result != UAT_OK)
  {
    printf("Failed to register +CREG handler: %d\n", result);
  }
  result = uAT_RegisterURC("+CGREG", cgreg_handler);
  if (result != UAT_OK)
  {
    printf("Failed to register +CGREG handler: %d\n", result);
  }

  result = uAT_RegisterCommand("OK", ok_handler);
  if (result != UAT_OK) {
    printf("Failed to register OK handler: %d\n", result);
  }
  result = uAT_RegisterCommand("ERROR", error_handler);
  if (result != UAT_OK)
  {
    printf("Failed to register ERROR handler: %d\n", result);
  }

  // retval = ATC_SendReceive(&hAtc, "ATI\r\n", 1000, NULL, 1000, 2, "\r\nOK\r\n", "\r\nERROR\r\n");
  // printf("ATC_SendReceive returned %d\n", retval);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  // MX_FREERTOS_Init();
  // Create the uAT parser task
  xTaskCreate(uAT_Task,
              "uAT_Task",
              512, // stack depth in words
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);

  // (Optional) your other application tasks
  xTaskCreate(App_Task,
              "AppTask",
              512,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// printf implementation for UART3.
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

// printf implementation for UART3.
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 0xFFFF);
  return len;
}

// scanf implementation for UART3. (not used in this code)
int _read(int file, char *ptr, int len)
{
  (void)file;
  (void)len;
  int ch = 0;
  HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  if (ch == 13)
  {
    ch = 10;
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  }
  else if (ch == 8)
  {
    ch = 0x30;
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  }

  *ptr = ch;

  return 1;
}

// Example application task
static void App_Task(void *pvParameters)
{
  (void)pvParameters; // ignore unused parameter
  printf("Application task started\r\n");
  char myBuff[1024];
  uAT_Result_t result;
  
  for (;;)
  {
    // e.g. periodically send an AT command
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
    
    printf("[%lu] sending 'AT!GSTATUS?' command\r\n", HAL_GetTick());
    result = uAT_SendCommand("AT!GSTATUS?");
    if (result != UAT_OK) {
      printf("Failed to send AT!GSTATUS? command: %d\r\n", result);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    if (uAT_UnregisterCommand("OK") != UAT_OK)
    {
      printf("Failed to unregister command\r\n");
    }

    result = uAT_SendReceive("ATI", "OK", myBuff, sizeof(myBuff), pdMS_TO_TICKS(1000));
    if (result != UAT_OK) {
      printf("Failed to uAT_SendReceive command: %d\r\n", result);
    } else {
      printf("[%lu] received: \n\n%s\r\n", HAL_GetTick(), myBuff);
    }
    
    result = uAT_RegisterCommand("OK", ok_handler);
    if (result != UAT_OK) {
      printf("Failed to re-register OK handler: %d\r\n", result);
    }
    
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
