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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc76xx_mqtt.h"
#include "stdio.h"
#include "uat_freertos.h" // your parser header
#include "string.h"
#include "mqtt_secrets.h"
#include "stdlib.h"

#include "bme280_tasks.h"
// #include "bme280.h"
// #include "bme280_porting.h"
#include "datalogger.h"

#include "w25qxx_hal.h"
#include "driver_w25qxx_interface.h"
#include "driver_w25qxx_basic.h"
#include "mpu6050_task.h"
#include "sensor_conversions.h"

#include "lwgps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Ring buffer configuration (64KB FIFO)
#define FLASH_RING_BUFFER_SIZE (64 * 1024)  // 64KB - can be changed later
#define RING_BUFFER_START_ADDRESS (DATA_START_ADDRESS)
#define RING_BUFFER_END_ADDRESS (RING_BUFFER_START_ADDRESS + FLASH_RING_BUFFER_SIZE)
#define RECORDS_PER_PAGE (W25Q64_PAGE_SIZE / sizeof(BME280_Record_t))

// Queue configuration
#define QUEUE_SEND_TIMEOUT_MS 1000
#define DATA_LOGGER_QUEUE_SIZE 20
#define MAX_QUEUE_WAIT_MS 1000

#define GPS_RX_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
static RC76XX_Handle_t mqttHandle;
// struct bme280_dev bme_device;
// struct bme280_data bme_comp_data;

// lwgps variables
uint8_t flag = 0;
uint8_t gpsRx[GPS_RX_SIZE]; 
lwgps_t hgps;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
// static void App_Task(void *pvParameters);
static void MQTT_Task(void *pvParameters);
void StartBme280Task(void *argument);
void StartW25QTestTask(void *argument);
void StartDataLoggerTask(void *argument);
void vGpsTaskStart(void *argument);

// // Queue interface functions (non-static, called from other tasks)
// BaseType_t DataLogger_QueueBME280Data(struct bme280_data *bme_data);
// BaseType_t DataLogger_QueueSystemEvent(const char *event_text);

// BME280 Task and helper functions
void StartBme280Task(void *argument);

void PrintBufferHex(const uint8_t *buffer, uint32_t size, uint32_t baseAddress);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    flag = 1;
  }
  /* If the huart1 buffer is full, mark the flag. */
}

// DMA completion callbacks
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hspi == &hspi1 && xW25QxxTaskHandle != NULL)
  {
    // Notify the waiting W25QXX task that TX is complete
    vTaskNotifyGiveFromISR(xW25QxxTaskHandle, &xHigherPriorityTaskWoken);

    // Perform context switch if necessary
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hspi == &hspi1 && xW25QxxTaskHandle != NULL)
  {
    // Notify the waiting W25QXX task that RX is complete
    vTaskNotifyGiveFromISR(xW25QxxTaskHandle, &xHigherPriorityTaskWoken);

    // Perform context switch if necessary
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hspi == &hspi1 && xW25QxxTaskHandle != NULL)
  {
    // Notify the waiting W25QXX task that TxRx is complete
    vTaskNotifyGiveFromISR(xW25QxxTaskHandle, &xHigherPriorityTaskWoken);

    // Perform context switch if necessary
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hspi == &hspi1 && xW25QxxTaskHandle != NULL)
  {
    // Notify the waiting task even on error so it doesn't hang
    vTaskNotifyGiveFromISR(xW25QxxTaskHandle, &xHigherPriorityTaskWoken);

    // Perform context switch if necessary
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
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

  /* Init scheduler */
  osKernelInitialize();
  // MX_FREERTOS_Init();

  BaseType_t TaskStatus;
  // Create the uAT parser task
  // Create I2C mutex
  SemaphoreHandle_t g_i2c_mutex = NULL;

  // In main() or initialization function:
  g_i2c_mutex = xSemaphoreCreateMutex();
  configASSERT(g_i2c_mutex != NULL);
  printf("I2C mutex created\n");
  if (xSemaphoreTake(g_i2c_mutex, portMAX_DELAY) != pdTRUE)
  {
    printf("Failed to take I2C mutex\n");
    Error_Handler();
  }
  printf("I2C mutex taken\n");
  if (xSemaphoreGive(g_i2c_mutex) != pdTRUE)
  {
    printf("Failed to give I2C mutex\n");
    Error_Handler();
  }
  printf("I2C mutex given\n");


  // Create uAT parser task
  TaskStatus = xTaskCreate(uAT_Task,
                           "uAT_Task",
                           512, // stack depth in words
                           NULL,
                           tskIDLE_PRIORITY + 1,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  printf("uAT task created\n");

  // // (Optional) your other application tasks
  // xTaskCreate(App_Task,
  //             "AppTask",
  //             512,
  //             NULL,
  //             tskIDLE_PRIORITY + 1,
  //             NULL);

  TaskStatus = xTaskCreate(MQTT_Task,
                           "MQTT_Task",
                           512 * 2,
                           NULL,
                           tskIDLE_PRIORITY + 1,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  printf("MQTT task created\n");

  // Create the BME280 task
  TaskStatus = xTaskCreate(StartBme280Task,
                           "BME280_Task",
                           512, // Adjust stack size as needed
                           (void *)g_i2c_mutex,
                           tskIDLE_PRIORITY + 1,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  printf("BME280 task created\n");
  // Create MPU6050 task
  TaskStatus = xTaskCreate(MPU6050_Task_Start,
                           "MPU6050",
                           512,
                           (void *)g_i2c_mutex,
                           tskIDLE_PRIORITY + 1,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  printf("MPU6050 task created\n");


  // Create the Data Logger task
  TaskStatus = xTaskCreate(StartDataLoggerTask,
                           "DataLogger",
                           2048, // Larger stack for this task
                           NULL,
                           tskIDLE_PRIORITY + 1,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  printf("DataLogger task created\n");

  // Create the GPS task
  TaskStatus = xTaskCreate(vGpsTaskStart,
                           "GPS_Task",
                           512,
                           NULL,
                           tskIDLE_PRIORITY + 1,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  printf("GPS task created\n");

  /* USER CODE END 2 */

  // /* Init scheduler */
  // osKernelInitialize();

  // /* Call init function for freertos objects (in cmsis_os2.c) */
  // MX_FREERTOS_Init();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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

/**
 * @brief  MQTT_Task
 *         Runs the RC76xx MQTT state machine:
 *         Initialize → NetworkAttach → ConfigMQTT → ConnectMQTT → loop Publish/Subscribe
 */
static void MQTT_Task(void *argument)
{
  RC76XX_Result_t res;
  TickType_t delay = pdMS_TO_TICKS(20000);
  SensorData_Combined_t sensor_data;
  SensorData_Combined_Int_t sensor_int_data;

  // Register all URC handlers
  res = RC76XX_RegisterURCHandlers(&mqttHandle);
  if (res != RC76XX_OK)
  {
    printf("Failed to register URC handlers: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  printf("Registered all URC handlers\r\n");

  /* 0) Reset the modem */
  printf("MQTT_Task: Resetting modem...\r\n");
  res = RC76XX_Reset(&mqttHandle);
  if (res != RC76XX_OK)
  {
    printf("Reset failed: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  vTaskDelay(delay);

  /* 1) Initialize modem */
  printf("MQTT_Task: Initializing modem...\r\n");
  res = RC76XX_Initialize(&mqttHandle);
  if (res != RC76XX_OK)
  {
    printf("Init failed: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  printf("Initialized, IMEI=%s\r\n", mqttHandle.imei);

  /* 2) Attach to network & get IP */
  printf("Attaching to network...\r\n");
  res = RC76XX_NetworkAttach(&mqttHandle);
  if (res != RC76XX_OK)
  {
    printf("NetworkAttach failed: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  printf("Network ready, IP=%s\r\n", mqttHandle.ip);

  /* 3) Configure MQTT */
  const char *broker = "mqtt3.thingspeak.com";
  const uint16_t port = 1883;
  const char *clientID = SECRET_MQTT_CLIENT_ID;
  const char *user = SECRET_MQTT_USERNAME;
  const char *pass = SECRET_MQTT_PASSWORD;
  const char *subTopic = "channels/2956054/subscribe";
  const char *pubTopic = "channels/2956054/publish";

  printf("Configuring MQTT %s:%u, ClientID=%s...\r\n", broker, port, clientID);
  res = RC76XX_ConfigMQTT(&mqttHandle, broker, port, clientID, pubTopic, user, pass, false, false, 120);
  if (res != RC76XX_OK)
  {
    printf("ConfigMQTT failed: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  printf("MQTT configured, cfg_id=%d\r\n", mqttHandle.mqtt_cfg_id);

  /* 4) Connect MQTT */
  printf("Connecting MQTT...\r\n");
  res = RC76XX_ConnectMQTT(&mqttHandle);
  if (res != RC76XX_OK)
  {
    printf("ConnectMQTT failed: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  vTaskDelay(delay);
  printf("MQTT connected\r\n");

  /* Subscribe to a topic */
  printf("Subscribing to %s...\r\n", subTopic);
  res = RC76XX_Subscribe(&mqttHandle, subTopic);
  if (res == RC76XX_OK)
  {
    printf("Subscribed to %s\r\n", subTopic);
  }
  else
  {
    printf("Subscribe failed: %d\r\n", res);
  }

  // Main MQTT transmission loop
  for (;;)
  {
    // Blocking until data is available in the queue
    if (xMQTTQueue != NULL &&
        xQueueReceive(xMQTTQueue, &sensor_data, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE)
    {
      // Convert to integer representation
      convert_combined_sensor_data_to_int_optimized(&sensor_data, &sensor_int_data);

      // Format payload for ThingSpeak with both BME280 and MPU6050 data
      char payload[300];

      // Build payload with available sensor data
      snprintf(payload, sizeof(payload),
               "field1=%ld.%02ld&field2=%lu.%02lu&field3=%lu.%02lu&field4=%d.%03d&field5=%d.%03d&field6=%d.%03d&field7=%d.%02d&field8=%d.%02d",
               // BME280 data (fields 1-3)
               sensor_int_data.bme_temp_whole, sensor_int_data.bme_temp_frac,   // Temperature
               sensor_int_data.bme_press_whole, sensor_int_data.bme_press_frac, // Pressure
               sensor_int_data.bme_hum_whole, sensor_int_data.bme_hum_frac,     // Humidity
               // MPU6050 data (fields 4-8)
               sensor_int_data.mpu_accel_x_whole, sensor_int_data.mpu_accel_x_frac, // Accel X
               sensor_int_data.mpu_accel_y_whole, sensor_int_data.mpu_accel_y_frac, // Accel Y
               sensor_int_data.mpu_accel_z_whole, sensor_int_data.mpu_accel_z_frac, // Accel Z
               sensor_int_data.mpu_gyro_x_whole, sensor_int_data.mpu_gyro_x_frac,   // Gyro X
               sensor_int_data.mpu_gyro_y_whole, sensor_int_data.mpu_gyro_y_frac    // Gyro Y
      );
      // snprintf(payload, sizeof(payload),
      //          "field1=%ld.%02ld&field2=%lu.%02lu&field3=%lu.%02lu&field4=%d.%03d&field5=%d.%02d",
      //          // BME280 (fields 1-3)
      //          sensor_int_data.bme_temp_whole, sensor_int_data.bme_temp_frac,
      //          sensor_int_data.bme_press_whole, sensor_int_data.bme_press_frac,
      //          sensor_int_data.bme_hum_whole, sensor_int_data.bme_hum_frac,
      //          // MPU6050 - Combined magnitude (fields 4-5)
      //          sensor_int_data.mpu_accel_x_whole, sensor_int_data.mpu_accel_x_frac, // Just X accel
      //          sensor_int_data.mpu_temp_whole, sensor_int_data.mpu_temp_frac        // MPU temp
      // );

      printf("MQTT: Publishing combined sensor data\r\n");
      // printf("BME280 Status: %s, MPU6050 Status: %s\r\n",
      //        sensor_int_data.bme_valid ? "Valid" : "Invalid",
      //        sensor_int_data.mpu_valid ? "Valid" : "Invalid");
      printf("MQTT Payload: %s\r\n", payload);

      // Publish to ThingSpeak
      res = RC76XX_Publish(&mqttHandle, pubTopic, payload);
      if (res == RC76XX_OK)
      {
        printf("MQTT: Publish successful\r\n");
      }
      else
      {
        printf("MQTT: Publish failed: %d\r\n", res);
      }
    }
    else
    {
      // No data received in 65 seconds - send heartbeat or handle timeout
      // printf("MQTT: No sensor data received in 65 seconds\r\n");

      // Optional: Send a status message or keep-alive
      // You could also fall back to direct global variable access here if needed
    }

    // Small delay before next iteration
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void vGpsTaskStart(void *argument)
{
  // Initialize GPS
  memset(gpsRx, 0, GPS_RX_SIZE); // Clear the buffer.
  HAL_UART_Receive_DMA(&huart6, gpsRx, GPS_RX_SIZE);

  if (lwgps_init(&hgps) != 1)
  {
    printf("GPS initialization failed\r\n");
    vTaskDelete(NULL);
  }

  for(;;)
  {
    if (flag == 1)
    {
      lwgps_process(&hgps, gpsRx, GPS_RX_SIZE);
      printf("---\r\n");
      printf("Valid status: %d\r\n", hgps.is_valid);
      printf("Time: %02d:%02d:%02d\r\n", hgps.hours, hgps.minutes, hgps.seconds);
      printf("Latitude: %f degrees\r\n", hgps.latitude);
      printf("Longitude: %f degrees\r\n", hgps.longitude);
      printf("Altitude: %f meters\r\n", hgps.altitude + hgps.geo_sep);
      printf("Speed: %.2f km/h\r\n", hgps.speed * 1.852);
      flag = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  // GPS task code here
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
