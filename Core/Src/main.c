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
#include "config.h"
#include "print.h"
#include "mqtt_secrets.h"
#include "rc76xx_mqtt.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "uat_freertos.h" // your parser header


#include "bme280_tasks.h"
// #include "bme280.h"
// #include "bme280_porting.h"
#include "datalogger.h"

// #include "w25qxx_hal.h"  // not use.
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
// #define FLASH_RING_BUFFER_SIZE (64 * 1024)  // 64KB - can be changed later
#define RING_BUFFER_START_ADDRESS (DATA_START_ADDRESS)
#define RING_BUFFER_END_ADDRESS (RING_BUFFER_START_ADDRESS + FLASH_RING_BUFFER_SIZE)
#define RECORDS_PER_PAGE (W25Q64_PAGE_SIZE / sizeof(BME280_Record_t))

// Queue configuration
#define QUEUE_SEND_TIMEOUT_MS 1000
// #define DATA_LOGGER_QUEUE_SIZE 20
#define MAX_QUEUE_WAIT_MS 1000
//#define QUEUE_SEND_TIMEOUT_MS 1000
// #define DATA_LOGGER_QUEUE_SIZE 20
//#define MAX_QUEUE_WAIT_MS 1000

#define GPS_RX_SIZE 512
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
volatile uint8_t flag = 0;
uint8_t gpsRx[GPS_RX_SIZE]; 
lwgps_t hgps;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
// static void App_Task(void *pvParameters);
static void MQTT_Task(void *pvParameters);
// void StartBme280Task(void *argument);
void StartW25QTestTask(void *argument);
void StartDataLoggerTask(void *argument);
void vGpsTaskStart(void *argument);

// // Queue interface functions (non-static, called from other tasks)
// BaseType_t DataLogger_QueueBME280Data(struct bme280_data *bme_data);
// BaseType_t DataLogger_QueueSystemEvent(const char *event_text);

// BME280 Task and helper functions
// void StartBme280Task(void *argument);

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

  /* Init scheduler */
  osKernelInitialize();
  // MX_FREERTOS_Init();
  // Create the print task
  BaseType_t printInitStatus = xPrintInit(CONFIG_TASK_PRIORITY_NORMAL);
  configASSERT(printInitStatus == pdPASS);
  DEBUG_PRINT_DEBUG("Print task created\n");

  // Initialize uAT parser on huart2
  uAT_Result_t result = uAT_Init(&huart2);
  if (result != UAT_OK) {
    // handle init error (blink LED, etc.)
    DEBUG_PRINT_ERROR("uAT parser initialization failed with error code: %d\n",
                      result);
    Error_Handler();
  }
  DEBUG_PRINT_DEBUG("uAT parser initialized\n");

  BaseType_t TaskStatus;
  // Create the uAT parser task
  // Create I2C mutex
  SemaphoreHandle_t g_i2c_mutex = NULL;

  // In main() or initialization function:
  g_i2c_mutex = xSemaphoreCreateMutex();
  configASSERT(g_i2c_mutex != NULL);
  DEBUG_PRINT_DEBUG("I2C mutex created\n");
  if (xSemaphoreTake(g_i2c_mutex, portMAX_DELAY) != pdTRUE)
  {
    DEBUG_PRINT_ERROR("Failed to take I2C mutex\n");
    Error_Handler();
  }
  DEBUG_PRINT_DEBUG("I2C mutex taken\n");
  if (xSemaphoreGive(g_i2c_mutex) != pdTRUE)
  {
    DEBUG_PRINT_ERROR("Failed to give I2C mutex\n");
    Error_Handler();
  }
  DEBUG_PRINT_DEBUG("I2C mutex given\n");

  // Create uAT parser task
  TaskStatus = xTaskCreate(uAT_Task,
                           "uAT_Task",
                           CONFIG_TASK_STACK_SIZE_UAT,
                           NULL,
                           CONFIG_TASK_PRIORITY_NORMAL,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  DEBUG_PRINT_DEBUG("uAT task created\n");

  // // (Optional) your other application tasks
  // xTaskCreate(App_Task,
  //             "AppTask",
  //             512,
  //             NULL,
  //             tskIDLE_PRIORITY + 1,
  //             NULL);

  TaskStatus = xTaskCreate(MQTT_Task,
                           "MQTT_Task",
                           CONFIG_TASK_STACK_SIZE_MQTT,
                           NULL,
                           CONFIG_TASK_PRIORITY_NORMAL,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  DEBUG_PRINT_DEBUG("MQTT task created\n");

  // Create the BME280 task
  TaskStatus = xTaskCreate(StartBme280Task,
                           "BME280_Task",
                           CONFIG_TASK_STACK_SIZE_BME280, // Adjust stack size as needed
                           (void *)g_i2c_mutex,
                           CONFIG_TASK_PRIORITY_NORMAL,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  DEBUG_PRINT_DEBUG("BME280 task created\n");
  // Create MPU6050 task
  TaskStatus = xTaskCreate(MPU6050_Task_Start,
                           "MPU6050",
                           CONFIG_TASK_STACK_SIZE_MPU6050,
                           (void *)g_i2c_mutex,
                           CONFIG_TASK_PRIORITY_NORMAL,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  DEBUG_PRINT_DEBUG("MPU6050 task created\n");

  // Create the Data Logger task
  TaskStatus = xTaskCreate(
                          StartDataLoggerTask, 
                          "DataLogger",
                          CONFIG_TASK_STACK_SIZE_DATALOGGER,
                          NULL, 
                          CONFIG_TASK_PRIORITY_NORMAL, 
                          NULL);
  configASSERT(TaskStatus == pdPASS);
  DEBUG_PRINT_DEBUG("DataLogger task created\n");

  // Create the GPS task
  TaskStatus = xTaskCreate(vGpsTaskStart,
                           "GPS_Task",
                           CONFIG_TASK_STACK_SIZE_GPS,
                           NULL,
                           CONFIG_TASK_PRIORITY_NORMAL,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  DEBUG_PRINT_DEBUG("GPS task created\n");

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
// int __io_putchar(int ch)
// {
//   HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
//   return ch;
// }

// // printf implementation for UART3.
// int _write(int file, char *ptr, int len)
// {
//   (void)file;
//   HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 0xFFFF);
//   return len;
// }

// // scanf implementation for UART3. (not used in this code)
// int _read(int file, char *ptr, int len)
// {
//   (void)file;
//   (void)len;
//   int ch = 0;
//   HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//   HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//   if (ch == 13)
//   {
//     ch = 10;
//     HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//   }
//   else if (ch == 8)
//   {
//     ch = 0x30;
//     HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//   }

//   *ptr = ch;

//   return 1;
// }

/**
 * @brief  MQTT_Task
 *         Runs the RC76xx MQTT state machine:
 *         Initialize → NetworkAttach → ConfigMQTT → ConnectMQTT → loop Publish/Subscribe
 */
static void MQTT_Task(void *argument)
{
  (void)argument;
  RC76XX_Result_t res;
  TickType_t delay = pdMS_TO_TICKS(20000);
  SensorData_Combined_t sensor_data;

  // Register all URC handlers
  res = RC76XX_RegisterURCHandlers(&mqttHandle);
  if (res != RC76XX_OK)
  {
    DEBUG_PRINT_ERROR("Failed to register URC handlers: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  DEBUG_PRINT_INFO("Registered all URC handlers\r\n");

  /* 0) Reset the modem */
  DEBUG_PRINT_INFO("MQTT_Task: Resetting modem...\r\n");
  res = RC76XX_Reset(&mqttHandle);
  if (res != RC76XX_OK)
  {
    DEBUG_PRINT_ERROR("Reset modem failed: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  vTaskDelay(delay);

  /* 1) Initialize modem */
  DEBUG_PRINT_INFO("MQTT_Task: Initializing modem...\r\n");
  res = RC76XX_Initialize(&mqttHandle);
  if (res != RC76XX_OK)
  {
    DEBUG_PRINT_ERROR("Init failed: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  DEBUG_PRINT_INFO("Initialized, IMEI=%s\r\n", mqttHandle.imei);

  /* 2) Attach to network & get IP */
  DEBUG_PRINT_INFO("Attaching to network...\r\n");
  res = RC76XX_NetworkAttach(&mqttHandle);
  if (res != RC76XX_OK)
  {
    DEBUG_PRINT_ERROR("NetworkAttach failed: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  DEBUG_PRINT_INFO("Network ready, IP=%s\r\n", mqttHandle.ip);

  /* 3) Configure MQTT */
  const char *broker = CONFIG_MQTT_BROKER_HOSTNAME;
  const uint16_t port = CONFIG_MQTT_BROKER_PORT;
  const char *clientID = CONFIG_MQTT_CLIENT_ID;
  const char *user = CONFIG_MQTT_USERNAME;
  const char *pass = CONFIG_MQTT_PASSWORD;
  const char *subTopic = CONFIG_MQTT_SUBSCRIBE_TOPIC;
  const char *pubTopic = CONFIG_MQTT_PUBLISH_TOPIC;

  DEBUG_PRINT_INFO("Configuring MQTT %s:%u, ClientID=%s...\r\n", broker, port, clientID);
  res = RC76XX_ConfigMQTT(&mqttHandle, broker, port, clientID, pubTopic, user, pass, false, false, 120);
  if (res != RC76XX_OK)
  {
    DEBUG_PRINT_ERROR("ConfigMQTT failed: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  DEBUG_PRINT_INFO("MQTT configured, cfg_id=%d\r\n", mqttHandle.mqtt_cfg_id);

  /* 4) Connect MQTT */
  DEBUG_PRINT_INFO("Connecting MQTT...\r\n");
  res = RC76XX_ConnectMQTT(&mqttHandle);
  if (res != RC76XX_OK)
  {
    DEBUG_PRINT_ERROR("ConnectMQTT failed: %d\r\n", res);
    vTaskSuspend(NULL);
  }
  vTaskDelay(delay);
  DEBUG_PRINT_INFO("MQTT connected\r\n");

  /* Subscribe to a topic */
  DEBUG_PRINT_INFO("Subscribing to %s...\r\n", subTopic);
  res = RC76XX_Subscribe(&mqttHandle, subTopic);
  if (res == RC76XX_OK)
  {
    DEBUG_PRINT_INFO("Subscribed to %s\r\n", subTopic);
  }
  else
  {
    DEBUG_PRINT_ERROR("Subscribe failed: %d\r\n", res);
  }

  // Main MQTT transmission loop
  for (;;)
  {
    // Blocking until data is available in the queue
    if (xMQTTQueue != NULL &&
        xQueueReceive(xMQTTQueue, &sensor_data, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE)
    {
      // Convert to integer representation

      // Enhanced payload with GPS data
      char payload[400]; // Increased size for GPS data

      // float version
      snprintf(payload, sizeof(payload), "field1=%02f&field2=%02f&field3=%02f&field4=%02f&field5=%03f&field6=%03f&field7=%03f&field8=%03f&lat=%06f&long=%06f",
               sensor_data.bme_temperature, sensor_data.bme_pressure, sensor_data.bme_humidity,
               sensor_data.mpu_accel_x, sensor_data.mpu_accel_y, sensor_data.mpu_accel_z,
               sensor_data.mpu_gyro_x, sensor_data.mpu_gyro_y,
               sensor_data.gps_latitude, sensor_data.gps_longitude);

      // printf("MQTT: Publishing combined sensor data with GPS\r\n");
      DEBUG_PRINT_INFO("BME280: %s, MPU6050: %s, GPS: %s\r\n",
             sensor_data.bme_valid ? "Valid" : "Invalid",
             sensor_data.mpu_valid ? "Valid" : "Invalid",
             sensor_data.gps_valid ? "Valid" : "Invalid");
      DEBUG_PRINT_DEBUG("MQTT Payload: %s\r\n", payload);

      // Publish to ThingSpeak
      res = RC76XX_Publish(&mqttHandle, pubTopic, payload);
      if (res == RC76XX_OK)
      {
        DEBUG_PRINT_INFO("MQTT: Publish successful\r\n");
      }
      else
      {
        DEBUG_PRINT_ERROR("MQTT: Publish failed: %d\r\n", res);
      }
    }
    else
    {
      // No data received in timeout period
      // Optional: Send heartbeat or handle timeout
    }

    // Small delay before next iteration
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void vGpsTaskStart(void *argument)
{
  (void)argument;
  DEBUG_PRINT_INFO("GPS Task Started\r\n");
  SensorData_Combined_t current_sensor_data;
  SensorData_Combined_Int_t gps_task_sensor_data;
  
  memset(gpsRx, 0, GPS_RX_SIZE);
  memset(&current_sensor_data, 0, sizeof(SensorData_Combined_t));
  memset(&gps_task_sensor_data, 0, sizeof(SensorData_Combined_Int_t));
  
  HAL_UART_Receive_DMA(&huart6, gpsRx, GPS_RX_SIZE);

  if (lwgps_init(&hgps) != 1)
  {
    DEBUG_PRINT_ERROR("GPS initialization failed\r\n");
    vTaskDelete(NULL);
  }

  for (;;)
  {
    if (flag == 1)
    {
      lwgps_process(&hgps, gpsRx, GPS_RX_SIZE);
      current_sensor_data.gps_longitude = hgps.longitude;
      current_sensor_data.gps_latitude = hgps.latitude;
      current_sensor_data.gps_altitude = hgps.altitude + hgps.geo_sep;
      current_sensor_data.gps_speed = hgps.speed * 1.852f;
      if(current_sensor_data.gps_longitude != 0 && current_sensor_data.gps_altitude != 0){
        // convert_combined_sensor_data_to_int_optimized(&current_sensor_data, &gps_task_sensor_data);

        float speed_kmh = hgps.speed * 1.852f;             // Convert knots to km/h
        float altitude_msl = hgps.altitude + hgps.geo_sep; // Mean sea level altitude

        BaseType_t result = DataLogger_UpdateGPSData(
            hgps.latitude,
                hgps.longitude,
                altitude_msl,
                speed_kmh);

        if (result != pdTRUE)
        {
          DEBUG_PRINT_ERROR("GPS: Failed to send data to datalogger\r\n");
        }
      }
          // // Send GPS data to datalogger if valid
          // if (hgps.is_valid)
          // {

      // }
      // else
      // {
      //   // Send invalid GPS data to clear previous valid data
      //   DataLogger_UpdateGPSData(0.0f, 0.0f, 0.0f, 0.0f);
      // }
      
      // DEBUG_PRINT_DEBUG("---\r\n");
      DEBUG_PRINT_DEBUG("Valid status: %d\r\n", hgps.is_valid);
      DEBUG_PRINT_DEBUG("Time: %02d:%02d:%02d\r\n", hgps.hours, hgps.minutes, hgps.seconds);
      DEBUG_PRINT_DEBUG("Latitude Float: %f\r\n", hgps.latitude);
      DEBUG_PRINT_DEBUG("Longitude Float: %f\r\n", hgps.longitude);
      DEBUG_PRINT_DEBUG("Altitude Float: %f\r\n", hgps.altitude + hgps.geo_sep);
      DEBUG_PRINT_DEBUG("Speed Float: %f\r\n", hgps.speed * 1.852f);
      DEBUG_PRINT_DEBUG("Dop: %f, %f, %f\r\n", hgps.dop_h, hgps.dop_v, hgps.dop_v);
      flag = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
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
