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

#include "bme280.h"
#include "bme280_porting.h"

#include "w25qxx_hal.h"
#include "driver_w25qxx_interface.h"
#include "driver_w25qxx_basic.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// W25Q64 specific parameters
#define W25Q64_SECTOR_SIZE      4096        // 4KB sectors
#define W25Q64_PAGE_SIZE        256         // 256 byte pages
#define W25Q64_TOTAL_SIZE       8388608     // 8MB total (8 * 1024 * 1024)
#define W25Q64_BLOCK_SIZE       65536       // 64KB blocks
#define DATA_START_ADDRESS      0x1000      // Start logging at 4KB offset (after test area)

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
struct bme280_dev bme_device;
struct bme280_data bme_comp_data;

// Define a structure to store BME280 readings with timestamp
typedef struct
{
  uint32_t timestamp; // Timestamp in milliseconds
  float temperature;  // Temperature in Celsius
  float pressure;     // Pressure in hPa
  float humidity;     // Humidity in %
} BME280_Record_t;

// W25Q64 specific variables
#define RECORDS_PER_SECTOR (W25Q64_SECTOR_SIZE / sizeof(BME280_Record_t))
static BME280_Record_t lastReadRecord;
static uint32_t nextWriteAddress = DATA_START_ADDRESS;
static SemaphoreHandle_t bme280DataMutex = NULL;

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
// static W25Q_StatusTypeDef W25Q_Flash_Init(void);
void PrintBufferHex(const uint8_t *buffer, uint32_t size, uint32_t baseAddress);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// W25Q64 utility functions
uint32_t W25Q64_GetSectorAddress(uint32_t address)
{
    return (address / W25Q64_SECTOR_SIZE) * W25Q64_SECTOR_SIZE;
}

uint32_t W25Q64_GetSectorNumber(uint32_t address)
{
    return address / W25Q64_SECTOR_SIZE;
}

uint32_t W25Q64_GetRecordsInSector(uint32_t sector_start_address)
{
    BME280_Record_t tempRecord;
    uint32_t record_count = 0;
    
    for (uint32_t addr = sector_start_address; 
         addr < (sector_start_address + W25Q64_SECTOR_SIZE); 
         addr += sizeof(BME280_Record_t))
    {
        if (w25qxx_basic_read(addr, (uint8_t *)&tempRecord, sizeof(BME280_Record_t)) == 0)
        {
            if (tempRecord.timestamp != 0xFFFFFFFF)
            {
                record_count++;
            }
            else
            {
                break; // Found empty record, stop counting
            }
        }
        else
        {
            break; // Read error, stop counting
        }
    }
    
    return record_count;
}

void W25Q64_PrintSectorInfo(uint32_t sector_address)
{
    uint32_t sector_num = W25Q64_GetSectorNumber(sector_address);
    uint32_t records_used = W25Q64_GetRecordsInSector(sector_address);
    
    printf("Sector %lu (0x%08lX): %lu/%d records used\r\n", 
           sector_num, sector_address, records_used, RECORDS_PER_SECTOR);
}

void W25Q64_PrintFlashInfo(void)
{
    printf("\r\n=== W25Q64 Flash Information ===\r\n");
    printf("Total Size: %d bytes (%.1f MB)\r\n", W25Q64_TOTAL_SIZE, W25Q64_TOTAL_SIZE / 1024.0f / 1024.0f);
    printf("Sector Size: %d bytes (%d KB)\r\n", W25Q64_SECTOR_SIZE, W25Q64_SECTOR_SIZE / 1024);
    printf("Page Size: %d bytes\r\n", W25Q64_PAGE_SIZE);
    printf("Total Sectors: %d\r\n", W25Q64_TOTAL_SIZE / W25Q64_SECTOR_SIZE);
    printf("BME280 Record Size: %d bytes\r\n", sizeof(BME280_Record_t));
    printf("Records per Sector: %d\r\n", RECORDS_PER_SECTOR);
    printf("Total Records Capacity: %d\r\n", (W25Q64_TOTAL_SIZE - DATA_START_ADDRESS) / sizeof(BME280_Record_t));
    printf("Data Start Address: 0x%08X\r\n", DATA_START_ADDRESS);
    printf("================================\r\n\r\n");
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
  bme280DataMutex = xSemaphoreCreateMutex();
  if (bme280DataMutex == NULL)
  {
    printf("Failed to create BME280 data mutex\r\n");
    Error_Handler();
  }
  BaseType_t TaskStatus;
  // Create the uAT parser task

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
                           NULL,
                           tskIDLE_PRIORITY + 1,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  printf("BME280 task created\n");

  // Create the Data Logger task
  TaskStatus = xTaskCreate(StartDataLoggerTask,
                           "DataLogger",
                           2048, // Larger stack for this task
                           NULL,
                           tskIDLE_PRIORITY + 1,
                           NULL);
  configASSERT(TaskStatus == pdPASS);
  printf("DataLogger task created\n");

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

  // /* 3) Configure MQTT */
  // const char *broker = "broker.hivemq.com";
  // const char *clientID = "QCX216";
  // const char *user = "";
  // const char *pass = "";
  const char *broker = "mqtt3.thingspeak.com";
  const uint16_t port = 1883;
  const char *clientID = SECRET_MQTT_CLIENT_ID;
  const char *user = SECRET_MQTT_USERNAME;
  const char *pass = SECRET_MQTT_PASSWORD;
  const char *subTopic = "channels/2956054/subscribe";
  const char *pubTopic = "channels/2956054/publish";

  printf("Configuring MQTT %s:%u, ClientID=%s...\r\n",
         broker, 1883, clientID);
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

  printf("Registered +KMQTT_DATA handler\r\n");
  /* Subscribe to a topic */

  // const char *subTopic = "home/LWTMessage";
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
  for (;;)
  {

    uint32_t now = HAL_GetTick();
    /* Publish a message */
    // const char *pubTopic = "home/LWTMessage";
    // Format BME280 data into MQTT payload for ThingSpeak
    char payload[100]; // Make sure this is large enough for your payload
    bme_calc_data_int_t bme_calc_data;

    // // Convert floating-point values to integer components
    convert_bme_data_to_int(&bme_comp_data, &bme_calc_data);

    // Format the payload
    sprintf(payload, "field1=%ld.%02ld&field2=%lu.%02lu&field3=%lu.%02lu",
            bme_calc_data.temp_whole, bme_calc_data.temp_frac,
            bme_calc_data.press_whole, bme_calc_data.press_frac,
            bme_calc_data.hum_whole, bme_calc_data.hum_frac);

    printf("MQTT Payload: %s\r\n", payload);
    printf("Publishing to %s: %s\r\n", pubTopic, payload);
    res = RC76XX_Publish(&mqttHandle, pubTopic, payload);
    if (res == RC76XX_OK)
    {
      printf("Publish OK\r\n");
    }
    else
    {
      printf("Publish failed: %d\r\n", res);
    }

    /* Wait before next cycle */
    vTaskDelay(delay * 3);
  }
}

// Task function for BME280
void StartBme280Task(void *argument)
{
  (void)argument;
  int8_t rslt;
  uint32_t meas_delay;
  struct bme280_settings settings;

  // Initialize the BME280 interface porting layer
  // Use BME280_I2C_ADDR_PRIM (0x76) or BME280_I2C_ADDR_SEC (0x77)
  rslt = bme280_interface_init(&bme_device, &hi2c1, BME280_I2C_ADDR_PRIM);
  if (rslt != BME280_OK)
  {
    printf("Failed to initialize BME280 interface. Error: %d\r\n", rslt);
    // Handle error: perhaps delete task or enter error loop
    vTaskDelete(NULL);
    return;
  }

  // Initialize the BME280 sensor
  rslt = bme280_init(&bme_device);
  if (rslt != BME280_OK)
  {
    printf("Failed to initialize BME280 sensor. Error: %d\r\n", rslt);
    // Handle error
    bme280_interface_deinit(&bme_device); // Clean up interface resources
    vTaskDelete(NULL);
    return;
  }
  printf("BME280 initialized successfully. Chip ID: 0x%X\r\n", bme_device.chip_id);
  vTaskDelay(10);
  // Configure sensor settings (example: normal mode, standard oversampling)
  // First get the current settings
  rslt = bme280_get_sensor_settings(&settings, &bme_device);
  if (rslt != BME280_OK)
  {
    printf("Failed to get BME280 sensor settings. Error: %d\r\n", rslt);
    bme280_interface_deinit(&bme_device);
    vTaskDelete(NULL);
    return;
  }
  vTaskDelay(10);

  // Now modify the settings
  settings.osr_h = BME280_OVERSAMPLING_1X;
  settings.osr_p = BME280_OVERSAMPLING_4X;
  settings.osr_t = BME280_OVERSAMPLING_2X;
  settings.filter = BME280_FILTER_COEFF_OFF;
  settings.standby_time = BME280_STANDBY_TIME_1000_MS; // Example, adjust as needed

  // Apply settings one by one instead of all at once

  // First apply humidity settings
  rslt = bme280_set_sensor_settings(BME280_SEL_OSR_HUM, &settings, &bme_device);
  if (rslt != BME280_OK)
  {
    printf("Failed to set humidity settings. Error: %d\r\n", rslt);
  }
  HAL_Delay(5);

  // Then apply temperature and pressure settings
  rslt = bme280_set_sensor_settings(BME280_SEL_OSR_TEMP | BME280_SEL_OSR_PRESS, &settings, &bme_device);
  if (rslt != BME280_OK)
  {
    printf("Failed to set temp/press settings. Error: %d\r\n", rslt);
  }
  HAL_Delay(5);

  // Finally apply filter and standby settings
  rslt = bme280_set_sensor_settings(BME280_SEL_FILTER | BME280_SEL_STANDBY, &settings, &bme_device);
  if (rslt != BME280_OK)
  {
    printf("Failed to set filter/standby settings. Error: %d\r\n", rslt);
  }

  // Set the sensor to normal mode
  rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme_device);
  if (rslt != BME280_OK)
  {
    printf("Failed to set BME280 sensor mode. Error: %d\r\n", rslt);
    // Handle error
    bme280_interface_deinit(&bme_device);
    vTaskDelete(NULL);
    return;
  }

  printf("Waiting for first measurement to complete...\r\n");
  HAL_Delay(50);
  uint8_t status;
  do
  {
    rslt = bme280_get_regs(BME280_REG_STATUS, &status, 1, &bme_device);
    HAL_Delay(10);
  } while (rslt == BME280_OK && (status & BME280_STATUS_MEAS_DONE) != 0);
  printf("First measurement complete, status: 0x%02X\r\n", status);

  // Calculate the minimum delay required between measurements based on settings
  rslt = bme280_cal_meas_delay(&meas_delay, &settings);
  if (rslt != BME280_OK)
  {
    printf("Failed to calculate measurement delay. Error: %d\r\n", rslt);
    // Handle error, but can continue
  }
  else
  {
    printf("Calculated measurement delay: %lu us\r\n", (unsigned long)meas_delay);
  }

  for (;;)
  {
    // Set sensor to forced mode (takes one measurement then goes to sleep)
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &bme_device);
    if (rslt != BME280_OK)
    {
      printf("Failed to set forced mode: %d\r\n", rslt);
    }

    // Wait for the measurement to complete
    HAL_Delay(50); // Short delay for measurement to start
    uint8_t status;
    do
    {
      rslt = bme280_get_regs(BME280_REG_STATUS, &status, 1, &bme_device);
      HAL_Delay(10);
    } while (rslt == BME280_OK && (status & BME280_STATUS_MEAS_DONE) != 0);

    // Now read the data
    rslt = bme280_get_sensor_data(BME280_ALL, &bme_comp_data, &bme_device);
    if (rslt == BME280_OK)
    {
      // Take mutex before updating the shared BME280 data
      if (bme280DataMutex != NULL && xSemaphoreTake(bme280DataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        // Data is already in bme_comp_data, which is a global variable
        // The DataLogger task will access this data

        // Convert and print the data (keep this for debugging)
        bme_calc_data_int_t bme_calc_data;
        convert_bme_data_to_int(&bme_comp_data, &bme_calc_data);

        printf("Temp: %ld.%02ld C, Press: %lu.%02lu hPa, Hum: %lu.%02lu %%\r\n",
               bme_calc_data.temp_whole, bme_calc_data.temp_frac,
               bme_calc_data.press_whole, bme_calc_data.press_frac,
               bme_calc_data.hum_whole, bme_calc_data.hum_frac);

        // Release the mutex
        xSemaphoreGive(bme280DataMutex);
      }
      else
      {
        printf("Failed to acquire BME280 data mutex for updating\r\n");
      }
    }
    else
    {
      printf("Failed to get sensor data: %d\r\n", rslt);
    }

    // Wait before next reading
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void StartDataLoggerTask(void *argument)
{
    (void)argument;
    uint8_t flash_status;
    uint8_t manufacturer;
    uint8_t device_id;
    BME280_Record_t record;
    uint32_t lastReadAddress = 0;
    
    // Delay for system init
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Data Logger Task starting...\r\n");
    
    // Create mutex for accessing BME280 data if it doesn't exist yet
    if (bme280DataMutex == NULL)
    {
        bme280DataMutex = xSemaphoreCreateMutex();
        if (bme280DataMutex == NULL)
        {
            printf("Failed to create BME280 data mutex\r\n");
            vTaskDelete(NULL);
            return;
        }
    }
    
    // Initialize W25Q64 flash using basic driver
    printf("Initializing W25Q64 flash...\r\n");
    flash_status = w25qxx_basic_init(W25Q64, W25QXX_INTERFACE_SPI, W25QXX_BOOL_FALSE);
    if (flash_status != 0)
    {
        printf("Failed to initialize W25Q64 flash (err=%d)\r\n", flash_status);
        vTaskDelete(NULL);
        return;
    }
    
    printf("W25Q64 flash initialized successfully\r\n");
    
    // Get manufacturer and device ID
    flash_status = w25qxx_basic_get_id(&manufacturer, &device_id);
    if (flash_status == 0)
    {
        printf("W25Q64: manufacturer is 0x%02X device id is 0x%02X\r\n", manufacturer, device_id);
        // Expected values for W25Q64: manufacturer = 0xEF, device_id = 0x16
        if (manufacturer == 0xEF && device_id == 0x16)
        {
            printf("W25Q64 chip confirmed!\r\n");
        }
        else
        {
            printf("Warning: Unexpected chip ID (expected 0xEF 0x16)\r\n");
        }
    }
    else
    {
        printf("Failed to read manufacturer/device ID\r\n");
    }
    
    // Print flash information
    W25Q64_PrintFlashInfo();
    
    // Test basic write/read functionality
    uint8_t test_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t read_data[8] = {0};
    
    printf("Testing W25Q64 write/read at address 0x00000000...\r\n");
    flash_status = w25qxx_basic_write(0x00000000, test_data, 8);
    if (flash_status == 0)
    {
        printf("Test write successful\r\n");
        
        flash_status = w25qxx_basic_read(0x00000000, read_data, 8);
        if (flash_status == 0)
        {
            printf("Test read successful: ");
            for (int i = 0; i < 8; i++)
            {
                printf("0x%02X ", read_data[i]);
            }
            printf("\r\n");
            
            // Verify data
            bool data_match = true;
            for (int i = 0; i < 8; i++)
            {
                if (test_data[i] != read_data[i])
                {
                    data_match = false;
                    break;
                }
            }
            
            if (data_match)
            {
                printf("W25Q64 test PASSED - data verified correctly\r\n");
            }
            else
            {
                printf("W25Q64 test FAILED - data mismatch\r\n");
            }
        }
        else
        {
            printf("Test read failed (err=%d)\r\n", flash_status);
        }
    }
    else
    {
        printf("Test write failed (err=%d)\r\n", flash_status);
    }
    
    // Find next write address by scanning for empty records
    nextWriteAddress = DATA_START_ADDRESS;
    BME280_Record_t tempRecord;
    uint32_t current_sector_start = W25Q64_GetSectorAddress(DATA_START_ADDRESS);
    
    printf("Scanning for next write address starting at 0x%08X...\r\n", DATA_START_ADDRESS);
    
    // Scan multiple sectors to find the last written record
    bool found_empty = false;
    uint32_t max_scan_address = DATA_START_ADDRESS + (10 * W25Q64_SECTOR_SIZE); // Scan first 10 sectors
    if (max_scan_address > W25Q64_TOTAL_SIZE)
    {
        max_scan_address = W25Q64_TOTAL_SIZE;
    }
    
    for (uint32_t addr = DATA_START_ADDRESS; addr < max_scan_address; addr += sizeof(BME280_Record_t))
    {
        flash_status = w25qxx_basic_read(addr, (uint8_t *)&tempRecord, sizeof(BME280_Record_t));
          if (flash_status != 0)
        {
            printf("Error reading flash at address 0x%08lX\r\n", addr);
            break;
        }
        
        // If we find an empty record (timestamp is 0xFFFFFFFF when erased), we've found our write position
        if (tempRecord.timestamp == 0xFFFFFFFF)
        {
            nextWriteAddress = addr;
            found_empty = true;
            break;
        }
        
        // Keep track of the last valid record we've read
        lastReadAddress = addr;
    }
    
    if (!found_empty)
    {
        printf("No empty space found in scanned area, starting at end of scan\r\n");
        nextWriteAddress = max_scan_address;
    }
    
    current_sector_start = W25Q64_GetSectorAddress(nextWriteAddress);
    uint32_t records_in_current_sector = (nextWriteAddress - current_sector_start) / sizeof(BME280_Record_t);
    
    printf("Next write address: 0x%08lX\r\n", nextWriteAddress);
    printf("Current sector start: 0x%08lX\r\n", current_sector_start);
    printf("Records in current sector: %lu/%d\r\n", records_in_current_sector, RECORDS_PER_SECTOR);
    
    // Print current sector information
    W25Q64_PrintSectorInfo(current_sector_start);
    
    // If we found existing records, read and display the last one
    if (lastReadAddress >= DATA_START_ADDRESS)
    {
        flash_status = w25qxx_basic_read(lastReadAddress, (uint8_t *)&lastReadRecord, sizeof(BME280_Record_t));
        if (flash_status == 0)
        {
            printf("Last record found at 0x%08lX:\r\n", lastReadAddress);
            printf("  Time: %lu ms, Temp: %.2f°C, Press: %.2f hPa, Hum: %.2f%%\r\n",
                   lastReadRecord.timestamp, lastReadRecord.temperature,
                   lastReadRecord.pressure, lastReadRecord.humidity);
        }
    }
    else
    {
        printf("No existing records found, starting fresh\r\n");
    }
    
    // Main data logging loop
    uint32_t log_counter = 0;
    uint32_t total_records_written = 0;
    
    printf("\r\n=== Starting Data Logging Loop ===\r\n");
    
    for (;;)
    {
        if (xSemaphoreTake(bme280DataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Prepare record from BME280 data
            bme_calc_data_int_t bme_calc_data;
            convert_bme_data_to_int(&bme_comp_data, &bme_calc_data);
            
            record.timestamp = HAL_GetTick();
            record.temperature = (float)bme_calc_data.temp_whole + (float)bme_calc_data.temp_frac / 100.0f;
            record.pressure = (float)bme_calc_data.press_whole + (float)bme_calc_data.press_frac / 100.0f;
            record.humidity = (float)bme_calc_data.hum_whole + (float)bme_calc_data.hum_frac / 100.0f;
            
            xSemaphoreGive(bme280DataMutex);
            
            // Check if current sector is full
            if (records_in_current_sector >= RECORDS_PER_SECTOR)
            {
                printf("\r\n--- Current sector full, moving to next sector ---\r\n");
                W25Q64_PrintSectorInfo(current_sector_start);
                
                current_sector_start += W25Q64_SECTOR_SIZE;
                nextWriteAddress = current_sector_start;
                records_in_current_sector = 0;
                
                // Check if we've reached the end of flash
                if (current_sector_start >= W25Q64_TOTAL_SIZE)
                {
                    printf("Flash memory full! Wrapping around to beginning...\r\n");
                    current_sector_start = DATA_START_ADDRESS;
                    nextWriteAddress = current_sector_start;
                    
                    // Optionally erase the chip here if you want to start fresh
                    // printf("Erasing entire chip...\r\n");
                    // w25qxx_basic_chip_erase();
                    // printf("Chip erased, starting fresh\r\n");
                    
                    printf("Warning: Overwriting old data\r\n");
                }
                
                printf("New sector: 0x%08lX\r\n", current_sector_start);
            }
            
            // Write record to flash
            log_counter++;
            printf("[%lu] Writing record to 0x%08lX (sector %lu, record %lu/%d)\r\n", 
                   log_counter, nextWriteAddress, 
                   W25Q64_GetSectorNumber(nextWriteAddress),
                   records_in_current_sector + 1, RECORDS_PER_SECTOR);
            
            flash_status = w25qxx_basic_write(nextWriteAddress, (uint8_t *)&record, sizeof(BME280_Record_t));
            if (flash_status == 0)
            {
                // Read back to verify
                flash_status = w25qxx_basic_read(nextWriteAddress, (uint8_t *)&lastReadRecord, sizeof(BME280_Record_t));
                if (flash_status == 0)
                {
                    printf("  ✓ Verified: T=%.2f°C, P=%.2f hPa, H=%.2f%%, Time=%lu ms\r\n",
                           lastReadRecord.temperature, lastReadRecord.pressure,
                           lastReadRecord.humidity, lastReadRecord.timestamp);
                    
                    // Print hex dump every 10th record for debugging
                    if (log_counter % 10 == 0)
                    {
                        printf("  Hex dump of record %lu:\r\n", log_counter);
                        PrintBufferHex((uint8_t *)&lastReadRecord, sizeof(BME280_Record_t), nextWriteAddress);
                    }
                    
                    nextWriteAddress += sizeof(BME280_Record_t);
                    records_in_current_sector++;
                    total_records_written++;
                    
                    // Print progress every 20 records
                    if (log_counter % 20 == 0)
                    {
                        printf("\r\n--- Progress Report ---\r\n");
                        printf("Total records written: %lu\r\n", total_records_written);
                        printf("Current sector progress: %lu/%d records\r\n", 
                               records_in_current_sector, RECORDS_PER_SECTOR);
                        printf("Flash usage: %d%% of total capacity\r\n",
                              (nextWriteAddress - DATA_START_ADDRESS) * 100 / (W25Q64_TOTAL_SIZE - DATA_START_ADDRESS));
                        printf("----------------------\r\n\r\n");
                    }
                }
                else
                {
                    printf("  ✗ Failed to read back record (err=%d)\r\n", flash_status);
                }
            }
            else
            {
                printf("  ✗ Failed to write record to flash (err=%d)\r\n", flash_status);
            }
        }
        else
        {
            printf("Failed to acquire BME280 data mutex\r\n");
        }
        
        // Wait before next logging cycle
        vTaskDelay(pdMS_TO_TICKS(10000)); // Log every 10 seconds
    }
    
    // Cleanup (this won't be reached in the infinite loop)
    printf("Cleaning up W25Q64 driver...\r\n");
    w25qxx_basic_deinit();
}

// Function to print a buffer in hex format
void PrintBufferHex(const uint8_t *buffer, uint32_t size, uint32_t baseAddress)
{
  printf("--- Buffer Hex Dump (Base Address: 0x%08lX) ---\r\n", baseAddress);

  // Print the buffer in lines of 32 bytes each
  for (uint32_t offset = 0; offset < size; offset += 32)
  {
    // Print address at the start of each line
    printf("0x%08lX: ", baseAddress + offset);

    // Print hex values
    for (uint32_t i = 0; i < 32 && (offset + i) < size; i++)
    {
      printf("%02X ", buffer[offset + i]);

      // Add extra space every 4 bytes for readability
      if ((i + 1) % 4 == 0)
      {
        printf(" ");
      }
    }

    // Fill remaining space if less than 32 bytes in this line
    uint32_t bytesInLine = ((offset + 32) <= size) ? 32 : (size - offset);
    for (uint32_t i = bytesInLine; i < 32; i++)
    {
      printf("   "); // 3 spaces for each missing byte
      if ((i + 1) % 4 == 0)
      {
        printf(" "); // Extra space every 4 bytes
      }
    }

    // Print ASCII representation
    printf(" | ");
    for (uint32_t i = 0; i < 32 && (offset + i) < size; i++)
    {
      // Print only printable ASCII characters, otherwise print a dot
      if (buffer[offset + i] >= 32 && buffer[offset + i] <= 126)
      {
        printf("%c", buffer[offset + i]);
      }
      else
      {
        printf(".");
      }
    }

    // printf("\r\n");
  }

  printf("--- End of Buffer Hex Dump ---\r\n");
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
