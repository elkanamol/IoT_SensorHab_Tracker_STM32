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
  uint32_t timestamp;   // Timestamp in milliseconds
  float temperature;    // Temperature in Celsius
  float pressure;       // Pressure in hPa
  float humidity;       // Humidity in %
  uint32_t crc;         // CRC32 of the entire record
} BME280_Record_t;

// W25Q64 specific variables with page buffering
#define RECORDS_PER_SECTOR (W25Q64_SECTOR_SIZE / sizeof(BME280_Record_t))
#define RECORDS_PER_PAGE (W25Q64_PAGE_SIZE / sizeof(BME280_Record_t))
#define PAGES_PER_SECTOR (W25Q64_SECTOR_SIZE / W25Q64_PAGE_SIZE)

static BME280_Record_t lastReadRecord;
static uint32_t nextWriteAddress = DATA_START_ADDRESS;
static SemaphoreHandle_t bme280DataMutex = NULL;

// Page buffering variables
#define RECORDS_PER_PAGE (W25Q64_PAGE_SIZE / sizeof(BME280_Record_t))  // 256/16 = 16 records per page
static BME280_Record_t page_buffer[RECORDS_PER_PAGE];
static uint8_t records_in_buffer = 0;
static uint32_t current_page_address = DATA_START_ADDRESS;

// FIFO management variables
static uint32_t oldest_record_address = DATA_START_ADDRESS;
static uint32_t newest_record_address = DATA_START_ADDRESS;
static uint32_t total_records_written = 0;
static bool flash_full = false;

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
uint8_t W25Q64_FlushPageBuffer(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

// W25Q64 utility functions
uint32_t W25Q64_GetSectorAddress(uint32_t address)
{
  return (address / W25Q64_SECTOR_SIZE) * W25Q64_SECTOR_SIZE;
}

uint32_t W25Q64_GetPageAddress(uint32_t address)
{
  return (address / W25Q64_PAGE_SIZE) * W25Q64_PAGE_SIZE;
}

uint32_t W25Q64_GetSectorNumber(uint32_t address)
{
  return address / W25Q64_SECTOR_SIZE;
}

uint32_t W25Q64_GetPageNumber(uint32_t address)
{
  return address / W25Q64_PAGE_SIZE;
}

void W25Q64_PrintFlashInfo(void)
{
  printf("\r\n=== W25Q64 Flash Information ===\r\n");
  printf("Total Size: %d bytes (%.1f MB)\r\n", W25Q64_TOTAL_SIZE, W25Q64_TOTAL_SIZE / 1024.0f / 1024.0f);
  printf("Sector Size: %d bytes (%d KB)\r\n", W25Q64_SECTOR_SIZE, W25Q64_SECTOR_SIZE / 1024);
  printf("Page Size: %d bytes\r\n", W25Q64_PAGE_SIZE);
  printf("BME280 Record Size: %d bytes\r\n", sizeof(BME280_Record_t));
  printf("Records per Page: %d\r\n", RECORDS_PER_PAGE);
  printf("Records per Sector: %d\r\n", RECORDS_PER_SECTOR);
  printf("Pages per Sector: %d\r\n", PAGES_PER_SECTOR);
  printf("Total Records Capacity: %d\r\n", (W25Q64_TOTAL_SIZE - DATA_START_ADDRESS) / sizeof(BME280_Record_t));
  printf("================================\r\n\r\n");
}

// Add record to page buffer (efficient approach)
uint8_t W25Q64_AddRecordToPageBuffer(BME280_Record_t *record)
{
    // Add record to RAM buffer
    page_buffer[records_in_buffer] = *record;
    records_in_buffer++;
    
    printf("Buffered record %d/%d: T=%.2f°C\r\n", 
           records_in_buffer, RECORDS_PER_PAGE, record->temperature);
    
    // When buffer is full, write entire page at once
    if (records_in_buffer >= RECORDS_PER_PAGE)
    {
        return W25Q64_FlushPageBuffer();
    }
    
    return 0; // Success, record buffered
}

// Flush page buffer to flash (writes 256 bytes at once)
uint8_t W25Q64_FlushPageBuffer(void)
{
  if (records_in_buffer == 0)
    return 0; // Nothing to flush

  printf("Flushing %d records to page 0x%08lX\r\n", records_in_buffer, current_page_address);

  // Fill remaining buffer with 0xFF (empty records)
  for (uint8_t i = records_in_buffer; i < RECORDS_PER_PAGE; i++)
  {
    memset(&page_buffer[i], 0xFF, sizeof(BME280_Record_t));
  }

  // Write entire 256-byte page at once (much more efficient!)
  uint8_t status = w25qxx_basic_write(current_page_address, (uint8_t *)page_buffer, W25Q64_PAGE_SIZE);

  if (status == 0)
  {
    printf("✓ Page written successfully (256 bytes)\r\n");

    // Move to next page
    current_page_address += W25Q64_PAGE_SIZE;

    // FIFO: Wrap around when reaching end of flash
    if (current_page_address >= W25Q64_TOTAL_SIZE)
    {
      printf("Flash full, wrapping to beginning (FIFO mode)\r\n");
      current_page_address = DATA_START_ADDRESS;
    }

    // Reset buffer
    records_in_buffer = 0;
    memset(page_buffer, 0xFF, sizeof(page_buffer));

    return 0;
  }
  else
  {
    printf("✗ Page write failed (err=%d)\r\n", status);
    return 1;
  }
}

// Force flush buffer (call before reset/power down)
void W25Q64_ForceFlush(void)
{
    if (records_in_buffer > 0)
    {
        printf("Force flushing %d records before shutdown\r\n", records_in_buffer);
        W25Q64_FlushPageBuffer();
    }
}

// Find the next write position by scanning flash
uint32_t W25Q64_FindNextWritePosition(void)
{
  BME280_Record_t tempRecord;
  uint32_t last_valid_address = DATA_START_ADDRESS;
  bool found_empty = false;

  printf("Scanning flash to find next write position...\r\n");

  // Scan page by page for efficiency
  for (uint32_t page_addr = DATA_START_ADDRESS; page_addr < W25Q64_TOTAL_SIZE; page_addr += W25Q64_PAGE_SIZE)
  {
    // Read first record of the page
    if (w25qxx_basic_read(page_addr, (uint8_t *)&tempRecord, sizeof(BME280_Record_t)) == 0)
    {
      if (tempRecord.timestamp == 0xFFFFFFFF)
      {
        // Found empty page
        found_empty = true;
        current_page_address = page_addr;
        break;
      }
      else
      {
        // Page has data, check all records in this page
        for (uint8_t record_idx = 0; record_idx < RECORDS_PER_PAGE; record_idx++)
        {
          uint32_t record_addr = page_addr + (record_idx * sizeof(BME280_Record_t));
          if (w25qxx_basic_read(record_addr, (uint8_t *)&tempRecord, sizeof(BME280_Record_t)) == 0)
          {
            if (tempRecord.timestamp == 0xFFFFFFFF)
            {
              // Found first empty record in this page
              found_empty = true;
              current_page_address = W25Q64_GetPageAddress(record_addr);
              records_in_buffer = record_idx; // Resume from this position in page
              break;
            }
            else
            {
              last_valid_address = record_addr;
            }
          }
        }
        if (found_empty)
          break;
      }
    }
  }

  if (!found_empty)
  {
    printf("Flash appears to be full, enabling FIFO mode\r\n");
    flash_full = true;
    current_page_address = DATA_START_ADDRESS;
    oldest_record_address = DATA_START_ADDRESS;
  }

  newest_record_address = last_valid_address;

  printf("Next write page: 0x%08lX\r\n", current_page_address);
  printf("Records already in current page: %d\r\n", records_in_buffer);

  return current_page_address;
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
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Data Logger Task starting with Page Buffering...\r\n");
    
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
    
    // Initialize page buffer
    memset(page_buffer, 0xFF, sizeof(page_buffer));
    records_in_buffer = 0;
    current_page_address = DATA_START_ADDRESS;
    
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
        printf("W25Q64: manufacturer=0x%02X, device_id=0x%02X\r\n", manufacturer, device_id);
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
    printf("\r\n=== W25Q64 Page Buffering Configuration ===\r\n");
    printf("Page Size: %d bytes\r\n", W25Q64_PAGE_SIZE);
    printf("BME280 Record Size: %d bytes\r\n", sizeof(BME280_Record_t));
    printf("Records per Page: %d\r\n", RECORDS_PER_PAGE);
    printf("Buffer Size: %d records (%d bytes)\r\n", RECORDS_PER_PAGE, W25Q64_PAGE_SIZE);
    printf("Starting Page Address: 0x%08lX\r\n", current_page_address);
    printf("==========================================\r\n\r\n");
    
    // Find next write position by scanning existing data
    printf("Scanning flash for existing data...\r\n");
    BME280_Record_t scan_record;
    bool found_empty_page = false;
    
    // Scan page by page to find where to resume
    for (uint32_t page_addr = DATA_START_ADDRESS; page_addr < W25Q64_TOTAL_SIZE; page_addr += W25Q64_PAGE_SIZE)
    {
        // Read first record of each page
        flash_status = w25qxx_basic_read(page_addr, (uint8_t *)&scan_record, sizeof(BME280_Record_t));
        if (flash_status == 0 && scan_record.timestamp == 0xFFFFFFFF)
        {
            // Found empty page
            current_page_address = page_addr;
            found_empty_page = true;
            printf("Found empty page at 0x%08lX\r\n", page_addr);
            break;
        }
    }
    
    if (!found_empty_page)
    {
        printf("Flash appears full, starting FIFO mode from beginning\r\n");
        current_page_address = DATA_START_ADDRESS;
    }
    
    // Test basic functionality
    printf("Testing W25Q64 basic operations...\r\n");
    uint8_t test_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t read_data[8] = {0};
    
    flash_status = w25qxx_basic_write(0x00000000, test_data, 8);
    if (flash_status == 0)
    {
        flash_status = w25qxx_basic_read(0x00000000, read_data, 8);
        if (flash_status == 0)
        {
            bool test_passed = true;
            for (int i = 0; i < 8; i++)
            {
                if (test_data[i] != read_data[i])
                {
                    test_passed = false;
                    break;
                }
            }
            printf("W25Q64 basic test: %s\r\n", test_passed ? "✓ PASSED" : "✗ FAILED");
        }
    }
    
    printf("\r\n=== Starting Page-Buffered Data Logging ===\r\n");
    printf("* Records will be buffered in RAM until page is full\r\n");
    printf("* Each page write will transfer 256 bytes efficiently\r\n");
    printf("* FIFO mode will activate when flash is full\r\n");
    printf("===============================================\r\n\r\n");
    
    flash_status = w25qxx_basic_chip_erase();
    if (flash_status == 0)
    {
        printf("Flash erased successfully\r\n");
    }
    else
    {
        printf("Flash erase failed (err=%d)\r\n", flash_status);
    }

    // Main data logging loop
    uint32_t log_counter = 0;
    uint32_t total_pages_written = 0;
    
    for (;;)
    {
        if (xSemaphoreTake(bme280DataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Get BME280 data
            bme_calc_data_int_t bme_calc_data;
            convert_bme_data_to_int(&bme_comp_data, &bme_calc_data);
            
            // Prepare record
            record.timestamp = HAL_GetTick();
            record.temperature = (float)bme_calc_data.temp_whole + (float)bme_calc_data.temp_frac / 100.0f;
            record.pressure = (float)bme_calc_data.press_whole + (float)bme_calc_data.press_frac / 100.0f;
            record.humidity = (float)bme_calc_data.hum_whole + (float)bme_calc_data.hum_frac / 100.0f;
            
            xSemaphoreGive(bme280DataMutex);
            
            // Add record to page buffer (this is the key change!)
            log_counter++;
            printf("[%lu] ", log_counter);
            
            uint8_t buffer_status = W25Q64_AddRecordToPageBuffer(&record);
            if (buffer_status == 0)
            {
                // Check if a page was written (buffer was flushed)
                if (records_in_buffer == 0)
                {
                    total_pages_written++;
                    printf("Page %lu written to flash (256 bytes)\r\n", total_pages_written);
                }
            }
            else
            {
                printf("Failed to buffer record (err=%d)\r\n", buffer_status);
            }
            
            // Progress report every 20 records
            if (log_counter % 20 == 0)
            {
                printf("\r\n--- Page Buffer Status ---\r\n");
                printf("Total records logged: %lu\r\n", log_counter);
                printf("Total pages written: %lu\r\n", total_pages_written);
                printf("Records in current buffer: %d/%d\r\n", records_in_buffer, RECORDS_PER_PAGE);
                printf("Current page address: 0x%08lX\r\n", current_page_address);
                printf("Next page write at: %d more records\r\n", RECORDS_PER_PAGE - records_in_buffer);
                
                // Calculate efficiency
                uint32_t total_flash_writes = total_pages_written * W25Q64_PAGE_SIZE;
                uint32_t data_written = log_counter * sizeof(BME280_Record_t);
                printf("Flash efficiency: %lu%% (%lu data bytes / %lu flash bytes)\r\n",
                       data_written * 100 / total_flash_writes, data_written, total_flash_writes);
                printf("-------------------------\r\n\r\n");
            }
            
            // Force flush every 100 records for safety
            if (log_counter % 100 == 0 && records_in_buffer > 0)
            {
                printf("Periodic flush: saving %d buffered records\r\n", records_in_buffer);
                W25Q64_FlushPageBuffer();
                total_pages_written++;
            }
        }
        else
        {
            printf("Failed to acquire BME280 data mutex\r\n");
        }
        
        // Wait before next logging cycle
        vTaskDelay(pdMS_TO_TICKS(1000)); // Log every 5 seconds
    }
    
    // Cleanup (won't be reached in infinite loop, but good practice)
    printf("Shutting down: flushing remaining buffer...\r\n");
    W25Q64_ForceFlush();
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
