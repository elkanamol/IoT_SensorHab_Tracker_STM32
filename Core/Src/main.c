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
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
static W25Q_HandleTypeDef hflash;
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

// Add this to your private variables section
#define MAX_RECORDS_IN_SECTOR (W25Q_DEFAULT_SECTOR_SIZE / sizeof(BME280_Record_t))
static BME280_Record_t lastReadRecord;
static uint32_t nextWriteAddress = 0;
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
static W25Q_StatusTypeDef W25Q_Flash_Init(void);
void PrintBufferHex(const uint8_t *buffer, uint32_t size, uint32_t baseAddress);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Initialize W25Q Flash
static W25Q_StatusTypeDef W25Q_Flash_Init(void)
{
  W25Q_ConfigTypeDef flashConfig;
  W25Q_StatusTypeDef status;
  uint8_t mfgID;
  uint16_t devID;

  // Configure the flash
  flashConfig.spi_handle = &hspi1;
  flashConfig.cs_port = SPI1_CS_GPIO_Port;
  flashConfig.cs_pin = SPI1_CS_Pin;
#ifdef W25QXX_USE_DMA
  // DMA configuration (if used)
  flashConfig.use_dma = 1; // Set to 1 to use DMA
  flashConfig.hdmatx = hspi1.hdmatx;
  flashConfig.hdmarx = hspi1.hdmarx;

  // Optional callbacks
  flashConfig.TxCpltCallback = HAL_SPI_TxCpltCallback;
  flashConfig.RxCpltCallback = HAL_SPI_RxCpltCallback;
  flashConfig.TxRxCpltCallback = HAL_SPI_TxRxCpltCallback;
#endif
  // Flash parameters (can use defaults)
  flashConfig.page_size = W25Q_DEFAULT_PAGE_SIZE;
  flashConfig.sector_size = W25Q_DEFAULT_SECTOR_SIZE;
  flashConfig.block_32k_size = W25Q_DEFAULT_BLOCK32_SIZE;
  flashConfig.block_64k_size = W25Q_DEFAULT_BLOCK64_SIZE;
  flashConfig.total_size_bytes = 8 * 1024 * 1024; // 8MB for W25Q64

  // SPI mode
  flashConfig.spi_mode = 1; // 1 for standard SPI

#ifdef W25QXX_USE_RTOS
  // IMPORTANT FIX: Initialize without RTOS first, then enable RTOS later
  flashConfig.use_rtos = 0; // First initialize without RTOS
#endif
  // Initialize the flash without RTOS support
  status = W25Q_Init(&hflash, &flashConfig);
  if (status != W25Q_OK)
  {
    printf("W25Q Flash initialization failed with error: %d\r\n", status);
    return status;
  }
  status = W25Q_ReadJEDECID(&hflash, &mfgID, &devID);
  if (status != W25Q_OK)
  {
    printf("Failed to read W25Q Flash JEDEC ID: %d\r\n", status);
    return status;
  }
  printf("W25Q Flash detected: Manufacturer ID: 0x%02X, Device ID: 0x%04X\r\n", mfgID, devID);

  return W25Q_OK;
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
  // Create the uAT parser task
  xTaskCreate(uAT_Task,
              "uAT_Task",
              512, // stack depth in words
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);

  // // (Optional) your other application tasks
  // xTaskCreate(App_Task,
  //             "AppTask",
  //             512,
  //             NULL,
  //             tskIDLE_PRIORITY + 1,
  //             NULL);

  xTaskCreate(MQTT_Task,
              "MQTT_Task",
              512 * 2,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);

  // Create the BME280 task
  xTaskCreate(StartBme280Task,
              "BME280_Task",
              512, // Adjust stack size as needed
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);

  // Create the Data Logger task
  xTaskCreate(StartDataLoggerTask,
              "DataLogger",
              512 * 2, // Larger stack for this task
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
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

void StartW25QTestTask(void *argument)
{
  (void)argument;
  uint8_t testData[256];
  uint8_t readData[256];
  W25Q_StatusTypeDef status;

  // Initialize test data
  for (int i = 0; i < 256; i++)
  {
    testData[i] = i;
  }

  // Wait a bit for other initializations to complete
  vTaskDelay(pdMS_TO_TICKS(1000));

  printf("W25Q Flash test starting...\r\n");

  // Erase a sector at address 0
  printf("Erasing sector at address 0...\r\n");
  status = W25Q_EraseSector_RTOS(&hflash, 0);
  if (status != W25Q_OK)
  {
    printf("Sector erase failed: %d\r\n", status);
    vTaskDelete(NULL);
    return;
  }

  // Write test data to the first page
  printf("Writing test data to first page...\r\n");
  status = W25Q_WritePage_RTOS(&hflash, 0, testData, 256);
  if (status != W25Q_OK)
  {
    printf("Page write failed: %d\r\n", status);
    vTaskDelete(NULL);
    return;
  }

  // Read back the data
  printf("Reading data back...\r\n");
  status = W25Q_Read_RTOS(&hflash, 0, readData, 256);
  if (status != W25Q_OK)
  {
    printf("Read failed: %d\r\n", status);
    vTaskDelete(NULL);
    return;
  }

  // Verify the data
  bool dataMatch = true;
  for (int i = 0; i < 256; i++)
  {
    if (readData[i] != testData[i])
    {
      printf("Data mismatch at index %d: expected %d, got %d\r\n", i, testData[i], readData[i]);
      dataMatch = false;
      break;
    }
  }

  if (dataMatch)
  {
    printf("W25Q Flash test passed! Data verified correctly.\r\n");
  }
  else
  {
    printf("W25Q Flash test failed! Data verification error.\r\n");
  }

  // Periodic status check
  for (;;)
  {
    uint8_t statusReg;
    W25Q_ReadStatusRegister1(&hflash, &statusReg);
    printf("W25Q Flash status: 0x%02X\r\n", statusReg);
    vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
  }
}

// Add this function implementation to your code
void StartDataLoggerTask(void *argument)
{
  (void)argument;
  W25Q_StatusTypeDef flash_status;
  BME280_Record_t record;
  uint32_t lastReadAddress = 0;

  // Wait for other tasks to initialize
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

  // 1. Initialize the W25Q Flash - using the non-RTOS version
  if (W25Q_Flash_Init() != W25Q_OK)
  {
    printf("Failed to initialize W25Q Flash\r\n");
    vTaskDelete(NULL);
    return;
  }
  printf("W25Q Flash initialized successfully\r\n");

  // Erase the first sector to prepare for data storage - using non-RTOS version
  printf("Erasing first sector...\r\n");
  flash_status = W25Q_EraseSector(&hflash, 0);
  if (flash_status != W25Q_OK)
  {
    printf("Failed to erase first sector: %d\r\n", flash_status);
    vTaskDelete(NULL);
    return;
  }
  printf("First sector erased successfully\r\n");

  // Find the next write address by scanning the first sector
  // This allows resuming after power cycles
  nextWriteAddress = 0;
  BME280_Record_t tempRecord;

  for (uint32_t addr = 0; addr < W25Q_DEFAULT_SECTOR_SIZE; addr += sizeof(BME280_Record_t))
  {
    flash_status = W25Q_Read(&hflash, addr, (uint8_t *)&tempRecord, sizeof(BME280_Record_t));
    if (flash_status != W25Q_OK)
    {
      printf("Error reading flash at address 0x%08lX: %d\r\n", addr, flash_status);
      break;
    }

    // If we find an empty record (timestamp is 0xFFFFFFFF when erased), we've found our write position
    if (tempRecord.timestamp == 0xFFFFFFFF)
    {
      nextWriteAddress = addr;
      break;
    }

    // Keep track of the last valid record we've read
    lastReadAddress = addr;
  }

  printf("Next write address: 0x%08lX\r\n", nextWriteAddress);

  // If we found existing records, read the last one
  if (lastReadAddress > 0)
  {
    flash_status = W25Q_Read(&hflash, lastReadAddress, (uint8_t *)&lastReadRecord, sizeof(BME280_Record_t));
    if (flash_status == W25Q_OK)
    {
      printf("Last record found - Time: %lu ms, Temp: %.2f°C, Press: %.2f hPa, Hum: %.2f%%\r\n",
             lastReadRecord.timestamp, lastReadRecord.temperature,
             lastReadRecord.pressure, lastReadRecord.humidity);
    }
  }

  // Main task loop
  for (;;)
  {
    // 2. Get BME280 values (using mutex for thread safety)
    if (xSemaphoreTake(bme280DataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      // Convert the BME280 data to our record format
      bme_calc_data_int_t bme_calc_data;
      convert_bme_data_to_int(&bme_comp_data, &bme_calc_data);

      record.timestamp = HAL_GetTick();
      record.temperature = (float)bme_calc_data.temp_whole + (float)bme_calc_data.temp_frac / 100.0f;
      record.pressure = (float)bme_calc_data.press_whole + (float)bme_calc_data.press_frac / 100.0f;
      record.humidity = (float)bme_calc_data.hum_whole + (float)bme_calc_data.hum_frac / 100.0f;

      // Release the mutex as soon as we've copied the data
      xSemaphoreGive(bme280DataMutex);

      // 3. Write to flash (only if we have space in the first sector)
      if (nextWriteAddress < W25Q_DEFAULT_SECTOR_SIZE)
      {
        printf("Writing record to flash at address 0x%08lX\r\n", nextWriteAddress);

        // Write the record to flash using non-RTOS version
        flash_status = W25Q_WritePage(&hflash, nextWriteAddress,
                                      (uint8_t *)&record, sizeof(BME280_Record_t));

        if (flash_status == W25Q_OK)
        {
          // 4. Read back the written data to verify
          flash_status = W25Q_Read(&hflash, nextWriteAddress,
                                   (uint8_t *)&lastReadRecord, sizeof(BME280_Record_t));

          if (flash_status == W25Q_OK)
          {
            // 5. Print the read data in both formatted and hex formats
            printf("Record written and read back - Time: %lu ms, Temp: %.2f°C, Press: %.2f hPa, Hum: %.2f%%\r\n",
                   lastReadRecord.timestamp, lastReadRecord.temperature,
                   lastReadRecord.pressure, lastReadRecord.humidity);
            // Print the raw bytes of the record in hex format
            PrintBufferHex((uint8_t *)&lastReadRecord, sizeof(BME280_Record_t), nextWriteAddress);

            // Update next write address
            nextWriteAddress += sizeof(BME280_Record_t);

            // If we've reached the end of the sector, wrap around
            if (nextWriteAddress >= W25Q_DEFAULT_SECTOR_SIZE)
            {
              printf("First sector full, will erase and start over on next cycle\r\n");

              // Erase the sector for the next cycle
              flash_status = W25Q_EraseSector(&hflash, 0);
              if (flash_status == W25Q_OK)
              {
                nextWriteAddress = 0;
                printf("First sector erased for reuse\r\n");
              }
              else
              {
                printf("Failed to erase first sector: %d\r\n", flash_status);
              }
            }
          }
          else
          {
            printf("Failed to read back record: %d\r\n", flash_status);
          }
        }
        else
        {
          printf("Failed to write record: %d\r\n", flash_status);
        }
      }
      else
      {
        printf("First sector full, reading and printing sector before erasing...\r\n");

        // Read and print the entire sector
        PrintBufferHex((uint8_t *)&lastReadRecord, sizeof(BME280_Record_t), nextWriteAddress);

        flash_status = W25Q_EraseSector(&hflash, 0);
        if (flash_status == W25Q_OK)
        {
          nextWriteAddress = 0;
          printf("First sector erased for reuse\r\n");
        }
        else
        {
          printf("Failed to erase first sector: %d\r\n", flash_status);
        }
      }

      // Every 5 cycles, read and print the sector contents
      static uint8_t cycleCount = 0;
      if (++cycleCount >= 5)
      {
        cycleCount = 0;
        printf("Periodic sector read:\r\n");
        PrintBufferHex((uint8_t *)&lastReadRecord, sizeof(BME280_Record_t), nextWriteAddress);
      }
    }
    else
    {
      printf("Failed to acquire BME280 data mutex\r\n");
    }

    // Wait before next logging cycle (e.g., log every 10 seconds)
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
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
