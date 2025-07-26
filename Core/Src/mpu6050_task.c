#include "driver_mpu6050_basic.h"
#include "mpu6050_task.h"
#include "sensor_conversions.h"
#include "datalogger.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>

#include "config.h"
#include "peripheral_init_helper.h"
#include "print.h"


// External I2C mutex
// extern SemaphoreHandle_t g_i2c_mutex = NULL;

/**
 * @brief Read MPU6050 sensor data with I2C mutex protection
 * @param data Pointer to data structure to fill
 * @return 0 if successful, 1 if failed
 */
static uint8_t mpu6050_read_data(SemaphoreHandle_t i2c_mutex, MPU6050_Data_t *data)
{
    DEBUG_PRINT_DEBUG("MPU6050: Reading sensor data...\r\n");
    float accel[3], gyro[3];
    uint8_t result = 1;  // Assume failure
    
    // Take I2C mutex
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(CONFIG_MPU6050_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        
        // Read accelerometer and gyroscope
        if (mpu6050_basic_read(accel, gyro) == 0) {
            // Read temperature
            if (mpu6050_basic_read_temperature(&data->temperature) == 0) {
                // Fill data structure
                data->accel_x = accel[0];
                data->accel_y = accel[1];
                data->accel_z = accel[2];
                data->gyro_x = gyro[0];
                data->gyro_y = gyro[1];
                data->gyro_z = gyro[2];
                data->timestamp = HAL_GetTick();
                result = 0;  // Success
            }
        }
        
        // Release I2C mutex
        if (xSemaphoreGive(i2c_mutex) != pdTRUE) {
            DEBUG_PRINT_DEBUG("MPU6050: Failed to give I2C mutex\r\n");
        }
    }
    
    return result;
}

/**
 * @brief Perform full initialization of the MPU6050 sensor with mutex protection
 * @param context Pointer to the I2C mutex semaphore
 * @return 0 on successful initialization, -1 on failure
 * @note Attempts to initialize the MPU6050 with a retry mechanism
 *       Requires I2C mutex to be taken before initialization
 */
static int8_t MPU6050_FullInit(void *context)
{
    if (context == NULL) {
        DEBUG_PRINT_ERROR("MPU6050: I2C mutex is NULL\r\n");
        return -1;
    }
    SemaphoreHandle_t i2c_mutex = (SemaphoreHandle_t)context;
    int8_t result = -1;
    DEBUG_PRINT_DEBUG("MPU6050: Trying to take I2C mutex for full init\r\n");
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(MPU6050_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        DEBUG_PRINT_DEBUG("MPU6050: Failed to take I2C mutex for full init\r\n");
        return -1;
    }
    DEBUG_PRINT_DEBUG("MPU6050: I2C mutex acquired for full init\r\n");
    vTaskDelay(pdMS_TO_TICKS(MPU6050_STARTUP_DELAY_MS));
    result = mpu6050_basic_init(CONFIG_MPU6050_I2C_ADDRESS);
    if (result != 0) {
        DEBUG_PRINT_DEBUG("MPU6050: First init attempt failed, retrying...\r\n");
        vTaskDelay(pdMS_TO_TICKS(MPU6050_RETRY_DELAY_MS));
        result = mpu6050_basic_init(CONFIG_MPU6050_I2C_ADDRESS);
    }
    xSemaphoreGive(i2c_mutex);
    DEBUG_PRINT_DEBUG("MPU6050: I2C mutex released after full init\r\n");
    return (result == 0) ? 0 : -1;
}


/**
 * @brief Main task function for MPU6050 sensor management
 * @param argument Pointer to I2C mutex semaphore for sensor communication
 * @details Handles MPU6050 sensor initialization, periodic data reading,
 *          error handling, and data logging. Runs continuously with 
 *          configurable read period and error retry mechanism.
 * @note Task will self-delete if initialization fails or encounters 
 *       persistent read errors beyond the configured threshold.
 */
void MPU6050_Task_Start(void *argument)
{
    MPU6050_Task_Handle_t MPU6050_Task_Handle;
    TickType_t last_wake_time;
    
    DEBUG_PRINT_DEBUG("MPU6050: Task started\r\n");    
    // Cast the argument
    MPU6050_Task_Handle.mutex = (SemaphoreHandle_t)argument;
    configASSERT(MPU6050_Task_Handle.mutex != NULL);
    // Validate the handle before using it
    if (MPU6050_Task_Handle.mutex == NULL) {
        DEBUG_PRINT_ERROR("MPU6050: ERROR - Mutex handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }
    vTaskDelay(CONFIG_MPU6050_STARTUP_DELAY_MS);

    InitRetryConfig_t mpu6050_init_cfg = {
        .init_func = MPU6050_FullInit,
        .context = MPU6050_Task_Handle.mutex,
        .mutex = MPU6050_Task_Handle.mutex,
        .max_retries = CONFIG_TASK_ERROR_THRESHOLD,
        .retry_delay_ms = CONFIG_TASK_ERROR_RETRY_DELAY_MS,
        .backoff_factor = CONFIG_TASK_ERROR_BACKOFF_FACTOR,
        .mutex_timeout_ms = CONFIG_TASK_ERROR_MUTEX_TIMEOUT_MS};
    InitResult_t res = Peripheral_InitWithRetry(&mpu6050_init_cfg);
    if (res != INIT_SUCCESS) {
      DEBUG_PRINT_ERROR("MPU6050: Initialization failed after retries\r\n");
      vTaskDelete(NULL);
      return;
    }
    DEBUG_PRINT_DEBUG("MPU6050: Sensor initialized successfully\r\n");
    
    // Initialize timing
    last_wake_time = xTaskGetTickCount();

    int error_count = 0;
    // Main task loop
    while (1) {
      // Read sensor data
      if (mpu6050_read_data(MPU6050_Task_Handle.mutex,
                            &MPU6050_Task_Handle.data) == 0) {
        error_count = 0;
        // Print data using integer representation
        // MPU6050_Task_PrintDataInt(&MPU6050_Task_Handle.data);  // ← Use
        // integer printing
        MPU6050_Task_PrintData(&MPU6050_Task_Handle.data);

        // Send to unified datalogger
        if (DataLogger_UpdateMPU6050Data(
                MPU6050_Task_Handle.data.accel_x,
                MPU6050_Task_Handle.data.accel_y,
                MPU6050_Task_Handle.data.accel_z,
                MPU6050_Task_Handle.data.gyro_x,
                MPU6050_Task_Handle.data.gyro_y,
                MPU6050_Task_Handle.data.gyro_z,
                MPU6050_Task_Handle.data.temperature) != pdTRUE) {
          DEBUG_PRINT_ERROR("MPU6050: Failed to update datalogger\r\n");
        }
      } else {
        error_count++;
        DEBUG_PRINT_ERROR(
            "MPU6050: ERROR - Failed to read sensor data %d times\r\n",
            error_count);
        if (error_count >= CONFIG_TASK_ERROR_THRESHOLD) {
          DEBUG_PRINT_ERROR(
              "MPU6050: Too many errors, attempting reinit...\r\n");
          MPU6050_FullInit(MPU6050_Task_Handle.mutex);
          error_count = 0;
        }
      }
      // Wait for next cycle
      vTaskDelayUntil(&last_wake_time,
                      pdMS_TO_TICKS(CONFIG_MPU6050_READ_PERIOD_MS));
    }
}


/**
 * @brief Print sensor data to console (keep original for reference)
 * @param data Pointer to sensor data
 */
void MPU6050_Task_PrintData(const MPU6050_Data_t *data)
{
    DEBUG_PRINT_INFO("MPU6050 [%d ms]: Accel(%.2f, %.2f, %.2f)g Gyro(%.1f, %.1f, %.1f)dps Temp:%.1f°C\r\n",
           (int)data->timestamp,
           data->accel_x, data->accel_y, data->accel_z,
           data->gyro_x, data->gyro_y, data->gyro_z,
           data->temperature);
}
