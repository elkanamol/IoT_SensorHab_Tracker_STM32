#include "driver_mpu6050_basic.h"
#include "mpu6050_task.h"
#include "sensor_conversions.h"
#include "datalogger.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>

#include "config.h"

// External I2C mutex
// extern SemaphoreHandle_t g_i2c_mutex = NULL;

/**
 * @brief Read MPU6050 sensor data with I2C mutex protection
 * @param data Pointer to data structure to fill
 * @return 0 if successful, 1 if failed
 */
static uint8_t mpu6050_read_data(SemaphoreHandle_t i2c_mutex, MPU6050_Data_t *data)
{
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
            printf("MPU6050: Failed to give I2C mutex\r\n");
        }
    }
    
    return result;
}

/**
 * @brief Initialize MPU6050 sensor with I2C mutex protection
 * @return 0 if successful, 1 if failed
 */
static uint8_t mpu6050_init_sensor(void)
{
    uint8_t result = 1;  // Assume failure

<<<<<<< Updated upstream
    vTaskDelay(pdMS_TO_TICKS(MPU6050_INIT_DELAY_MS));
<<<<<<< HEAD
    result = mpu6050_basic_init(MPU6050_ADDRESS_AD0_LOW);
=======
    vTaskDelay(pdMS_TO_TICKS(CONFIG_MPU6050_STARTUP_DELAY_MS));
    result = mpu6050_basic_init(CONFIG_MPU6050_I2C_ADDRESS);
>>>>>>> Stashed changes
=======
    result = mpu6050_basic_init(CONFIG_MPU6050_I2C_ADDRESS);
>>>>>>> 7a8ab737b2e350dd096e98fab5f8517cf1fb63a5
    // If first attempt fails, try once more with longer delay
    if (result != 0)
    {
        printf("MPU6050: First init attempt failed, retrying...\r\n");
<<<<<<< Updated upstream
        vTaskDelay(pdMS_TO_TICKS(MPU6050_RETRY_DELAY_MS)); // Longer delay
<<<<<<< HEAD
        result = mpu6050_basic_init(MPU6050_ADDRESS_AD0_LOW);
=======
        vTaskDelay(pdMS_TO_TICKS(CONFIG_MPU6050_RETRY_DELAY_MS)); // Longer delay
        result = mpu6050_basic_init(CONFIG_MPU6050_I2C_ADDRESS);
>>>>>>> Stashed changes
=======
        result = mpu6050_basic_init(CONFIG_MPU6050_I2C_ADDRESS);
>>>>>>> 7a8ab737b2e350dd096e98fab5f8517cf1fb63a5
    }
    return result;
}

/**
 * @brief Main MPU6050 task function
 * @param argument Task argument (I2C mutex)
 */
void MPU6050_Task_Start(void *argument)
{
    MPU6050_Task_Handle_t MPU6050_Task_Handle;
    TickType_t last_wake_time;
    
    printf("MPU6050: Task started\r\n");    
    // Cast the argument
    MPU6050_Task_Handle.mutex = (SemaphoreHandle_t)argument;
    configASSERT(MPU6050_Task_Handle.mutex != NULL);
    // Validate the handle before using it
    if (MPU6050_Task_Handle.mutex == NULL) {
        printf("MPU6050: ERROR - Mutex handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }
    
    
    vTaskDelay(pdMS_TO_TICKS(CONFIG_MPU6050_STARTUP_DELAY_MS)); // Wait longer to let BME280 finish init
    
    printf("MPU6050: Attempting to take mutex with timeout...\r\n");
    
    // Use timeout instead of portMAX_DELAY for debugging
    if (xSemaphoreTake(MPU6050_Task_Handle.mutex,
                       pdMS_TO_TICKS(CONFIG_MPU6050_MUTEX_TIMEOUT_MS)) !=
        pdTRUE) {
      printf("MPU6050: Failed to take I2C mutex within timeout\r\n");
      vTaskDelete(NULL);
      return;
    }

    printf("MPU6050: Successfully acquired mutex!\r\n");
    
    // Initialize sensor
    if (mpu6050_init_sensor() != 0) {
        printf("MPU6050: ERROR - Initialization failed\r\n");
        xSemaphoreGive(MPU6050_Task_Handle.mutex);
        vTaskDelete(NULL);
        return;
    }
    
    if (xSemaphoreGive(MPU6050_Task_Handle.mutex) != pdTRUE)
    {
        printf("MPU6050: Failed to give I2C mutex\r\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("MPU6050: Sensor initialized successfully\r\n");
    
    // Initialize timing
    last_wake_time = xTaskGetTickCount();
    
    // Main task loop
    while (1) {
        
        // Read sensor data
        if (mpu6050_read_data(MPU6050_Task_Handle.mutex, &MPU6050_Task_Handle.data) == 0) {
            // Print data using integer representation
            // MPU6050_Task_PrintDataInt(&MPU6050_Task_Handle.data);  // ← Use integer printing
            MPU6050_Task_PrintData(&MPU6050_Task_Handle.data);

            // Send to unified datalogger
            if (DataLogger_UpdateMPU6050Data(
                    MPU6050_Task_Handle.data.accel_x,
                    MPU6050_Task_Handle.data.accel_y,
                    MPU6050_Task_Handle.data.accel_z,
                    MPU6050_Task_Handle.data.gyro_x,
                    MPU6050_Task_Handle.data.gyro_y,
                    MPU6050_Task_Handle.data.gyro_z,
                    MPU6050_Task_Handle.data.temperature) != pdTRUE)
            {
                printf("MPU6050: Failed to update datalogger\r\n");
            }
        } else {
            printf("MPU6050: ERROR - Failed to read sensor data\r\n");
        }
        
        // Wait for next cycle
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONFIG_MPU6050_READ_PERIOD_MS));
    }
}


/**
 * @brief Print sensor data to console (keep original for reference)
 * @param data Pointer to sensor data
 */
void MPU6050_Task_PrintData(const MPU6050_Data_t *data)
{
    printf("MPU6050 [%d ms]: Accel(%.2f, %.2f, %.2f)g Gyro(%.1f, %.1f, %.1f)dps Temp:%.1f°C\r\n",
           (int)data->timestamp,
           data->accel_x, data->accel_y, data->accel_z,
           data->gyro_x, data->gyro_y, data->gyro_z,
           data->temperature);
}
