#include "driver_mpu6050_basic.h"
#include "mpu6050_task.h"
#include "datalogger.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>

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
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        
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
        xSemaphoreGive(i2c_mutex);
    }
    
    return result;
}

/**
 * @brief Initialize MPU6050 sensor with I2C mutex protection
 * @return 0 if successful, 1 if failed
 */
static uint8_t mpu6050_init_sensor(SemaphoreHandle_t i2c_mutex)
{
    uint8_t result = 1;  // Assume failure

    vTaskDelay(pdMS_TO_TICKS(200));
    result = mpu6050_basic_init(MPU6050_ADDRESS_AD0_LOW);
    // If first attempt fails, try once more with longer delay
    if (result != 0)
    {
        printf("MPU6050: First init attempt failed, retrying...\r\n");
        vTaskDelay(pdMS_TO_TICKS(500)); // Longer delay
        result = mpu6050_basic_init(MPU6050_ADDRESS_AD0_LOW);
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
    printf("MPU6050: Received argument = %p\r\n", argument);
    
    // Cast the argument
    MPU6050_Task_Handle.mutex = (SemaphoreHandle_t)argument;
    printf("MPU6050: Mutex handle = %p\r\n", (void*)MPU6050_Task_Handle.mutex);
    configASSERT(MPU6050_Task_Handle.mutex != NULL);
    // Validate the handle before using it
    if (MPU6050_Task_Handle.mutex == NULL) {
        printf("MPU6050: ERROR - Mutex handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }
    
    // Additional validation - check if it's a valid FreeRTOS object
    // Try a non-blocking take first
    printf("MPU6050: Testing mutex with non-blocking take...\r\n");
    BaseType_t test_result = xSemaphoreTake(MPU6050_Task_Handle.mutex, 0);
    if (test_result == pdTRUE) {
        printf("MPU6050: Non-blocking take successful, giving back...\r\n");
        xSemaphoreGive(MPU6050_Task_Handle.mutex);
    } else {
        printf("MPU6050: Non-blocking take failed (expected if BME280 has it)\r\n");
    }
    
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait longer to let BME280 finish init
    
    printf("MPU6050: Attempting to take mutex with timeout...\r\n");
    
    // Use timeout instead of portMAX_DELAY for debugging
    if (xSemaphoreTake(MPU6050_Task_Handle.mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        printf("MPU6050: Failed to take I2C mutex within 5 seconds\r\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("MPU6050: Successfully acquired mutex!\r\n");
    
    // Initialize sensor
    if (mpu6050_init_sensor(MPU6050_Task_Handle.mutex) != 0) {
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
        // if (xSemaphoreTake(MPU6050_Task_Handle.mutex, portMAX_DELAY) != pdTRUE)
        // {
        //     printf("MPU6050: Failed to take I2C mutex\r\n");
        //     continue;
        // }
        
        // Read sensor data
        if (mpu6050_read_data(MPU6050_Task_Handle.mutex, &MPU6050_Task_Handle.data) == 0) {
            // Print data using integer representation
            MPU6050_Task_PrintDataInt(&MPU6050_Task_Handle.data);  // ← Use integer printing
            
            // Send to unified datalogger
            if (DataLogger_UpdateMPU6050Data(
                MPU6050_Task_Handle.data.accel_x,
                MPU6050_Task_Handle.data.accel_y,
                MPU6050_Task_Handle.data.accel_z,
                MPU6050_Task_Handle.data.gyro_x,
                MPU6050_Task_Handle.data.gyro_y,
                MPU6050_Task_Handle.data.gyro_z,
                MPU6050_Task_Handle.data.temperature) != pdTRUE) {
                printf("MPU6050: Failed to update datalogger\r\n");
            }
            
        } else {
            printf("MPU6050: ERROR - Failed to read sensor data\r\n");
        }
        
        // if (xSemaphoreGive(MPU6050_Task_Handle.mutex) != pdTRUE)
        // {
        //     printf("MPU6050: Failed to give I2C mutex\r\n");
        // }
        
        // Wait for next cycle
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MPU6050_READ_PERIOD_MS));
    }
}

/**
 * @brief Convert MPU6050 float data to integer representation
 * @param float_data Pointer to float data structure
 * @param int_data Pointer to integer data structure to fill
 */
void convert_mpu6050_data_to_int(const MPU6050_Data_t *float_data, MPU6050_Data_Int_t *int_data)
{
    if (float_data == NULL || int_data == NULL) {
        return;
    }
    
    // Convert accelerometer data (g)
    int_data->accel_x_whole = (int16_t)float_data->accel_x;
    int_data->accel_x_frac = (int16_t)((float_data->accel_x - int_data->accel_x_whole) * 1000);
    
    int_data->accel_y_whole = (int16_t)float_data->accel_y;
    int_data->accel_y_frac = (int16_t)((float_data->accel_y - int_data->accel_y_whole) * 1000);
    
    int_data->accel_z_whole = (int16_t)float_data->accel_z;
    int_data->accel_z_frac = (int16_t)((float_data->accel_z - int_data->accel_z_whole) * 1000);
    
    // Convert gyroscope data (dps)
    int_data->gyro_x_whole = (int16_t)float_data->gyro_x;
    int_data->gyro_x_frac = (int16_t)((float_data->gyro_x - int_data->gyro_x_whole) * 100);
    
    int_data->gyro_y_whole = (int16_t)float_data->gyro_y;
    int_data->gyro_y_frac = (int16_t)((float_data->gyro_y - int_data->gyro_y_whole) * 100);
    
    int_data->gyro_z_whole = (int16_t)float_data->gyro_z;
    int_data->gyro_z_frac = (int16_t)((float_data->gyro_z - int_data->gyro_z_whole) * 100);
    
    // Convert temperature data (°C)
    int_data->temp_whole = (int16_t)float_data->temperature;
    int_data->temp_frac = (int16_t)((float_data->temperature - int_data->temp_whole) * 100);
    
    // Handle negative fractional parts
    if (int_data->accel_x_frac < 0) int_data->accel_x_frac = -int_data->accel_x_frac;
    if (int_data->accel_y_frac < 0) int_data->accel_y_frac = -int_data->accel_y_frac;
    if (int_data->accel_z_frac < 0) int_data->accel_z_frac = -int_data->accel_z_frac;
    if (int_data->gyro_x_frac < 0) int_data->gyro_x_frac = -int_data->gyro_x_frac;
    if (int_data->gyro_y_frac < 0) int_data->gyro_y_frac = -int_data->gyro_y_frac;
    if (int_data->gyro_z_frac < 0) int_data->gyro_z_frac = -int_data->gyro_z_frac;
    if (int_data->temp_frac < 0) int_data->temp_frac = -int_data->temp_frac;
    
    int_data->timestamp = float_data->timestamp;
}

/**
 * @brief Print MPU6050 sensor data using integer representation
 * @param data Pointer to float sensor data
 */
void MPU6050_Task_PrintDataInt(const MPU6050_Data_t *data)
{
    MPU6050_Data_Int_t int_data;
    
    if (data == NULL) {
        printf("MPU6050: Invalid data\r\n");
        return;
    }
    
    // Convert float data to integer representation
    convert_mpu6050_data_to_int(data, &int_data);
    
    // Print using integer arithmetic
    printf("MPU6050 [%lu ms]: Accel(%d.%03d, %d.%03d, %d.%03d)g Gyro(%d.%02d, %d.%02d, %d.%02d)dps Temp: %d.%02d°C\r\n",
           int_data.timestamp,
           int_data.accel_x_whole, int_data.accel_x_frac,
           int_data.accel_y_whole, int_data.accel_y_frac,
           int_data.accel_z_whole, int_data.accel_z_frac,
           int_data.gyro_x_whole, int_data.gyro_x_frac,
           int_data.gyro_y_whole, int_data.gyro_y_frac,
           int_data.gyro_z_whole, int_data.gyro_z_frac,
           int_data.temp_whole, int_data.temp_frac);
}

/**
 * @brief Print sensor data to console (keep original for reference)
 * @param data Pointer to sensor data
 */
void MPU6050_Task_PrintData(const MPU6050_Data_t *data)
{
    printf("MPU6050 [%lu ms]: Accel(%.2f, %.2f, %.2f)g Gyro(%.1f, %.1f, %.1f)dps Temp:%.1f°C\r\n",
           data->timestamp,
           data->accel_x, data->accel_y, data->accel_z,
           data->gyro_x, data->gyro_y, data->gyro_z,
           data->temperature);
}
