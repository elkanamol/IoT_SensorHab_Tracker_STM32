#ifndef MPU6050_TASK_H
#define MPU6050_TASK_H

#include "driver_mpu6050_basic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include <stdio.h>

// Task configuration
#define MPU6050_READ_PERIOD_MS 1000  // Read every 1 second
#define MPU6050_INIT_DELAY_MS 200
#define MPU6050_RETRY_DELAY_MS 500
#define MPU6050_STARTUP_DELAY_MS 2000
#define MPU6050_MUTEX_TIMEOUT_MS 5000

// MPU6050 data structure (float)
typedef struct {
    float accel_x, accel_y, accel_z;  // Accelerometer (g)
    float gyro_x, gyro_y, gyro_z;     // Gyroscope (dps)
    float temperature;                // Temperature (°C)
    uint32_t timestamp;               // Timestamp (ms)
} MPU6050_Data_t;

// MPU6050 data structure (integer representation)
typedef struct {
    // Accelerometer (g) - split into whole and fractional parts
    int16_t accel_x_whole, accel_x_frac;
    int16_t accel_y_whole, accel_y_frac;
    int16_t accel_z_whole, accel_z_frac;
    
    // Gyroscope (dps) - split into whole and fractional parts
    int16_t gyro_x_whole, gyro_x_frac;
    int16_t gyro_y_whole, gyro_y_frac;
    int16_t gyro_z_whole, gyro_z_frac;
    
    // Temperature (°C) - split into whole and fractional parts
    int16_t temp_whole, temp_frac;
    
    uint32_t timestamp;
} MPU6050_Data_Int_t;

typedef struct {
    SemaphoreHandle_t mutex;
    MPU6050_Data_t data;
} MPU6050_Task_Handle_t;

// Public functions
void MPU6050_Task_Start(void *argument);
void MPU6050_Task_PrintData(const MPU6050_Data_t *data);
//void MPU6050_Task_PrintDataInt(const MPU6050_Data_t *data);
// void convert_mpu6050_data_to_int(const MPU6050_Data_t *float_data, MPU6050_Data_Int_t *int_data);
// void convert_mpu6050_data_to_int_optimized(const MPU6050_Data_t *float_data, MPU6050_Data_Int_t *int_data);

#endif // MPU6050_TASK_H
