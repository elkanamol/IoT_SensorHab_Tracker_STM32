#include "bme280_porting.h"
#include "sensor_conversions.h"

/**
 * @brief Convert BME280 float data to integer representation
 */
void convert_bme_data_to_int(const struct bme280_data *float_data, bme_calc_data_int_t *int_data)
{
    if (float_data == NULL || int_data == NULL) {
        return;
    }
    
    CONVERT_SENSOR_FIELD(float_data->temperature, int_data->temp_whole, int_data->temp_frac, 100);
    CONVERT_SENSOR_FIELD(float_data->pressure, int_data->press_whole, int_data->press_frac, 100);
    CONVERT_SENSOR_FIELD(float_data->humidity, int_data->hum_whole, int_data->hum_frac, 100);
}

/**
 * @brief Optimized MPU6050 float to integer conversion
 */
void convert_mpu6050_data_to_int_optimized(const MPU6050_Data_t *float_data, MPU6050_Data_Int_t *int_data)
{
    if (float_data == NULL || int_data == NULL) {
        return;
    }
    
    // Convert all accelerometer data (3 decimal places)
    CONVERT_SENSOR_FIELD(float_data->accel_x, int_data->accel_x_whole, int_data->accel_x_frac, 1000);
    CONVERT_SENSOR_FIELD(float_data->accel_y, int_data->accel_y_whole, int_data->accel_y_frac, 1000);
    CONVERT_SENSOR_FIELD(float_data->accel_z, int_data->accel_z_whole, int_data->accel_z_frac, 1000);
    
    // Convert all gyroscope data (2 decimal places)
    CONVERT_SENSOR_FIELD(float_data->gyro_x, int_data->gyro_x_whole, int_data->gyro_x_frac, 100);
    CONVERT_SENSOR_FIELD(float_data->gyro_y, int_data->gyro_y_whole, int_data->gyro_y_frac, 100);
    CONVERT_SENSOR_FIELD(float_data->gyro_z, int_data->gyro_z_whole, int_data->gyro_z_frac, 100);
    
    // Convert temperature (2 decimal places)
    CONVERT_SENSOR_FIELD(float_data->temperature, int_data->temp_whole, int_data->temp_frac, 100);
    
    int_data->timestamp = float_data->timestamp;
}

/**
 * @brief Optimized combined sensor data conversion
 */
void convert_combined_sensor_data_to_int_optimized(const SensorData_Combined_t *float_data, 
                                                   SensorData_Combined_Int_t *int_data)
{
    if (float_data == NULL || int_data == NULL) {
        return;
    }
    
    // Convert BME280 data
    if (float_data->bme_valid) {
        CONVERT_SENSOR_FIELD(float_data->bme_temperature, int_data->bme_temp_whole, int_data->bme_temp_frac, 100);
        CONVERT_SENSOR_FIELD(float_data->bme_pressure, int_data->bme_press_whole, int_data->bme_press_frac, 100);
        CONVERT_SENSOR_FIELD(float_data->bme_humidity, int_data->bme_hum_whole, int_data->bme_hum_frac, 100);
        int_data->bme_valid = 1;
    } else {
        ZERO_SENSOR_FIELDS(int_data->bme_temp_whole, int_data->bme_temp_frac);
        ZERO_SENSOR_FIELDS(int_data->bme_press_whole, int_data->bme_press_frac);
        ZERO_SENSOR_FIELDS(int_data->bme_hum_whole, int_data->bme_hum_frac);
        int_data->bme_valid = 0;
    }
    
    // Convert MPU6050 data
    if (float_data->mpu_valid) {
        // Accelerometer (3 decimal places)
        CONVERT_SENSOR_FIELD(float_data->mpu_accel_x, int_data->mpu_accel_x_whole, int_data->mpu_accel_x_frac, 1000);
        CONVERT_SENSOR_FIELD(float_data->mpu_accel_y, int_data->mpu_accel_y_whole, int_data->mpu_accel_y_frac, 1000);
        CONVERT_SENSOR_FIELD(float_data->mpu_accel_z, int_data->mpu_accel_z_whole, int_data->mpu_accel_z_frac, 1000);
        
        // Gyroscope (2 decimal places)
        CONVERT_SENSOR_FIELD(float_data->mpu_gyro_x, int_data->mpu_gyro_x_whole, int_data->mpu_gyro_x_frac, 100);
        CONVERT_SENSOR_FIELD(float_data->mpu_gyro_y, int_data->mpu_gyro_y_whole, int_data->mpu_gyro_y_frac, 100);
        CONVERT_SENSOR_FIELD(float_data->mpu_gyro_z, int_data->mpu_gyro_z_whole, int_data->mpu_gyro_z_frac, 100);
        
        // Temperature (2 decimal places)
        CONVERT_SENSOR_FIELD(float_data->mpu_temperature, int_data->mpu_temp_whole, int_data->mpu_temp_frac, 100);
        int_data->mpu_valid = 1;
    } else {
        // Zero all MPU fields
        ZERO_SENSOR_FIELDS(int_data->mpu_accel_x_whole, int_data->mpu_accel_x_frac);
        ZERO_SENSOR_FIELDS(int_data->mpu_accel_y_whole, int_data->mpu_accel_y_frac);
        ZERO_SENSOR_FIELDS(int_data->mpu_accel_z_whole, int_data->mpu_accel_z_frac);
        ZERO_SENSOR_FIELDS(int_data->mpu_gyro_x_whole, int_data->mpu_gyro_x_frac);
        ZERO_SENSOR_FIELDS(int_data->mpu_gyro_y_whole, int_data->mpu_gyro_y_frac);
        ZERO_SENSOR_FIELDS(int_data->mpu_gyro_z_whole, int_data->mpu_gyro_z_frac);
        ZERO_SENSOR_FIELDS(int_data->mpu_temp_whole, int_data->mpu_temp_frac);
        int_data->mpu_valid = 0;
    }
    
    int_data->timestamp = float_data->timestamp;
}