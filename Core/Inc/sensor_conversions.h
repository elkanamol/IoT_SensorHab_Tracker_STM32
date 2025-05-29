#ifndef SENSOR_CONVERSIONS_H
#define SENSOR_CONVERSIONS_H

#include "main.h"
#include "bme280.h" // For bme280_data_t
#include "bme280_porting.h" // For bme_calc_data_int_t
#include "mpu6050_task.h" // For MPU6050_Data_t
#include "datalogger.h" // For SensorData_Combined_t
// Improved conversion macro that handles both signed and unsigned types
#define CONVERT_SENSOR_FIELD(src, dst_whole, dst_frac, scale) \
    do { \
        (dst_whole) = (typeof(dst_whole))(src); \
        float temp_frac = ((src) - (dst_whole)) * (scale); \
        (dst_frac) = (typeof(dst_frac))(temp_frac < 0 ? -temp_frac : temp_frac); \
    } while(0)

#define ZERO_SENSOR_FIELDS(whole, frac) \
    do { \
        (whole) = 0; \
        (frac) = 0; \
    } while(0)

// Function declarations (only the new optimized combined function)
void convert_combined_sensor_data_to_int_optimized(const SensorData_Combined_t *float_data, SensorData_Combined_Int_t *int_data);
void convert_bme_data_to_int(const struct bme280_data *float_data, bme_calc_data_int_t *int_data);

#endif /* SENSOR_CONVERSIONS_H */