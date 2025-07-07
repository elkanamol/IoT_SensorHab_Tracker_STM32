#ifndef BME280_TASKS_H
#define BME280_TASKS_H

#include "bme280.h"
#include "bme280_porting.h"
// External variables (declared here, defined in bme280_tasks.c)
extern I2C_HandleTypeDef hi2c1;
extern struct bme280_dev bme_device;
extern struct bme280_data bme_comp_data;

#define BME280_STARTUP_DELAY_MS 2000
#define BME280_CONFIG_DELAY_MS 10
#define BME280_SETTINGS_DELAY_MS 5
#define BME280_FIRST_MEASUREMENT_DELAY_MS 100
#define BME280_FORCED_MODE_DELAY_MS 50
#define BME280_MEASUREMENT_INTERVAL_MS 1000

/**
 * @brief Starts the BME280 sensor task
 * 
 * Initializes and runs the task for reading and processing data from the BME280 environmental sensor
 * 
 * @param argument Pointer to task arguments (unused)
 */
void StartBme280Task(void *argument);

// BME280 helper function declarations (NOT static - external linkage)
int8_t BME280_InitializeInterface(void);
int8_t BME280_InitializeSensor(void);
int8_t BME280_ConfigureSettings(void);
int8_t BME280_ApplySettings(struct bme280_settings *settings);
int8_t BME280_ValidateFirstMeasurement(void);
int8_t BME280_TakeForcedMeasurement(void);
void BME280_PrintMeasurementData(void);

#endif
