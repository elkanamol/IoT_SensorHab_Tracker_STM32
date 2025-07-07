#include "bme280_tasks.h"
#include "bme280_defs.h"
#include "bme280_porting.h" 
#include "bme280.h"
#include "sensor_conversions.h" 
#include "main.h"
#include "stdio.h"
#include "datalogger.h"
#include "config.h"

// Define global variables here (not in header!)
struct bme280_dev bme_device;
struct bme280_data bme_comp_data;

/**
 * @brief FreeRTOS task for initializing and managing BME280 sensor operations
 *
 * This task performs the following steps:
 * 1. Initialize the I2C interface for the BME280 sensor
 * 2. Initialize the BME280 sensor
 * 3. Configure sensor settings
 * 4. Validate first measurement
 * 5. Start continuous measurement loop
 *
 * If any initialization step fails, the task will delete itself
 *
 * @param argument FreeRTOS task argument (unused)
 */
void StartBme280Task(void *argument)
{
    SemaphoreHandle_t i2c_mutex = (SemaphoreHandle_t)argument;
    
    // Validate argument before any delays
    if (i2c_mutex == NULL) {
        printf("BME280: I2C mutex is NULL\r\n");
        vTaskDelete(NULL);
        return;
    }
    
    // Use named constant for startup delay
    vTaskDelay(pdMS_TO_TICKS(CONFIG_BME280_STARTUP_DELAY_MS));
    printf("Starting BME280 task...\r\n");
    
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) != pdTRUE)
    {
        printf("BME280: Failed to take I2C mutex\r\n");
        vTaskDelete(NULL);
        return;
    }

    // Initialize BME280 step by step
    if (BME280_InitializeInterface() != BME280_OK)
    {
        printf("BME280 interface initialization failed\r\n");
        xSemaphoreGive(i2c_mutex);  
        vTaskDelete(NULL);
        return;
    }

    if (BME280_InitializeSensor() != BME280_OK)
    {
        printf("BME280 sensor initialization failed\r\n");
        xSemaphoreGive(i2c_mutex);  
        vTaskDelete(NULL);
        return;
    }

    if (BME280_ConfigureSettings() != BME280_OK)
    {
        printf("BME280 settings configuration failed\r\n");
        xSemaphoreGive(i2c_mutex);  
        vTaskDelete(NULL);
        return;
    }

    if (BME280_ValidateFirstMeasurement() != BME280_OK)
    {
        printf("BME280 first measurement validation failed\r\n");
        xSemaphoreGive(i2c_mutex); 
        vTaskDelete(NULL);
        return;
    }
    
    if (xSemaphoreGive(i2c_mutex) != pdTRUE)
    {
        printf("BME280: Failed to give I2C mutex\r\n");
        vTaskDelete(NULL);
        return;
    }
    printf("BME280 initialization complete, starting measurements\r\n");

    // Start main measurement loop
    int8_t rslt;

    for (;;)
    {
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) != pdTRUE)
        {
            printf("BME280: Failed to take I2C mutex\r\n");
            continue;
        }
        
        // Take forced measurement (power efficient)
        rslt = BME280_TakeForcedMeasurement();

        if (rslt == BME280_OK)
        {
            if (DataLogger_UpdateBME280Data(&bme_comp_data) != pdTRUE)
            {
                printf("BME280: Failed to update datalogger\r\n");
            }

            // Print data for debugging
            BME280_PrintMeasurementData();
        }
        else
        {
            printf("BME280: Measurement failed, error: %d\r\n", rslt);
        }
        
        if (xSemaphoreGive(i2c_mutex) != pdTRUE)
        {
            printf("BME280: Failed to give I2C mutex\r\n");
        }
        
        // Wait before next measurement (1 second interval)
        vTaskDelay(pdMS_TO_TICKS(CONFIG_BME280_MEASUREMENT_INTERVAL_MS));
    }
}

/**
 * @brief Initialize BME280 I2C interface
 * @retval BME280_OK: Success, BME280_E_*: Error
 */
int8_t BME280_InitializeInterface(void)
{
    int8_t rslt = bme280_interface_init(&bme_device, &hi2c1, BME280_I2C_ADDR_PRIM);
    if (rslt != BME280_OK)
    {
        printf("Failed to initialize BME280 interface. Error: %d\r\n", rslt);
        return rslt;
    }

    printf("BME280 I2C interface initialized successfully\r\n");
    return BME280_OK;
}

/**
 * @brief Initialize BME280 sensor and verify chip ID
 * @retval BME280_OK: Success, BME280_E_*: Error
 */
int8_t BME280_InitializeSensor(void)
{
    int8_t rslt = bme280_init(&bme_device);
    if (rslt != BME280_OK)
    {
        printf("Failed to initialize BME280 sensor. Error: %d\r\n", rslt);
        bme280_interface_deinit(&bme_device);
        return rslt;
    }

    printf("BME280 sensor initialized successfully. Chip ID: 0x%X\r\n", bme_device.chip_id);
    vTaskDelay(pdMS_TO_TICKS(BME280_CONFIG_DELAY_MS));
    return BME280_OK;
}

/**
 * @brief Configure BME280 sensor settings for environmental monitoring
 * @retval BME280_OK: Success, BME280_E_*: Error
 */
int8_t BME280_ConfigureSettings(void)
{
    struct bme280_settings settings;
    int8_t rslt;

    // Get current settings
    rslt = bme280_get_sensor_settings(&settings, &bme_device);
    if (rslt != BME280_OK)
    {
        printf("Failed to get BME280 sensor settings. Error: %d\r\n", rslt);
        return rslt;
    }

    vTaskDelay(pdMS_TO_TICKS(BME280_CONFIG_DELAY_MS));

    // Configure optimal settings for environmental monitoring
    settings.osr_h = CONFIG_BME280_OVERSAMPLING_HUMIDITY;   // Humidity: 1x (fast, adequate)
    settings.osr_p = CONFIG_BME280_OVERSAMPLING_PRESSURE;   // Pressure: 4x (higher precision)
    settings.osr_t = CONFIG_BME280_OVERSAMPLING_TEMP;   // Temperature: 2x (good balance)
    settings.filter = CONFIG_BME280_FILTER_COEFFICIENT; // No filtering (real-time data)
    settings.standby_time = BME280_STANDBY_TIME_1000_MS;

    printf("Applying BME280 settings...\r\n");
    return BME280_ApplySettings(&settings);
}

/**
 * @brief Apply BME280 settings in correct sequence
 * @param settings Pointer to settings structure
 * @retval BME280_OK: Success, BME280_E_*: Error
 */
int8_t BME280_ApplySettings(struct bme280_settings *settings)
{
    if (settings == NULL) {
        printf("BME280: Invalid settings pointer\r\n");
        return BME280_E_NULL_PTR;
    }
    
    int8_t rslt;

    // Apply humidity settings first (BME280 requirement)
    rslt = bme280_set_sensor_settings(BME280_SEL_OSR_HUM, settings, &bme_device);
    if (rslt != BME280_OK)
    {
        printf("Failed to set humidity settings. Error: %d\r\n", rslt);
        return rslt;
    }
    vTaskDelay(pdMS_TO_TICKS(BME280_SETTINGS_DELAY_MS));

    // Apply temperature and pressure settings
    rslt = bme280_set_sensor_settings(BME280_SEL_OSR_TEMP | BME280_SEL_OSR_PRESS, settings, &bme_device);
    if (rslt != BME280_OK)
    {
        printf("Failed to set temp/press settings. Error: %d\r\n", rslt);
        return rslt;
    }
    vTaskDelay(pdMS_TO_TICKS(BME280_SETTINGS_DELAY_MS));

    // Apply filter and standby settings
    rslt = bme280_set_sensor_settings(BME280_SEL_FILTER | BME280_SEL_STANDBY, settings, &bme_device);
    if (rslt != BME280_OK)
    {
        printf("Failed to set filter/standby settings. Error: %d\r\n", rslt);
        return rslt;
    }

    printf("BME280 settings applied successfully\r\n");
    return BME280_OK;
}

/**
 * @brief Validate BME280 operation with first measurement
 * @retval BME280_OK: Success, BME280_E_*: Error
 */
int8_t BME280_ValidateFirstMeasurement(void)
{
    int8_t rslt;

    // Set sensor to normal mode for validation
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme_device);
    if (rslt != BME280_OK)
    {
        printf("Failed to set BME280 sensor mode. Error: %d\r\n", rslt);
        return rslt;
    }

    printf("Waiting for first measurement to complete...\r\n");
    vTaskDelay(pdMS_TO_TICKS(
        BME280_FIRST_MEASUREMENT_DELAY_MS)); // Wait for first measurement

    // Take first measurement to validate everything works
    rslt = bme280_get_sensor_data(BME280_ALL, &bme_comp_data, &bme_device);
    if (rslt != BME280_OK)
    {
        printf("Failed to get first measurement. Error: %d\r\n", rslt);
        return rslt;
    }

    printf("First measurement successful - BME280 ready for operation\r\n");
    return BME280_OK;
}

/**
 * @brief Take single forced measurement (power efficient)
 * @retval BME280_OK: Success, BME280_E_*: Error
 */
int8_t BME280_TakeForcedMeasurement(void)
{
    int8_t rslt;

    // Set sensor to forced mode (takes one measurement then sleeps)
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &bme_device);
    if (rslt != BME280_OK)
    {
        return rslt;
    }

    // Fixed delay based on oversampling settings
    // T×2 + P×4 + H×1 + margins = ~50ms covers all cases
    vTaskDelay(pdMS_TO_TICKS(BME280_FORCED_MODE_DELAY_MS));

    // Read measurement data
    rslt = bme280_get_sensor_data(BME280_ALL, &bme_comp_data, &bme_device);
    return rslt;
}

void BME280_PrintMeasurementData(void)
{
    printf("BME280: %.2f°C, %.2f hPa, %.2f%%\r\n", 
           bme_comp_data.temperature, bme_comp_data.pressure, bme_comp_data.humidity);
}

// /**
//  * @brief Print BME280 measurement data for debugging
//  */
// void BME280_PrintMeasurementData_int(void)
// {
//     bme_calc_data_int_t bme_calc_data;
//     convert_bme_data_to_int(&bme_comp_data, &bme_calc_data);

//     printf("BME280: %d.%02d°C, %d.%02d hPa, %d.%02d%%\r\n",
//            bme_calc_data.temp_whole, bme_calc_data.temp_frac,
//            bme_calc_data.press_whole, bme_calc_data.press_frac,
//            bme_calc_data.hum_whole, bme_calc_data.hum_frac);
// }

