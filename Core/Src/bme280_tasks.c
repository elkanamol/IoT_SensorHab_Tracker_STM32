#include "bme280_tasks.h"
#include "bme280_porting.h"
#include "bme280.h"
#include "main.h"
#include "stdio.h"
#include "datalogger.h"

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
    (void)argument;

    printf("Starting BME280 task...\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Initialize BME280 step by step
    if (BME280_InitializeInterface() != BME280_OK)
    {
        printf("BME280 interface initialization failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    if (BME280_InitializeSensor() != BME280_OK)
    {
        printf("BME280 sensor initialization failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    if (BME280_ConfigureSettings() != BME280_OK)
    {
        printf("BME280 settings configuration failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    if (BME280_ValidateFirstMeasurement() != BME280_OK)
    {
        printf("BME280 first measurement validation failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    printf("BME280 initialization complete, starting measurements\r\n");

    // Start main measurement loop
    int8_t rslt;

    for (;;)
    {
        // Take forced measurement (power efficient)
        rslt = BME280_TakeForcedMeasurement();

        if (rslt == BME280_OK)
        {
            // Send original float data to data logger via queue
            if (DataLogger_QueueBME280Data(&bme_comp_data) != pdTRUE)
            {
                printf("BME280: Failed to queue data (queue full)\r\n");
            }

            // Print data for debugging
            BME280_PrintMeasurementData();
        }
        else
        {
            printf("BME280: Measurement failed, error: %d\r\n", rslt);
        }

        // Wait before next measurement (1 second interval)
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    vTaskDelay(pdMS_TO_TICKS(10));
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

    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure optimal settings for environmental monitoring
    settings.osr_h = BME280_OVERSAMPLING_1X;   // Humidity: 1x (fast, adequate)
    settings.osr_p = BME280_OVERSAMPLING_4X;   // Pressure: 4x (higher precision)
    settings.osr_t = BME280_OVERSAMPLING_2X;   // Temperature: 2x (good balance)
    settings.filter = BME280_FILTER_COEFF_OFF; // No filtering (real-time data)
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
    int8_t rslt;

    // Apply humidity settings first (BME280 requirement)
    rslt = bme280_set_sensor_settings(BME280_SEL_OSR_HUM, settings, &bme_device);
    if (rslt != BME280_OK)
    {
        printf("Failed to set humidity settings. Error: %d\r\n", rslt);
        return rslt;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Apply temperature and pressure settings
    rslt = bme280_set_sensor_settings(BME280_SEL_OSR_TEMP | BME280_SEL_OSR_PRESS, settings, &bme_device);
    if (rslt != BME280_OK)
    {
        printf("Failed to set temp/press settings. Error: %d\r\n", rslt);
        return rslt;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

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
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for first measurement

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
    vTaskDelay(pdMS_TO_TICKS(50));

    // Read measurement data
    rslt = bme280_get_sensor_data(BME280_ALL, &bme_comp_data, &bme_device);
    return rslt;
}

/**
 * @brief Print BME280 measurement data for debugging
 */
void BME280_PrintMeasurementData(void)
{
    // Convert for printf only (since printf doesn't support float)
    bme_calc_data_int_t bme_calc_data;
    convert_bme_data_to_int(&bme_comp_data, &bme_calc_data);
    char temp_str[16], press_str[16], hum_str[16];

    // Use sprintf for float conversion (if supported)
    sprintf(temp_str, "%d.%02d", (int)bme_comp_data.temperature, (int)((bme_comp_data.temperature - (int)bme_comp_data.temperature) * 100));
    sprintf(press_str, "%d.%02d", (int)bme_comp_data.pressure, (int)((bme_comp_data.pressure - (int)bme_comp_data.pressure) * 100));
    sprintf(hum_str, "%d.%02d", (int)bme_comp_data.humidity, (int)((bme_comp_data.humidity - (int)bme_comp_data.humidity) * 100));

    printf("BME280: %s°C, %shPa, %s%%\r\n", temp_str, press_str, hum_str);
}

// /**
//  * @brief Print BME280 measurement data for debugging
//  */
// void BME280_PrintMeasurementData(void)
// {
//     // Convert for printf only (since printf doesn't support float)
//     bme_calc_data_int_t bme_calc_data;
//     convert_bme_data_to_int(&bme_comp_data, &bme_calc_data);
//     char temp_str[16], press_str[16], hum_str[16];

//     // Use sprintf for float conversion (if supported)
//     sprintf(temp_str, "%.2f", bme_comp_data.temperature);
//     sprintf(press_str, "%.2f", bme_comp_data.pressure);
//     sprintf(hum_str, "%.2f", bme_comp_data.humidity);

//     printf("BME280: %s°C, %shPa, %s%%\r\n", temp_str, press_str, hum_str);
// }
