#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "bme280.h"           // For BME280_OVERSAMPLING_* and BME280_FILTER_* enums
#include "driver_mpu6050.h"   // For MPU6050_ADDRESS_* enums
#include "mqtt_secrets.h"     // For SECRET_MQTT_* credentials
#include "FreeRTOS.h"         // For tskIDLE_PRIORITY
#include "task.h"

// MQTT & Network Configuration

#define CONFIG_MQTT_BROKER_HOSTNAME      "mqtt3.thingspeak.com"
#define CONFIG_MQTT_BROKER_PORT          1883
#define CONFIG_MQTT_CLIENT_ID            SECRET_MQTT_CLIENT_ID
#define CONFIG_MQTT_USERNAME             SECRET_MQTT_USERNAME
#define CONFIG_MQTT_PASSWORD             SECRET_MQTT_PASSWORD
#define CONFIG_MQTT_PUBLISH_TOPIC        "channels/2956054/publish"
#define CONFIG_MQTT_SUBSCRIBE_TOPIC      "channels/2956054/subscribe"

// Datalogger & Storage Configuration
// How often the datalogger sends aggregated data to the MQTT task
#define CONFIG_DATALOGGER_MQTT_TRANSMISSION_INTERVAL_MS  (60 * 1000) // 60 seconds

// How often the maintenance loop forces a save of the RAM buffer to flash
#define CONFIG_DATALOGGER_MAINTENANCE_FLUSH_INTERVAL_S   60

// How often the maintenance loop prints a status message
#define CONFIG_DATALOGGER_MAINTENANCE_STATUS_INTERVAL_S  120

// The size of the ring buffer on the external W25Qxx flash chip
#define CONFIG_DATALOGGER_FLASH_RING_BUFFER_SIZE         (64 * 1024) // 64KB

// The depth of the FreeRTOS queue for buffering incoming sensor data
#define CONFIG_DATALOGGER_QUEUE_SIZE                     20

// Sensor Task Timing Configuration
// Defines the main polling rate for each sensor task in milliseconds.
#define CONFIG_BME280_MEASUREMENT_INTERVAL_MS      1000
#define CONFIG_MPU6050_READ_PERIOD_MS              1000

// Defines initial delays for tasks to allow system stabilization.
#define CONFIG_BME280_CONFIG_DELAY_MS              10
#define CONFIG_BME280_STARTUP_DELAY_MS 1000
#define CONFIG_MPU6050_STARTUP_DELAY_MS 5000 // Should be > BME280 startup
#define CONFIG_DATALOGGER_QUEUE_SEND_TIMEOUT_MS   1000
#define CONFIG_BME280_SETTINGS_DELAY_MS           10
#define CONFIG_BME280_FIRST_MEASUREMENT_DELAY_MS  1000
#define CONFIG_BME280_FORCED_MODE_DELAY_MS        50
#define CONFIG_MPU6050_RETRY_DELAY_MS 100

// Timeout for MPU6050 task to wait for the shared I2C bus mutex.
#define CONFIG_MPU6050_MUTEX_TIMEOUT_MS 10000

// BME280 Sensor Hardware Settings
// These settings trade accuracy for power consumption and measurement time.
// Valid values: BME280_OVERSAMPLING_1X, _2X, _4X, _8X, _16X
#define CONFIG_BME280_OVERSAMPLING_HUMIDITY      BME280_OVERSAMPLING_1X
#define CONFIG_BME280_OVERSAMPLING_PRESSURE    BME280_OVERSAMPLING_4X
#define CONFIG_BME280_OVERSAMPLING_TEMP        BME280_OVERSAMPLING_2X

// IIR filter coefficient to smooth out short-term fluctuations.
// Valid values: BME280_FILTER_COEFF_OFF, _2, _4, _8, _16
#define CONFIG_BME280_FILTER_COEFFICIENT       BME280_FILTER_COEFF_OFF

// MPU6050 Sensor Hardware Settings
// I2C address of the MPU6050 sensor. Depends on the state of the AD0 pin.
// Valid values: MPU6050_ADDRESS_AD0_LOW (0x68), MPU6050_ADDRESS_AD0_HIGH (0x69)
#define CONFIG_MPU6050_I2C_ADDRESS             MPU6050_ADDRESS_AD0_LOW

// FreeRTOS Task Configuration
// --- Task Priorities ---
// Define general priority levels for the application.
#define CONFIG_TASK_PRIORITY_NORMAL            (tskIDLE_PRIORITY + 1)
#define CONFIG_TASK_PRIORITY_HIGH              (tskIDLE_PRIORITY + 2)

// --- Task Stack Sizes (in words) ---
// Adjust these based on task complexity and memory usage.
#define CONFIG_TASK_STACK_SIZE_UAT             (512)
#define CONFIG_TASK_STACK_SIZE_MQTT            (512 * 2)
#define CONFIG_TASK_STACK_SIZE_BME280 (1024)
#define CONFIG_TASK_STACK_SIZE_MPU6050 (1024)
#define CONFIG_TASK_STACK_SIZE_DATALOGGER      (2048)
#define CONFIG_TASK_STACK_SIZE_GPS             (1536)

// Add missing BME280 delays (tune as needed)
#define CONFIG_BME280_SETTINGS_DELAY_MS 10
#define CONFIG_BME280_FORCED_MODE_DELAY_MS        50

// Add missing MPU6050 retry delay
#define CONFIG_MPU6050_RETRY_DELAY_MS             100

// Add missing DATA_START_ADDRESS (adjust as needed for your flash layout)
//#define DATA_START_ADDRESS                        0x000000

#endif // APP_CONFIG_H
