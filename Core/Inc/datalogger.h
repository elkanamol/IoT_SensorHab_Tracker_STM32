#ifndef DATALOGGER_H
#define DATALOGGER_H

#include "main.h"
#include "bme280.h"
#include "driver_w25qxx_basic.h"
#include "FreeRTOS.h"
#include "queue.h"

// Ring buffer configuration (64KB FIFO)
#define FLASH_RING_BUFFER_SIZE (64 * 1024)
#define RING_BUFFER_START_ADDRESS (DATA_START_ADDRESS)
#define RING_BUFFER_END_ADDRESS (RING_BUFFER_START_ADDRESS + FLASH_RING_BUFFER_SIZE)

// Sector-based definitions
#define W25Q64_SECTOR_SIZE 4096
#define UNIFIED_RECORD_SIZE sizeof(SensorData_Combined_t)
#define RECORDS_PER_SECTOR (W25Q64_SECTOR_SIZE / UNIFIED_RECORD_SIZE)
#define SECTOR_BUFFER_SIZE W25Q64_SECTOR_SIZE

// Queue configuration
#define DATA_LOGGER_QUEUE_SIZE 20
#define MAX_QUEUE_WAIT_MS 1000
#define QUEUE_SEND_TIMEOUT_MS 1000

// MQTT transmission interval
#define MQTT_TRANSMISSION_INTERVAL_MS (60 * 1000)  // 60 seconds
#define MAINTENANCE_FLUSH_INTERVAL_SEC 60
#define MAINTENANCE_STATUS_INTERVAL_SEC 120
#define MAINTENANCE_PERIODIC_CHECK_MS 1000


// Sensor update types
typedef enum {
    SENSOR_UPDATE_BME280,
    SENSOR_UPDATE_MPU6050,
    SENSOR_UPDATE_GPS,           // Add GPS update type
    SENSOR_UPDATE_SYSTEM_EVENT,
    SENSOR_UPDATE_FORCE_SAVE,
} SensorUpdateType_t;

// Unified sensor data structure (this goes to flash)
typedef struct {
    uint32_t timestamp;
    uint32_t record_id;
    
    // BME280 data
    float bme_temperature;
    float bme_pressure;
    float bme_humidity;
    uint8_t bme_valid;
    uint8_t bme_padding[3];
    
    // MPU6050 data
    float mpu_accel_x, mpu_accel_y, mpu_accel_z;
    float mpu_gyro_x, mpu_gyro_y, mpu_gyro_z;
    float mpu_temperature;
    uint8_t mpu_valid;
    uint8_t mpu_padding[3];
    
    // GPS data
    float gps_latitude;
    float gps_longitude;
    float gps_altitude;
    float gps_speed;
    uint8_t gps_valid;
    uint8_t gps_satellites;
    uint8_t gps_padding[2];
    
    uint32_t crc;
    uint8_t reserved[8];  // Reduced reserved space due to GPS data
} SensorData_Combined_t;

// Integer representation for MQTT transmission
typedef struct {
    // BME280 integer data
    int32_t bme_temp_whole, bme_temp_frac;
    uint32_t bme_press_whole, bme_press_frac;
    uint32_t bme_hum_whole, bme_hum_frac;
    uint8_t bme_valid;
    
    // MPU6050 integer data
    int16_t mpu_accel_x_whole, mpu_accel_x_frac;
    int16_t mpu_accel_y_whole, mpu_accel_y_frac;
    int16_t mpu_accel_z_whole, mpu_accel_z_frac;
    int16_t mpu_gyro_x_whole, mpu_gyro_x_frac;
    int16_t mpu_gyro_y_whole, mpu_gyro_y_frac;
    int16_t mpu_gyro_z_whole, mpu_gyro_z_frac;
    int16_t mpu_temp_whole, mpu_temp_frac;
    uint8_t mpu_valid;
    
    // GPS integer data
    int32_t gps_lat_whole, gps_lat_frac;
    int32_t gps_lon_whole, gps_lon_frac;
    int16_t gps_alt_whole, gps_alt_frac;
    uint16_t gps_speed_whole, gps_speed_frac;
    uint8_t gps_valid;
    uint8_t gps_satellites;
    
    uint32_t timestamp;
} SensorData_Combined_Int_t;

// Sensor update message
typedef struct {
    SensorUpdateType_t type;
    uint32_t timestamp;
    union {
        struct bme280_data bme_data;
        struct {
            float accel_x, accel_y, accel_z;
            float gyro_x, gyro_y, gyro_z;
            float temperature;
        } mpu_data;
        struct {
            float latitude;
            float longitude;
            float altitude;
            float speed;
            uint8_t satellites;
            uint8_t is_valid;
        } gps_data;
        char system_event[32];
    } data;
} SensorUpdateMessage_t;

// External variables
extern QueueHandle_t xDataLoggerQueue;
extern QueueHandle_t xMQTTQueue;

// Public API functions
void StartDataLoggerTask(void *argument);

BaseType_t DataLogger_UpdateBME280Data(struct bme280_data *bme_data);
BaseType_t DataLogger_UpdateMPU6050Data(float ax, float ay, float az, float gx, float gy, float gz, float temp);
BaseType_t DataLogger_UpdateGPSData(float lat, float lon, float alt, float speed);
// BaseType_t DataLogger_QueueSystemEvent(const char *event_text);
BaseType_t DataLogger_ForceSave(void);

#endif /* DATALOGGER_H */
