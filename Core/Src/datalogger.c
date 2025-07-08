#include "datalogger.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sensor_conversions.h"
#include <string.h>
#include "print.h"

// Private variables
static uint32_t ring_buffer_write_address = RING_BUFFER_START_ADDRESS;
static uint32_t total_records_written = 0;
static uint32_t record_counter = 0;

// Unified sector buffer
static uint8_t sector_buffer[SECTOR_BUFFER_SIZE];
static uint16_t records_in_buffer = 0;
static uint32_t current_sector_address = RING_BUFFER_START_ADDRESS;

// Current sensor data (updated by different sensors)
static SensorData_Combined_t current_sensor_data = {0};
static TickType_t last_mqtt_transmission = 0;
static TickType_t last_flash_save = 0;

// Public variables
QueueHandle_t xDataLoggerQueue = NULL;
QueueHandle_t xMQTTQueue = NULL;

// Private function prototypes
static uint8_t DataLogger_Initialize(void);
static void DataLogger_FindWritePosition(void);
static void DataLogger_ProcessSensorUpdate(SensorUpdateMessage_t *message);
static void DataLogger_SaveCurrentDataToFlash(void);
static void DataLogger_FlushSectorBuffer(void);
static void DataLogger_PeriodicMaintenance(void);
static void DataLogger_PrintStatus(void);
static uint32_t DataLogger_CalculateCRC(SensorData_Combined_t *record);
static void DataLogger_InitializeRecord(SensorData_Combined_t *record);
static void DataLogger_CheckPeriodicSave(void);
static void DataLogger_CheckMQTTTransmission(void);

/**
 * @brief Initialize Data Logger subsystem
 * @retval 0: Success, 1: Error
 */
static uint8_t DataLogger_Initialize(void)
{
    uint8_t flash_status;
    uint8_t manufacturer, device_id;
    
    DEBUG_PRINT_DEBUG("DataLogger: Initializing unified sensor logging system...\r\n");
    
    // Create data logger queue
    xDataLoggerQueue = xQueueCreate(DATA_LOGGER_QUEUE_SIZE, sizeof(SensorUpdateMessage_t));
    if (xDataLoggerQueue == NULL) {
        DEBUG_PRINT_ERROR("DataLogger: Failed to create data logger queue\r\n");
        return 1;
    }
    
    // Create MQTT queue
    xMQTTQueue = xQueueCreate(5, sizeof(SensorData_Combined_t));
    if (xMQTTQueue == NULL) {
        DEBUG_PRINT_ERROR("DataLogger: Failed to create MQTT queue\r\n");
        return 1;
    }
    
    // Initialize sector buffer
    memset(sector_buffer, 0xFF, sizeof(sector_buffer));
    records_in_buffer = 0;
    
    // Initialize W25Q64 flash
    flash_status = w25qxx_basic_init(W25Q64, W25QXX_INTERFACE_SPI, W25QXX_BOOL_FALSE);
    if (flash_status != 0) {
        DEBUG_PRINT_ERROR("DataLogger: Failed to initialize W25Q64 flash (err=%d)\r\n", flash_status);
        return 1;
    }
    
    // Verify chip ID
    flash_status = w25qxx_basic_get_id(&manufacturer, &device_id);
    if (flash_status == 0) {
        DEBUG_PRINT_DEBUG("DataLogger: W25Q64 ID - manufacturer=0x%02X, device=0x%02X\r\n", manufacturer, device_id);
        if (manufacturer != 0xEF || device_id != 0x16) {
            DEBUG_PRINT_ERROR("DataLogger: Warning - Unexpected chip ID\r\n");
        }
    }
    
    // Find current write position in ring buffer
    DataLogger_FindWritePosition();
    
    DEBUG_PRINT_DEBUG("DataLogger: Initialization complete\r\n");
    return 0;
}

/**
 * @brief Initialize a record with default values and 0xFF padding
 */
static void DataLogger_InitializeRecord(SensorData_Combined_t *record)
{
    if (record == NULL) {
        DEBUG_PRINT_ERROR("DataLogger: Invalid record pointer\r\n");
        return;
    }
    memset(record, 0xFF, sizeof(SensorData_Combined_t));
    
    // Set default values for non-0xFF fields
    record->timestamp = 0;
    record->record_id = 0;
    record->bme_temperature = 0.0f;
    record->bme_pressure = 0.0f;
    record->bme_humidity = 0.0f;
    record->bme_valid = 0;
    record->mpu_accel_x = 0.0f;
    record->mpu_accel_y = 0.0f;
    record->mpu_accel_z = 0.0f;
    record->mpu_gyro_x = 0.0f;
    record->mpu_gyro_y = 0.0f;
    record->mpu_gyro_z = 0.0f;
    record->mpu_temperature = 0.0f;
    record->mpu_valid = 0;
    record->gps_latitude = 0.0f;
    record->gps_longitude = 0.0f;
    record->gps_altitude = 0.0f;
    record->gps_speed = 0.0f;
    record->gps_valid = 0;
    record->gps_satellites = 0;
    record->crc = 0;
}

/**
 * @brief Calculate CRC for unified record
 */
static uint32_t DataLogger_CalculateCRC(SensorData_Combined_t *record)
{
    uint32_t crc = 0;
    uint8_t *data = (uint8_t *)record;
    size_t len = sizeof(SensorData_Combined_t) - sizeof(uint32_t) - 16; // Exclude CRC and reserved
    
    for (size_t i = 0; i < len; i++) {
        crc += data[i];
    }
    return crc;
}

/**
 * @brief Save current sensor data to flash
 */
static void DataLogger_SaveCurrentDataToFlash(void)
{
    SensorData_Combined_t record_to_save;
    
    // Copy current data
    record_to_save = current_sensor_data;
    
    // Update record metadata
    record_to_save.timestamp = HAL_GetTick();
    record_to_save.record_id = ++record_counter;
    
    // Fill reserved area with 0xFF
    memset(record_to_save.reserved, 0xFF, sizeof(record_to_save.reserved));
    
    // Calculate CRC
    record_to_save.crc = DataLogger_CalculateCRC(&record_to_save);
    
    // Add to sector buffer
    uint32_t offset = records_in_buffer * sizeof(SensorData_Combined_t);
    memcpy(&sector_buffer[offset], &record_to_save, sizeof(SensorData_Combined_t));
    records_in_buffer++;

    DEBUG_PRINT_DEBUG(
        "DataLogger: Saved record #%lu - BME280(%s) MPU6050(%s)\r\n",
        record_to_save.record_id,
        record_to_save.bme_valid ? "Valid" : "Invalid",
        record_to_save.mpu_valid ? "Valid" : "Invalid");

    // Flush when sector buffer is full
    if (records_in_buffer >= RECORDS_PER_SECTOR) {
        DataLogger_FlushSectorBuffer();
    }
    
    last_flash_save = xTaskGetTickCount();
}

/**
 * @brief Update BME280 data in current sensor data
 */
BaseType_t DataLogger_UpdateBME280Data(struct bme280_data *bme_data)
{
    if (bme_data == NULL) {
        DEBUG_PRINT_ERROR("DataLogger: Invalid BME280 data pointer\r\n");
        return pdFALSE;
    }
    SensorUpdateMessage_t message = {
        .type = SENSOR_UPDATE_BME280,
        .timestamp = HAL_GetTick(),
        .data.bme_data = *bme_data
    };

    return xQueueSend(xDataLoggerQueue, &message, pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS));
}

/**
 * @brief Update MPU6050 data in current sensor data
 */
BaseType_t DataLogger_UpdateMPU6050Data(float ax, float ay, float az, float gx, float gy, float gz, float temp)
{
    SensorUpdateMessage_t message = {
        .type = SENSOR_UPDATE_MPU6050,
        .timestamp = HAL_GetTick(),
        .data.mpu_data = {
            .accel_x = ax, .accel_y = ay, .accel_z = az,
            .gyro_x = gx, .gyro_y = gy, .gyro_z = gz,
            .temperature = temp
        }
    };

    return xQueueSend(xDataLoggerQueue, &message, pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS));
}

/**
 * @brief Force save current data to flash
 */
BaseType_t DataLogger_ForceSave(void)
{
    SensorUpdateMessage_t message = {
        .type = SENSOR_UPDATE_FORCE_SAVE,
        .timestamp = HAL_GetTick()
    };

    return xQueueSend(xDataLoggerQueue, &message, pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS));
}

// Add GPS update function
/**
 * @brief Update GPS data in current sensor data
 * 
 * @param lat Latitude value
 * @param lon Longitude value
 * @param alt Altitude value
 * @param speed Speed value
 * @return BaseType_t Result of sending GPS update message to queue
 */
BaseType_t DataLogger_UpdateGPSData(float lat, float lon, float alt, float speed)
{
    if (lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f) {
        DEBUG_PRINT_ERROR("DataLogger: Invalid GPS coordinates: lat=%f, lon=%f\r\n", lat, lon);
        return pdFALSE;
    }
    SensorUpdateMessage_t message = {
        .type = SENSOR_UPDATE_GPS,
        .timestamp = HAL_GetTick(),
        .data.gps_data = {
            .latitude = lat,
            .longitude = lon,
            .altitude = alt,
            .speed = speed}};

    return xQueueSend(xDataLoggerQueue, &message, pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS));
}
/**
 * @brief Process sensor update messages
 */
static void DataLogger_ProcessSensorUpdate(SensorUpdateMessage_t *message)
{
    if (message == NULL) {
        DEBUG_PRINT_ERROR("DataLogger: Received NULL message\r\n");
        return;
    }
    switch (message->type)
    {
        case SENSOR_UPDATE_BME280:
            // Update BME280 data in current record
            current_sensor_data.bme_temperature = message->data.bme_data.temperature;
            current_sensor_data.bme_pressure = message->data.bme_data.pressure;
            current_sensor_data.bme_humidity = message->data.bme_data.humidity;
            current_sensor_data.bme_valid = 1;
            
            // printf("DataLogger: BME280 data updated\r\n");
            break;
            
        case SENSOR_UPDATE_MPU6050:
            // Update MPU6050 data in current record
            current_sensor_data.mpu_accel_x = message->data.mpu_data.accel_x;
            current_sensor_data.mpu_accel_y = message->data.mpu_data.accel_y;
            current_sensor_data.mpu_accel_z = message->data.mpu_data.accel_z;
            current_sensor_data.mpu_gyro_x = message->data.mpu_data.gyro_x;
            current_sensor_data.mpu_gyro_y = message->data.mpu_data.gyro_y;
            current_sensor_data.mpu_gyro_z = message->data.mpu_data.gyro_z;
            current_sensor_data.mpu_temperature = message->data.mpu_data.temperature;
            current_sensor_data.mpu_valid = 1;
            
            // printf("DataLogger: MPU6050 data updated\r\n");
            break;
            
        case SENSOR_UPDATE_GPS:  // Add GPS case
            current_sensor_data.gps_latitude = message->data.gps_data.latitude;
            current_sensor_data.gps_longitude = message->data.gps_data.longitude;
            current_sensor_data.gps_altitude = message->data.gps_data.altitude;
            current_sensor_data.gps_speed = message->data.gps_data.speed;
            current_sensor_data.gps_satellites = message->data.gps_data.satellites;
            current_sensor_data.gps_valid = 1;
            break;
            
        case SENSOR_UPDATE_FORCE_SAVE:
            DataLogger_SaveCurrentDataToFlash();
            break;
            
        case SENSOR_UPDATE_SYSTEM_EVENT:
            DEBUG_PRINT_DEBUG("System Event: %s\r\n", message->data.system_event);
            break;
            
        default:
          DEBUG_PRINT_DEBUG("DataLogger: Unknown update type: %d\r\n",
                            message->type);
          break;
    }
}

/**
 * @brief Check if it's time for periodic save (every 10 seconds if data is available)
 */
static void DataLogger_CheckPeriodicSave(void)
{
    TickType_t current_time = xTaskGetTickCount();
    
    // Save every 10 seconds if we have valid data
    if ((current_time - last_flash_save) >=
            pdMS_TO_TICKS(MAINTENANCE_PERIODIC_CHECK_MS) &&
        (current_sensor_data.bme_valid || current_sensor_data.mpu_valid ||
         current_sensor_data.gps_valid)) {
      DataLogger_SaveCurrentDataToFlash();
    }
}

/**
 * @brief Check if it's time to send MQTT data (every 60 seconds)
 */
static void DataLogger_CheckMQTTTransmission(void)
{
    TickType_t current_time = xTaskGetTickCount();
    
    if ((current_time - last_mqtt_transmission) >= pdMS_TO_TICKS(MQTT_TRANSMISSION_INTERVAL_MS))
    {
        if (xMQTTQueue != NULL && (current_sensor_data.bme_valid || current_sensor_data.mpu_valid))
        {
            if (xQueueSend(xMQTTQueue, &current_sensor_data, 0) == pdTRUE)
            {
                DEBUG_PRINT_DEBUG("DataLogger: Sent combined sensor data to MQTT queue\r\n");
                last_mqtt_transmission = current_time;
            }
            else
            {
              DEBUG_PRINT_ERROR(
                  "DataLogger: Failed to send to MQTT queue (queue full)\r\n");
            }
        }
    }
}

/**
 * @brief Main DataLogger task
 */
void StartDataLoggerTask(void *argument)
{
    (void)argument;
    SensorUpdateMessage_t received_message;
    
    DEBUG_PRINT_DEBUG("DataLogger: Starting unified sensor data logging system...\r\n");
    
    // Initialize system
    if (DataLogger_Initialize() != 0) {
        DEBUG_PRINT_ERROR("DataLogger: Failed to initialize\r\n");
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize current sensor data
    DataLogger_InitializeRecord(&current_sensor_data);
    
    // Initialize timing
    last_flash_save = xTaskGetTickCount();
    last_mqtt_transmission = xTaskGetTickCount();
    
    DEBUG_PRINT_DEBUG("DataLogger: Unified system initialized successfully\r\n");
    
    // Main processing loop
    for (;;)
    {
        // Wait for sensor updates
        if (xQueueReceive(xDataLoggerQueue, &received_message, pdMS_TO_TICKS(MAX_QUEUE_WAIT_MS)) == pdTRUE)
        {
            DataLogger_ProcessSensorUpdate(&received_message);
        }
        else
        {
            // Timeout - perform periodic tasks
            DataLogger_PeriodicMaintenance();
        }
        
        // Check for periodic save and MQTT transmission
        DataLogger_CheckPeriodicSave();
        DataLogger_CheckMQTTTransmission();
    }
}

/**
 * @brief Flush sector buffer to flash with ring buffer management
 */
static void DataLogger_FlushSectorBuffer(void)
{
    if (records_in_buffer == 0) return;
    
    uint32_t used_bytes = records_in_buffer * sizeof(SensorData_Combined_t);
    
    DEBUG_PRINT_DEBUG("DataLogger: Flushing sector: %d records (%lu bytes) at 0x%08lX\r\n", 
           records_in_buffer, (unsigned long)used_bytes, (unsigned long)current_sector_address);
    
    // Fill remaining buffer with 0xFF
    if (used_bytes < SECTOR_BUFFER_SIZE) {
        memset(&sector_buffer[used_bytes], 0xFF, SECTOR_BUFFER_SIZE - used_bytes);
    }
    
    // Write entire 4KB sector
    uint8_t status = w25qxx_basic_write(current_sector_address, 
                                        sector_buffer, 
                                        SECTOR_BUFFER_SIZE);
    
    if (status == 0) {
        total_records_written += records_in_buffer;
        
        // Advance to next sector
        current_sector_address += SECTOR_BUFFER_SIZE;
        ring_buffer_write_address = current_sector_address;
        
        // Ring buffer wrap-around
        if (current_sector_address >= RING_BUFFER_END_ADDRESS) {
            DEBUG_PRINT_DEBUG("DataLogger: Ring buffer full, wrapping to start (FIFO)\r\n");
            current_sector_address = RING_BUFFER_START_ADDRESS;
            ring_buffer_write_address = RING_BUFFER_START_ADDRESS;
        }
        
        // Reset buffer
        records_in_buffer = 0;
        memset(sector_buffer, 0xFF, sizeof(sector_buffer));
        
        DEBUG_PRINT_DEBUG("DataLogger: Sector written successfully\r\n");
    } else {
        DEBUG_PRINT_ERROR("DataLogger: Sector write failed (err=%d)\r\n", status);
    }
}

/**
 * @brief Find current write position in ring buffer
 */
static void DataLogger_FindWritePosition(void)
{
    // printf("DataLogger: Scanning ring buffer for write position...\r\n");
    SensorData_Combined_t scan_record;
    bool found_empty = false;
    
    // Scan by sectors
    for (uint32_t addr = RING_BUFFER_START_ADDRESS; 
         addr < RING_BUFFER_END_ADDRESS; 
         addr += W25Q64_SECTOR_SIZE)
    {
        uint8_t status = w25qxx_basic_read(addr, (uint8_t *)&scan_record, sizeof(SensorData_Combined_t));
        if (status == 0 && scan_record.timestamp == 0xFFFFFFFF) {
            current_sector_address = addr;
            ring_buffer_write_address = addr;
            found_empty = true;
            DEBUG_PRINT_DEBUG("DataLogger: Found empty sector at 0x%08lX\r\n", (unsigned long)addr);
            break;
        }
    }
    
    if (!found_empty) {
        DEBUG_PRINT_DEBUG("DataLogger: Ring buffer full, starting FIFO from beginning\r\n");
        current_sector_address = RING_BUFFER_START_ADDRESS;
        ring_buffer_write_address = RING_BUFFER_START_ADDRESS;
    }
}

/**
 * @brief Periodic maintenance (called on queue timeout)
 */
static void DataLogger_PeriodicMaintenance(void)
{
    static uint32_t maintenance_counter = 0;
    maintenance_counter++;
    
    // Force flush every 60 seconds if we have buffered records
    if ((maintenance_counter % MAINTENANCE_FLUSH_INTERVAL_SEC == 0 )&& (records_in_buffer > 0)) {
        DEBUG_PRINT_DEBUG("DataLogger: Periodic maintenance - flushing %d buffered records\r\n", records_in_buffer);
        DataLogger_FlushSectorBuffer();
    }
    
    // Print status every 120 seconds
    if (maintenance_counter % MAINTENANCE_STATUS_INTERVAL_SEC == 0) {
        DataLogger_PrintStatus();
    }
}

/**
 * @brief Print current datalogger status
 */
static void DataLogger_PrintStatus(void)
{
    DEBUG_PRINT_DEBUG("\r\n=== DataLogger Status ===\r\n");
    DEBUG_PRINT_DEBUG("Write Address: 0x%08lX\r\n", (unsigned long)ring_buffer_write_address);
    DEBUG_PRINT_DEBUG("Current Sector: 0x%08lX\r\n", (unsigned long)current_sector_address);
    DEBUG_PRINT_DEBUG("Records in Buffer: %d/%d\r\n", records_in_buffer, RECORDS_PER_SECTOR);
    DEBUG_PRINT_DEBUG("Total Records Written: %lu\r\n", (unsigned long)total_records_written);
    DEBUG_PRINT_DEBUG("Current Record ID: %lu\r\n", (unsigned long)record_counter);
    DEBUG_PRINT_DEBUG("BME280 Valid: %s\r\n", current_sensor_data.bme_valid ? "Yes" : "No");
    DEBUG_PRINT_DEBUG("MPU6050 Valid: %s\r\n", current_sensor_data.mpu_valid ? "Yes" : "No");
    DEBUG_PRINT_DEBUG("GPS Valid: %s (%d sats)\r\n", current_sensor_data.gps_valid ? "Yes" : "No", current_sensor_data.gps_satellites);
    DEBUG_PRINT_DEBUG("==========================\r\n\r\n");
}
