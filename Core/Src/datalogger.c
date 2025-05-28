#include "datalogger.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"

// Private variables (internal to this module)
static uint32_t ring_buffer_write_address = RING_BUFFER_START_ADDRESS;
static uint32_t total_records_written = 0;

// Sector-based variables
static uint8_t sector_buffer[SECTOR_BUFFER_SIZE];  // 4KB buffer instead of 256B
static uint16_t records_in_buffer = 0;             // uint16_t for larger count (~256 records)
static uint32_t current_sector_address = RING_BUFFER_START_ADDRESS;


// Public variables (accessible from other modules)
QueueHandle_t xDataLoggerQueue = NULL;

// Private function prototypes
static uint8_t DataLogger_Initialize(void);
static void DataLogger_FindWritePosition(void);
static void DataLogger_ProcessMessage(LogMessage_t *message);
static void DataLogger_AddRecordToBuffer(BME280_Record_t *record);
static void DataLogger_FlushSectorBuffer(void);
static void DataLogger_PeriodicMaintenance(void);
static void DataLogger_PrintConfiguration(void);
static void DataLogger_PrintStatus(void);
static void DataLogger_Shutdown(void);
static uint32_t DataLogger_CalculateCRC(BME280_Record_t *record);

/**
 * @brief DataLogger FreeRTOS Task
 */
void StartDataLoggerTask(void *argument)
{
    (void)argument;
    LogMessage_t received_message;
    
    // vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Data Logger Task starting with Queue-based architecture...\r\n");
    
    // Initialize all subsystems
    if (DataLogger_Initialize() != 0) {
        printf("Failed to initialize Data Logger\r\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("Data Logger initialized successfully\r\n");
    DataLogger_PrintConfiguration();
    
    // Main processing loop
    for (;;)
    {
        // Wait for messages from other tasks
        if (xQueueReceive(xDataLoggerQueue, &received_message, pdMS_TO_TICKS(MAX_QUEUE_WAIT_MS)) == pdTRUE)
        {
            // Process received message
            DataLogger_ProcessMessage(&received_message);
        }
        else
        {
            // Timeout - perform periodic maintenance
            DataLogger_PeriodicMaintenance();
        }
    }
    
    // Cleanup (won't be reached)
    DataLogger_Shutdown();
}

/**
 * @brief Initialize Data Logger subsystem
 * @retval 0: Success, 1: Error
 */
static uint8_t DataLogger_Initialize(void)
{
    uint8_t flash_status;
    uint8_t manufacturer, device_id;
    
    // Create queue for receiving data from other tasks
    xDataLoggerQueue = xQueueCreate(DATA_LOGGER_QUEUE_SIZE, sizeof(LogMessage_t));
    if (xDataLoggerQueue == NULL) {
        printf("Failed to create data logger queue\r\n");
        return 1;
    }
    
    // Initialize sector buffer
    memset(sector_buffer, 0xFF, sizeof(sector_buffer));
    records_in_buffer = 0;
    
    // Initialize W25Q64 flash
    flash_status = w25qxx_basic_init(W25Q64, W25QXX_INTERFACE_SPI, W25QXX_BOOL_FALSE);
    if (flash_status != 0) {
        printf("Failed to initialize W25Q64 flash (err=%d)\r\n", flash_status);
        return 1;
    }
    
    // Verify chip ID
    flash_status = w25qxx_basic_get_id(&manufacturer, &device_id);
    if (flash_status == 0) {
        printf("W25Q64: manufacturer=0x%02X, device_id=0x%02X\r\n", manufacturer, device_id);
        if (manufacturer != 0xEF || device_id != 0x16) {
            printf("Warning: Unexpected chip ID\r\n");
        }
    }
    
    // Find current write position in ring buffer
    DataLogger_FindWritePosition();
    
    return 0;
}

/**
 * @brief Find current write position in ring buffer
 */
static void DataLogger_FindWritePosition(void)
{
    printf("Scanning 64KB ring buffer for write position...\r\n");
    BME280_Record_t scan_record;
    bool found_empty = false;
    
    // Scan by sectors instead of pages
    for (uint32_t addr = RING_BUFFER_START_ADDRESS; 
         addr < RING_BUFFER_END_ADDRESS; 
         addr += W25Q64_SECTOR_SIZE)  // 4KB increments instead of 256B
    {
        uint8_t status = w25qxx_basic_read(addr, (uint8_t *)&scan_record, sizeof(BME280_Record_t));
        if (status == 0 && scan_record.timestamp == 0xFFFFFFFF) {
            current_sector_address = addr;
            ring_buffer_write_address = addr;
            found_empty = true;
            printf("Found empty sector at 0x%08lX\r\n", addr);
            break;
        }
    }
    
    if (!found_empty) {
        printf("Ring buffer full, starting FIFO from beginning\r\n");
        current_sector_address = RING_BUFFER_START_ADDRESS;
        ring_buffer_write_address = RING_BUFFER_START_ADDRESS;
    }
}

/**
 * @brief Process received message
 */
static void DataLogger_ProcessMessage(LogMessage_t *message)
{
    BME280_Record_t record;
    
    switch (message->type)
    {
        case LOG_MSG_BME280_DATA:
            // Use original float data directly (no conversion needed!)
            record.timestamp = message->timestamp;
            record.temperature = message->data.bme_data.temperature;
            record.pressure = message->data.bme_data.pressure;
            record.humidity = message->data.bme_data.humidity;
            record.crc = DataLogger_CalculateCRC(&record);
            
            // printf("Logging BME280: T=%.2f°C, P=%.2fhPa, H=%.2f%%\r\n",
            //        record.temperature, record.pressure, record.humidity);
            
            DataLogger_AddRecordToBuffer(&record);
            break;
            
        case LOG_MSG_SYSTEM_EVENT:
            printf("System Event: %s\r\n", message->data.system_event);
            // Could log system events to flash as well
            break;
            
        case LOG_MSG_MQTT_STATUS:
            printf("MQTT Status: %d\r\n", message->data.mqtt_status);
            break;
            
        default:
            printf("Unknown message type: %d\r\n", message->type);
            break;
    }
}

/**
 * @brief Add record to page buffer with ring buffer management
 */
static void DataLogger_AddRecordToBuffer(BME280_Record_t *record)
{
    // Copy to sector buffer at correct offset
    uint32_t offset = records_in_buffer * sizeof(BME280_Record_t);
    memcpy(&sector_buffer[offset], record, sizeof(BME280_Record_t));
    records_in_buffer++;
    
    // Flush when sector buffer is full
    if (records_in_buffer >= RECORDS_PER_SECTOR) {
        DataLogger_FlushSectorBuffer();  // New function name
    }
}

/**
 * @brief Flush page buffer with 64KB ring buffer management
 */
static void DataLogger_FlushSectorBuffer(void)
{
    if (records_in_buffer == 0) return;
    
    uint32_t used_bytes = records_in_buffer * sizeof(BME280_Record_t);
    
    printf("Flushing 4KB sector: %d records (%lu bytes) at 0x%08lX\r\n", 
           records_in_buffer, used_bytes, current_sector_address);
    
    // Fill remaining buffer with 0xFF
    if (used_bytes < SECTOR_BUFFER_SIZE) {
        memset(&sector_buffer[used_bytes], 0xFF, SECTOR_BUFFER_SIZE - used_bytes);
    }
    
    // Write entire 4KB sector - MUCH more efficient!
    uint8_t status = w25qxx_basic_write(current_sector_address, 
                                        sector_buffer, 
                                        SECTOR_BUFFER_SIZE);  // 4096 bytes instead of 256
    
    if (status == 0) {
        total_records_written += records_in_buffer;
        
        // Advance to next 4KB sector
        current_sector_address += SECTOR_BUFFER_SIZE;
        ring_buffer_write_address = current_sector_address;
        
        // Ring buffer wrap-around (64KB FIFO)
        if (current_sector_address >= RING_BUFFER_END_ADDRESS) {
            printf("Ring buffer full (64KB), wrapping to start (FIFO)\r\n");
            current_sector_address = RING_BUFFER_START_ADDRESS;
            ring_buffer_write_address = RING_BUFFER_START_ADDRESS;
        }
        
        // Reset buffer
        records_in_buffer = 0;
        memset(sector_buffer, 0xFF, sizeof(sector_buffer));
        
        printf("4KB sector written successfully\r\n");
    } else {
        printf("Sector write failed (err=%d)\r\n", status);
    }
}

/**
 * @brief Periodic maintenance (called on queue timeout)
 */
static void DataLogger_PeriodicMaintenance(void)
{
    static uint32_t maintenance_counter = 0;
    maintenance_counter++;
    
    //Force flush every 60 seconds instead of 30 (larger buffer)
    if (maintenance_counter % 60 == 0 && records_in_buffer > 0) {
        printf("Periodic maintenance: flushing %d buffered records\r\n", records_in_buffer);
        DataLogger_FlushSectorBuffer();  // Updated function name
    }
    
    // Print status every 120 seconds instead of 60
    if (maintenance_counter % 120 == 0) {
        DataLogger_PrintStatus();
    }
}

/**
 * @brief Print current configuration
 */
static void DataLogger_PrintConfiguration(void)
{
    printf("\r\n=== Data Logger Configuration (4KB Sector Buffering) ===\r\n");
    printf("Ring Buffer Size: %d KB\r\n", FLASH_RING_BUFFER_SIZE / 1024);
    printf("Ring Buffer Range: 0x%08lX - 0x%08lX\r\n", 
           (uint32_t)RING_BUFFER_START_ADDRESS, (uint32_t)RING_BUFFER_END_ADDRESS);
    printf("Queue Size: %d messages\r\n", DATA_LOGGER_QUEUE_SIZE);
    printf("Records per Sector: %d\r\n", RECORDS_PER_SECTOR);
    printf("Sector Buffer Size: %d bytes\r\n", SECTOR_BUFFER_SIZE);
    
    printf("Total Ring Buffer Capacity: %d records\r\n", 
           FLASH_RING_BUFFER_SIZE / (int)sizeof(BME280_Record_t));
    printf("=======================================================\r\n\r\n");
}

/**
 * @brief Print current status
 */
static void DataLogger_PrintStatus(void)
{
    printf("\r\n=== Data Logger Status ===\r\n");
    printf("Ring Buffer Write Address: 0x%08lX\r\n", ring_buffer_write_address);
    printf("Current Sector Address: 0x%08lX\r\n", current_sector_address);
    printf("Records in Buffer: %d/%d\r\n", records_in_buffer, RECORDS_PER_SECTOR);
    printf("Total Records Written: %lu\r\n", total_records_written);
    printf("Ring Buffer Usage: %.1f%%\r\n", 
           ((float)(ring_buffer_write_address - RING_BUFFER_START_ADDRESS) / FLASH_RING_BUFFER_SIZE) * 100.0f);
    printf("=========================\r\n\r\n");
}

/**
 * @brief Shutdown data logger
 */
static void DataLogger_Shutdown(void)
{
    printf("Data Logger shutting down...\r\n");
    
    // Force flush any remaining buffered data
    if (records_in_buffer > 0) {
        printf("Flushing %d remaining records\r\n", records_in_buffer);
        DataLogger_FlushSectorBuffer();  // Updated function name
    }
    
    // Deinitialize flash
    w25qxx_basic_deinit();
    
    // Delete queue if it exists
    if (xDataLoggerQueue != NULL) {
        vQueueDelete(xDataLoggerQueue);
        xDataLoggerQueue = NULL;
    }
    
    printf("Data Logger shutdown complete\r\n");
}

/**
 * @brief Calculate CRC32 for record integrity
 */
static uint32_t DataLogger_CalculateCRC(BME280_Record_t *record)
{
    // Simple CRC calculation (implement proper CRC32 if needed)
    uint32_t crc = 0;
    uint8_t *data = (uint8_t *)record;
    size_t len = sizeof(BME280_Record_t) - sizeof(uint32_t); // Exclude CRC field
    
    for (size_t i = 0; i < len; i++) {
        crc += data[i];
    }
    return crc;
}

/**
 * @brief Send BME280 data to logger (called from BME280 task)
 */
BaseType_t DataLogger_QueueBME280Data(struct bme280_data *bme_data)
{
    LogMessage_t message = {
        .type = LOG_MSG_BME280_DATA,
        .timestamp = HAL_GetTick(),
        .data.bme_data = *bme_data  // Use original float values!
    };

    return xQueueSend(xDataLoggerQueue, &message, pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS));
}

/**
 * @brief Send system event to logger
 */
BaseType_t DataLogger_QueueSystemEvent(const char *event_text)
{
    LogMessage_t message = {
        .type = LOG_MSG_SYSTEM_EVENT,
        .timestamp = HAL_GetTick()
    };
    
    strncpy(message.data.system_event, event_text, sizeof(message.data.system_event) - 1);
    message.data.system_event[sizeof(message.data.system_event) - 1] = '\0';

    return xQueueSend(xDataLoggerQueue, &message, pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS));
}