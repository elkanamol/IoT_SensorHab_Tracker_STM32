#ifndef DATALOGGER_H
#define DATALOGGER_H

#include "main.h"
#include "bme280.h"
#include "w25qxx_hal.h"
#include "driver_w25qxx_basic.h"
#include "FreeRTOS.h"
#include "queue.h"

// Ring buffer configuration (64KB FIFO)
#define FLASH_RING_BUFFER_SIZE (64 * 1024)  // 64KB - can be changed later
#define RING_BUFFER_START_ADDRESS (DATA_START_ADDRESS)
#define RING_BUFFER_END_ADDRESS (RING_BUFFER_START_ADDRESS + FLASH_RING_BUFFER_SIZE)
#define RECORDS_PER_PAGE (W25Q64_PAGE_SIZE / sizeof(BME280_Record_t))

// Queue configuration
#define DATA_LOGGER_QUEUE_SIZE 20
#define MAX_QUEUE_WAIT_MS 1000
#define QUEUE_SEND_TIMEOUT_MS 1000

// Data logger message types
typedef enum {
    LOG_MSG_BME280_DATA,
    LOG_MSG_SYSTEM_EVENT,
    LOG_MSG_MQTT_STATUS,
    // Add more message types as needed
} LogMsgType_t;

// Queue message structure
typedef struct {
    LogMsgType_t type;
    uint32_t timestamp;
    union {
        struct bme280_data bme_data;    // Use original float data
        char system_event[32];
        uint8_t mqtt_status;
        // Add more data types as needed
    } data;
} LogMessage_t;

// External variables (declared here, defined in datalogger.c)
extern QueueHandle_t xDataLoggerQueue;

// Public API functions (called from other modules)
void StartDataLoggerTask(void *argument);
BaseType_t DataLogger_QueueBME280Data(struct bme280_data *bme_data);
BaseType_t DataLogger_QueueSystemEvent(const char *event_text);

#endif /* DATALOGGER_H */