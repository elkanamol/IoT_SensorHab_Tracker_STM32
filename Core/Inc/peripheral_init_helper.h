#ifndef PERIPHERAL_INIT_HELPER_H
#define PERIPHERAL_INIT_HELPER_H

#include "FreeRTOS.h"
#include "semphr.h"

typedef enum {
    INIT_SUCCESS = 0,
    INIT_FAILED  = 1,
    INIT_TIMEOUT = 2
} InitResult_t;

typedef struct {
    int8_t (*init_func)(void *context);      // Pointer to initialization function
    void *context;                          // Context for the function (sensor struct, etc.)
    SemaphoreHandle_t mutex;                // Mutex for thread safety
    uint32_t max_retries;                   // Max attempts (0 = infinite)
    uint32_t retry_delay_ms;                // Delay between retries
    uint32_t backoff_factor;                // Multiplier for exponential backoff (1 = fixed)
    uint32_t mutex_timeout_ms;              // Timeout for taking mutex
} InitRetryConfig_t;

InitResult_t Peripheral_InitWithRetry(const InitRetryConfig_t *cfg);

#endif // PERIPHERAL_INIT_HELPER_H
