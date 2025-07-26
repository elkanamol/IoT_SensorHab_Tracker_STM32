#include "peripheral_init_helper.h"

/**
 * @brief Attempts to initialize a peripheral with configurable retry mechanism
 *
 * Initializes a peripheral by calling the provided initialization function,
 * with support for multiple retry attempts and exponential backoff.
 *
 * @param cfg Pointer to configuration structure defining initialization retry parameters
 * @return InitResult_t Indicates whether initialization succeeded or failed
 */
InitResult_t Peripheral_InitWithRetry(const InitRetryConfig_t *cfg)
{
    uint32_t attempt = 0;
    uint32_t delay = cfg->retry_delay_ms;
    int8_t result;

    while (cfg->max_retries == 0 || attempt < cfg->max_retries) {
        result = cfg->init_func(cfg->context);
        if (result == 0) { // Success
            return INIT_SUCCESS;
        }
        attempt++;
        vTaskDelay(pdMS_TO_TICKS(delay));
        delay *= cfg->backoff_factor;
    }
    return INIT_FAILED;
}
