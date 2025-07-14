#include "peripheral_init_helper.h"

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
