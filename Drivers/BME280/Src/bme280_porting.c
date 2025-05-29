#include "bme280_porting.h"
#include "bme280.h" // For BME280_OK, BME280_E_COMM_FAIL etc.
#include <string.h> // For NULL
#include <stdio.h> // For printf

// Static instance of the porting context.
// This design assumes one BME280 sensor instance managed by this porting layer.
// If multiple BME280 sensors (or other I2C DMA devices sharing callbacks) exist,
// a more sophisticated context management for callbacks would be needed.
static bme280_port_ctx_t g_bme280_port_ctx;

// Forward declarations for static I2C and delay functions
static BME280_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr);
static BME280_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr);
static BME280_INTF_RET_TYPE user_i2c_read_blocking(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr);
static BME280_INTF_RET_TYPE user_i2c_write_blocking(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr);
static void user_delay_us(uint32_t period_us, void *intf_ptr);

int8_t bme280_interface_init(struct bme280_dev *dev, I2C_HandleTypeDef *i2c_handle, uint8_t bme280_i2c_addr)
{
    if (dev == NULL || i2c_handle == NULL)
    {
        return BME280_E_NULL_PTR;
    }

    // Initialize the porting context
    g_bme280_port_ctx.i2c_handle = i2c_handle;
    g_bme280_port_ctx.i2c_addr_shifted = (uint16_t)(bme280_i2c_addr << 1);
    g_bme280_port_ctx.dma_result = BME280_OK; // Default to OK

    // Create a binary semaphore for DMA synchronization
    // It's taken before starting DMA and given in the DMA completion/error ISR
    g_bme280_port_ctx.dma_sem = xSemaphoreCreateBinary();
    if (g_bme280_port_ctx.dma_sem == NULL)
    {
        // Failed to create semaphore
        return BME280_E_COMM_FAIL; // Or a more specific error
    }

    // Assign the BME280 driver function pointers
    // dev->read = user_i2c_read;     // uncomment to use non-blocking I2C (DMA)
    // dev->write = user_i2c_write;   // uncomment to use non-blocking I2C (DMA)
    dev->read = user_i2c_read_blocking;
    dev->write = user_i2c_write_blocking;
    dev->delay_us = user_delay_us;
    dev->intf = BME280_I2C_INTF;
    dev->intf_ptr = &g_bme280_port_ctx; // Pass our context to the R/W functions

    return BME280_OK;
}

int8_t bme280_interface_deinit(struct bme280_dev *dev)
{
    if (dev == NULL || dev->intf_ptr == NULL)
    {
        return BME280_E_NULL_PTR;
    }

    bme280_port_ctx_t *port_ctx = (bme280_port_ctx_t *)dev->intf_ptr;

    if (port_ctx->dma_sem != NULL)
    {
        vSemaphoreDelete(port_ctx->dma_sem);
        port_ctx->dma_sem = NULL;
    }
    // Clear the intf_ptr to indicate deinitialization
    dev->intf_ptr = NULL;
    return BME280_OK;
}

static BME280_INTF_RET_TYPE user_i2c_read_blocking(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    if (intf_ptr == NULL)
    {
        return BME280_E_NULL_PTR;
    }
    bme280_port_ctx_t *port_ctx = (bme280_port_ctx_t *)intf_ptr;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(port_ctx->i2c_handle,
                                                port_ctx->i2c_addr_shifted,
                                                reg_addr,
                                                I2C_MEMADD_SIZE_8BIT,
                                                read_data,
                                                len,
                                                1000);

    return (status == HAL_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}


static BME280_INTF_RET_TYPE user_i2c_write_blocking(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr)
{
    if (intf_ptr == NULL)
    {
        return BME280_E_NULL_PTR;
    }
    bme280_port_ctx_t *port_ctx = (bme280_port_ctx_t *)intf_ptr;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(port_ctx->i2c_handle,
                                                 port_ctx->i2c_addr_shifted,
                                                 reg_addr,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 (uint8_t *)write_data,
                                                 len,
                                                 1000);

    return (status == HAL_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}
static void user_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr; // Unused in this implementation

    if (period_us == 0)
    {
        return;
    }
    // Convert microseconds to FreeRTOS ticks, rounding up.
    // Minimum delay is 1 tick.
    uint32_t period_ms = (period_us + 999) / 1000; // Round up to nearest millisecond
    if (period_ms == 0 && period_us > 0)

    { // Ensure at least 1ms ms vTaskDelay if any us delay requested
        period_ms = 1;
    }
    vTaskDelay(pdMS_TO_TICKS(period_ms));
}

// Function to debug raw sensor data
void debug_bme280_raw_data(struct bme280_dev *dev)
{
    uint8_t data[8];
    int8_t rslt = bme280_get_regs(BME280_REG_DATA, data, 8, dev);

    if (rslt == BME280_OK)
    {
        printf("Raw data registers: ");
        for (int i = 0; i < 8; i++)
        {
            printf("0x%02X ", data[i]);
        }
        printf("\r\n");

        // Check if data is valid (not 0x80 0x00 0x00 pattern)
        if (data[0] == 0x80 && data[1] == 0x00 && data[2] == 0x00)
        {
            printf("WARNING: Data appears to be invalid/not ready\r\n");
        }
    }
    else
    {
        printf("Failed to read raw data registers: %d\r\n", rslt);
    }
}

// void convert_bme_data_to_int(struct bme280_data *bme_comp_data, bme_calc_data_int_t *bme_calc_data)
// {

//     bme_calc_data->temp_whole = (int32_t)bme_comp_data->temperature;
//     bme_calc_data->temp_frac = (int32_t)((bme_comp_data->temperature - bme_calc_data->temp_whole) * 100 + 0.5); // Round to nearest

//     bme_calc_data->press_pa = (uint32_t)(bme_comp_data->pressure + 0.5); // Round to nearest Pa
//     bme_calc_data->press_whole = bme_calc_data->press_pa / 100;
//     bme_calc_data->press_frac = bme_calc_data->press_pa % 100;

//     bme_calc_data->hum_whole = (uint32_t)bme_comp_data->humidity;
//     bme_calc_data->hum_frac = (uint32_t)((bme_comp_data->humidity - bme_calc_data->hum_whole) * 100 + 0.5); // Round to nearest
// }

// --- HAL I2C DMA Callback Wrappers ---
// These functions should be called from the global HAL I2C callbacks in stm32xxxx_it.c or similar

void bme280_hal_i2c_mem_tx_cplt_callback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Check if this callback is for the I2C peripheral used by BME280
    if (g_bme280_port_ctx.i2c_handle != NULL && hi2c->Instance == g_bme280_port_ctx.i2c_handle->Instance)
    {
        g_bme280_port_ctx.dma_result = BME280_OK;
        if (g_bme280_port_ctx.dma_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_bme280_port_ctx.dma_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    // Add else if blocks block if other I2C DMA devices share these callbacks
}

void bme280_hal_i2c_mem_rx_cplt_callback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (g_bme280_port_ctx.i2c_handle != NULL && hi2c->Instance == g_bme280_port_ctx.i2c_handle->Instance)
    {
        g_bme280_port_ctx.dma_result = BME280_OK;
        if (g_bme280_port_ctx.dma_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_bme280_port_ctx.dma_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void bme280_hal_i2c_error_callback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (g_bme280_port_ctx.i2c_handle != NULL && hi2c->Instance == g_bme280_port_ctx.i2c_handle->Instance)
    {

        g_bme280_port_ctx.dma_result = BME280_E_COMM_FAIL;
        if (g_bme280_port_ctx.dma_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_bme280_port_ctx.dma_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}



/*
// those function need to fix, its is not working in DMA mode.
// for the moment i use the polling mode.

static BME280_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    if (intf_ptr == NULL)
    {
        return BME280_E_NULL_PTR;
    }
    bme280_port_ctx_t *port_ctx = (bme280_port_ctx_t *)intf_ptr;

    port_ctx->dma_result = BME280_E_COMM_FAIL; // Assume failure until success

    // Ensure semaphore is taken (cleared) before starting a new transaction
    // This handles cases where a previous timeout might have occurred without the ISR giving the semaphore.
    // A zero timeout attempts to take it if available, effectively resetting its state if previously given.
    xSemaphoreTake(port_ctx->dma_sem, (TickType_t)0);

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(port_ctx->i2c_handle,
                                                    port_ctx->i2c_addr_shifted,
                                                    reg_addr,
                                                    I2C_MEMADD_SIZE_8BIT,
                                                    read_data,
                                                    len);

    if (status != HAL_OK)
    {

        // Log the specific HAL error
        printf("I2C Error: %d, HAL Error: %lu\r\n", BME280_E_COMM_FAIL, HAL_I2C_GetError(port_ctx->i2c_handle));
        return BME280_E_COMM_FAIL;
    }

    // Wait for the DMA completion ISR to give the semaphore
    if (xSemaphoreTake(port_ctx->dma_sem, pdMS_TO_TICKS(BME280_I2C_DMA_TIMEOUT_MS)) == pdTRUE)
    {
        // Semaphore was obtained, check the result from ISR
        return port_ctx->dma_result;
    }
    else
    {
        // Timeout waiting for semaphore
        // Attempt to abort the I2C transfer if possible/necessary
        // Use the available HAL function
        HAL_I2C_Master_Abort_IT(port_ctx->i2c_handle, port_ctx->i2c_addr_shifted);
        return BME280_E_COMM_FAIL; // Timeout
    }
}
static BME280_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr)
{
    if (intf_ptr == NULL)
    {
        return BME280_E_NULL_PTR;
    }

    bme280_port_ctx_t *port_ctx = (bme280_port_ctx_t *)intf_ptr;

    port_ctx->dma_result = BME280_E_COMM_FAIL; // Assume failure

    xSemaphoreTake(port_ctx->dma_sem, (TickType_t)0); // Clear semaphore

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(port_ctx->i2c_handle,
                                                     port_ctx->i2c_addr_shifted,
                                                     reg_addr,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     (uint8_t *)write_data, // HAL API expects non-const
                                                     len);

    if (status != HAL_OK)
    {
        return BME280_E_COMM_FAIL;
    }

    if (xSemaphoreTake(port_ctx->dma_sem, pdMS_TO_TICKS(BME280_I2C_DMA_TIMEOUT_MS)) == pdTRUE)
    {
        return port_ctx->dma_result;
    }
    else
    {
        // Use the available HAL function
        HAL_I2C_Master_Abort_IT(port_ctx->i2c_handle, port_ctx->i2c_addr_shifted);
        return BME280_E_COMM_FAIL; // Timeout
    }
}

// --- HAL I2C DMA Callback Wrappers ---
// These functions should be called from the global HAL I2C callbacks in stm32xxxx_it.c or similar

void bme280_hal_i2c_mem_tx_cplt_callback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Check if this callback is for the I2C peripheral used by BME280
    if (g_bme280_port_ctx.i2c_handle != NULL && hi2c->Instance == g_bme280_port_ctx.i2c_handle->Instance)
    {
        g_bme280_port_ctx.dma_result = BME280_OK;
        if (g_bme280_port_ctx.dma_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_bme280_port_ctx.dma_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    // Add else if blocks block if other I2C DMA devices share these callbacks
}

void bme280_hal_i2c_mem_rx_cplt_callback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (g_bme280_port_ctx.i2c_handle != NULL && hi2c->Instance == g_bme280_port_ctx.i2c_handle->Instance)
    {
        g_bme280_port_ctx.dma_result = BME280_OK;
        if (g_bme280_port_ctx.dma_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_bme280_port_ctx.dma_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void bme280_hal_i2c_error_callback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (g_bme280_port_ctx.i2c_handle != NULL && hi2c->Instance == g_bme280_port_ctx.i2c_handle->Instance)
    {

        g_bme280_port_ctx.dma_result = BME280_E_COMM_FAIL;
        if (g_bme280_port_ctx.dma_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_bme280_port_ctx.dma_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

*/