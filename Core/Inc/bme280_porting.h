#ifndef BME280_PORTING_H_
#define BME280_PORTING_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f7xx_hal.h"
#include "bme280.h"

#include "FreeRTOS.h"
#include "semphr.h"

#define BME280_I2C_DMA_TIMEOUT_MS  1000

// Structure to hold all port-specific context
typedef struct
{
    I2C_HandleTypeDef *i2c_handle; // Pointer to the STM32 I2C handle
    uint16_t i2c_addr_shifted;      // BME280 I2C address (7-bit address left-shifted by 1)
    SemaphoreHandle_t dma_sem;     // Semaphore for DMA transaction synchronization
    volatile int8_t dma_result;    // Result of the DMA transaction (BME280_OK or BME280_E_COMM_FAIL)
} bme280_port_ctx_t;

typedef struct
{
    int32_t temp_whole;
    int32_t temp_frac;

    uint32_t press_pa;
    uint32_t press_whole;
    uint32_t press_frac;

    uint32_t hum_whole;
    uint32_t hum_frac;
} bme_calc_data_int_t;

/**
 * @brief Initializes the BME280 interface porting layer.
 * @param[in,out] dev : Pointer to the BME280 device structure.
 * @param[in] i2c_handle : Pointer to the I2C_HandleTypeDef for BME280 communication.
 * @param[in] bme280_i2c_addr : The 7-bit I2C address of the BME280 sensor (e.g., BME280_I2C_ADDR_PRIM or BME280_I2C_ADDR_SEC).
 * @return BME280_OK on success, BME280_E_NULL_PTR or BME280_E_COMM_FAIL on failure.
 */
int8_t bme280_interface_init(struct bme280_dev *dev, I2C_HandleTypeDef *i2c_handle, uint8_t bme280_i2c_addr);

/**
 * @brief De-initializes the BME280 interface porting layer.
 * (Mainly for cleaning up resources like semaphores if dynamically allocated,
 * or if the sensor is to be powered down and re-initialized later)
 * @param[in] dev : Pointer to the BME280 device structure.
 * @return BME280_OK on success.
 */
int8_t bme280_interface_deinit(struct bme280_dev *dev);

/*
 * The following functions are not meant to be called directly by the application,
 * but are called by the BME280 driver via function pointers.
 * They are exposed here only to allow the HAL I2C DMA callbacks to be defined
 * in bme280_porting.c and to signal completion.
 * For a cleaner approach in very complex systems with many I2C devices,
 * a registration mechanism for HAL callbacks might be used.
 */

/**
 * @brief HAL I2C Master Tx Transfer completed callback.
 * This function *must* be called from the global HAL_I2C_MemTxCpltCallback
 * if the I2C instance matches the one used for BME280.
 * @param[in] hi2c : Pointer to the I2C_HandleTypeDef that completed.
 */
void bme280_hal_i2c_mem_tx_cplt_callback(I2C_HandleTypeDef *hi2c);

/**
 * @brief HAL I2C Master Rx Transfer completed callback.
 * This function *must* be called from the global HAL_I2C_MemRxCpltCallback
 * if the I2C instance matches the one used for BME280.
 * @param[in] hi2c : Pointer to the I2C_HandleTypeDef that completed.
 */
void bme280_hal_i2c_mem_rx_cplt_callback(I2C_HandleTypeDef *hi2c);

/**
 * @brief HAL I2C Error callback.
 * This function *must* be called from the global HAL_I2C_ErrorCallback
 * if the I2C instance matches the one used for BME280.
 * @param[in] hi2c : Pointer to the I2C_HandleTypeDef that erred.
 */
void bme280_hal_i2c_error_callback(I2C_HandleTypeDef *hi2c);

void debug_bme280_raw_data(struct bme280_dev *dev);

/**
 * @brief Converts BME280 sensor data from floating-point to integer representation.
 * @param[in] bme_comp_data Pointer to the compensated BME280 sensor data structure.
 * @param[out] bme_calc_data Pointer to the integer-based calculated sensor data structure.
 */
void convert_bme_data_to_int(struct bme280_data *bme_comp_data, bme_calc_data_int_t *bme_calc_data);

#ifdef __cplusplus
}
#endif

#endif /* BME280_PORTING_H_ */