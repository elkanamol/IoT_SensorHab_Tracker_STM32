#ifndef DRIVER_W25QXX_INTERFACE_H
#define DRIVER_W25QXX_INTERFACE_H

#include "driver_w25qxx.h"
#include "stm32f7xx_hal.h" // Adjust include to your series
// #include "stdint.h"
#include "main.h"

//------------------------------------------------------------------------------
// Chip-select control (change to your board_config or port as needed)
//------------------------------------------------------------------------------
#define W25QXX_CS_PORT SPI1_CS_GPIO_Port
#define W25QXX_CS_PIN SPI1_CS_Pin

#define W25QXX_CS_LOW() HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_RESET)
#define W25QXX_CS_HIGH() HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_SET)

//------------------------------------------------------------------------------
// SPI, Delay and Debug “hooks”
//------------------------------------------------------------------------------
/**
 * @defgroup w25qxx_interface_driver w25qxx interface driver function
 * @brief    w25qxx interface driver modules
 * @ingroup  w25qxx_driver
 * @{
 */

/**
 * @brief  interface spi qspi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi qspi init failed
 * @note   none
 */
uint8_t w25qxx_interface_spi_qspi_init(void);

/**
 * @brief  interface spi qspi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi qspi deinit failed
 * @note   none
 */
uint8_t w25qxx_interface_spi_qspi_deinit(void);

/**
 * @brief      interface spi qspi bus write read
 * @param[in]  instruction sent instruction
 * @param[in]  instruction_line instruction phy lines
 * @param[in]  address register address
 * @param[in]  address_line address phy lines
 * @param[in]  address_len address length
 * @param[in]  alternate register address
 * @param[in]  alternate_line alternate phy lines
 * @param[in]  alternate_len alternate length
 * @param[in]  dummy dummy cycle
 * @param[in]  *in_buf pointer to a input buffer
 * @param[in]  in_len input length
 * @param[out] *out_buf pointer to a output buffer
 * @param[in]  out_len output length
 * @param[in]  data_line data phy lines
 * @return     status code
 *             - 0 success
 *             - 1 write read failed
 * @note       none
 */
uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
                                             uint32_t address, uint8_t address_line, uint8_t address_len,
                                             uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                                             uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                                             uint8_t *out_buf, uint32_t out_len, uint8_t data_line);

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void w25qxx_interface_delay_ms(uint32_t ms);

/**
 * @brief     interface delay us
 * @param[in] us time
 * @note      none
 */
void w25qxx_interface_delay_us(uint32_t us);

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void w25qxx_interface_debug_print(const char *const fmt, ...);

// Add the helper function declaration
uint8_t spi_write_read(uint8_t *in_buf, uint32_t in_len, uint8_t *out_buf, uint32_t out_len);

/**
 * @}
 */
#endif // DRIVER_W25QXX_INTERFACE_H