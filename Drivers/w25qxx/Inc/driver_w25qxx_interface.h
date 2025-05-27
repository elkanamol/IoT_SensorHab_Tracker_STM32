#ifndef DRIVER_W25QXX_INTERFACE_H
#define DRIVER_W25QXX_INTERFACE_H

#include "driver_w25qxx.h"
#include "stm32f7xx_hal.h" // Adjust include to your series
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


/* W25Q64 specific defines */
#define W25Q64_SECTOR_SIZE      4096        // 4KB sectors
#define W25Q64_PAGE_SIZE        256         // 256 byte pages
#define W25Q64_TOTAL_SIZE       8388608     // 8MB total (8 * 1024 * 1024)
#define W25Q64_BLOCK_SIZE       65536       // 64KB blocks
#define DATA_START_ADDRESS      0x1000      // Start logging at 4KB offset (after test area)

#define W25QXX_CS_PORT SPI1_CS_GPIO_Port
#define W25QXX_CS_PIN SPI1_CS_Pin

#define W25QXX_CS_LOW() HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_RESET)
#define W25QXX_CS_HIGH() HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_SET)

// DMA Configuration
#define W25QXX_DMA_TIMEOUT_MS 1000
#define CACHE_LINE_SIZE 32

// External variables that need to be defined in main.c or elsewhere
extern TaskHandle_t xW25QxxTaskHandle;
extern SemaphoreHandle_t xSpiMutex;

// Define a structure to store BME280 readings with timestamp
typedef struct
{
    uint32_t timestamp; // Timestamp in milliseconds
    float temperature;  // Temperature in Celsius
    float pressure;     // Pressure in hPa
    float humidity;     // Humidity in %
    uint32_t crc;       // CRC32 of the entire record
} BME280_Record_t;

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

// Add the helper function declaration (now DMA-enabled)
uint8_t spi_write_read(uint8_t *in_buf, uint32_t in_len, uint8_t *out_buf, uint32_t out_len);

// W25Q64 utility functions
/**
 * @brief     get the sector address from a given address
 * @param[in] address input address
 * @return    sector address
 * @note      none
 */
uint32_t W25Q64_GetSectorAddress(uint32_t address);

/**
 * @brief     get the page address from a given address
 * @param[in] address input address
 * @return    page address
 * @note      none
 */
uint32_t W25Q64_GetPageAddress(uint32_t address);

/**
 * @brief     get the sector number from a given address
 * @param[in] address input address
 * @return    sector number
 * @note      none
 */
uint32_t W25Q64_GetSectorNumber(uint32_t address);

/**
 * @brief     get the page number from a given address
 * @param[in] address input address
 * @return    page number
 * @note      none
 */
uint32_t W25Q64_GetPageNumber(uint32_t address);

/**
 * @brief     print buffer contents in hexadecimal format
 * @param[in] buffer pointer to the buffer to be printed
 * @param[in] size number of bytes to print from the buffer
 * @param[in] baseAddress optional base address for offset display
 * @note      useful for debugging and inspecting memory contents
 */
void PrintBufferHex(const uint8_t *buffer, uint32_t size, uint32_t baseAddress);

/**
 * @}
 */


#endif // DRIVER_W25QXX_INTERFACE_H