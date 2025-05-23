#ifndef W25QXX_HAL_H
#define W25QXX_HAL_H

#include <stm32f756xx.h>
#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Define common HAL status codes for the W25QXX driver
typedef enum
{
    W25Q_OK = 0x00U,
    W25Q_ERROR = 0x01U,
    W25Q_BUSY = 0x02U,
    W25Q_TIMEOUT = 0x03U,
    W25Q_PARAM_ERR = 0x04U,
    W25Q_PROG_ERR = 0x05U, // Programming error detected
    W25Q_ERASE_ERR = 0x06U // Erase error detected
} W25Q_StatusTypeDef;

// Driver states
#define W25Q_STATE_READY      0
#define W25Q_STATE_BUSY_TX    1
#define W25Q_STATE_BUSY_RX    2
#define W25Q_STATE_BUSY_TX_RX 3

// --- W25QXX Chip Specific Definitions (Common for W25Qxx Series) ---
// Commands
#define W25Q_CMD_WRITE_ENABLE 0x06U
#define W25Q_CMD_VOLATILE_SR_WRITE_EN 0x50U // Enable Write Status Register
#define W25Q_CMD_WRITE_DISABLE 0x04U
#define W25Q_CMD_READ_STATUS_REG1 0x05U // Read Status Register-1
#define W25Q_CMD_READ_STATUS_REG2 0x35U // Read Status Register-2
#define W25Q_CMD_WRITE_STATUS_REG 0x01U // Write Status Register
#define W25Q_CMD_READ_DATA 0x03U        // Read Data
#define W25Q_CMD_FAST_READ 0x0BU        // Fast Read (with dummy byte)
#define W25Q_CMD_PAGE_PROGRAM 0x02U
#define W25Q_CMD_SECTOR_ERASE_4KB 0x20U // Sector Erase (4KB)
#define W25Q_CMD_BLOCK_ERASE_32KB 0x52U // Block Erase (32KB)
#define W25Q_CMD_BLOCK_ERASE_64KB 0xD8U // Block Erase (64KB)
#define W25Q_CMD_CHIP_ERASE 0xC7U       // or 0x60U
#define W25Q_CMD_POWER_DOWN 0xB9U
#define W25Q_CMD_RELEASE_POWER_DOWN_ID 0xABU  // Release Power-Down / Device ID
#define W25Q_CMD_MANUFACTURER_DEVICE_ID 0x90U // Read Manufacturer/Device ID
#define W25Q_CMD_JEDEC_ID 0x9FU               // Read JEDEC ID

// Status Register 1 Bits
#define W25Q_SR1_BUSY_BIT (1U << 0) // Write In Progress (WIP)
#define W25Q_SR1_WEL_BIT (1U << 1)  // Write Enable Latch (WEL)
#define W25Q_SR1_BP0_BIT (1U << 2)  // Block Protect bits (2:4)
#define W25Q_SR1_BP1_BIT (1U << 3)
#define W25Q_SR1_BP2_BIT (1U << 4)
#define W25Q_SR1_TB_BIT (1U << 5)   // Top/Bottom protect bit
#define W25Q_SR1_SRP0_BIT (1U << 6) // Status Register Protect 0
#define W25Q_SR1_SRP1_BIT (1U << 7) // Status Register Protect 1

// Status Register 2 Bits (0:1), the rest is reseved. 
#define W25Q_SR2_QE_BIT (1U << 1)  // Quad Enable (important for QSPI)
#define W25Q_SR2_SRL_BIT (1U << 0) // Status Register Lock

// Memory Organization (Typical for W25Qxx)
#define W25Q_DEFAULT_PAGE_SIZE 256U      // bytes
#define W25Q_DEFAULT_SECTOR_SIZE 4096U   // 4 KB
#define W25Q_DEFAULT_BLOCK32_SIZE 32768U // 32 KB
#define W25Q_DEFAULT_BLOCK64_SIZE 65536U // 64 KB

// Define a general timeout for communication operations (e.g., waiting for SPI transfer)
#define W25Q_COM_TIMEOUT_MS 100

// Define a maximum timeout for the W25Q_WaitForReady function,
// especially useful if the flash is not connected or faulty.
#define W25Q_MAX_READY_TIMEOUT_MS 1000 // 1 second

// for specific W25Q64BV chip
#define W25Q64BV_MFG_ID 0xEF
#define W25Q64BV_DEV_ID 0x4017

// dummy data in useal using 0xAA or 0x00
#define W25Q_DATA_DUMMY 0xAA

// Forward declaration
typedef struct W25Q_HandleTypeDef_s W25Q_HandleTypeDef;

/**
 * @brief Configuration structure for the W25QXX flash.
 * This structure holds the hardware and chip-specific parameters.
 */
typedef struct
{
    // Pointer to underlying SPI/QSPI peripheral handler (e.g., specific to STM32 HAL)
    SPI_HandleTypeDef *spi_handle; // Explicitly using STM32's SPI_HandleTypeDef
    // GPIO pin for Chip Select (CS) and its port
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    // DMA-related members
    uint8_t use_dma;           // Flag to enable/disable DMA mode
    DMA_HandleTypeDef *hdmatx; // DMA handle for transmit
    DMA_HandleTypeDef *hdmarx; // DMA handle for receive
    
    // Optional callback for DMA completion
    void (*TxCpltCallback)(W25Q_HandleTypeDef *hflash);
    void (*RxCpltCallback)(W25Q_HandleTypeDef *hflash);
    void (*TxRxCpltCallback)(W25Q_HandleTypeDef *hflash);

    // Flash chip specific parameters (can be overridden during init if known)
    uint32_t page_size;
    uint32_t sector_size;
    uint32_t block_32k_size;
    uint32_t block_64k_size;
    uint32_t total_size_bytes; // Total flash memory size in bytes (e.g., 16*1024*1024 for 128Mbit)

    // Optional: SPI/QSPI mode (future expansion)
    uint8_t spi_mode; // 1: SPI, 4: QSPI, 8: OctoSPI
} W25Q_ConfigTypeDef;

/**
 * @brief W25QXX Driver Handle structure.
 * This structure contains instance-specific data and configuration.
 */
typedef struct W25Q_HandleTypeDef_s
{
    W25Q_ConfigTypeDef config;
    uint8_t mfg_id;
    uint16_t dev_id;
    
    // New state tracking for DMA operations
    volatile uint8_t state;     // Current state (READY, BUSY_TX, BUSY_RX)
    volatile uint8_t dma_error; // DMA error flag
    volatile uint8_t tx_complete;
    volatile uint8_t rx_complete;
} W25Q_HandleTypeDef;

// --- Public Function Prototypes ---

/**
 * @brief Initializes the W25QXX flash driver instance.
 * This function configures the SPI peripheral and verifies flash presence.
 * @param hflash Pointer to the W25Q_HandleTypeDef structure.
 * @param pFlashConfig Pointer to the W25Q_ConfigTypeDef with initial hardware config.
 * @retval W25Q_StatusTypeDef Status of the initialization (W25Q_OK if successful)
 */
W25Q_StatusTypeDef W25Q_Init(W25Q_HandleTypeDef *hflash, const W25Q_ConfigTypeDef *pFlashConfig);

/**
 * @brief Deinitializes the W25QXX flash driver instance.
 * Resets the SPI peripheral to its default state.
 * @param hflash Pointer to the W25Q_HandleTypeDef structure.
 * @retval W25Q_StatusTypeDef Status of the deinitialization (W25Q_OK if successful)
 */
W25Q_StatusTypeDef W25Q_DeInit(W25Q_HandleTypeDef *hflash);

/**
 * @brief Reads the JEDEC ID (Manufacturer and Device ID) from the W25QXX flash.
 * @param hflash Pointer to the W25Q_HandleTypeDef structure.
 * @param pManufacturerID Pointer to store the Manufacturer ID.
 * @param pDeviceID Pointer to store the Device ID.
 * @retval W25Q_StatusTypeDef Status of the operation.
 */
W25Q_StatusTypeDef W25Q_ReadJEDECID(W25Q_HandleTypeDef *hflash, uint8_t *pManufacturerID, uint16_t *pDeviceID);

/**
 * @brief Reads the Status Register 1 from the W25QXX flash.
 * @param hflash Pointer to the W25Q_HandleTypeDef structure.
 * @param pStatusReg1 Pointer to store the Status Register 1 value.
 * @retval W25Q_StatusTypeDef Status of the operation.
 */
W25Q_StatusTypeDef W25Q_ReadStatusRegister1(W25Q_HandleTypeDef *hflash, uint8_t *pStatusReg1);

/**
 * @brief Reads the Status Register 2 from the W25QXX flash.
 * @param hflash Pointer to the W25Q_HandleTypeDef structure.
 * @param pStatusReg2 Pointer to store the Status Register 2 value.
 * @retval W25Q_StatusTypeDef Status of the operation.
 */
W25Q_StatusTypeDef W25Q_ReadStatusRegister2(W25Q_HandleTypeDef *hflash, uint8_t *pStatusReg2);

/**
 * @brief Calculates the approximate timeout for a flash operation based on size.
 * This is a basic estimation and may need fine-tuning.
 * @param hflash Pointer to the W25Q_HandleTypeDef structure.
 * @param operation_type W25Q_CMD_PAGE_PROGRAM, W25Q_CMD_SECTOR_ERASE_4KB, etc.
 * @param data_size Size of data for page program (in bytes) or 0 for erase.
 * @retval uint32_t Timeout value in milliseconds. Returns 0 if unknown operation.
 */
uint32_t W25Q_CalculateTimeout(const W25Q_HandleTypeDef *hflash, uint8_t operation_type, uint32_t data_size);

/**
 * @brief Reads data from the W25QXX flash at a specified address.
 * @param hflash Pointer to the W25Q_HandleTypeDef structure.
 * @param Address The starting address to read from.
 * @param pBuffer Pointer to the buffer where read data will be stored.
 * @param Size Number of bytes to read.
 * @retval W25Q_StatusTypeDef Status of the operation.
 */
W25Q_StatusTypeDef W25Q_Read(W25Q_HandleTypeDef *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size);

/**
 * @brief Writes data to a page in the W25QXX flash.
 * Note: Data must not exceed hflash->config.page_size.
 * The flash must be erased prior to writing.
 * @param hflash Pointer to the W25Q_HandleTypeDef structure.
 * @param Address The starting address to write to (must be page aligned).
 * @param pBuffer Pointer to the data to be written.
 * @param Size Number of bytes to write (max hflash->config.page_size).
 * @retval W25Q_StatusTypeDef Status of the operation.
 */
W25Q_StatusTypeDef W25Q_WritePage(W25Q_HandleTypeDef *hflash, uint32_t Address, const uint8_t *pBuffer, uint32_t Size);

/**
 * @brief Erases a 4KB sector in the W25QXX flash.
 * @param hflash Pointer to the W25Q_HandleTypeDef structure.
 * @param SectorAddress The starting address of the sector to erase (must be sector aligned).
 * @retval W25Q_StatusTypeDef Status of the operation.
 */
W25Q_StatusTypeDef W25Q_EraseSector(W25Q_HandleTypeDef *hflash, uint32_t SectorAddress);

// Public DMA function prototypes
W25Q_StatusTypeDef W25Q_Read_DMA(W25Q_HandleTypeDef *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size);
W25Q_StatusTypeDef W25Q_WritePage_DMA(W25Q_HandleTypeDef *hflash, uint32_t Address, const uint8_t *pBuffer, uint32_t Size);
W25Q_StatusTypeDef W25Q_WaitForDMATransferComplete(W25Q_HandleTypeDef *hflash, uint32_t Timeout);
#endif // W25QXX_HAL_H