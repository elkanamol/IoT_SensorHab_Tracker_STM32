#ifndef W25QXX_HAL_H
#define W25QXX_HAL_H

#include <stm32f756xx.h>
#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Configuration Options */
/* Comment out any option not needed in your application */
/* Note, the DMA and RTOS are not yet working well and need to FIX it */
// #define W25QXX_USE_DMA /* Enable DMA functionality */
// #define W25QXX_USE_RTOS   /* Enable FreeRTOS functionality */

/* Include FreeRTOS headers only if RTOS is enabled */
#ifdef W25QXX_USE_RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#endif
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
 */
typedef struct
{
    /* Base fields (always included) */
    SPI_HandleTypeDef *spi_handle;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    /* Flash chip specific parameters */
    uint32_t page_size;
    uint32_t sector_size;
    uint32_t block_32k_size;
    uint32_t block_64k_size;
    uint32_t total_size_bytes;
    uint8_t spi_mode;

#ifdef W25QXX_USE_DMA
    /* DMA-related members */
    uint8_t use_dma;
    DMA_HandleTypeDef *hdmatx;
    DMA_HandleTypeDef *hdmarx;

    /* Optional callback for DMA completion */
    void (*TxCpltCallback)(W25Q_HandleTypeDef *hflash);
    void (*RxCpltCallback)(W25Q_HandleTypeDef *hflash);
    void (*TxRxCpltCallback)(W25Q_HandleTypeDef *hflash);
#endif

#ifdef W25QXX_USE_RTOS
    /* FreeRTOS support flag */
    uint8_t use_rtos;
#endif
} W25Q_ConfigTypeDef;

/**
 * @brief W25QXX Driver Handle structure.
 */
typedef struct W25Q_HandleTypeDef_s
{
    /* Base fields */
    W25Q_ConfigTypeDef config;
    uint8_t mfg_id;
    uint16_t dev_id;

#ifdef W25QXX_USE_DMA
    /* State tracking for DMA operations */
    volatile uint8_t state;
    volatile uint8_t dma_error;
    volatile uint8_t tx_complete;
    volatile uint8_t rx_complete;
#endif

#ifdef W25QXX_USE_RTOS
    /* FreeRTOS synchronization primitives */
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t tx_semaphore;
    SemaphoreHandle_t rx_semaphore;
    SemaphoreHandle_t txrx_semaphore;

    /* FreeRTOS task and queue handles */
    TaskHandle_t task_handle;
    QueueHandle_t cmd_queue;
#endif
} W25Q_HandleTypeDef;

#ifdef W25QXX_USE_RTOS
/* Command types for flash operations (for FreeRTOS queue) */
typedef enum {
    W25Q_CMD_READ,
    W25Q_CMD_WRITE_PAGE,
    W25Q_CMD_ERASE_SECTOR,
    W25Q_CMD_READ_ID,
    W25Q_CMD_READ_STATUS
} W25Q_CommandType;

/* Command structure for flash operations (for FreeRTOS queue) */
typedef struct {
    W25Q_CommandType cmd_type;
    uint32_t address;
    uint8_t *buffer;
    uint32_t size;
    W25Q_StatusTypeDef *status;
    SemaphoreHandle_t completion_semaphore;
} W25Q_QueueItem;
#endif

/* --- Base/Core API Functions --- */

W25Q_StatusTypeDef W25Q_Init(W25Q_HandleTypeDef *hflash, const W25Q_ConfigTypeDef *pFlashConfig);
W25Q_StatusTypeDef W25Q_DeInit(W25Q_HandleTypeDef *hflash);
W25Q_StatusTypeDef W25Q_ReadJEDECID(W25Q_HandleTypeDef *hflash, uint8_t *pManufacturerID, uint16_t *pDeviceID);
W25Q_StatusTypeDef W25Q_ReadStatusRegister1(W25Q_HandleTypeDef *hflash, uint8_t *pStatusReg1);
W25Q_StatusTypeDef W25Q_ReadStatusRegister2(W25Q_HandleTypeDef *hflash, uint8_t *pStatusReg2);
uint32_t W25Q_CalculateTimeout(const W25Q_HandleTypeDef *hflash, uint8_t operation_type, uint32_t data_size);
W25Q_StatusTypeDef W25Q_Read(W25Q_HandleTypeDef *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size);
W25Q_StatusTypeDef W25Q_WritePage(W25Q_HandleTypeDef *hflash, uint32_t Address, const uint8_t *pBuffer, uint32_t Size);
W25Q_StatusTypeDef W25Q_EraseSector(W25Q_HandleTypeDef *hflash, uint32_t SectorAddress);

#ifdef W25QXX_USE_DMA
/* --- DMA API Functions --- */
W25Q_StatusTypeDef W25Q_Read_DMA(W25Q_HandleTypeDef *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size);
W25Q_StatusTypeDef W25Q_WritePage_DMA(W25Q_HandleTypeDef *hflash, uint32_t Address, const uint8_t *pBuffer, uint32_t Size);
W25Q_StatusTypeDef W25Q_WaitForDMATransferComplete(W25Q_HandleTypeDef *hflash, uint32_t Timeout);
#endif

#ifdef W25QXX_USE_RTOS
/* RTOS-specific function prototypes */
W25Q_StatusTypeDef W25Q_StartTask(W25Q_HandleTypeDef *hflash);
W25Q_StatusTypeDef W25Q_StopTask(W25Q_HandleTypeDef *hflash);
W25Q_StatusTypeDef W25Q_Read_RTOS(W25Q_HandleTypeDef *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size);
W25Q_StatusTypeDef W25Q_WritePage_RTOS(W25Q_HandleTypeDef *hflash, uint32_t Address, const uint8_t *pBuffer, uint32_t Size);
W25Q_StatusTypeDef W25Q_EraseSector_RTOS(W25Q_HandleTypeDef *hflash, uint32_t SectorAddress);
W25Q_StatusTypeDef W25Q_ReadJEDECID_RTOS(W25Q_HandleTypeDef *hflash, uint8_t *pManufacturerID, uint16_t *pDeviceID);
#endif
#endif // W25QXX_HAL_H