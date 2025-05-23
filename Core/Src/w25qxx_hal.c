#include "w25qxx_hal.h"
#include <string.h> // For memcpy

// --- Private Functions Prototypes ---
static void W25Q_CS_Enable(W25Q_HandleTypeDef *hflash);
static void W25Q_CS_Disable(W25Q_HandleTypeDef *hflash);
static W25Q_StatusTypeDef W25Q_SPI_Transmit(W25Q_HandleTypeDef *hflash, uint8_t *pData, uint16_t Size);
static W25Q_StatusTypeDef W25Q_SPI_Receive(W25Q_HandleTypeDef *hflash, uint8_t *pData, uint16_t Size);
static W25Q_StatusTypeDef W25Q_SPI_TransmitReceive(W25Q_HandleTypeDef *hflash, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
static W25Q_StatusTypeDef W25Q_WriteEnable(W25Q_HandleTypeDef *hflash);
static W25Q_StatusTypeDef W25Q_WaitForReady(W25Q_HandleTypeDef *hflash, uint32_t Timeout);
static W25Q_StatusTypeDef W25Q_SendCmdAddress(W25Q_HandleTypeDef *hflash, uint8_t cmd, uint32_t address);
static W25Q_StatusTypeDef W25Q_SendCmdAddressData(W25Q_HandleTypeDef *hflash, uint8_t cmd, uint32_t address, const uint8_t *pData, uint32_t size);

// Single global pointer to the W25Q handle
static W25Q_HandleTypeDef* g_w25q_handle = NULL;

// --- Private Functions Implementation ---

/**
 * @brief Enables the Chip Select (CS) pin for the W25Q Flash memory device
 * @param hflash Pointer to the W25Q Flash memory handle
 * @note Sets the Chip Select pin to active low state, preparing the device for communication
 */
static void W25Q_CS_Enable(W25Q_HandleTypeDef *hflash)
{
    HAL_GPIO_WritePin(hflash->config.cs_port, hflash->config.cs_pin, GPIO_PIN_RESET); // Active Low
}

/**
 * @brief Disables the Chip Select (CS) pin for the W25Q Flash memory device
 * @param hflash Pointer to the W25Q Flash memory handle
 * @note Sets the Chip Select pin to inactive high state, ending communication with the device
 */
static void W25Q_CS_Disable(W25Q_HandleTypeDef *hflash)
{
    HAL_GPIO_WritePin(hflash->config.cs_port, hflash->config.cs_pin, GPIO_PIN_SET); // Inactive High
}

/**
 * @brief Transmits data over SPI for the W25Q Flash memory device
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param pData Pointer to the data buffer to be transmitted
 * @param Size Number of bytes to transmit
 * @return W25Q_StatusTypeDef Status of the SPI transmission operation
 *         - W25Q_OK: Transmission successful
 *         - W25Q_PARAM_ERR: Invalid input parameters
 *         - W25Q_ERROR: Transmission failed
 */
static W25Q_StatusTypeDef W25Q_SPI_Transmit(W25Q_HandleTypeDef *hflash, uint8_t *pData, uint16_t Size)
{
    if (hflash == NULL || pData == NULL || Size == 0 )
    {
        return W25Q_PARAM_ERR;
    }
    if (HAL_SPI_Transmit(hflash->config.spi_handle, pData, Size, W25Q_COM_TIMEOUT_MS) == HAL_OK)
    {
        return W25Q_OK;
    }
    return W25Q_ERROR;
}

static W25Q_StatusTypeDef W25Q_SPI_Transmit_DMA(W25Q_HandleTypeDef *hflash, uint8_t *pData, uint16_t Size)
{
    if (hflash == NULL || pData == NULL || Size == 0 || hflash->config.hdmatx == NULL)
    {
        return W25Q_PARAM_ERR;
    }
    
    hflash->state = W25Q_STATE_BUSY_TX;
    hflash->tx_complete = 0;
    hflash->dma_error = 0;
    
    if (HAL_SPI_Transmit_DMA(hflash->config.spi_handle, pData, Size) == HAL_OK)
    {
        return W25Q_OK;
    }
    
    hflash->state = W25Q_STATE_READY;
    return W25Q_ERROR;
}
/**
 * @brief Receives data over SPI for the W25Q Flash memory device
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param pData Pointer to the buffer where received data will be stored
 * @param Size Number of bytes to receive
 * @return W25Q_StatusTypeDef Status of the SPI receive operation
 *         - W25Q_OK: Reception successful
 *         - W25Q_PARAM_ERR: Invalid input parameters
 *         - W25Q_ERROR: Reception failed
 */
static W25Q_StatusTypeDef W25Q_SPI_Receive(W25Q_HandleTypeDef *hflash, uint8_t *pData, uint16_t Size)
{
    if (hflash == NULL || pData == NULL || Size == 0)
    {
        return W25Q_PARAM_ERR;
    }
    if (HAL_SPI_Receive(hflash->config.spi_handle, pData, Size, W25Q_COM_TIMEOUT_MS) == HAL_OK)
    {
        return W25Q_OK;
    }
    return W25Q_ERROR;
}

/**
 * @brief Transmits and receives data over SPI for the W25Q Flash memory device
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param pTxData Pointer to the data buffer to be transmitted
 * @param pRxData Pointer to the buffer where received data will be stored
 * @param Size Number of bytes to transmit and receive
 * @return W25Q_StatusTypeDef Status of the SPI transmit-receive operation
 *         - W25Q_OK: Transmission and reception successful
 *         - W25Q_PARAM_ERR: Invalid input parameters
 *         - W25Q_ERROR: Transmission or reception failed
 */
static W25Q_StatusTypeDef W25Q_SPI_TransmitReceive(W25Q_HandleTypeDef *hflash, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    if (hflash == NULL || pTxData == NULL || pRxData == NULL || Size == 0)
    {
        return W25Q_PARAM_ERR;
    }
    if (HAL_SPI_TransmitReceive(hflash->config.spi_handle, pTxData, pRxData, Size, W25Q_COM_TIMEOUT_MS) == HAL_OK)
    {
        return W25Q_OK;
    }
    return W25Q_ERROR;
}

/**
 * @brief Enable write operations for the W25Q Flash memory device
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 *
 * @return W25Q_StatusTypeDef Status of the write enable operation
 *         - W25Q_OK if write enable is successful
 *         - W25Q_ERROR if write enable fails
 *         - W25Q_PARAM_ERR if the handle is NULL
 *
 * @note This function sends the Write Enable command to allow subsequent write operations
 */
static W25Q_StatusTypeDef W25Q_WriteEnable(W25Q_HandleTypeDef *hflash)
{
    if (hflash == NULL)
    {
        return W25Q_PARAM_ERR;
    }
    W25Q_StatusTypeDef status;
    uint8_t cmd = W25Q_CMD_WRITE_ENABLE;

    W25Q_CS_Enable(hflash);
    status = W25Q_SPI_Transmit(hflash, &cmd, 1);
    W25Q_CS_Disable(hflash);

    if (status != W25Q_OK)
    {
        return status;
    }

    // Optional: Verify WEL bit is set in status register
    uint8_t sr1;
    status = W25Q_ReadStatusRegister1(hflash, &sr1);
    if (status != W25Q_OK || !(sr1 & W25Q_SR1_WEL_BIT))
    {
        return W25Q_ERROR; // WEL not set
    }
    return W25Q_OK;
}

/**
 * @brief Wait for the W25Q Flash memory device to become ready.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param Timeout Maximum time to wait for the device to become ready (in milliseconds)
 *
 * @return W25Q_StatusTypeDef Status of the operation
 *         - W25Q_OK if the device is ready
 *         - W25Q_ERROR if there's an error reading the status register
 *         - W25Q_TIMEOUT if the device does not become ready within the specified timeout
 *         - W25Q_PARAM_ERR if the handle is NULL
 *
 * @note This function polls the device's status register to check if the Write-In-Progress (WIP) bit is cleared
 */
static W25Q_StatusTypeDef W25Q_WaitForReady(W25Q_HandleTypeDef *hflash, uint32_t Timeout)
{
    if (hflash == NULL)
    {
        return W25Q_PARAM_ERR;
    }
    uint8_t status_reg1;
    uint32_t start_time = HAL_GetTick(); // Get current tick for timeout

    do
    {
        if (W25Q_ReadStatusRegister1(hflash, &status_reg1) != W25Q_OK)
        {
            return W25Q_ERROR; // Error reading status register
        }
        if ((HAL_GetTick() - start_time) > Timeout)
        {
            return W25Q_TIMEOUT; // Timeout occurred
        }
    } while (status_reg1 & W25Q_SR1_BUSY_BIT); // Loop while WIP bit is set

    return W25Q_OK;
}

/**
 * @brief Helper function to send a command with a 24-bit address to the W25Q Flash memory device.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param cmd Command byte to be sent
 * @param address 24-bit memory address to send
 *
 * @return W25Q_StatusTypeDef Status of the operation (W25Q_OK on success)
 *
 * @note This is an internal function for sending command and address to the Flash device
 */
static W25Q_StatusTypeDef W25Q_SendCmdAddress(W25Q_HandleTypeDef *hflash, uint8_t cmd, uint32_t address)
{
    if (hflash == NULL || cmd == 0)
    {
        return W25Q_PARAM_ERR;
    }
    uint8_t tx_buffer[4]; // Command + 3-byte address
    tx_buffer[0] = cmd;
    tx_buffer[1] = (address >> 16) & 0xFF;
    tx_buffer[2] = (address >> 8) & 0xFF;
    tx_buffer[3] = address & 0xFF;

    return W25Q_SPI_Transmit(hflash, tx_buffer, 4);
}

/**
 * @brief Helper function, sends a command with address and data to the W25Q Flash memory device.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param cmd Command byte to be sent
 * @param address 24-bit memory address to write to
 * @param pData Pointer to the data buffer to be transmitted
 * @param size Number of bytes to transmit
 *
 * @return W25Q_StatusTypeDef Status of the operation (W25Q_OK on success)
 *
 * @note This is an internal function for sending command, address, and data to the Flash device
 */
static W25Q_StatusTypeDef W25Q_SendCmdAddressData(W25Q_HandleTypeDef *hflash, uint8_t cmd, uint32_t address, const uint8_t *pData, uint32_t size)
{
    if (hflash == NULL || pData == NULL || size == 0)
    {
        return W25Q_PARAM_ERR;
    }
    W25Q_StatusTypeDef status;
    uint8_t tx_header[4]; // Command + 3-byte address
    tx_header[0] = cmd;
    tx_header[1] = (address >> 16) & 0xFF;
    tx_header[2] = (address >> 8) & 0xFF;
    tx_header[3] = address & 0xFF;

    W25Q_CS_Enable(hflash);
    status = W25Q_SPI_Transmit(hflash, tx_header, 4);
    if (status != W25Q_OK)
    {
        W25Q_CS_Disable(hflash);
        return status;
    }
    status = W25Q_SPI_Transmit(hflash, (uint8_t *)pData, size); // Cast to uint8_t* for HAL_SPI_Transmit
    W25Q_CS_Disable(hflash);
    return status;
}

// --- Public Functions Implementation ---

W25Q_StatusTypeDef W25Q_Init(W25Q_HandleTypeDef *hflash, const W25Q_ConfigTypeDef *pFlashConfig)
{
    if (hflash == NULL || pFlashConfig == NULL || pFlashConfig->spi_handle == NULL ||
        pFlashConfig->cs_port == NULL || pFlashConfig->cs_pin == 0)
    {
        return W25Q_PARAM_ERR;
    }

    // Copy configuration to the handle
    memcpy(&hflash->config, pFlashConfig, sizeof(W25Q_ConfigTypeDef));

    // Set default chip parameters if not explicitly provided (e.g., if total_size_bytes is 0)
    if (hflash->config.page_size == 0)
        hflash->config.page_size = W25Q_DEFAULT_PAGE_SIZE;
    if (hflash->config.sector_size == 0)
        hflash->config.sector_size = W25Q_DEFAULT_SECTOR_SIZE;
    if (hflash->config.block_32k_size == 0)
        hflash->config.block_32k_size = W25Q_DEFAULT_BLOCK32_SIZE;
    if (hflash->config.block_64k_size == 0)
        hflash->config.block_64k_size = W25Q_DEFAULT_BLOCK64_SIZE;
    // total_size_bytes should ideally be known and set by the user based on the actual chip.

    W25Q_CS_Disable(hflash);

    // Store the global handle for callbacks
    g_w25q_handle = hflash;

    // Initialize state
    hflash->state = W25Q_STATE_READY;
    hflash->tx_complete = 0;
    hflash->rx_complete = 0;
    hflash->dma_error = 0;

    // Verify communication with the flash by reading JEDEC ID
    uint8_t mfg_id_read;
    uint16_t dev_id_read;
    if (W25Q_ReadJEDECID(hflash, &mfg_id_read, &dev_id_read) != W25Q_OK)
    {
        return W25Q_ERROR;
    }

    // Store the discovered IDs in the handle
    hflash->mfg_id = mfg_id_read;
    hflash->dev_id = dev_id_read;

    // You might want to add checks here for expected Manufacturer/Device IDs
    // For Winbond, Manufacturer ID is typically 0xEF. Device ID varies (e.g., 0x4016 for W25Q32FV, 0x4018 for W25Q128FV)
    if (hflash->mfg_id != 0xEF)
    {
        // Log an error or return a specific error code
        return W25Q_ERROR;
    }
    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_DeInit(W25Q_HandleTypeDef *hflash)
{
    if (hflash == NULL)
    {
        return W25Q_PARAM_ERR;
    }
    W25Q_CS_Disable(hflash); // Ensure CS is inactive
    
    g_w25q_handle = NULL;

    hflash->config.spi_handle = NULL; // Reset SPI handle
    hflash->config.cs_port = NULL;     // Reset CS port
    hflash->config.cs_pin = 0;         // Reset CS pin

    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_ReadJEDECID(W25Q_HandleTypeDef *hflash, uint8_t *pManufacturerID, uint16_t *pDeviceID)
{
    if (hflash == NULL || pManufacturerID == NULL || pDeviceID == NULL)
    {
        return W25Q_PARAM_ERR;
    }

    uint8_t tx_buffer[4] = {W25Q_CMD_JEDEC_ID, W25Q_DATA_DUMMY, W25Q_DATA_DUMMY, W25Q_DATA_DUMMY}; // Cmd + 3 dummy bytes
    uint8_t rx_buffer[4];

    W25Q_CS_Enable(hflash);
    W25Q_StatusTypeDef status = W25Q_SPI_TransmitReceive(hflash, tx_buffer, rx_buffer, 4);
    W25Q_CS_Disable(hflash);

    if (status == W25Q_OK)
    {
        *pManufacturerID = rx_buffer[1]; // JEDEC ID format: Mfr. ID (1 byte), Device ID (2 bytes)
        *pDeviceID = (uint16_t)(rx_buffer[2] << 8) | rx_buffer[3];
    }
    return status;
}

W25Q_StatusTypeDef W25Q_ReadStatusRegister1(W25Q_HandleTypeDef *hflash, uint8_t *pStatusReg1)
{
    if (hflash == NULL || pStatusReg1 == NULL)
    {
        return W25Q_PARAM_ERR;
    }

    uint8_t tx_cmd = W25Q_CMD_READ_STATUS_REG1;
    uint8_t rx_byte;

    W25Q_CS_Enable(hflash);
    W25Q_StatusTypeDef status = W25Q_SPI_Transmit(hflash, &tx_cmd, 1);
    if (status == W25Q_OK)
    {
        status = W25Q_SPI_Receive(hflash, &rx_byte, 1);
    }
    W25Q_CS_Disable(hflash);

    if (status == W25Q_OK)
    {
        *pStatusReg1 = rx_byte;
    }
    return status;
}

W25Q_StatusTypeDef W25Q_ReadStatusRegister2(W25Q_HandleTypeDef *hflash, uint8_t *pStatusReg2)
{
    if (hflash == NULL || pStatusReg2 == NULL)
    {
        return W25Q_PARAM_ERR;
    }

    uint8_t tx_cmd = W25Q_CMD_READ_STATUS_REG2;
    uint8_t rx_byte;

    W25Q_CS_Enable(hflash);
    W25Q_StatusTypeDef status = W25Q_SPI_Transmit(hflash, &tx_cmd, 1);
    if (status == W25Q_OK)
    {
        status = W25Q_SPI_Receive(hflash, &rx_byte, 1);
    }
    W25Q_CS_Disable(hflash);

    if (status == W25Q_OK)
    {
        *pStatusReg2 = rx_byte;
    }
    return status;
}

uint32_t W25Q_CalculateTimeout(const W25Q_HandleTypeDef *hflash, uint8_t operation_type, uint32_t data_size)
{
    // These are typical maximum times from W25Qxx datasheets.
    // Always refer to the specific datasheet for your part number.
    // These are *maximum* times. Average times are much lower.
    // For safety, we use max times.
    if (hflash == NULL)
        return 0; // Invalid handle

    switch (operation_type)
    {
    case W25Q_CMD_PAGE_PROGRAM:
        // Max page program time: ~3ms per page (256 bytes)
        return (data_size / hflash->config.page_size) * 3 + 5; // +5ms buffer

    case W25Q_CMD_SECTOR_ERASE_4KB:
        // Max sector erase time: ~400ms
        return 400;

    case W25Q_CMD_BLOCK_ERASE_32KB:
        // Max 32KB block erase time: ~1.5s
        return 1500;

    case W25Q_CMD_BLOCK_ERASE_64KB:
        // Max 64KB block erase time: ~2s
        return 2000;

    case W25Q_CMD_CHIP_ERASE:
        // per the datasheet the W25Q64BV have max of 30 sec erase time. 
        if (hflash->dev_id == W25Q64BV_DEV_ID)
            return 30000;
        // Provide a generic, very safe fallback
        return 60000; // 60 seconds for larger chips or safety

    default:
        return 0; // Unknown operation type
    }
}

W25Q_StatusTypeDef W25Q_Read(W25Q_HandleTypeDef *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size)
{
    if (hflash == NULL || pBuffer == NULL || Size == 0)
    {
        return W25Q_PARAM_ERR;
    }

    uint8_t tx_header[5];              // Command + 3-byte address + 1 dummy byte for Fast Read
    tx_header[0] = W25Q_CMD_FAST_READ; // Using Fast Read for better performance
    tx_header[1] = (Address >> 16) & 0xFF;
    tx_header[2] = (Address >> 8) & 0xFF;
    tx_header[3] = Address & 0xFF;
    tx_header[4] = W25Q_DATA_DUMMY; // Dummy byte

    W25Q_CS_Enable(hflash);
    W25Q_StatusTypeDef status = W25Q_SPI_Transmit(hflash, tx_header, 5); // Send command, address, dummy
    if (status == W25Q_OK)
    {
        status = W25Q_SPI_Receive(hflash, pBuffer, Size); // Receive data
    }
    W25Q_CS_Disable(hflash);
    return status;
}

W25Q_StatusTypeDef W25Q_WritePage(W25Q_HandleTypeDef *hflash, uint32_t Address, const uint8_t *pBuffer, uint32_t Size)
{
    if (hflash == NULL || pBuffer == NULL || Size == 0 || Size > hflash->config.page_size || (Address % hflash->config.page_size) != 0)
    {
        return W25Q_PARAM_ERR;
    }

    W25Q_StatusTypeDef status;
    uint32_t timeout_ms = W25Q_CalculateTimeout(hflash, W25Q_CMD_PAGE_PROGRAM, Size);

    // 1. Write Enable
    status = W25Q_WriteEnable(hflash);
    if (status != W25Q_OK)
    {
        return status;
    }

    // 2. Send Page Program Command + Address + Data
    status = W25Q_SendCmdAddressData(hflash, W25Q_CMD_PAGE_PROGRAM, Address, pBuffer, Size);
    if (status != W25Q_OK)
    {
        return status;
    }

    // 3. Wait for Write In Progress (WIP) bit to clear
    status = W25Q_WaitForReady(hflash, timeout_ms);
    if (status != W25Q_OK)
    {
        return status;
    }

    // Optional: Verify successful program by checking status register bits (e.g., Program Fail)
    // This requires specific bit definitions for program/erase fail.
    // uint8_t sr2;
    // W25Q_ReadStatusRegister2(hflash, &sr2);
    // if (sr2 & W25Q_SR2_PFAIL_BIT) return W25Q_PROG_ERR; // If such a bit exists

    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_EraseSector(W25Q_HandleTypeDef *hflash, uint32_t SectorAddress)
{
    // Address must be aligned to sector boundary
    if (hflash == NULL || (SectorAddress % hflash->config.sector_size) != 0)
    {
        return W25Q_PARAM_ERR;
    }

    W25Q_StatusTypeDef status;
    uint32_t timeout_ms = W25Q_CalculateTimeout(hflash, W25Q_CMD_SECTOR_ERASE_4KB, 0); // 0 for data_size for erase, only one sector!

    // 1. Write Enable
    status = W25Q_WriteEnable(hflash);
    if (status != W25Q_OK)
    {
        return status;
    }

    // 2. Send Sector Erase Command + Address
    W25Q_CS_Enable(hflash);
    status = W25Q_SendCmdAddress(hflash, W25Q_CMD_SECTOR_ERASE_4KB, SectorAddress);
    W25Q_CS_Disable(hflash); // De-assert CS after sending command and address
    if (status != W25Q_OK)
    {
        return status;
    }

    // 3. Wait for Write In Progress (WIP) bit to clear
    status = W25Q_WaitForReady(hflash, timeout_ms);
    if (status != W25Q_OK)
    {
        return status;
    }
    return W25Q_OK;
}

// DMA completion callbacks
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // We know there's only one flash chip, so just use the global handle
    if (g_w25q_handle != NULL && g_w25q_handle->config.spi_handle == hspi)
    {
        g_w25q_handle->tx_complete = 1;
        g_w25q_handle->state = W25Q_STATE_READY;
        
        // Deassert CS pin
        W25Q_CS_Disable(g_w25q_handle);
        
        // Call user callback if defined
        if (g_w25q_handle->config.TxCpltCallback != NULL)
        {
            g_w25q_handle->config.TxCpltCallback(g_w25q_handle);
        }
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (g_w25q_handle != NULL && g_w25q_handle->config.spi_handle == hspi)
    {
        g_w25q_handle->rx_complete = 1;
        g_w25q_handle->state = W25Q_STATE_READY;
        
        // Deassert CS pin
        W25Q_CS_Disable(g_w25q_handle);
        
        // Call user callback if defined
        if (g_w25q_handle->config.RxCpltCallback != NULL)
        {
            g_w25q_handle->config.RxCpltCallback(g_w25q_handle);
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (g_w25q_handle != NULL && g_w25q_handle->config.spi_handle == hspi)
    {
        g_w25q_handle->dma_error = 1;
        g_w25q_handle->state = W25Q_STATE_READY;
        
        // Deassert CS pin
        W25Q_CS_Disable(g_w25q_handle);
        
        // // Call error callback if defined
        // if (g_w25q_handle->config.ErrorCallback != NULL)
        // {
        //     g_w25q_handle->config.ErrorCallback(g_w25q_handle);
        // }
    }
}

W25Q_StatusTypeDef W25Q_WaitForDMATransferComplete(W25Q_HandleTypeDef *hflash, uint32_t Timeout)
{
    if (hflash == NULL)
    {
        return W25Q_PARAM_ERR;
    }
    uint32_t tickstart = HAL_GetTick();
    
    while (hflash->state != W25Q_STATE_READY)
    {
        if ((HAL_GetTick() - tickstart) > Timeout)
        {
            return W25Q_TIMEOUT;
        }
    }
    
    if (hflash->dma_error)
    {
        return W25Q_ERROR;
    }
    
    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_Read_DMA(W25Q_HandleTypeDef *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size)
{
    if (hflash == NULL || pBuffer == NULL || Size == 0 || hflash->config.hdmarx == NULL)
    {
        return W25Q_PARAM_ERR;
    }

    uint8_t tx_header[5];              // Command + 3-byte address + 1 dummy byte for Fast Read
    tx_header[0] = W25Q_CMD_FAST_READ; // Using Fast Read for better performance
    tx_header[1] = (Address >> 16) & 0xFF;
    tx_header[2] = (Address >> 8) & 0xFF;
    tx_header[3] = Address & 0xFF;
    tx_header[4] = W25Q_DATA_DUMMY; // Dummy byte

    // Set state to busy
    hflash->state = W25Q_STATE_BUSY_RX;
    hflash->rx_complete = 0;
    hflash->dma_error = 0;

    // Enable chip select
    W25Q_CS_Enable(hflash);
    
    // Send command and address (blocking)
    W25Q_StatusTypeDef status = W25Q_SPI_Transmit(hflash, tx_header, 5);
    if (status != W25Q_OK)
    {
        W25Q_CS_Disable(hflash);
        hflash->state = W25Q_STATE_READY;
        return status;
    }
    
    // Start DMA receive
    if (HAL_SPI_Receive_DMA(hflash->config.spi_handle, pBuffer, Size) != HAL_OK)
    {
        W25Q_CS_Disable(hflash);
        hflash->state = W25Q_STATE_READY;
        return W25Q_ERROR;
    }
    
    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_WritePage_DMA(W25Q_HandleTypeDef *hflash, uint32_t Address, const uint8_t *pBuffer, uint32_t Size)
{
    if (hflash == NULL || pBuffer == NULL || Size == 0 || Size > hflash->config.page_size || 
        (Address % hflash->config.page_size) != 0 || hflash->config.hdmatx == NULL)
    {
        return W25Q_PARAM_ERR;
    }

    // 1. Write Enable (blocking)
    W25Q_StatusTypeDef status = W25Q_WriteEnable(hflash);
    if (status != W25Q_OK)
    {
        return status;
    }

    uint8_t tx_header[4]; // Command + 3-byte address
    tx_header[0] = W25Q_CMD_PAGE_PROGRAM;
    tx_header[1] = (Address >> 16) & 0xFF;
    tx_header[2] = (Address >> 8) & 0xFF;
    tx_header[3] = Address & 0xFF;

    // Set state to busy
    hflash->state = W25Q_STATE_BUSY_TX;
    hflash->tx_complete = 0;
    hflash->dma_error = 0;

    // Enable chip select
    W25Q_CS_Enable(hflash);
    
    // Send command and address (blocking)
    status = W25Q_SPI_Transmit(hflash, tx_header, 4);
    if (status != W25Q_OK)
    {
        W25Q_CS_Disable(hflash);
        hflash->state = W25Q_STATE_READY;
        return status;
    }
    
    // Start DMA transmit for data
    if (HAL_SPI_Transmit_DMA(hflash->config.spi_handle, (uint8_t*)pBuffer, Size) != HAL_OK)
    {
        W25Q_CS_Disable(hflash);
        hflash->state = W25Q_STATE_READY;
        return W25Q_ERROR;
    }
    
    return W25Q_OK;
}