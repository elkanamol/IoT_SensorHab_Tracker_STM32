#include "w25qxx_hal.h"
#include <string.h> // For memcpy

/* --- Global Variables --- */
static W25Q_HandleTypeDef *g_w25q_handle = NULL;

/* --- Private Functions Prototypes --- */
/* Core/Base private functions */
static void W25Q_CS_Enable(W25Q_HandleTypeDef *hflash);
static void W25Q_CS_Disable(W25Q_HandleTypeDef *hflash);
static W25Q_StatusTypeDef W25Q_SPI_Transmit(W25Q_HandleTypeDef *hflash, uint8_t *pData, uint16_t Size);
static W25Q_StatusTypeDef W25Q_SPI_Receive(W25Q_HandleTypeDef *hflash, uint8_t *pData, uint16_t Size);
static W25Q_StatusTypeDef W25Q_SPI_TransmitReceive(W25Q_HandleTypeDef *hflash, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
static W25Q_StatusTypeDef W25Q_WriteEnable(W25Q_HandleTypeDef *hflash);
static W25Q_StatusTypeDef W25Q_WaitForReady(W25Q_HandleTypeDef *hflash, uint32_t Timeout);
static W25Q_StatusTypeDef W25Q_SendCmdAddress(W25Q_HandleTypeDef *hflash, uint8_t cmd, uint32_t address);
static W25Q_StatusTypeDef W25Q_SendCmdAddressData(W25Q_HandleTypeDef *hflash, uint8_t cmd, uint32_t address, const uint8_t *pData, uint32_t size);

#ifdef W25QXX_USE_DMA
/* DMA-specific private functions */
static W25Q_StatusTypeDef W25Q_SPI_Transmit_DMA(W25Q_HandleTypeDef *hflash, uint8_t *pData, uint16_t Size);
static void W25Q_TxCpltCallback(W25Q_HandleTypeDef *hflash);
static void W25Q_RxCpltCallback(W25Q_HandleTypeDef *hflash);
static void W25Q_TxRxCpltCallback(W25Q_HandleTypeDef *hflash);
static void W25Q_DMAErrorCallback(W25Q_HandleTypeDef *hflash);
#endif

#ifdef W25QXX_USE_RTOS
static void W25Q_Task(void *argument);
#endif
// --- Private Functions Implementation ---

/* === CORE/BASE FUNCTIONS IMPLEMENTATION === */

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
 * @param pData Pointer to the buffer containing data to transmit
 * @param Size Number of bytes to transmit
 * @return W25Q_StatusTypeDef Status of the SPI transmission operation
 *         - W25Q_OK: Transmission successful
 *         - W25Q_PARAM_ERR: Invalid input parameters
 *         - W25Q_ERROR: Transmission failed
 */
static W25Q_StatusTypeDef W25Q_SPI_Transmit(W25Q_HandleTypeDef *hflash, uint8_t *pData, uint16_t Size)
{
    // Base implementation
    if (hflash == NULL || pData == NULL || Size == 0)
    {
        return W25Q_PARAM_ERR;
    }
    if (HAL_SPI_Transmit(hflash->config.spi_handle, pData, Size, W25Q_COM_TIMEOUT_MS) == HAL_OK)
    {
        return W25Q_OK;
    }
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

/* === PUBLIC CORE/BASE API IMPLEMENTATION === */

/**
 * @brief Initialize a W25Q Flash memory handle with the provided configuration
 *
 * This function sets up a W25Q Flash memory handle by validating input parameters,
 * copying configuration, setting default chip parameters, and initializing
 * optional DMA and RTOS-specific resources.
 *
 * @param hflash Pointer to the W25Q Flash memory handle to be initialized
 * @param pFlashConfig Pointer to the configuration parameters for the Flash memory
 *
 * @return W25Q_StatusTypeDef Status of the initialization process
 *         - W25Q_OK: Successful initialization
 *         - W25Q_PARAM_ERR: Invalid input parameters
 *         - W25Q_ERROR: Resource allocation failure
 */
W25Q_StatusTypeDef W25Q_Init(W25Q_HandleTypeDef *hflash, const W25Q_ConfigTypeDef *pFlashConfig)
{
    // Base implementation with conditional sections
    if (hflash == NULL || pFlashConfig == NULL || pFlashConfig->spi_handle == NULL ||
        pFlashConfig->cs_port == NULL || pFlashConfig->cs_pin == 0)
    {
        return W25Q_PARAM_ERR;
    }

    // Copy configuration to the handle
    memcpy(&hflash->config, pFlashConfig, sizeof(W25Q_ConfigTypeDef));

    // Set default chip parameters if not explicitly provided
    if (hflash->config.page_size == 0)
        hflash->config.page_size = W25Q_DEFAULT_PAGE_SIZE;
    if (hflash->config.sector_size == 0)
        hflash->config.sector_size = W25Q_DEFAULT_SECTOR_SIZE;
    if (hflash->config.block_32k_size == 0)
        hflash->config.block_32k_size = W25Q_DEFAULT_BLOCK32_SIZE;
    if (hflash->config.block_64k_size == 0)
        hflash->config.block_64k_size = W25Q_DEFAULT_BLOCK64_SIZE;

    W25Q_CS_Disable(hflash);

    // Store the global handle for callbacks
    g_w25q_handle = hflash;

#ifdef W25QXX_USE_DMA
    // Initialize DMA state - THIS SECTION IS CONDITIONAL
    hflash->state = W25Q_STATE_READY;
    hflash->tx_complete = 0;
    hflash->rx_complete = 0;
    hflash->dma_error = 0;
#endif

#ifdef W25QXX_USE_RTOS
    // Initialize FreeRTOS synchronization primitives - THIS SECTION IS CONDITIONAL
    if (hflash->config.use_rtos)
    {
        hflash->mutex = xSemaphoreCreateMutex();
        hflash->tx_semaphore = xSemaphoreCreateBinary();
        hflash->rx_semaphore = xSemaphoreCreateBinary();
        hflash->txrx_semaphore = xSemaphoreCreateBinary();

        if (hflash->mutex == NULL || hflash->tx_semaphore == NULL || hflash->rx_semaphore == NULL || hflash->txrx_semaphore == NULL || hflash->cmd_queue == NULL)
        {
            W25Q_DeInit(hflash);
            return W25Q_ERROR;
        }

        hflash->cmd_queue = xQueueCreate(10, sizeof(W25Q_QueueItem));
        if (hflash->cmd_queue == NULL)
        {
            W25Q_DeInit(hflash);
            return W25Q_ERROR;
        }
    }
#endif

    // IMPORTANT: Read JEDEC ID during initialization (always using direct SPI)
    uint8_t mfg_id_read;
    uint16_t dev_id_read;
    W25Q_StatusTypeDef status = W25Q_ReadJEDECID(hflash, &mfg_id_read, &dev_id_read);

    if (status != W25Q_OK)
    {
        W25Q_DeInit(hflash);
        return W25Q_ERROR;
    }

    // Parse JEDEC ID from response
    hflash->mfg_id = mfg_id_read;
    hflash->dev_id = dev_id_read;

    // Check for expected Manufacturer ID
    if (hflash->mfg_id != W25Q64BV_MFG_ID)
    {
        W25Q_DeInit(hflash);
        return W25Q_ERROR;
    }

#ifdef W25QXX_USE_RTOS
    // Start the FreeRTOS task - THIS SECTION IS CONDITIONAL
    if (hflash->config.use_rtos)
    {
        W25Q_StartTask(hflash);
    }
#endif

    return W25Q_OK;
}

/**
 * @brief Deinitialize the W25Q Flash memory device
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 * @return W25Q_StatusTypeDef Status of the deinitialization operation
 *
 * This function performs cleanup and reset operations for the W25Q Flash memory device,
 * including releasing RTOS resources if applicable and resetting device configuration.
 */
W25Q_StatusTypeDef W25Q_DeInit(W25Q_HandleTypeDef *hflash)
{
    if (hflash == NULL)
    {
        return W25Q_PARAM_ERR;
    }
#ifdef W25QXX_USE_RTOS
    // Stop the FreeRTOS task if it's running
    if (hflash->config.use_rtos) {
        W25Q_StopTask(hflash);
        
        // Delete FreeRTOS synchronization primitives
        if (hflash->mutex != NULL) {
            vSemaphoreDelete(hflash->mutex);
            hflash->mutex = NULL;
        }
        if (hflash->tx_semaphore != NULL) {
            vSemaphoreDelete(hflash->tx_semaphore);
            hflash->tx_semaphore = NULL;
        }
        if (hflash->rx_semaphore != NULL) {
            vSemaphoreDelete(hflash->rx_semaphore);
            hflash->rx_semaphore = NULL;
        }
        if (hflash->txrx_semaphore != NULL) {
            vSemaphoreDelete(hflash->txrx_semaphore);
            hflash->txrx_semaphore = NULL;
        }
        if (hflash->cmd_queue != NULL) {
            vQueueDelete(hflash->cmd_queue);
            hflash->cmd_queue = NULL;
        }
    }
#endif
    W25Q_CS_Disable(hflash); // Ensure CS is inactive
    
    g_w25q_handle = NULL;

    hflash->config.spi_handle = NULL; // Reset SPI handle
    hflash->config.cs_port = NULL;     // Reset CS port
    hflash->config.cs_pin = 0;         // Reset CS pin

    return W25Q_OK;
}

/**
 * @brief Reads the JEDEC ID of the W25Q Flash memory device.
 *
 * This function retrieves the manufacturer ID and device ID from the W25Q Flash memory
 * using the JEDEC ID command. It supports both polling and RTOS-based communication modes.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param pManufacturerID Pointer to store the manufacturer ID
 * @param pDeviceID Pointer to store the device ID
 *
 * @return W25Q_StatusTypeDef Status of the JEDEC ID read operation
 *         - W25Q_OK: Successfully read JEDEC ID
 *         - W25Q_PARAM_ERR: Invalid input parameters
 *         - Other error codes may be returned based on communication status
 */
W25Q_StatusTypeDef W25Q_ReadJEDECID(W25Q_HandleTypeDef *hflash, uint8_t *pManufacturerID, uint16_t *pDeviceID)
{
    if (hflash == NULL || pManufacturerID == NULL || pDeviceID == NULL)
    {
        return W25Q_PARAM_ERR;
    }

#ifdef W25QXX_USE_RTOS
    // If RTOS is enabled, task is created, and we're not in an ISR, use the thread-safe version
    if (hflash->config.use_rtos && hflash->task_handle != NULL && !xPortIsInsideInterrupt()) {
        return W25Q_ReadJEDECID_RTOS(hflash, pManufacturerID, pDeviceID);
    }
#endif

    // Regular polling implementation
    uint8_t tx_buffer[4] = {W25Q_CMD_JEDEC_ID, W25Q_DATA_DUMMY, W25Q_DATA_DUMMY, W25Q_DATA_DUMMY};
    uint8_t rx_buffer[4];

    W25Q_CS_Enable(hflash);
    W25Q_StatusTypeDef status = W25Q_SPI_TransmitReceive(hflash, tx_buffer, rx_buffer, 4);
    W25Q_CS_Disable(hflash);

    if (status == W25Q_OK)
    {
        *pManufacturerID = rx_buffer[1];
        *pDeviceID = (uint16_t)(rx_buffer[2] << 8) | rx_buffer[3];
    }
    return status;
}

/**
 * @brief Reads the first status register of the W25Q Flash memory device.
 *
 * This function retrieves the contents of the first status register from the W25Q Flash memory
 * using the appropriate read status register command.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param pStatusReg1 Pointer to store the value of the first status register
 *
 * @return W25Q_StatusTypeDef Status of the status register read operation
 *         - W25Q_OK: Successfully read status register
 *         - W25Q_PARAM_ERR: Invalid input parameters
 *         - Other error codes may be returned based on communication status
 */
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

/**
 * @brief Reads the second status register of the W25Q Flash memory device.
 *
 * This function retrieves the contents of the second status register from the W25Q Flash memory
 * using the appropriate read status register command.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param pStatusReg2 Pointer to store the value of the second status register
 *
 * @return W25Q_StatusTypeDef Status of the status register read operation
 *         - W25Q_OK: Successfully read status register
 *         - W25Q_PARAM_ERR: Invalid input parameters
 *         - Other error codes may be returned based on communication status
 */
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

/**
 * @brief Calculates the timeout duration for various W25Qxx flash memory operations.
 *
 * This function determines the maximum expected time for different flash memory operations
 * based on the device configuration and operation type. The timeout values are derived from
 * typical W25Qxx datasheet specifications.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param operation_type Type of flash memory operation (e.g., page program, sector erase)
 * @param data_size Size of data involved in the operation
 *
 * @return uint32_t Timeout duration in milliseconds
 *         - Returns 0 for invalid handle or unknown operation type
 *         - Timeout values are maximum times from datasheets
 *
 * @note Timeout values are conservative estimates and may vary by specific device model
 */
uint32_t W25Q_CalculateTimeout(const W25Q_HandleTypeDef *hflash, uint8_t operation_type, uint32_t data_size)
{
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

/**
 * @brief Reads data from the W25Q Flash memory device.
 *
 * This function reads a specified number of bytes from a given address in the flash memory.
 * It supports multiple read modes including standard polling, DMA, and RTOS-based transfers.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param Address Starting memory address to read from
 * @param pBuffer Pointer to the buffer where read data will be stored
 * @param Size Number of bytes to read
 *
 * @return W25Q_StatusTypeDef
 *         - W25Q_OK if read is successful
 *         - W25Q_PARAM_ERR for invalid parameters
 *         - Error status from underlying SPI or DMA transfer
 *
 * @note Supports optional DMA and RTOS transfer modes based on configuration
 */
W25Q_StatusTypeDef W25Q_Read(W25Q_HandleTypeDef *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size)
{
    if (hflash == NULL || pBuffer == NULL || Size == 0)
    {
        return W25Q_PARAM_ERR;
    }

#ifdef W25QXX_USE_RTOS
    // RTOS-specific routing
    if (hflash->config.use_rtos && !xPortIsInsideInterrupt()) {
        return W25Q_Read_RTOS(hflash, Address, pBuffer, Size);
    }
#endif

#ifdef W25QXX_USE_DMA
    // DMA-specific routing
    if (hflash->config.use_dma && hflash->config.hdmarx != NULL && Size > 16)
    {
        return W25Q_Read_DMA(hflash, Address, pBuffer, Size);
    }
#endif

    // Regular polling implementation - ALWAYS AVAILABLE
    uint8_t tx_header[5];
    tx_header[0] = W25Q_CMD_FAST_READ;
    tx_header[1] = (Address >> 16) & 0xFF;
    tx_header[2] = (Address >> 8) & 0xFF;
    tx_header[3] = Address & 0xFF;
    tx_header[4] = W25Q_DATA_DUMMY;

    W25Q_CS_Enable(hflash);
    W25Q_StatusTypeDef status = W25Q_SPI_Transmit(hflash, tx_header, 5);
    if (status == W25Q_OK)
    {
        status = W25Q_SPI_Receive(hflash, pBuffer, Size);
    }
    W25Q_CS_Disable(hflash);
    return status;
}

/**
 * @brief Write a page of data to a W25QXX flash memory device
 *
 * @param hflash Pointer to the flash device handle
 * @param Address Starting memory address to write to
 * @param pBuffer Pointer to the data buffer to be written
 * @param Size Number of bytes to write (must not exceed page size)
 *
 * @return W25Q_StatusTypeDef
 *         - W25Q_OK if write is successful
 *         - W25Q_PARAM_ERR for invalid parameters
 *         - Error status from underlying SPI or DMA transfer
 *
 * @note Supports optional DMA and RTOS transfer modes based on configuration
 * @note Writes are limited to a single page and must be page-aligned
 */
W25Q_StatusTypeDef W25Q_WritePage(W25Q_HandleTypeDef *hflash, uint32_t Address, const uint8_t *pBuffer, uint32_t Size)
{
    if (hflash == NULL || pBuffer == NULL || Size == 0 || Size > hflash->config.page_size /*|| (Address % hflash->config.page_size) != 0*/)
    {
        return W25Q_PARAM_ERR;
    }

#ifdef W25QXX_USE_RTOS
    // If RTOS is enabled and we're not in an ISR, use the thread-safe version
    if (hflash->config.use_rtos && !xPortIsInsideInterrupt()) {
        return W25Q_WritePage_RTOS(hflash, Address, pBuffer, Size);
    }
#endif

#ifdef W25QXX_USE_DMA
    // If DMA is configured and buffer is suitable, use DMA version
    if (hflash->config.use_dma && hflash->config.hdmatx != NULL && Size > 16)
    {
        W25Q_StatusTypeDef status = W25Q_WriteEnable(hflash);
        if (status != W25Q_OK)
        {
            return status;
        }

        status = W25Q_WritePage_DMA(hflash, Address, pBuffer, Size);
        if (status != W25Q_OK)
        {
            return status;
        }

        uint32_t timeout_ms = W25Q_CalculateTimeout(hflash, W25Q_CMD_PAGE_PROGRAM, Size);
        return W25Q_WaitForDMATransferComplete(hflash, timeout_ms);
    }
#endif

    // Regular polling implementation
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

    return W25Q_OK;
}

/**
 * @brief Erases a single 4KB sector of the W25QXX flash memory.
 *
 * @param hflash Pointer to the W25Q flash device handle
 * @param SectorAddress Base address of the sector to be erased (must be sector-aligned)
 * @return W25Q_StatusTypeDef Status of the erase operation (W25Q_OK if successful)
 *
 * @note The sector address must be aligned to the sector size of the flash device.
 * @note This function supports both polling and DMA modes depending on configuration.
 */
W25Q_StatusTypeDef W25Q_EraseSector(W25Q_HandleTypeDef *hflash, uint32_t SectorAddress)
{
    // Address must be aligned to sector boundary
    if (hflash == NULL || (SectorAddress % hflash->config.sector_size) != 0)
    {
        return W25Q_PARAM_ERR;
    }

#ifdef W25QXX_USE_RTOS
    // If RTOS is enabled and we're not in an ISR, use the thread-safe version
    if (hflash->config.use_rtos && !xPortIsInsideInterrupt()) {
        return W25Q_EraseSector_RTOS(hflash, SectorAddress);
    }
#endif

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

// DMA spesific functions
#ifdef W25QXX_USE_DMA
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
#ifdef W25QXX_USE_DMA
    if (g_w25q_handle != NULL && g_w25q_handle->config.spi_handle == hspi)
    {
        W25Q_TxCpltCallback(g_w25q_handle);
    }
#endif
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
#ifdef W25QXX_USE_DMA
    if (g_w25q_handle != NULL && g_w25q_handle->config.spi_handle == hspi)
    {
        W25Q_RxCpltCallback(g_w25q_handle);
    }
#endif
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
#ifdef W25QXX_USE_DMA
    if (g_w25q_handle != NULL && g_w25q_handle->config.spi_handle == hspi)
    {
        W25Q_TxRxCpltCallback(g_w25q_handle);
    }
#endif
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
#ifdef W25QXX_USE_DMA
    if (g_w25q_handle != NULL && g_w25q_handle->config.spi_handle == hspi)
    {
        W25Q_DMAErrorCallback(g_w25q_handle);
    }
#endif
}

/**
 * @brief Wait for a DMA transfer to complete for the W25Q flash device.
 *
 * @param hflash Pointer to the W25Q flash device handle
 * @param Timeout Maximum time to wait for transfer completion in milliseconds
 * @return W25Q_StatusTypeDef Status of the wait operation
 *         - W25Q_OK if transfer completed successfully
 *         - W25Q_TIMEOUT if transfer did not complete within specified time
 *         - W25Q_ERROR if a DMA error occurred
 *         - W25Q_PARAM_ERR if invalid handle is provided
 */
W25Q_StatusTypeDef W25Q_WaitForDMATransferComplete(W25Q_HandleTypeDef *hflash, uint32_t Timeout)
{
#ifdef W25QXX_USE_DMA
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

    return W25Q_OK
#else
    return W25Q_ERROR;
#endif
}

/**
 * @brief Reads data from the W25Q flash memory using DMA transfer.
 *
 * @param hflash Pointer to the W25Q flash device handle
 * @param Address Starting memory address to read from
 * @param pBuffer Pointer to the buffer where read data will be stored
 * @param Size Number of bytes to read
 * @return W25Q_StatusTypeDef Status of the read operation
 *         - W25Q_OK if read was successful
 *         - W25Q_PARAM_ERR if invalid parameters are provided
 *         - W25Q_ERROR if DMA transfer failed
 * @note This function uses DMA for efficient data transfer and requires W25QXX_USE_DMA to be defined
 */
W25Q_StatusTypeDef W25Q_Read_DMA(W25Q_HandleTypeDef *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size)
{
#ifdef W25QXX_USE_DMA
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
#else
    return W25Q_ERROR;
#endif
}

/**
 * @brief Writes a page of data to the W25QXX flash memory using DMA.
 *
 * @param hflash Pointer to the W25Q flash handle
 * @param Address Memory address to write the page
 * @param pBuffer Pointer to the data buffer to be written
 * @param Size Number of bytes to write (must not exceed page size)
 *
 * @return W25Q_StatusTypeDef
 *         - W25Q_OK if write operation is successful
 *         - W25Q_PARAM_ERR if invalid parameters are provided
 *         - W25Q_ERROR if DMA transfer failed
 *
 * @note This function uses DMA for efficient page programming and requires W25QXX_USE_DMA to be defined
 */
W25Q_StatusTypeDef W25Q_WritePage_DMA(W25Q_HandleTypeDef *hflash, uint32_t Address, const uint8_t *pBuffer, uint32_t Size)
{
#ifdef W25QXX_USE_DMA
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
#else
    return W25Q_ERROR;
#endif
}

/**
 * @brief Transmits data over SPI using DMA for the W25Q Flash memory device
 * @param hflash Pointer to the W25Q Flash memory handle
 * @param pData Pointer to the buffer containing data to transmit
 * @param Size Number of bytes to transmit
 * @return W25Q_StatusTypeDef Status of the DMA SPI transmission operation
 *         - W25Q_OK: DMA Transmission successful
 *         - W25Q_PARAM_ERR: Invalid input parameters
 *         - W25Q_ERROR: DMA Transmission failed
 * @note This function is only available when W25QXX_USE_DMA is defined
 */
static W25Q_StatusTypeDef W25Q_SPI_Transmit_DMA(W25Q_HandleTypeDef *hflash, uint8_t *pData, uint16_t Size)
{
#ifdef W25QXX_USE_DMA
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
#else
    // If DMA is not enabled, this function should not be called
    (void)hflash;
    (void)pData;
    (void)Size;
    return W25Q_ERROR;
#endif
}

/**
 * @brief Callback function for transmit (TX) DMA completion
 *
 * This function is called when a DMA transmission to the W25Q Flash memory is complete.
 * It updates the flash handle state, disables the chip select, and optionally signals
 * completion via an RTOS semaphore or invokes a user-defined callback.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 */
static void W25Q_TxCpltCallback(W25Q_HandleTypeDef *hflash)
{
#ifdef W25QXX_USE_DMA
    // DMA state variables are available
    hflash->tx_complete = 1;
    hflash->state = W25Q_STATE_READY;

    // Deassert CS pin
    W25Q_CS_Disable(hflash);

#ifdef W25QXX_USE_RTOS
    // RTOS-specific code - add this guard
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Signal completion via semaphore if in RTOS mode
    if (hflash->config.use_rtos && hflash->tx_semaphore != NULL)
    {
        xSemaphoreGiveFromISR(hflash->tx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
#endif

    // Call user callback if defined
    if (hflash->config.TxCpltCallback != NULL)
    {
        hflash->config.TxCpltCallback(hflash);
    }
#endif
}

/**
 * @brief Callback function for receive (RX) DMA completion
 *
 * This function is called when a DMA receive operation to the W25Q Flash memory is complete.
 * It updates the flash handle state, disables the chip select, and optionally signals
 * completion via an RTOS semaphore or invokes a user-defined callback.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 */
static void W25Q_RxCpltCallback(W25Q_HandleTypeDef *hflash)
{
#ifdef W25QXX_USE_DMA
    // DMA state variables are available
    hflash->tx_complete = 1;
    hflash->state = W25Q_STATE_READY;

    // Deassert CS pin
    W25Q_CS_Disable(hflash);

#ifdef W25QXX_USE_RTOS
    // RTOS-specific code - add this guard
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Signal completion via semaphore if in RTOS mode
    if (hflash->config.use_rtos && hflash->tx_semaphore != NULL)
    {
        xSemaphoreGiveFromISR(hflash->tx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
#endif

    // Call user callback if defined
    if (hflash->config.TxCpltCallback != NULL)
    {
        hflash->config.TxCpltCallback(hflash);
    }
#endif
}

/**
 * @brief Callback function for combined TX/RX DMA completion
 *
 * This function is called when a DMA transmit and receive operation to the W25Q Flash memory is complete.
 * It updates the flash handle state, disables the chip select, and optionally signals
 * completion via an RTOS semaphore or invokes a user-defined callback.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 */
static void W25Q_TxRxCpltCallback(W25Q_HandleTypeDef *hflash)
{
#ifdef W25QXX_USE_DMA
    // DMA state variables are available
    hflash->tx_complete = 1;
    hflash->state = W25Q_STATE_READY;

    // Deassert CS pin
    W25Q_CS_Disable(hflash);

#ifdef W25QXX_USE_RTOS
    // RTOS-specific code - add this guard
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Signal completion via semaphore if in RTOS mode
    if (hflash->config.use_rtos && hflash->tx_semaphore != NULL)
    {
        xSemaphoreGiveFromISR(hflash->tx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
#endif

    // Call user callback if defined
    if (hflash->config.TxCpltCallback != NULL)
    {
        hflash->config.TxCpltCallback(hflash);
    }
#endif
}

/**
 * @brief Callback function for DMA error handling
 *
 * This function is called when a DMA error occurs during SPI communication with the W25Q Flash memory.
 * It sets the error flag, resets the flash handle state, disables the chip select, and
 * optionally signals semaphores in an RTOS environment to unblock waiting tasks.
 *
 * @param hflash Pointer to the W25Q Flash memory handle
 */
static void W25Q_DMAErrorCallback(W25Q_HandleTypeDef *hflash)
{
#ifdef W25QXX_USE_DMA
#ifdef W25QXX_USE_RTOS
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif

    hflash->dma_error = 1;
    hflash->state = W25Q_STATE_READY;

    // Deassert CS pin
    W25Q_CS_Disable(hflash);

#ifdef W25QXX_USE_RTOS
    // Signal all semaphores to unblock any waiting tasks
    if (hflash->config.use_rtos)
    {
        if (hflash->tx_semaphore != NULL)
        {
            xSemaphoreGiveFromISR(hflash->tx_semaphore, &xHigherPriorityTaskWoken);
        }
        if (hflash->rx_semaphore != NULL)
        {
            xSemaphoreGiveFromISR(hflash->rx_semaphore, &xHigherPriorityTaskWoken);
        }
        if (hflash->txrx_semaphore != NULL)
        {
            xSemaphoreGiveFromISR(hflash->txrx_semaphore, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
#endif
#endif // W25QXX_USE_RTOS
}
#endif // W25QXX_USE_DMA

// FreeRTOS specific functions
#ifdef W25QXX_USE_RTOS

/**
 * @brief Starts a FreeRTOS task for the W25Q flash device.
 *
 * @param hflash Pointer to the W25Q flash device handle
 * @return W25Q_StatusTypeDef Status of task creation
 *         - W25Q_OK: Task created successfully
 *         - W25Q_PARAM_ERR: Invalid handle or RTOS not configured
 *         - W25Q_ERROR: Failed to create task
 */
W25Q_StatusTypeDef W25Q_StartTask(W25Q_HandleTypeDef *hflash)
{
    if (hflash == NULL || !hflash->config.use_rtos) {
        return W25Q_PARAM_ERR;
    }
    
    // Create the task if it doesn't exist
    if (hflash->task_handle == NULL) {
        BaseType_t result = xTaskCreate(
            W25Q_Task,                  // Task function
            "W25Q_Task",                // Task name
            configMINIMAL_STACK_SIZE*2, // Stack size (adjust as needed)
            (void*)hflash,              // Parameter passed to the task
            tskIDLE_PRIORITY + 1,       // Priority (adjust as needed)
            &hflash->task_handle        // Task handle
        );
        
        if (result != pdPASS) {
            return W25Q_ERROR;
        }
    }
    
    return W25Q_OK;
}

/**
 * @brief Stops the FreeRTOS task for the W25Q flash device.
 *
 * @param hflash Pointer to the W25Q flash device handle
 * @return W25Q_StatusTypeDef Status of task stopping
 *         - W25Q_OK: Task stopped successfully
 *         - W25Q_PARAM_ERR: Invalid handle or RTOS not configured
 *         - W25Q_ERROR: Failed to stop task
 */
W25Q_StatusTypeDef W25Q_StopTask(W25Q_HandleTypeDef *hflash)
{
    if (hflash == NULL || !hflash->config.use_rtos) {
        return W25Q_PARAM_ERR;
    }
    
    // Delete the task if it exists
    if (hflash->task_handle != NULL) {
        vTaskDelete(hflash->task_handle);
        hflash->task_handle = NULL;
    }
    
    return W25Q_OK;
}

/**
 * @brief Reads data from the W25Q flash memory using FreeRTOS.
 *
 * @param hflash Pointer to the W25Q flash device handle
 * @param Address Starting memory address to read from
 * @param pBuffer Pointer to the buffer where read data will be stored
 * @param Size Number of bytes to read
 * @return W25Q_StatusTypeDef Status of the read operation
 *         - W25Q_OK if read was successful
 *         - W25Q_PARAM_ERR if invalid parameters are provided
 *         - W25Q_ERROR if DMA transfer failed
 */
W25Q_StatusTypeDef W25Q_Read_RTOS(W25Q_HandleTypeDef *hflash, uint32_t Address, uint8_t *pBuffer, uint32_t Size)
{
    if (hflash == NULL || pBuffer == NULL || Size == 0 || !hflash->config.use_rtos) {
        return W25Q_PARAM_ERR;
    }
    
    W25Q_QueueItem item;
    W25Q_StatusTypeDef status = W25Q_OK;
    SemaphoreHandle_t completion_semaphore;
    
    // Create a semaphore for this operation
    completion_semaphore = xSemaphoreCreateBinary();
    if (completion_semaphore == NULL) {
        return W25Q_ERROR;
    }
    
    // Prepare the command
    item.cmd_type = W25Q_CMD_READ;
    item.address = Address;
    item.buffer = pBuffer;
    item.size = Size;
    item.status = &status;
    item.completion_semaphore = completion_semaphore;
    
    // Send the command to the queue
    if (xQueueSend(hflash->cmd_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        vSemaphoreDelete(completion_semaphore);
        return W25Q_BUSY;
    }
    
    // Wait for the operation to complete
    if (xSemaphoreTake(completion_semaphore, pdMS_TO_TICKS(W25Q_MAX_READY_TIMEOUT_MS)) != pdTRUE) {
        status = W25Q_TIMEOUT;
    }
    
    // Clean up
    vSemaphoreDelete(completion_semaphore);
    
    return status;
}

/**
 * @brief Writes a page of data to the W25QXX flash memory using FreeRTOS.
 *
 * @param hflash Pointer to the W25Q flash handle
 * @param Address Memory address to write the page
 * @param pBuffer Pointer to the data buffer to be written
 * @param Size Number of bytes to write (must not exceed page size)
 * @return W25Q_StatusTypeDef Status of the write operation
 *         - W25Q_OK if write was successful
 *         - W25Q_PARAM_ERR if invalid parameters are provided
 *         - W25Q_ERROR if DMA transfer failed
 *         - W25Q_TIMEOUT if operation timed out
 *         - W25Q_BUSY if device is busy
 */
W25Q_StatusTypeDef W25Q_WritePage_RTOS(W25Q_HandleTypeDef *hflash, uint32_t Address, const uint8_t *pBuffer, uint32_t Size)
{
    if (hflash == NULL || pBuffer == NULL || Size == 0 || Size > hflash->config.page_size || 
        (Address % hflash->config.page_size) != 0 || !hflash->config.use_rtos) {
        return W25Q_PARAM_ERR;
    }
    
    W25Q_QueueItem item;
    W25Q_StatusTypeDef status = W25Q_OK;
    SemaphoreHandle_t completion_semaphore;
    
    // Create a semaphore for this operation
    completion_semaphore = xSemaphoreCreateBinary();
    if (completion_semaphore == NULL) {
        return W25Q_ERROR;
    }
    
    // Prepare the command
    item.cmd_type = W25Q_CMD_WRITE_PAGE;
    item.address = Address;
    item.buffer = (uint8_t*)pBuffer;  // Cast away const for the queue item
    item.size = Size;
    item.status = &status;
    item.completion_semaphore = completion_semaphore;
    
    // Send the command to the queue
    if (xQueueSend(hflash->cmd_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        vSemaphoreDelete(completion_semaphore);
        return W25Q_BUSY;
    }
    
    // Wait for the operation to complete with appropriate timeout
    uint32_t timeout = W25Q_CalculateTimeout(hflash, W25Q_CMD_PAGE_PROGRAM, Size);
    if (xSemaphoreTake(completion_semaphore, pdMS_TO_TICKS(timeout + 100)) != pdTRUE) {
        status = W25Q_TIMEOUT;
    }
    
    // Clean up
    vSemaphoreDelete(completion_semaphore);
    
    return status;
}

/**
 * @brief Erase a sector of the W25Q Flash memory using RTOS
 *
 * @param hflash Pointer to the W25Q flash handle
 * @param SectorAddress Address of the sector to be erased (must be sector-aligned)
 * @return W25Q_StatusTypeDef
 *         - W25Q_OK if sector erase was successful
 *         - W25Q_PARAM_ERR if invalid parameters are provided
 *         - W25Q_ERROR if operation failed
 *         - W25Q_TIMEOUT if operation timed out
 *         - W25Q_BUSY if device is busy
 */
W25Q_StatusTypeDef W25Q_EraseSector_RTOS(W25Q_HandleTypeDef *hflash, uint32_t SectorAddress)
{
    if (hflash == NULL || (SectorAddress % hflash->config.sector_size) != 0 || !hflash->config.use_rtos) {
        return W25Q_PARAM_ERR;
    }
    
    W25Q_QueueItem item;
    W25Q_StatusTypeDef status = W25Q_OK;
    SemaphoreHandle_t completion_semaphore;
    
    // Create a semaphore for this operation
    completion_semaphore = xSemaphoreCreateBinary();
    if (completion_semaphore == NULL) {
        return W25Q_ERROR;
    }
    
    // Prepare the command
    item.cmd_type = W25Q_CMD_ERASE_SECTOR;
    item.address = SectorAddress;
    item.buffer = NULL;
    item.size = 0;
    item.status = &status;
    item.completion_semaphore = completion_semaphore;
    
    // Send the command to the queue
    if (xQueueSend(hflash->cmd_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        vSemaphoreDelete(completion_semaphore);
        return W25Q_BUSY;
    }
    
    // Wait for the operation to complete with appropriate timeout
    // uint32_t timeout = W25Q_CalculateTimeout(hflash, W25Q_CMD_SECTOR_ERASE_4KB, 0);
    if (xSemaphoreTake(completion_semaphore, pdMS_TO_TICKS(10000)) != pdTRUE) {
        status = W25Q_TIMEOUT;
    }
    
    // Clean up
    vSemaphoreDelete(completion_semaphore);
    
    return status;
}

/**
 * @brief Reads the JEDEC ID of the W25QXX flash memory using FreeRTOS.
 *
 * @param hflash Pointer to the W25Q flash handle
 * @param pManufacturerID Pointer to store the manufacturer ID
 * @param pDeviceID Pointer to store the device ID
 * @return W25Q_StatusTypeDef Status of the read operation
 *         - W25Q_OK if read was successful
 *         - W25Q_PARAM_ERR if invalid parameters are provided
 *         - W25Q_ERROR if operation failed
 *         - W25Q_TIMEOUT if operation timed out
 *         - W25Q_BUSY if device is busy
 */
W25Q_StatusTypeDef W25Q_ReadJEDECID_RTOS(W25Q_HandleTypeDef *hflash, uint8_t *pManufacturerID, uint16_t *pDeviceID)
{
    if (hflash == NULL || pManufacturerID == NULL || pDeviceID == NULL || !hflash->config.use_rtos) {
        return W25Q_PARAM_ERR;
    }
    
    W25Q_QueueItem item;
    W25Q_StatusTypeDef status = W25Q_OK;
    SemaphoreHandle_t completion_semaphore;
    uint8_t buffer[3]; // Temporary buffer for ID data
    
    // Create a semaphore for this operation
    completion_semaphore = xSemaphoreCreateBinary();
    if (completion_semaphore == NULL) {
        return W25Q_ERROR;
    }
    
    // Prepare the command
    item.cmd_type = W25Q_CMD_READ_ID;
    item.address = 0; // Not used for this command
    item.buffer = buffer;
    item.size = 0; // Not used for this command
    item.status = &status;
    item.completion_semaphore = completion_semaphore;
    
    // Send the command to the queue
    if (xQueueSend(hflash->cmd_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        vSemaphoreDelete(completion_semaphore);
        return W25Q_BUSY;
    }
    
    // Wait for the operation to complete
    if (xSemaphoreTake(completion_semaphore, pdMS_TO_TICKS(100)) != pdTRUE) {
        status = W25Q_TIMEOUT;
    } else {
        // Copy the results if the operation was successful
        if (status == W25Q_OK) {
            *pManufacturerID = buffer[0];
            *pDeviceID = (uint16_t)(buffer[1] << 8) | buffer[2];
        }
    }
    
    // Clean up
    vSemaphoreDelete(completion_semaphore);
    
    return status;
}

#ifdef W25QXX_USE_RTOS
/**
 * @brief FreeRTOS task function for handling flash memory operations asynchronously
 *
 * This task manages queued flash memory operations in a thread-safe manner using FreeRTOS primitives.
 * It supports various operations such as reading, writing pages, erasing sectors, reading device ID,
 * and reading status registers.
 *
 * @param argument Pointer to the W25Q Flash memory handle used for context
 *
 * @note This is an internal task function for managing flash operations in an RTOS environment
 */
static void W25Q_Task(void *argument)
{
    W25Q_HandleTypeDef *hflash = (W25Q_HandleTypeDef *)argument;
    W25Q_QueueItem item;
    W25Q_StatusTypeDef status;

    for (;;)
    {
        // Wait for a command from the queue
        if (xQueueReceive(hflash->cmd_queue, &item, portMAX_DELAY) == pdTRUE)
        {
            // Take the mutex to ensure exclusive access to the flash
            if (xSemaphoreTake(hflash->mutex, portMAX_DELAY) == pdTRUE)
            {
                // Process the command
                switch (item.cmd_type)
                {
                case W25Q_CMD_READ:
                    status = W25Q_Read_DMA(hflash, item.address, item.buffer, item.size);
                    if (status == W25Q_OK)
                    {
                        // Wait for DMA completion
                        xSemaphoreTake(hflash->rx_semaphore, portMAX_DELAY);
                        status = hflash->dma_error ? W25Q_ERROR : W25Q_OK;
                    }
                    break;

                case W25Q_CMD_WRITE_PAGE:
                    status = W25Q_WritePage_DMA(hflash, item.address, item.buffer, item.size);
                    if (status == W25Q_OK)
                    {
                        // Wait for DMA completion
                        xSemaphoreTake(hflash->tx_semaphore, portMAX_DELAY);
                        status = hflash->dma_error ? W25Q_ERROR : W25Q_OK;

                        // Wait for flash to complete the write operation
                        uint32_t timeout = W25Q_CalculateTimeout(hflash, W25Q_CMD_PAGE_PROGRAM, item.size);
                        W25Q_WaitForReady(hflash, timeout);
                    }
                    break;

                case W25Q_CMD_ERASE_SECTOR:
                    status = W25Q_EraseSector(hflash, item.address);
                    break;

                case W25Q_CMD_READ_ID:
                    status = W25Q_ReadJEDECID(hflash,
                                              (uint8_t *)item.buffer,
                                              (uint16_t *)(item.buffer + sizeof(uint8_t)));
                    break;

                case W25Q_CMD_READ_STATUS:
                    status = W25Q_ReadStatusRegister1(hflash, item.buffer);
                    break;

                default:
                    status = W25Q_PARAM_ERR;
                    break;
                }

                // Release the mutex
                xSemaphoreGive(hflash->mutex);

                // Set the status if requested
                if (item.status != NULL)
                {
                    *item.status = status;
                }

                // Signal completion to the caller
                if (item.completion_semaphore != NULL)
                {
                    xSemaphoreGive(item.completion_semaphore);
                }
            }
        }
    }
}
#endif
#endif // W25QXX_USE_RTOS
