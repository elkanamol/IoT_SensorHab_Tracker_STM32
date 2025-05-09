/**
 * @file uat_freertos.c
 * @brief Implementation of FreeRTOS-friendly uAT command parser
 *
 * This file implements the uAT command parser interface defined in uat_freertos.h.
 * It uses FreeRTOS primitives for thread safety and provides both DMA and
 * interrupt-driven UART reception options.
 *
 * Implementation notes:
 * - Uses stream buffers for UART reception buffering
 * - Uses semaphores for synchronization
 * - Supports circular DMA buffer handling
 *
 * @author [Elkana Molson]
 * @date [06/05/2025]
 */

#include "stm32f756xx.h"
#include "stm32f7xx_hal.h" // or your HAL header
#include "stm32f7xx_hal_uart.h"
#include "usart.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "main.h"

#include "uat_freertos.h"

// Configuration: define UAT_USE_DMA for DMA-based RX, otherwise interrupt-driven RX

#ifndef UAT_CONFIG_DEFINED
// Default configuration
#define UAT_USE_DMA
#endif

// for standard uAT, change these to match your needs
#define UAT_RX_BUFFER_SIZE 256     // size of RX buffer
#define UAT_TX_BUFFER_SIZE 256     // size of TX buffer, if using with IP or long fields you need to increase this
#define UAT_MAX_CMD_HANDLERS 10    // max number of command handlers
#define UAT_LINE_TERMINATOR "\r\n" // this is the default for uAT to parse in a CRLF-terminated line

#ifdef UAT_USE_DMA // for DMA-based RX
#define UART_DMA_RX_SIZE 256
static uint8_t uart_dma_rx_buf[UART_DMA_RX_SIZE];
static volatile size_t dma_last_pos __attribute__((aligned(4))) = 0;
#endif

// Forward declaration
struct uAT_HandleStruct;
typedef void (*uAT_CommandHandler)(const char *args);

/**
 * @brief Command handler entry structure
 *
 * Stores a command string and its associated handler function.
 * Used in the command dispatch system.
 */
typedef struct
{
    const char *command;         ///< Command string to match
    uAT_CommandHandler handler;  ///< Function to call when command is received
} uAT_CommandEntry;

/**
 * @brief Main uAT handle structure
 *
 * Contains all state information for the uAT parser, including
 * UART handle, synchronization primitives, and command handlers.
 */
typedef struct uAT_HandleStruct
{
    UART_HandleTypeDef *huart;                          // UART handle that connect to modem (e.g. UART2)
    StreamBufferHandle_t rxStream;                      // Stream buffer for RX
    SemaphoreHandle_t txComplete;                       // For UART transmission
    SemaphoreHandle_t txMutex;                          // For UART transmission
    SemaphoreHandle_t handlerMutex;                     // For command handler management
    SemaphoreHandle_t sendReceiveSem;                   // For SendReceive
    uint8_t txBuffer[UAT_TX_BUFFER_SIZE];               // Transmit buffer
    uAT_CommandEntry cmdHandlers[UAT_MAX_CMD_HANDLERS]; // Registered commands
    size_t cmdCount;                                    // Number of registered commands

    // SendReceive state
    bool inSendReceive;  // True if currently in SendReceive
    char *srBuffer;      // Buffer for SendReceive
    size_t srBufferSize; // Size of srBuffer
    size_t srBufferPos;  // Current position in srBuffer
} uAT_Handle_t;

static uAT_Handle_t uat;

// Push single received byte into stream buffer
static inline void uAT_PushRxByte(uint8_t byte)
{
    BaseType_t xHigher = pdFALSE;
    xStreamBufferSendFromISR(uat.rxStream, &byte, 1, &xHigher);
    portYIELD_FROM_ISR(xHigher);
}

#ifdef UAT_USE_DMA
// Idle line IRQ handler to copy new DMA data into stream buffer
/**
 * @brief Handles UART IDLE line interrupt for DMA-based reception
 * 
 * This function processes incoming data from the DMA receive buffer when an IDLE line
 * interrupt occurs. It manages circular buffer wrapping and transfers received data
 * to the stream buffer, tracking the last processed position.
 * 
 * @note This function is designed to be called from the UART IDLE line interrupt handler
 * @note Uses critical sections to safely update shared position tracking variable
 */
void uAT_UART_IdleHandler(void)
{
    UBaseType_t uxSavedInterruptStatus;
    size_t current_pos = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(uat.huart->hdmarx);
    size_t last_pos;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Enter critical section
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    last_pos = dma_last_pos;
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    
    // No new data
    if (current_pos == last_pos) {
        return;
    }
    
    // Handle normal case (no buffer wrap)
    if (current_pos > last_pos) {
        size_t data_len = current_pos - last_pos;
        xStreamBufferSendFromISR(uat.rxStream,
                          &uart_dma_rx_buf[last_pos], 
                          data_len, 
                          &xHigherPriorityTaskWoken);
    }
    // Handle buffer wrap case
    else {
        // Data from last_pos to end of buffer
        size_t tail_len = UART_DMA_RX_SIZE - last_pos;
        if (tail_len > 0) {
            xStreamBufferSendFromISR(uat.rxStream,
                              &uart_dma_rx_buf[last_pos], 
                              tail_len, 
                              &xHigherPriorityTaskWoken);
        }
        
        // Data from start of buffer to current position
        if (current_pos > 0) {
            xStreamBufferSendFromISR(uat.rxStream,
                              &uart_dma_rx_buf[0], 
                              current_pos, 
                              &xHigherPriorityTaskWoken);
        }
    }
    
    // Update position tracking
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    dma_last_pos = current_pos;
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    
    // Yield if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ISR: called from UART IRQ when IDLE flag set
// add to USARTx_IRQHandler in stm32f7xx_it.c file
// Note: the checking and calling uAT_UART_IdleHandler() is done before USARTx_IRQHandler.
void USARTx_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(uat.huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(uat.huart);
        uAT_UART_IdleHandler();
    }
    HAL_UART_IRQHandler(uat.huart);
}

#else
// Byte-by-byte interrupt-driven receive
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t rxByte;
    if (huart == uat.huart)
    {
        uAT_PushRxByte(rxByte);
        HAL_UART_Receive_IT(uat.huart, &rxByte, 1);
    }
}
#endif

// Common TX complete callback (for both IT and DMA)
/**
 * @brief Callback function for UART transmission complete event
 * 
 * This function is called when a UART transmission is completed. It gives a binary semaphore
 * to signal the transmission is finished, allowing waiting tasks to proceed.
 * 
 * @param huart Pointer to the UART handle that completed transmission
 * 
 * @note This is an ISR (Interrupt Service Routine) callback function
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == uat.huart)
    {
        BaseType_t xHigher = pdFALSE;
        xSemaphoreGiveFromISR(uat.txComplete, &xHigher);
        portYIELD_FROM_ISR(xHigher);
    }
}

// === CORE API ===

uAT_Result_t uAT_Init(UART_HandleTypeDef *huart)
{
    if (!huart)
    {
        return UAT_ERR_INVALID_ARG;
    }

    memset(&uat, 0, sizeof(uat));
    uat.huart = huart;
    uat.rxStream = xStreamBufferCreate(UAT_RX_BUFFER_SIZE, 1);
    uat.txComplete = xSemaphoreCreateBinary();
    uat.txMutex = xSemaphoreCreateMutex();
    uat.handlerMutex = xSemaphoreCreateMutex();
    uat.sendReceiveSem = xSemaphoreCreateBinary();
    uat.inSendReceive = false;
    uat.srBuffer = NULL;
    uat.srBufferSize = 0;
    uat.srBufferPos = 0;
    
    if (!uat.rxStream || !uat.txComplete || !uat.txMutex || 
        !uat.handlerMutex || !uat.sendReceiveSem)
        return UAT_ERR_RESOURCE;

#ifdef UAT_USE_DMA
    // Reset DMA position tracking
    dma_last_pos = 0;
    
    // start circular DMA reception
    __HAL_RCC_DMA1_CLK_ENABLE();
    if (HAL_UART_Receive_DMA(huart, uart_dma_rx_buf, UART_DMA_RX_SIZE) != HAL_OK)
        return UAT_ERR_INIT_FAIL;

    // ensure IDLE interrupt enabled
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(uat.huart, UART_IT_IDLE);
#else
    // start byte-by-byte IRQ reception
    static uint8_t dummy;
    if (HAL_UART_Receive_IT(huart, &dummy, 1) != HAL_OK)
        return UAT_ERR_INIT_FAIL;
#endif

    return UAT_OK;
}

uAT_Result_t uAT_RegisterCommand(const char *cmd, uAT_CommandHandler handler)
{
    if (!cmd || !handler)
    {
        return UAT_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(uat.handlerMutex, portMAX_DELAY) != pdTRUE)
        return UAT_ERR_BUSY;

    uAT_Result_t result = UAT_ERR_RESOURCE;
    if (uat.cmdCount < UAT_MAX_CMD_HANDLERS)
    {
        uat.cmdHandlers[uat.cmdCount++] = (uAT_CommandEntry){cmd, handler};
        result = UAT_OK;
    }
    
    xSemaphoreGive(uat.handlerMutex);
    return result;
}

// Note: This function should be called with handlerMutex already taken
uAT_Result_t uAT_UnregisterCommand(const char *cmd)
{
    if (!cmd) 
        return UAT_ERR_INVALID_ARG;
        
    for (size_t i = 0; i < uat.cmdCount; i++)
    {
        if (strcmp(uat.cmdHandlers[i].command, cmd) == 0)
        {
            for (size_t j = i; j < uat.cmdCount - 1; j++)
            {
                uat.cmdHandlers[j] = uat.cmdHandlers[j + 1];
            }
            uat.cmdCount--;
            return UAT_OK;
        }
    }
    
    return UAT_ERR_NOT_FOUND;
}

// Helper function to safely append data to the SendReceive buffer
static void uAT_AppendToResponseBuffer(const char *data, size_t len)
{
    // Check if we're in a SendReceive operation and have a valid buffer
    if (!uat.inSendReceive || !uat.srBuffer || uat.srBufferPos >= uat.srBufferSize - 1)
        return;
    
    // Calculate how much space is left in the buffer
    size_t spaceLeft = uat.srBufferSize - uat.srBufferPos - 1; // -1 for null terminator
    
    // Limit copy to available space
    if (len > spaceLeft)
        len = spaceLeft;
    
    // Copy data to buffer
    if (len > 0) {
        memcpy(uat.srBuffer + uat.srBufferPos, data, len);
        uat.srBufferPos += len;
        uat.srBuffer[uat.srBufferPos] = '\0'; // Ensure null termination
    }
}

static void uAT_CommandHandler_SendReceive(const char *args)
{
    // Signal that we've received the expected response
    xSemaphoreGive(uat.sendReceiveSem);
}

// Helper function to safely clean up SendReceive state
static void uAT_CleanupSendReceiveState(const char *expected)
{
    // This function should be called with handlerMutex already taken
    uAT_UnregisterCommand(expected);
    uat.inSendReceive = false;
    uat.srBuffer = NULL;
    uat.srBufferSize = 0;
    uat.srBufferPos = 0;
}

// Helper function to take mutex and clean up SendReceive state
static void uAT_SafeCleanupSendReceiveState(const char *expected, TickType_t timeoutTicks)
{
    if (xSemaphoreTake(uat.handlerMutex, timeoutTicks) == pdTRUE) {
        uAT_CleanupSendReceiveState(expected);
        xSemaphoreGive(uat.handlerMutex);
    }
}

// Helper function to set up SendReceive state
static uAT_Result_t uAT_SetupSendReceiveState(const char *expected, char *outBuf, size_t bufLen)
{
    // This function should be called with handlerMutex already taken
    
    // Set up the SendReceive state
    uat.inSendReceive = true;
    uat.srBuffer = outBuf;
    uat.srBufferSize = bufLen;
    uat.srBufferPos = 0;
    
    // Register the command handler for the expected response
    if (uat.cmdCount < UAT_MAX_CMD_HANDLERS) {
        uat.cmdHandlers[uat.cmdCount++] = (uAT_CommandEntry){expected, uAT_CommandHandler_SendReceive};
        return UAT_OK;
    }
    
    // If we couldn't register the handler, reset the state
    uat.inSendReceive = false;
    uat.srBuffer = NULL;
    uat.srBufferSize = 0;
    uat.srBufferPos = 0;
    
    return UAT_ERR_RESOURCE;
}

/**
 * @brief Sends an AT command and waits for a specific response
 * 
 * Implementation details:
 * 1. Takes handlerMutex to ensure exclusive access to command handlers
 * 2. Registers a temporary handler for the expected response
 * 3. Sends the command using uAT_SendCommand()
 * 4. Waits for the response with timeout
 * 5. Cleans up by unregistering the temporary handler
 * 
 * @param cmd Command to send
 * @param expected Expected response prefix till end of line
 * @param outBuf Buffer to store the response from beginning of the response till end of line of the `expected` prefix
 * @param bufLen Size of outBuf
 * @param timeoutTicks Maximum time to wait for response
 * @return UAT_OK on success, error code otherwise
 */
uAT_Result_t uAT_SendReceive(const char *cmd, const char *expected, char *outBuf, size_t bufLen, TickType_t timeoutTicks)
{
    if (!cmd || !expected || !outBuf || bufLen == 0) {
        return UAT_ERR_INVALID_ARG;
    }

    // Clear the output buffer
    outBuf[0] = '\0';
    
    // 1) Serialize access to SendReceive operation
    if (xSemaphoreTake(uat.handlerMutex, timeoutTicks) != pdTRUE) {
        return UAT_ERR_BUSY;
    }
    
    // Check if we're already in a SendReceive operation
    if (uat.inSendReceive) {
        xSemaphoreGive(uat.handlerMutex);
        return UAT_ERR_BUSY;
    }
    
    // Set up the SendReceive state
    uAT_Result_t result = uAT_SetupSendReceiveState(expected, outBuf, bufLen);
    xSemaphoreGive(uat.handlerMutex);
    
    if (result != UAT_OK) {
        return UAT_ERR_INT;
    }
    
    // 2) Send the AT command
    printf("Sending command: %s\n", cmd);
    if (UAT_OK != uAT_SendCommand(cmd)) {
        uAT_SafeCleanupSendReceiveState(expected, timeoutTicks);
        return UAT_ERR_SEND_FAIL;
    }
    
    // 3) Wait for the callback to fire (or timeout)
    if (xSemaphoreTake(uat.sendReceiveSem, timeoutTicks) != pdTRUE) {
        uAT_SafeCleanupSendReceiveState(expected, portMAX_DELAY);
        return UAT_ERR_TIMEOUT;
    }
    
    // 4) Success - unregister the command handler
    uAT_SafeCleanupSendReceiveState(expected, portMAX_DELAY);
    return UAT_OK;
}

uAT_Result_t uAT_SendCommand(const char *cmd)
{
    if (!cmd)
        return UAT_ERR_INVALID_ARG; // invalid command argument

    if (xSemaphoreTake(uat.txMutex, pdMS_TO_TICKS(500)) != pdTRUE)
        return UAT_ERR_BUSY;      // failed to take mutex, check if uart is busy

    int len = snprintf((char *)uat.txBuffer,
                       UAT_TX_BUFFER_SIZE, "%s\r\n", cmd);

    if (len <= 0 || len >= UAT_TX_BUFFER_SIZE) 
    {
        xSemaphoreGive(uat.txMutex);
        return UAT_ERR_INVALID_ARG; // buffer isn't valid length
    }

    // choose DMA or IT transmit automatically by HAL
    if (HAL_UART_Transmit_DMA(uat.huart, uat.txBuffer, len) != HAL_OK)
    {
        xSemaphoreGive(uat.txMutex);
        return UAT_ERR_SEND_FAIL;  // failed to transmit
    }
    
    // wait for completion with timeout
    BaseType_t result = xSemaphoreTake(uat.txComplete, pdMS_TO_TICKS(1000));
    xSemaphoreGive(uat.txMutex);

    return (result == pdTRUE) ? UAT_OK : UAT_ERR_TIMEOUT;
}

//
/**
 * @brief Helper function that dispatches an incoming AT command
 *  to the appropriate registered handler.
 *
 * Iterates through registered command handlers to find a matching command prefix.
 * When a match is found, the corresponding handler is called with the command arguments.
 *
 * @param line Received command line to dispatch
 * @param len Length of the received command line
 * @return bool True if a matching handler was found and executed, false otherwise
 */
static bool uAT_DispatchCommand(const char *line, size_t len)
{
    
    for (size_t i = 0; i < uat.cmdCount; i++) {
        const char *cmd = uat.cmdHandlers[i].command;
        size_t cmdLen = strlen(cmd);
        // 
        if (strncmp(line, cmd, cmdLen) == 0) {
            const char *args = line + cmdLen;
            // Skip leading spaces in the 
            while (*args == ' ') {
                args++;
            }
            
            // Store handler to call after releasing mutex
            uAT_CommandHandler handler = uat.cmdHandlers[i].handler;
            xSemaphoreGive(uat.handlerMutex);
            
            // Call handler outside critical section
            handler(args);
            return true;
        }
    }
    return false;
}

/**
 * @brief Halpe function that receive data from a stream buffer until a delimiter is found
 *
 * Reads characters from the stream buffer into a destination buffer until
 * either the maximum length is reached, the delimiter is found, or no more
 * data is available within the specified timeout.
 *
 * @param stream Stream buffer to read from
 * @param dest Destination buffer to store received characters
 * @param maxLen Maximum number of characters to read
 * @param delim Delimiter string to search for
 * @param ticksToWait Maximum time to wait for data (in RTOS ticks)
 * @return Number of characters received
 */
size_t xStreamBufferReceiveUntilDelimiter(
    StreamBufferHandle_t stream,
    char *dest,
    size_t maxLen,
    const char *delim,
    TickType_t ticksToWait)
{
    size_t total = 0;
    char ch;
    while (total < maxLen - 1)
    {
        if (xStreamBufferReceive(stream, &ch, 1, ticksToWait) != 1)
            break;
        dest[total++] = ch;
        dest[total] = '\0';
        if (strstr(dest, delim))
            break;
    }
    dest[total] = '\0';
    return total;
}

/**
 * @brief FreeRTOS task for handling UAT (UART AT) command processing
 *
 * This task continuously monitors the UAT receive stream for incoming commands.
 * When a complete command is received, it attempts to dispatch the command 
 * to a registered handler. In SendReceive mode, it also captures the response.
 *
 * @param params Unused task parameters
 */
void uAT_Task(void *params)
{
    (void)params;
    uint8_t lineBuf[UAT_RX_BUFFER_SIZE];
    printf("uAT_Task started\r\n");

    while (1) {
        size_t len = xStreamBufferReceiveUntilDelimiter(
            uat.rxStream,
            (char *)lineBuf,
            sizeof(lineBuf),
            UAT_LINE_TERMINATOR,
            pdMS_TO_TICKS(1000));

        if (len > 0) {
            if (xSemaphoreTake(uat.handlerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Always capture response if in SendReceive mode
                if (uat.inSendReceive) {
                    uAT_AppendToResponseBuffer((char *)lineBuf, len);
                }
                
                // Dispatch to appropriate handler
                if (!uAT_DispatchCommand((char *)lineBuf, len)) {
                    // No handler found
                    xSemaphoreGive(uat.handlerMutex);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


uAT_Result_t uAT_Reset(void)
{
    // Stop any ongoing transfers
    HAL_UART_AbortReceive(uat.huart);
    HAL_UART_AbortTransmit(uat.huart);
    
    // Clear stream buffer
    xStreamBufferReset(uat.rxStream);
    
    // Reset DMA
    dma_last_pos = 0;
    if (HAL_UART_Receive_DMA(uat.huart, uart_dma_rx_buf, UART_DMA_RX_SIZE) != HAL_OK)
    {
        return UAT_ERR_INIT_FAIL;
    }

    // Re-enable IDLE interrupt
    __HAL_UART_CLEAR_IDLEFLAG(uat.huart);
    __HAL_UART_ENABLE_IT(uat.huart, UART_IT_IDLE);

    return UAT_OK;
}

/**
 * @brief Register a URC handler with high priority
 * 
 * This function registers a command handler for Unsolicited Result Codes (URCs)
 * with higher priority than regular command handlers by inserting it at the
 * beginning of the handler array.
 * 
 * @param cmd Command string to match
 * @param handler Function to call when the command is received
 * @return UAT_OK if successful, error code otherwise
 */
uAT_Result_t uAT_RegisterURC(const char *cmd, uAT_CommandHandler handler)
{
    if (!cmd || !handler) {
        return UAT_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(uat.handlerMutex, portMAX_DELAY) != pdTRUE)
        return UAT_ERR_BUSY;

    uAT_Result_t result = UAT_ERR_RESOURCE;
    if (uat.cmdCount < UAT_MAX_CMD_HANDLERS) {
        // Shift existing handlers to make room at the beginning
        for (size_t i = uat.cmdCount; i > 0; i--) {
            uat.cmdHandlers[i] = uat.cmdHandlers[i-1];
        }
        
        // Insert the URC handler at the beginning
        uat.cmdHandlers[0] = (uAT_CommandEntry){cmd, handler};
        uat.cmdCount++;
        result = UAT_OK;
    }
    
    xSemaphoreGive(uat.handlerMutex);
    return result;
}
