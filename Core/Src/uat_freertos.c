/*
 * uat_freertos.c - FreeRTOS-friendly uAT command parser with IRQ and DMA support
 */

#include "stm32f756xx.h"
#include "stm32f7xx_hal.h" // or your HAL header
#include <stm32f7xx_hal_uart.h>
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
#define UAT_USE_DMA

#define UAT_RX_BUFFER_SIZE 256
#define UAT_TX_BUFFER_SIZE 128
#define UAT_MAX_CMD_HANDLERS 10
#define UAT_LINE_TERMINATOR "\r\n"

#ifdef UAT_USE_DMA
#define UART_DMA_RX_SIZE 256
static uint8_t uart_dma_rx_buf[UART_DMA_RX_SIZE];
static volatile size_t dma_last_pos __attribute__((aligned(4))) = 0;
#endif

// Forward declaration
struct uAT_HandleStruct;
typedef void (*uAT_CommandHandler)(const char *args);

// uAT command callback entry
typedef struct
{
    const char *command;
    uAT_CommandHandler handler;
} uAT_CommandEntry;

// Handle structure
typedef struct uAT_HandleStruct
{
    UART_HandleTypeDef *huart;
    StreamBufferHandle_t rxStream;
    SemaphoreHandle_t txComplete;
    SemaphoreHandle_t txMutex;         // For UART transmission
    SemaphoreHandle_t handlerMutex;    // For command handler management
    SemaphoreHandle_t sendReceiveSem;
    uint8_t txBuffer[UAT_TX_BUFFER_SIZE];
    uAT_CommandEntry cmdHandlers[UAT_MAX_CMD_HANDLERS];
    size_t cmdCount;
    
    // SendReceive state
    bool inSendReceive;
    char *srBuffer;
    size_t srBufferSize;
    size_t srBufferPos;
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
void uAT_UART_IdleHandler(void)
{
    size_t pos = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(uat.huart->hdmarx);
    size_t count;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (pos != dma_last_pos)
    {
        if (pos > dma_last_pos)
        {
            count = pos - dma_last_pos;
            xStreamBufferSendFromISR(uat.rxStream,
                              &uart_dma_rx_buf[dma_last_pos], count, &xHigherPriorityTaskWoken);
        }
        else
        {
            // First, send data from dma_last_pos to the end of buffer
            count = UART_DMA_RX_SIZE - dma_last_pos;
            if (count > 0) {
                xStreamBufferSendFromISR(uat.rxStream,
                                  &uart_dma_rx_buf[dma_last_pos], count, &xHigherPriorityTaskWoken);
            }
            
            // Then, send data from beginning of buffer to pos, but only if pos > 0
            if (pos > 0) {
                xStreamBufferSendFromISR(uat.rxStream,
                                  &uart_dma_rx_buf[0], pos, &xHigherPriorityTaskWoken);
            }
        }
        dma_last_pos = pos;
    }
    
    // Yield if a higher priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// // ISR: called from UART IRQ when IDLE flag set
// void USARTx_IRQHandler(void)
// {
//     if (__HAL_UART_GET_FLAG(uat.huart, UART_FLAG_IDLE))
//     {
//         __HAL_UART_CLEAR_IDLEFLAG(uat.huart);
//         uAT_UART_IdleHandler();
//     }
//     HAL_UART_IRQHandler(uat.huart);
// }

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

bool uAT_Init(UART_HandleTypeDef *huart)
{
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
        return false;

#ifdef UAT_USE_DMA
    // Reset DMA position tracking
    dma_last_pos = 0;
    
    // start circular DMA reception
    __HAL_RCC_DMA1_CLK_ENABLE();
    if (HAL_UART_Receive_DMA(huart, uart_dma_rx_buf, UART_DMA_RX_SIZE) != HAL_OK)
        return false;
        
    // ensure IDLE interrupt enabled
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(uat.huart, UART_IT_IDLE);
#else
    // start byte-by-byte IRQ reception
    static uint8_t dummy;
    if (HAL_UART_Receive_IT(huart, &dummy, 1) != HAL_OK)
        return false;
#endif

    return true;
}

bool uAT_RegisterCommand(const char *cmd, uAT_CommandHandler handler)
{
    if (xSemaphoreTake(uat.handlerMutex, portMAX_DELAY) != pdTRUE)
        return false;
        
    bool result = false;
    if (uat.cmdCount < UAT_MAX_CMD_HANDLERS)
    {
        uat.cmdHandlers[uat.cmdCount++] = (uAT_CommandEntry){cmd, handler};
        result = true;
    }
    
    xSemaphoreGive(uat.handlerMutex);
    return result;
}

uAT_Result_t uAT_UnregisterCommand(const char *cmd)
{
    if (!cmd)
        return UAT_ERR_INVALID_ARG;
    
    // Note: This function should be called with handlerMutex already taken
    
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

uAT_Result_t uAT_SendReceive(const char *cmd, const char *expected, char *outBuf, size_t bufLen, TickType_t timeoutTicks)
{
    if (!cmd || !expected || !outBuf || bufLen == 0)
    {
        return UAT_ERR_INVALID_ARG;
    }

    // Clear the output buffer
    outBuf[0] = '\0';
    
    // 1) Serialize access to SendReceive operation
    if (xSemaphoreTake(uat.handlerMutex, timeoutTicks) != pdTRUE)
    {
        return UAT_ERR_BUSY;
    }
    
    // Check if we're already in a SendReceive operation
    if (uat.inSendReceive)
    {
        xSemaphoreGive(uat.handlerMutex);
        return UAT_ERR_BUSY;
    }
    
    // Set up the SendReceive state
    uat.inSendReceive = true;
    uat.srBuffer = outBuf;
    uat.srBufferSize = bufLen;
    uat.srBufferPos = 0;
    
    // Register the command handler for the expected response
    bool registered = false;
    if (uat.cmdCount < UAT_MAX_CMD_HANDLERS)
    {
        uat.cmdHandlers[uat.cmdCount++] = (uAT_CommandEntry){expected, uAT_CommandHandler_SendReceive};
        registered = true;
    }
    
    xSemaphoreGive(uat.handlerMutex);
    
    if (!registered)
    {
        // Clean up if registration failed
        if (xSemaphoreTake(uat.handlerMutex, portMAX_DELAY) == pdTRUE)
        {
            uat.inSendReceive = false;
            uat.srBuffer = NULL;
            uat.srBufferSize = 0;
            uat.srBufferPos = 0;
            xSemaphoreGive(uat.handlerMutex);
        }
        return UAT_ERR_INT;
    }
    
    printf("Sending command: %s\n", cmd);
    // 2) Send the AT command (this function has its own mutex protection)
    if (!uAT_SendCommand(cmd))
    {
        // Need to take mutex again to unregister the command
        if (xSemaphoreTake(uat.handlerMutex, timeoutTicks) == pdTRUE)
        {
            uAT_UnregisterCommand(expected);
            uat.inSendReceive = false;
            uat.srBuffer = NULL;
            uat.srBufferSize = 0;
            uat.srBufferPos = 0;
            xSemaphoreGive(uat.handlerMutex);
        }
        return UAT_ERR_SEND_FAIL;
    }
    
    // 3) Wait for the callback to fire (or timeout)
    if (xSemaphoreTake(uat.sendReceiveSem, timeoutTicks) != pdTRUE)
    {
        // Timed out - need to take mutex to unregister the command
        if (xSemaphoreTake(uat.handlerMutex, portMAX_DELAY) == pdTRUE)
        {
            uAT_UnregisterCommand(expected);
            uat.inSendReceive = false;
            uat.srBuffer = NULL;
            uat.srBufferSize = 0;
            uat.srBufferPos = 0;
            xSemaphoreGive(uat.handlerMutex);
        }
        return UAT_TIMEOUT;
    }
    
    // 4) Success - unregister the command handler
    if (xSemaphoreTake(uat.handlerMutex, portMAX_DELAY) == pdTRUE)
    {
        uAT_UnregisterCommand(expected);
        uat.inSendReceive = false;
        uat.srBuffer = NULL;
        uat.srBufferSize = 0;
        // Don't reset srBufferPos as it contains the final length
        xSemaphoreGive(uat.handlerMutex);
    }
    
    return UAT_OK;
}



bool uAT_SendCommand(const char *cmd)
{
    if (xSemaphoreTake(uat.txMutex, pdMS_TO_TICKS(500)) != pdTRUE)
        return false;
        
    int len = snprintf((char *)uat.txBuffer,
                       UAT_TX_BUFFER_SIZE, "%s\r\n", cmd);
    if (len <= 0 || len >= UAT_TX_BUFFER_SIZE)
    {
        xSemaphoreGive(uat.txMutex);
        return false;
    }

    // choose DMA or IT transmit automatically by HAL
    if (HAL_UART_Transmit_DMA(uat.huart, uat.txBuffer, len) != HAL_OK)
    {
        xSemaphoreGive(uat.txMutex);
        return false;
    }
    
    // wait for completion with timeout
    BaseType_t result = xSemaphoreTake(uat.txComplete, pdMS_TO_TICKS(1000));
    xSemaphoreGive(uat.txMutex);
    
    return (result == pdTRUE);
}

void uAT_Task(void *params)
{
    (void)params; // clear warning of unused parameter
    uint8_t lineBuf[UAT_RX_BUFFER_SIZE];
    char crlfStr[] = "\r\n";
    printf("uAT_Task started\r\n");

    while (1)
    {
        size_t len = xStreamBufferReceiveUntilDelimiter(
            uat.rxStream,
            (char *)lineBuf,
            sizeof(lineBuf),
            UAT_LINE_TERMINATOR,
            pdMS_TO_TICKS(1000));

        if (len > 0)
        {
            // Take mutex before accessing command handlers
            if (xSemaphoreTake(uat.handlerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                // If we're in a SendReceive operation, capture this line
                if (uat.inSendReceive)
                {
                    // Append the line to the response buffer
                    uAT_AppendToResponseBuffer((char *)lineBuf, len);
                    
                    // Append CRLF
                    // uAT_AppendToResponseBuffer(crlfStr, 2);
                }
                
                // dispatch to registered handlers
                for (size_t i = 0; i < uat.cmdCount; i++)
                {
                    const char *cmd = uat.cmdHandlers[i].command;
                    size_t cmdLen = strlen(cmd);
                    if (strncmp((char *)lineBuf, cmd, cmdLen) == 0)
                    {
                        const char *args = (char *)lineBuf + cmdLen;
                        while (*args == ' ')
                            args++;
                            
                        // Store the handler to call after releasing mutex
                        uAT_CommandHandler handler = uat.cmdHandlers[i].handler;
                        xSemaphoreGive(uat.handlerMutex);
                        
                        // Call the handler outside of the critical section
                        handler(args);
                        break;
                    }
                }
                
                // If we get here without breaking, no handler was found
                xSemaphoreGive(uat.handlerMutex);
            }
        }
        
        // Small delay to prevent tight loops
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

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
    return total;
}

bool uAT_Reset(void)
{
    // Stop any ongoing transfers
    HAL_UART_AbortReceive(uat.huart);
    HAL_UART_AbortTransmit(uat.huart);
    
    // Clear stream buffer
    xStreamBufferReset(uat.rxStream);
    
    // Reset DMA
    dma_last_pos = 0;
    HAL_UART_Receive_DMA(uat.huart, uart_dma_rx_buf, UART_DMA_RX_SIZE);
    
    // Re-enable IDLE interrupt
    __HAL_UART_CLEAR_IDLEFLAG(uat.huart);
    __HAL_UART_ENABLE_IT(uat.huart, UART_IT_IDLE);
    
    return true;
}

