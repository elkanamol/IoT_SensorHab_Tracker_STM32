#ifndef UAT_FREERTOS_H
#define UAT_FREERTOS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f7xx_hal.h" // or your HAL header
#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"
#include <stddef.h>
#include <stdbool.h>

// Configuration macros (can override before include)
#ifndef UAT_RX_BUFFER_SIZE
#define UAT_RX_BUFFER_SIZE 256
#endif
#ifndef UAT_TX_BUFFER_SIZE
#define UAT_TX_BUFFER_SIZE 128
#endif
#ifndef UAT_MAX_CMD_HANDLERS
#define UAT_MAX_CMD_HANDLERS 10
#endif
#ifndef UAT_LINE_TERMINATOR
#define UAT_LINE_TERMINATOR "\r\n"
#endif
#ifndef UAT_USE_DMA
#define UAT_USE_DMA
#endif

    /** Result codes for sync transactions */
    typedef enum
    {
        UAT_OK,             // Expected reply received
        UAT_TIMEOUT,        // No reply within timeout
        UAT_ERR_BUSY,       // Another transaction in progress
        UAT_ERR_NOMATCH,    // Got reply, but didn’t match expected prefix
        UAT_ERR_INT,        // Internal error (mutex, memory, etc)
        UAT_ERR_INVALID_ARG, // Invalid argument
        UAT_ERR_NOT_FOUND, // Command not found
        UAT_ERR_SEND_FAIL, // Failed to send command
    } uAT_Result_t;


    // Uncomment to enable circular DMA RX (uses UART IDLE interrupt)
    // #define UAT_USE_DMA

    // Forward declaration of the uAT handle (opaque in user code)
    typedef struct uAT_HandleStruct uAT_Handle_t;

    // Command handler callback prototype
    // args points to the first character after the registered command
    // Ex: if command == "OK", handler receives "param1,param2" when lineBuf == "OK param1,param2\r\n"
    typedef void (*uAT_CommandHandler)(const char *args);

    // API


    /**
     * @brief  Initialize the uAT parser module
     * @param  huart Pointer to HAL UART handle
     * @return true if successful, false on allocation or config error
     */
    bool uAT_Init(UART_HandleTypeDef *huart);

    /**
     * @brief  Register a command string and its handler
     * @param  cmd     Null-terminated string to match at start of line
     * @param  handler Function called when a line beginning with cmd arrives
     * @return true if registered, false if handler table is full
     */
    bool uAT_RegisterCommand(const char *cmd, uAT_CommandHandler handler);

    /**
     * @brief  Unregister a previously registered command
     * Note: This function should be called with `uat.handlerMutex` already taken
     * @param  cmd Null-terminated string of the command to unregister
     * @return Result of the unregistration operation (uAT_Result_t)
     */
    uAT_Result_t uAT_UnregisterCommand(const char *cmd);

    /**
     * @brief  Send an AT-style command (appends CR+LF)
     * @param  cmd Null-terminated command string without terminator
     * @return true on success, false on UART error or timeout
     */
    bool uAT_SendCommand(const char *cmd);

    /**
     * @brief  Send a command and wait for a specific response prefix.
     * @param  cmd            Null-terminated AT command (no CRLF)
     * @param  expected       Prefix to match (e.g. "OK" or "+CREG")
     * @param  outBuf         Buffer to receive the full line (incl. CRLF)
     * @param  bufLen         Length of outBuf
     * @param  timeoutTicks   How many RTOS ticks to wait
     * @return one of uAT_Result_t
     */
    uAT_Result_t uAT_SendReceive(const char *cmd,
                                 const char *expected,
                                 char *outBuf,
                                 size_t bufLen,
                                 TickType_t timeoutTicks);

    /**
     * @brief  FreeRTOS task to process incoming lines and dispatch handlers
     * @param  params Unused
     */
    void uAT_Task(void *params);

// ISR hooks (implement or forward in application IRQ)
#ifdef UAT_USE_DMA
    /**
     * @brief  Must be called from UART IRQ handler on IDLE line event
     *         Extracts new bytes from DMA buffer into stream buffer
     */
    void uAT_UART_IdleHandler(void);
#endif

    /**
     * @brief  Receive data from a stream buffer until a delimiter is found
     * @param  stream     Handle to the stream buffer
     * @param  dest       Destination buffer to store received data
     * @param  maxLen     Maximum number of bytes to receive
     * @param  delim      Null-terminated string delimiter to stop receiving
     * @param  ticksToWait Maximum time to wait for data
     * @return Number of bytes received, or 0 if no data or delimiter not found
     */
    size_t xStreamBufferReceiveUntilDelimiter(
        StreamBufferHandle_t stream,
        char *dest,
        size_t maxLen,
        const char *delim,
        TickType_t ticksToWait);

#ifdef __cplusplus
}
#endif

#endif // UAT_FREERTOS_H
