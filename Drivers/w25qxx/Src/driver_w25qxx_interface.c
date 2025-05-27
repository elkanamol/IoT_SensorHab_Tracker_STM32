#include "driver_w25qxx_interface.h"
#include "main.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdarg.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

// FreeRTOS synchronization objects (to be defined in main.c)
TaskHandle_t xW25QxxTaskHandle = NULL;
SemaphoreHandle_t xSpiMutex = NULL;

//------------------------------------------------------------------------------
// Private helper functions
//------------------------------------------------------------------------------

/**
 * @brief Wait for DMA completion using FreeRTOS task notification
 * @return HAL status code
 */
static HAL_StatusTypeDef w25qxx_wait_for_dma_completion(void)
{
    // Store current task handle if not already stored
    if (xW25QxxTaskHandle == NULL) {
        xW25QxxTaskHandle = xTaskGetCurrentTaskHandle();
    }
    
    // Wait for the notification from the SPI DMA completion ISR
    BaseType_t ret;
    ret = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(W25QXX_DMA_TIMEOUT_MS));
    if (ret == 0) {
        // Timeout occurred
        w25qxx_interface_debug_print("W25QXX SPI DMA Timeout!\n");
        // HAL_SPI_DMAStop(&hspi1);
        // return HAL_TIMEOUT;
    }
    
    // Check for any HAL errors that might have occurred during DMA transfer
    if (hspi1.ErrorCode != HAL_SPI_ERROR_NONE) {
        w25qxx_interface_debug_print("W25QXX SPI DMA Error: 0x%lx\n", hspi1.ErrorCode);
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

//------------------------------------------------------------------------------
// Interface implementation
//------------------------------------------------------------------------------

uint8_t w25qxx_interface_spi_qspi_init(void)
{
    /* Create mutex for SPI access if not already created */
    if (xSpiMutex == NULL) {
        xSpiMutex = xSemaphoreCreateMutex();
        if (xSpiMutex == NULL) {
            return 1; /* Failed to create mutex */
        }
    }
    
    /* set cs high to release spi */
    W25QXX_CS_HIGH();
    return 0;
}

uint8_t w25qxx_interface_spi_qspi_deinit(void)
{
    /* Delete mutex if it exists */
    if (xSpiMutex != NULL) {
        vSemaphoreDelete(xSpiMutex);
        xSpiMutex = NULL;
    }
    
    /* set cs high to release spi */
    W25QXX_CS_HIGH();
    return 0;
}

uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
                                             uint32_t address, uint8_t address_line, uint8_t address_len,
                                             uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                                             uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                                             uint8_t *out_buf, uint32_t out_len, uint8_t data_line)
{
    if ((instruction_line != 0) || (address_line != 0) || (alternate_line != 0) || (dummy != 0) || (data_line != 1))
    {
        return 1;
    }
    /* spi write read, calling DMA-enabled spi_write_read implementation*/
    return spi_write_read(in_buf, in_len, out_buf, out_len);
}

uint8_t spi_write_read(uint8_t *in_buf, uint32_t in_len, uint8_t *out_buf, uint32_t out_len)
{
    HAL_StatusTypeDef hal_status;
    
    // Acquire mutex if SPI peripheral is shared by multiple tasks
    if (xSpiMutex != NULL) {
        if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) != pdTRUE) {
            return 1; // Failed to acquire mutex
        }
    }
    
    /* set cs low */
    W25QXX_CS_LOW();
    
    /* if in_len > 0 */
    if (in_len > 0)
    {
        // Cache coherency: Clean D-Cache for transmit buffer
        SCB_CleanDCache_by_Addr((uint32_t*)in_buf, in_len);
        
        /* transmit the input buffer using DMA */
        hal_status = HAL_SPI_Transmit_DMA(&hspi1, in_buf, in_len);
        if (hal_status != HAL_OK)
        {
            w25qxx_interface_debug_print("HAL_SPI_Transmit_DMA failed: %d\n", hal_status);
            /* set cs high */
            W25QXX_CS_HIGH();
            if (xSpiMutex != NULL) xSemaphoreGive(xSpiMutex);
            return 1;
        }
        
        // Wait for DMA completion
        hal_status = w25qxx_wait_for_dma_completion();
        if (hal_status != HAL_OK) {
            /* set cs high */
            W25QXX_CS_HIGH();
            if (xSpiMutex != NULL) xSemaphoreGive(xSpiMutex);
            return 1;
        }
    }
    
    /* if out_len > 0 it means receive operation */
    if (out_len > 0)
    {
        /* receive to the output buffer using DMA */
        hal_status = HAL_SPI_Receive_DMA(&hspi1, out_buf, out_len);
        if (hal_status != HAL_OK)
        {
            w25qxx_interface_debug_print("HAL_SPI_Receive_DMA failed: %d\n", hal_status);
            /* set cs high */
            W25QXX_CS_HIGH();
            if (xSpiMutex != NULL) xSemaphoreGive(xSpiMutex);
            return 1;
        }
        
        // Wait for DMA completion
        hal_status = w25qxx_wait_for_dma_completion();
        if (hal_status != HAL_OK) {
            /* set cs high */
            W25QXX_CS_HIGH();
            if (xSpiMutex != NULL) xSemaphoreGive(xSpiMutex);
            return 1;
        }
        
        // Cache coherency: Invalidate D-Cache for receive buffer
        SCB_InvalidateDCache_by_Addr((uint32_t*)out_buf, out_len);
    }
    
    /* set cs high */
    W25QXX_CS_HIGH();
    
    // Release mutex
    if (xSpiMutex != NULL) xSemaphoreGive(xSpiMutex);
    
    return 0;
}

void w25qxx_interface_delay_ms(uint32_t ms)
{
    /* delay ms */
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void w25qxx_interface_delay_us(uint32_t us)
{
    /* delay us */
    if (us >= 1000)
    {
        vTaskDelay(pdMS_TO_TICKS(us / 1000));
    }
    else
    {
        // Simple busy wait for microseconds
        volatile uint32_t count = us * (SystemCoreClock / 1000000) / 10;
        while (count--);
    }
}

void w25qxx_interface_debug_print(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    printf("%s", buf);
}

// W25Q64 utility functions
uint32_t W25Q64_GetSectorAddress(uint32_t address)
{
    return (address / W25Q64_SECTOR_SIZE) * W25Q64_SECTOR_SIZE;
}

uint32_t W25Q64_GetPageAddress(uint32_t address)
{
    return (address / W25Q64_PAGE_SIZE) * W25Q64_PAGE_SIZE;
}

uint32_t W25Q64_GetSectorNumber(uint32_t address)
{
    return address / W25Q64_SECTOR_SIZE;
}

uint32_t W25Q64_GetPageNumber(uint32_t address)
{
    return address / W25Q64_PAGE_SIZE;
}

// Function to print a buffer in hex format
void PrintBufferHex(const uint8_t *buffer, uint32_t size, uint32_t baseAddress)
{
    printf("--- Buffer Hex Dump (Base Address: 0x%08lX) ---\r\n", baseAddress);

    // Print the buffer in lines of 32 bytes each
    for (uint32_t offset = 0; offset < size; offset += 32)
    {
        // Print address at the start of each line
        printf("0x%08lX: ", baseAddress + offset);

        // Print hex values
        for (uint32_t i = 0; i < 32 && (offset + i) < size; i++)
        {
            printf("%02X ", buffer[offset + i]);

            // Add extra space every 4 bytes for readability
            if ((i + 1) % 4 == 0)
            {
                printf(" ");
            }
        }

        // Fill remaining space if less than 32 bytes in this line
        uint32_t bytesInLine = ((offset + 32) <= size) ? 32 : (size - offset);
        for (uint32_t i = bytesInLine; i < 32; i++)
        {
            printf("   "); // 3 spaces for each missing byte
            if ((i + 1) % 4 == 0)
            {
                printf(" "); // Extra space every 4 bytes
            }
        }

        // Print ASCII representation
        printf(" | ");
        for (uint32_t i = 0; i < 32 && (offset + i) < size; i++)
        {
            // Print only printable ASCII characters, otherwise print a dot
            if (buffer[offset + i] >= 32 && buffer[offset + i] <= 126)
            {
                printf("%c", buffer[offset + i]);
            }
            else
            {
                printf(".");
            }
        }

        // printf("\r\n");
    }

    printf("--- End of Buffer Hex Dump ---\r\n");
}