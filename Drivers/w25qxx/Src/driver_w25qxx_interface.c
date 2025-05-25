#include "driver_w25qxx_interface.h"
#include "main.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdarg.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

uint8_t w25qxx_interface_spi_qspi_init(void)
{
    W25QXX_CS_HIGH();
    return 0;
}

uint8_t w25qxx_interface_spi_qspi_deinit(void)
{
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
    
    return spi_write_read(in_buf, in_len, out_buf, out_len);
}

uint8_t spi_write_read(uint8_t *in_buf, uint32_t in_len, uint8_t *out_buf, uint32_t out_len)
{
    uint8_t res;
    
    /* set cs low */
    W25QXX_CS_LOW();
    
    /* if in_len > 0 */
    if (in_len > 0)
    {
        /* transmit the input buffer */
        res = HAL_SPI_Transmit(&hspi1, in_buf, in_len, 1000);
        if (res != HAL_OK)
        {
            /* set cs high */
            W25QXX_CS_HIGH();
            return 1;
        }
    }
    
    /* if out_len > 0 */
    if (out_len > 0)
    {
        /* receive to the output buffer */
        res = HAL_SPI_Receive(&hspi1, out_buf, out_len, 1000);
        if (res != HAL_OK)
        {
            /* set cs high */
            W25QXX_CS_HIGH();
            return 1;
        }
    }
    
    /* set cs high */
    W25QXX_CS_HIGH();
    
    return 0;
}

void w25qxx_interface_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void w25qxx_interface_delay_us(uint32_t us)
{
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