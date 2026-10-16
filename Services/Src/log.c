#include "log.h"
#include "board.h"
#include "cmsis_os.h"
#include <stdarg.h>
#include <stdio.h>

/* 互斥锁句柄 */
static osMutexId_t log_mutex;

static const osMutexAttr_t log_mutex_attributes =
{
    .name = "LogMutex",
    .attr_bits = osMutexPrioInherit,
};

bool Log_Init(void)
{
    if(log_mutex != NULL)
    {
        return true;
    }

    log_mutex = osMutexNew(&log_mutex_attributes);

    return log_mutex != NULL;
}

void Log_Printf(const char *format, ...)
{
    static char buffer[256];
    va_list args;
    int length;
    uint16_t send_length;

    /* 仅允许在调度器运行后的任务中调用 */
    if(__get_IPSR() != 0U || osKernelGetState() != osKernelRunning)
    {
        return;
    }

    if(format == NULL || log_mutex == NULL || debug_uart == NULL || debug_uart->huart == NULL || BSP_USART_IsInitialized(debug_uart) == 0U)
    {
        return;
    }

    if(osMutexAcquire(log_mutex, osWaitForever) != osOK)
    {
        return;
    }

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if(length > 0)
    {
        if(length >= (int)sizeof(buffer))
        {
            send_length = (uint16_t)(sizeof(buffer) - 1U);
        }
        else
        {
            send_length = (uint16_t)length;
        }

        HAL_UART_Transmit(debug_uart->huart, (uint8_t *)buffer, send_length, 100U);
    }

    osMutexRelease(log_mutex);
}
