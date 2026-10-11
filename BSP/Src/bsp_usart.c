#include "bsp_usart.h"
#include "board.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 初始化串口对象 */
void BSP_USART_Init(BSP_USART_t usart, UART_HandleTypeDef *huart )
{
    if(usart == NULL) return;
    
    usart->huart = huart;
    usart->rx_buffer = NULL;
    usart->rx_buffer_size = 0;
    usart->rx_callback = NULL;
    usart->is_initialized = 1;
    usart->rx_restart_status = HAL_OK;
}
/* 设置接收回调函数 */
void BSP_USART_SetRxCallback(BSP_USART_t usart, void (*callback)(uint8_t *data, uint16_t len))
{
    if(usart == NULL) return;
    usart->rx_callback = callback;
}
/* 设置接收缓冲区和大小 */
void BSP_USART_SetRxBuffer(BSP_USART_t usart, uint8_t* buffer,uint16_t size)
{
    if(usart == NULL) return;
    usart->rx_buffer = buffer;
    usart->rx_buffer_size = size;
}
/* 查询初始化状态 */
uint8_t BSP_USART_IsInitialized(BSP_USART_t usart)
{
    if(usart == NULL) return 0;
    return usart->is_initialized;
}
/* 发送一个字符 */
void BSP_USART_SendByte(BSP_USART_t usart, uint8_t data)
{
    if(usart == NULL || usart->is_initialized != 1) return;
    HAL_UART_Transmit(usart->huart, &data, 1, HAL_MAX_DELAY);
}
/* 发送字符串 */
void BSP_USART_SendData(BSP_USART_t usart, uint8_t *data, uint16_t len)
{
    if(usart == NULL || usart->is_initialized != 1) return;
    HAL_UART_Transmit(usart->huart, data, len, HAL_MAX_DELAY);
}
/* 格式化输出 */
void BSP_USART_Printf(BSP_USART_t usart, const char *format,...)
{
    if(usart == NULL || usart->is_initialized != 1) return;
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    BSP_USART_SendData(usart, (uint8_t*)buffer, strlen(buffer));
}
/* 重定向printf */
int fputc(int ch, FILE *f)
{
    BSP_USART_SendByte(debug_uart, (uint8_t)ch);
    return ch;
}
/* DMA中断接收 */
HAL_StatusTypeDef BSP_USART_StartReceiveToIdle(BSP_USART_t usart)
{
    HAL_StatusTypeDef status;

    /* 检查串口对象 */
    if(usart == NULL || usart->is_initialized == 0U)
    {
        return HAL_ERROR;
    }

    /* 检测DMA配置 */
    if(usart->huart == NULL || usart->huart->hdmarx == NULL)
    {
        return HAL_ERROR;
    }

    /* 检测有效接收缓冲区 */
    if(usart->rx_buffer == NULL || usart->rx_buffer_size == 0U)
    {
        return HAL_ERROR;
    }

    status = HAL_UARTEx_ReceiveToIdle_DMA(usart->huart, usart->rx_buffer, usart->rx_buffer_size);

    if(status == HAL_OK)
    {
        /* 关闭半传输中断，保留空闲和接收满事件 */
        __HAL_DMA_DISABLE_IT(usart->huart->hdmarx, DMA_IT_HT);
    }
    return status;
}
/* 在接收空闲或缓冲区收满时调用 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BSP_USART_t usart = motor_uart;

    if(huart == NULL || usart == NULL || usart->is_initialized == 0U || huart != usart->huart)
    {
        return;
    }

    /* 半传输事件不结束本次接收 */
    if(HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
    {
        return;
    }

    if(usart->rx_buffer == NULL || size == 0U || size > usart->rx_buffer_size)
    {
        return;
    }

    /* 先交接数据，再允许 DMA 重用缓冲区 */
    if(usart->rx_callback != NULL)
    {
        usart->rx_callback(usart->rx_buffer, size);
    }

    /* Normal 模式需要重新启动接收 */
    usart->rx_restart_status = BSP_USART_StartReceiveToIdle(usart);
}



