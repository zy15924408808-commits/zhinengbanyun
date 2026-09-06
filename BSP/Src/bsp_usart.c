#include "bsp_usart.h"
#include "board.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 初始化串口对象 */
void BSP_USART_Iint(BSP_USART_t usart, UART_HandleTypeDef *huart )
{
    if(usart == NULL) return;
    
    usart->huart = huart;
    usart->rx_buffer = NULL;
    usart->rx_buffer_size = 0;
    usart->rx_callback = NULL;
    usart->is_initialized = 1;
}
/* 发送一个字符 */
void BSP_USART_SendByte(BSP_USART_t usart, uint8_t data)
{
    if(usart == NULL || usart->is_initialized != 1) return;
    HAL_UART_Transmit(usart->huart, &data,1, HAL_MAX_DELAY);
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






