#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "main.h"

/* 串口对象 */
struct bsp_usart
{
    UART_HandleTypeDef *huart;
    
    /* DMA缓冲区 */
    uint8_t     *rx_buffer;
    uint16_t    rx_buffer_size;
    /* 回调函数 */
    void (*rx_callback)(uint8_t *data, uint16_t len);
    /* 状态标志,检测是否完成初始化 */
    uint8_t     is_initialized;
    
};

typedef struct bsp_usart *BSP_USART_t;

void BSP_USART_Iint(BSP_USART_t usart, UART_HandleTypeDef *huart );
void BSP_USART_SendByte(BSP_USART_t usart, uint8_t data);
void BSP_USART_SendData(BSP_USART_t usart, uint8_t *data, uint16_t len);
void BSP_USART_Printf(BSP_USART_t usart, const char *format,...);



#endif /* __BSP_USART_H__ */
