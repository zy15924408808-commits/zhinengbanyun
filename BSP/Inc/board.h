#ifndef __BOARD_H__
#define __BOARD_H__

#include "main.h"
#include "bsp_led.h"
#include "bsp_usart.h"
#include "usart.h"

extern BSP_LED_t led1;
extern BSP_LED_t led2;
extern BSP_LED_t led3;
    
extern BSP_USART_t debug_uart;      /* UART5  调试串口*/
extern BSP_USART_t motor_uart;      /* UART4  底盘电机控制 */
extern BSP_USART_t camera_uart;     /* USART1 摄像头 */
extern BSP_USART_t gyro_uart;       /* USART2 陀螺仪 */

void Board_Init(void);


#endif /* __BOARD_H__*/
