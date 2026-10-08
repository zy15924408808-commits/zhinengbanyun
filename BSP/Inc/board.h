#ifndef __BOARD_H__
#define __BOARD_H__

#include "main.h"
#include "bsp_led.h"
#include "bsp_usart.h"
#include "usart.h"
#include "bsp_bno080.h"
#include "i2c.h"
#include <stdio.h>
#include "bsp_oled.h"
#include "spi.h"
#include "bsp_power.h"

extern BSP_LED_t led1;
extern BSP_LED_t led2;
extern BSP_LED_t led3;
    
extern BSP_USART_t debug_uart;      /* USART5  调试串口*/
extern BSP_USART_t motor_uart;      /* USART4  底盘电机控制 */
extern BSP_USART_t camera_uart;     /* USART1 摄像头 */

extern BSP_BNO080_t imu;

extern BSP_OLED_t oled;

void Board_Init(void);


#endif /* __BOARD_H__*/
