#ifndef BSP_X42S_H
#define BSP_X42S_H

#include "bsp_usart.h"

/*Emm 协议中的旋转方向*/
typedef enum
{
    BSP_X42S_CW     = 0U,
    BSP_X42S_CCW    = 1U
}BSP_X42S_Direction_t;

/* 电机对象 */
struct bsp_x42s
{
    BSP_USART_t usart;
    uint8_t address;
};

typedef struct bsp_x42s *BSP_X42S_t;

/* 绑定串口与电机地址 */
HAL_StatusTypeDef BSP_X42S_Init(BSP_X42S_t motor, BSP_USART_t usart, uint8_t address);

/* 驱动使能：1开启，0关闭 */
HAL_StatusTypeDef BSP_X42S_Enable(BSP_X42S_t motor, uint8_t enable);

/* 速度控制：速度单位 RPM, 加速度参数范围 0~255 */
HAL_StatusTypeDef BSP_X42S_SetSpeed(BSP_X42S_t motor, BSP_X42S_Direction_t direction, uint16_t speed_rpm, uint8_t acceleration);


#endif /* BSP_X42S_H */
