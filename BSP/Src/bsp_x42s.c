#include "bsp_x42s.h"

/* 检查电机对象和串口是否可用 */
static bool BSP_X42S_IsReady(BSP_X42S_t motor)
{
    if(motor == NULL || motor->usart == NULL || motor->usart->huart == NULL)
    {
        return false;
    }

    return BSP_USART_IsInitialized(motor->usart) != 0U;
}

/* 发送一帧已经组好的命令：长度为0表示组帧失败 */
static HAL_StatusTypeDef BSP_X42S_Send(BSP_X42S_t motor, uint8_t *frame, uint8_t len)
{
    if(len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(motor->usart->huart, frame, len, 100U);
}

/* 绑定电机使用的串口与通信地址 */
HAL_StatusTypeDef BSP_X42S_Init(BSP_X42S_t motor, BSP_USART_t usart, uint8_t address)
{
    /* 检查对象指针 */
    if(motor == NULL || usart == NULL)
    {
        return HAL_ERROR;
    }

    /* 串口必须已经初始化 */
    if(usart->huart == NULL || BSP_USART_IsInitialized(usart) == 0U)
    {
        return HAL_ERROR;
    }

    /* 单电机对象不使用广播地址 */
    if(address == 0U)
    {
        return HAL_ERROR;
    }

    motor->usart = usart;
    motor->address = address;

    return HAL_OK;
}


/* 控制电机驱动使能 */
HAL_StatusTypeDef BSP_X42S_Enable(BSP_X42S_t motor, uint8_t enable)
{
    uint8_t frame[6];

    if(!BSP_X42S_IsReady(motor) || enable > 1U)
    {
        return HAL_ERROR;
    }
    return BSP_X42S_Send(motor, frame, X42S_BuildEnable(motor->address, enable == 1U, frame));
}

/* 控制电机速度 */
HAL_StatusTypeDef BSP_X42S_SetSpeed(BSP_X42S_t motor, BSP_X42S_Direction_t direction, uint16_t speed_rpm, uint8_t acceleration)
{
    uint8_t frame[8];

    if(!BSP_X42S_IsReady(motor))
    {
        return HAL_ERROR;
    }

    /* 方向、转速范围由协议层检查，不合法时返回长度0 */
    return BSP_X42S_Send(motor, frame, X42S_BuildSpeed(motor->address, (uint8_t)direction, speed_rpm, acceleration, frame));
}


