#include "bsp_x42s.h"

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

    /* 绑定电机使用的串口与通信地址 */
    motor->usart = usart;
    motor->address = address;

    return HAL_OK;
}

/* 控制电机驱动使能 */
HAL_StatusTypeDef BSP_X42S_Enable(BSP_X42S_t motor, uint8_t enable)
{
    uint8_t command[6];

    /* 检查电机和串口对象 */
    if(motor == NULL || motor->usart == NULL)
    {
        return HAL_ERROR;
    }
    
    if(motor->usart->huart ==NULL || BSP_USART_IsInitialized(motor->usart) == 0U)
    {
        return HAL_ERROR;
    }

    /* 地址不能为广播地址，使能值只能为0或1 */
    if(motor->address == 0U || enable > 1U)
    {
        return HAL_ERROR;
    }

    /* 组装Emm命令 */
    command[0] = motor->address;
    command[1] = 0xF3;
    command[2] = 0xAB;
    command[3] = enable;
    command[4] = 0x00;
    command[5] = 0x6B;

    return HAL_UART_Transmit(motor->usart->huart, command,sizeof(command), 100U);
}

HAL_StatusTypeDef BSP_X42S_SetSpeed(BSP_X42S_t motor, BSP_X42S_Direction_t direction, uint16_t speed_rpm, uint8_t acceleration)
{
    uint8_t command[8];

    /* 检查电机和串口对象 */
    if(motor == NULL || motor->usart == NULL)
    {
        return HAL_ERROR;
    }

    if(motor->usart->huart == NULL || BSP_USART_IsInitialized(motor->usart) == 0U)
    {
        return HAL_ERROR;
    }

    /* 检查参数 */
    if(motor->address == 0U || direction > BSP_X42S_CCW || speed_rpm > 3000U)
    {
        return HAL_ERROR;
    }

    /* 组装 Emm F6 速度命令 */
    command[0] = motor->address;
    command[1] = 0xF6;
    command[2] = (uint8_t)direction;
    command[3] = (uint8_t)(speed_rpm >> 8);
    command[4] = (uint8_t)(speed_rpm & 0xFFU);
    command[5] = acceleration;
    command[6] = 0x00;
    command[7] = 0x6B;

    return HAL_UART_Transmit(motor->usart->huart, command, sizeof(command), 100U);
}
