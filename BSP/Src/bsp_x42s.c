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
/* 控制电机速度 */
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
void BSP_X42S_ParserInit(BSP_X42S_Parser_t *parser)
{
    if(parser == NULL)
    {
        return;
    }

    parser->used = 0U;
}
/* 解析电机发送的数据 */
bool BSP_X42S_ParseByte(BSP_X42S_Parser_t *parser, uint8_t byte, BSP_X42S_Response_t *response)
{
    if(parser == NULL || response == NULL)
    {
        return false;
    }

    if(parser->used >= sizeof(parser->buffer))
    {
        parser->used = 0U;
    }

    parser->buffer[parser->used++] = byte;

    if(parser->used < sizeof(parser->buffer))
    {
        return false;
    }
    /* 电机有效应答，4个字节，buffer[0]电机地址，buffer[1]功能码(0xF3或0xF6)，buffer[2]状态，buffer[3]尾帧 */
    if(parser->buffer[0] != 0U && (parser->buffer[1] == 0xF3U || parser->buffer[1] == 0xF6U) && parser->buffer[3] == 0x6BU)
    {
        response->address = parser->buffer[0];
        response->function = parser->buffer[1];
        response->status = parser->buffer[2];
        parser->used = 0U;
        return true;
    }

    /* 不匹配时保留后三个字节，继续查找应答 */
    parser->buffer[0] = parser->buffer[1];
    parser->buffer[1] = parser->buffer[2];
    parser->buffer[2] = parser->buffer[3];
    parser->used = 3U;
    return false;
}

