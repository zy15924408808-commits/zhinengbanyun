#include "app_tasks.h"
#include "board.h"
#include "cmsis_os.h"
#include "log.h"
#include "app_chassis.h"
#include <stdio.h>
#include <string.h>

typedef struct
{
  uint16_t len;
  uint8_t data[64];
}MotorRxChunk_t;

static osMessageQueueId_t motor_rx_queue;
static osMessageQueueId_t motor_ack_queue;
static volatile uint32_t motor_rx_drop_count = 0U;
static osThreadId_t MotorRxTaskHandle;
static const osThreadAttr_t MotorRxTask_attributes = {
  .name = "MotorRxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
static void MotorUartRxCallback(uint8_t *data, uint16_t len);
static void StartMotorRxTask(void *argument);

void App_RTOS_Init(void)
{
    if(!Log_Init())
    {
        Error_Handler();
        return;
    }

    motor_rx_queue = osMessageQueueNew(8U, sizeof(MotorRxChunk_t), NULL);

    if (motor_rx_queue == NULL)
    {
        Error_Handler();
        return;
    }

    /* 暂存已经解析的电机应答 */
    motor_ack_queue = osMessageQueueNew(8U, sizeof(X42S_Response_t), NULL);
    if(motor_ack_queue == NULL)
    {
        Error_Handler();
        return;
    }

    BSP_USART_SetRxCallback(motor_uart, MotorUartRxCallback);

    MotorRxTaskHandle = osThreadNew(StartMotorRxTask, NULL, &MotorRxTask_attributes);

    if (MotorRxTaskHandle == NULL)
    {
        Error_Handler();
        return;
    }
}

static void StartMotorRxTask(void *argument)
{
    MotorRxChunk_t message;
    HAL_StatusTypeDef status;
    uint16_t i;

    X42S_Parser_t parser;
    X42S_Response_t response;

    X42S_ParserInit(&parser);
    /* 任务运行后再启动 DMA，确保调度器和队列已经运行 */
    status = BSP_USART_StartReceiveToIdle(motor_uart);
    /* 电机串口的DMA接收启动*/
    Log_Printf("Motor RX DMA start:%d\r\n", (int)status);

    for (;;)
    {
        if (osMessageQueueGet(motor_rx_queue,&message,NULL,osKernelGetTickFreq()) == osOK)
        {
          for (i = 0U; i < message.len; i++)
          {
            if (X42S_ParseByte(&parser,message.data[i],&response))
            {
                /* 把完整应答交给控制任务 */
                if(osMessageQueuePut(motor_ack_queue, &response, 0U, 0U) != osOK)
                {
                    /* 队列满，解析好的回复放不进队列 */
                    Log_Printf("Motor ACK queue put FAIL\r\n");
                }
                if(response.function == X42S_FUNC_READ_POS)
                {
                    /* 收到0x36读位置的回复。addr是电机地址(1,2,3,4)，pos 是累计位置，65536 为一圈 */
                    Log_Printf("Motor POS addr:%u pos:%ld\r\n", (unsigned int)response.address,(long)response.position);
                }
                else
                {
                    /* 收到使能（F3）或调速（F6）命令的确认 */
                    Log_Printf("Motor ACK addr:%u function:0x%02X status:0x%02X\r\n",
                        (unsigned int)response.address,
                        (unsigned int)response.function,
                        (unsigned int)response.status); 
                }
            }
          }
        }
        else
        {
            /* 1 秒内没收到任何数据时打印的诊断信息，不是错误。
             * state 0x22 表示串口正在接收；
             * remaining:64 表示 DMA 缓冲区里还剩 64 字节没用，也就是没收到东西；
             * error 是串口错误码，全 0 表示正常 
            */
            Log_Printf("RX state:0x%02X remaining:%lu error:0x%08lX\r\n",
                   (unsigned int)motor_uart->huart->RxState,
                   (unsigned long)__HAL_DMA_GET_COUNTER(
                       motor_uart->huart->hdmarx),
                   (unsigned long)HAL_UART_GetError(
                       motor_uart->huart));
            /* 和上一条一起打印。
             * restart 是重新启动 DMA 接收的返回值；
             * dropped 是累计丢掉的数据包数，一直是 0 才正常 
            */
            Log_Printf("RX restart:%d dropped:%lu\r\n",
                   (int)motor_uart->rx_restart_status,
                   (unsigned long)motor_rx_drop_count);
        }
    }
}

static void MotorUartRxCallback(uint8_t *data, uint16_t len)
{
  MotorRxChunk_t message = {0};

  if(data == NULL || len == 0U || len > sizeof(message.data) || motor_rx_queue == NULL)
  {
    motor_rx_drop_count++;
    return;
  }

  message.len = len;
  memcpy(message.data, data, len);

  if(osMessageQueuePut(motor_rx_queue, &message, 0U, 0U) != osOK)
  {
    motor_rx_drop_count++;
  }

}

/* 等待指定地址、功能码的应答，收到后把完整内容写入 out：超时单位为内核 tick */
static HAL_StatusTypeDef Motor_WaitResponse(uint8_t address, uint8_t function, uint32_t timeout_ticks, X42S_Response_t *out)
{
    X42S_Response_t response;
    osStatus_t queue_status;
    uint32_t start_tick;
    uint32_t elapsed;
    if(motor_ack_queue == NULL || address == 0U || timeout_ticks == 0u || out == NULL)
    {
        return HAL_ERROR;
    }
    start_tick = osKernelGetTickCount();
    for(;;)
    {
        elapsed = osKernelGetTickCount() - start_tick;
        if(elapsed >= timeout_ticks)
        {
            return HAL_TIMEOUT;
        }
        queue_status = osMessageQueueGet(motor_ack_queue, &response, NULL, timeout_ticks - elapsed);
        if(queue_status != osOK)
        {
            return (queue_status == osErrorTimeout) ? HAL_TIMEOUT : HAL_ERROR;
        }
        /* 不是正在等待的应答，继续等待 */
        if(response.address != address || response.function != function)
        {
            continue;
        }

        *out = response;
        return (response.status == X42S_STATUS_OK) ? HAL_OK : HAL_ERROR;
    }
}

/* 等待控制命令（F3/F6）确认应答 */
static HAL_StatusTypeDef Motor_WaitAck(uint8_t address, uint8_t function, uint32_t timeout_ticks)
{
    X42S_Response_t response;

    return Motor_WaitResponse(address, function, timeout_ticks, &response);
}

/* 读取一台电机的实时位置：发送请求并等待应答，单位为编码器计数（65536 = 一圈） */
static HAL_StatusTypeDef Motor_ReadPosition(BSP_X42S_t motor, uint32_t timeout_ticks, int32_t *position)
{
    X42S_Response_t response;
    HAL_StatusTypeDef status;

    if(motor == NULL || position == NULL)
    {
        return HAL_ERROR;
    }

    osMessageQueueReset(motor_ack_queue);
    status = BSP_X42S_RequestPosition(motor);
    if(status != HAL_OK)
    {
        return status;
    }

    status = Motor_WaitResponse(motor->address, X42S_FUNC_READ_POS, timeout_ticks, &response);
    if(status == HAL_OK)
    {
        *position = response.position;
    }

    return status;
}

void App_ControlTask(void *argument)
{
    int32_t pos_start[4] = {0};
    int32_t pos_end[4] = {0};
    BSP_X42S_t motors[4] = {motor1, motor2, motor3, motor4};
    int16_t test_rpm[4] = {0};
    HAL_StatusTypeDef status;
    uint8_t i;
    uint8_t failed;
    /* 内核每秒1000tick，等待上限200ms*/
    uint32_t ack_timeout = 200U;

    App_Chassis_Init();

    /* 等待电机上电启动 */
    osDelay(2000);
    failed = 0U;

    /*先使能四台电机*/
    for(i = 0U; i < 4U; i++)
    {
        osMessageQueueReset(motor_ack_queue);
        status = BSP_X42S_Enable(motors[i], 1U);
        /* 使能命令发出去了 */
        Log_Printf("Motor %u enable TX:%d\r\n",(unsigned int)(i + 1U), (int)status);
        if(status == HAL_OK)
        {
            status = Motor_WaitAck(motors[i]->address, 0xF3U, ack_timeout);
            /* 收到了电机的使能确认 */
            Log_Printf("Motor %u enable confirm:%d\r\n", (unsigned int)motors[i]->address, (int)status);
        }
        if(status != HAL_OK)
        {
            failed = 1U;
            break;
        }
        osDelay(10);
    }
    /* 记录起始位置 */
    for(i = 0U; i < 4U; i++)
    {
        status = Motor_ReadPosition(motors[i], ack_timeout, &pos_start[i]);
        /* 读起始位置成功 */
        Log_Printf("Motor %u start pos read:%d\r\n", (unsigned int)motors[i]->address, (int)status);        
    }
    /* 根据运动分量计算轮速：先验证左横移 */
    status = App_Chassis_CalcWheelRpm(30, 30, 0, test_rpm);
    
    if(status != HAL_OK)
    {
        failed = 1U;
        /* 麦轮轮速计算失败，比如算出来超过了 3000 RPM */
        Log_Printf("Chassis wheel calculation FAIL\r\n");
    }
    else
    {
        /* 算出来的四个轮子转速，顺序是左前、右前、左后、右后 */
        Log_Printf("Wheel RPM: %d %d %d %d\r\n", (int)test_rpm[0], (int)test_rpm[1], (int)test_rpm[2], (int)test_rpm[3]);
    }
    /* 计算和使能均成功后，逐台发送轮速 */
    if(failed == 0U)
    {
        for(i = 0U; i < 4U; i++)
        {
            osMessageQueueReset(motor_ack_queue);
            status = App_Chassis_SetOneWheelRpm(i, test_rpm[i]);
            /* 调速命令发出去了 */
            Log_Printf("Motor %u speed TX:%d\r\n", (unsigned int)motors[i]->address, (int)status);
            if(status == HAL_OK)
            {
                status = Motor_WaitAck(motors[i]->address, 0xF6U, ack_timeout);
                /* 调速命令发收到了确认 */
                Log_Printf("Motor %u speed confirm:%d\r\n", (unsigned int)motors[i]->address, (int)status);

            }

            if(status != HAL_OK)
            {
                failed = 1U;
                break;
            }
            osDelay(10);
        }
    }

    if(failed == 0U)
    {
        osDelay(3000);
    }

    /* 正常结束或发送异常，都尝试停止电机 */
    for(i = 0U; i < 4U; i++)
    {
        osMessageQueueReset(motor_ack_queue);
        
        status = App_Chassis_SetOneWheelRpm(i, 0);
        /* 停转命令（转速设为 0）发出去了 */
        Log_Printf("Motor %u stop TX:%d\r\n", (unsigned int)motors[i]->address, (int)status);
        if(status == HAL_OK)
        {
            status = Motor_WaitAck(motors[i]->address, 0xF6U, ack_timeout);
            /* 停转命令收到了确认 */
            Log_Printf("Motor %u stop confirm:%d\r\n",(unsigned int)motors[i]->address, (int)status);
        }
    }
    osDelay(10);
    /* 停稳后读取结束位置，打印每台电机转过的计数 */
    osDelay(200);
    for(i = 0U; i < 4U; i++)
    {
        status = Motor_ReadPosition(motors[i], ack_timeout, &pos_end[i]);
        /* 结束位置减去起始位置，就是这一轮转了多少，括号里是读取函数的返回值 */
        Log_Printf("Motor %u delta:%ld (%d)\r\n", (unsigned int)motors[i]->address, 
                  (long)(pos_end[i] - pos_start[i]), (int)status);
    }
    /* 关闭全部电机驱动 */
    for(i = 0U; i < 4U; i++)
    {
        osMessageQueueReset(motor_ack_queue);
        status = BSP_X42S_Enable(motors[i], 0U);
        /* 关闭使能（松轴）的命令发出去了 */
        Log_Printf("Motor %u disable TX%d", (unsigned int)(i + 1U), (int)status);
        if(status == HAL_OK)
        {
            status = Motor_WaitAck(motors[i]->address, 0xF3U, ack_timeout);
            /* 关闭使能（松轴）的命令收到了确认 */
            Log_Printf("Motor %u disable confirm:%d\r\n", (unsigned int)motors[i]->address,(int)status);
        }
        osDelay(10);
    }
    for(;;)
    {
        osDelay(1000);
    }
}

