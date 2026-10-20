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
    motor_ack_queue = osMessageQueueNew(8U, sizeof(BSP_X42S_Response_t), NULL);
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

    BSP_X42S_Parser_t parser;
    BSP_X42S_Response_t response;

    BSP_X42S_ParserInit(&parser);
    /* 任务运行后再启动 DMA，确保调度器和队列已经运行 */
    status = BSP_USART_StartReceiveToIdle(motor_uart);
    Log_Printf("Motor RX DMA start:%d\r\n", (int)status);

    for (;;)
    {
        if (osMessageQueueGet(motor_rx_queue,&message,NULL,osKernelGetTickFreq()) == osOK)
        {
          for (i = 0U; i < message.len; i++)
          {
            if (BSP_X42S_ParseByte(&parser,message.data[i],&response))
            {
                /* 把完整应答交给控制任务 */
                if(osMessageQueuePut(motor_ack_queue, &response, 0U, 0U) != osOK)
                {
                    Log_Printf("Motor ACK queue put FAIL\r\n");
                }
                Log_Printf("Motor ACK addr:%u function:0x%02X status:0x%02X\r\n",
                      (unsigned int)response.address,
                      (unsigned int)response.function,
                      (unsigned int)response.status);
            }
          }
        }
        else
        {
            Log_Printf("RX state:0x%02X remaining:%lu error:0x%08lX\r\n",
                   (unsigned int)motor_uart->huart->RxState,
                   (unsigned long)__HAL_DMA_GET_COUNTER(
                       motor_uart->huart->hdmarx),
                   (unsigned long)HAL_UART_GetError(
                       motor_uart->huart));

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

/*等待指定地址、功能码的应答：超时单位为内核 tick */
static HAL_StatusTypeDef Motor_WaitAck(uint8_t address, uint8_t function, uint32_t timeout_ticks)
{
    BSP_X42S_Response_t response;
    osStatus_t queue_status;
    uint32_t start_tick;
    uint32_t elapsed;

    if(motor_ack_queue == NULL || address == 0U || timeout_ticks == 0U)
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
            return (queue_status == osErrorTimeout) ? HAL_TIMEOUT :HAL_ERROR;
        }
        /* 不是正在等待的应答，继续等待 */
        if(response.address != address || response.function != function)
        {
            continue;
        }

        return (response.status == 0x02U) ? HAL_OK : HAL_ERROR;
    }
}

void App_ControlTask(void *argument)
{
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
        Log_Printf("Motor %u enable TX:%d\r\n",(unsigned int)(i + 1U), (int)status);
        if(status == HAL_OK)
        {
            status = Motor_WaitAck(motors[i]->address, 0xF3U, ack_timeout);
            Log_Printf("Motor %u enable confirm:%d\r\n", (unsigned int)motors[i]->address, (int)status);
        }
        if(status != HAL_OK)
        {
            failed = 1U;
            break;
        }
        osDelay(10);
    }
    /* 根据运动分量计算轮速：先验证左横移 */
    status = App_Chassis_CalcWheelRpm(30, 30, 0, test_rpm);
    if(status != HAL_OK)
    {
        failed = 1U;
        Log_Printf("Chassis wheel calculation FAIL\r\n");
    }
    else
    {
        Log_Printf("Wheel RPM: %d %d %d %d\r\n", (int)test_rpm[0], (int)test_rpm[1], (int)test_rpm[2], (int)test_rpm[3]);
    }
    /* 计算和使能均成功后，逐台发送轮速 */
    if(failed == 0U)
    {
        for(i = 0U; i < 4U; i++)
        {
            osMessageQueueReset(motor_ack_queue);
            status = App_Chassis_SetOneWheelRpm(i, test_rpm[i]);
            Log_Printf("Motor %u speed TX:%d\r\n", (unsigned int)motors[i]->address, (int)status);
            if(status == HAL_OK)
            {
                status = Motor_WaitAck(motors[i]->address, 0xF6U, ack_timeout);
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
        Log_Printf("Motor %u stop TX:%d\r\n", (unsigned int)motors[i]->address, (int)status);
        if(status == HAL_OK)
        {
            status = Motor_WaitAck(motors[i]->address, 0xF6U, ack_timeout);
            Log_Printf("Motor %u stop confirm:%d\r\n",(unsigned int)motors[i]->address, (int)status);

        }
    }
    osDelay(10);

    /* 关闭全部电机驱动 */
    for(i = 0U; i < 4U; i++)
    {
        osMessageQueueReset(motor_ack_queue);
        status = BSP_X42S_Enable(motors[i], 0U);
        Log_Printf("Motor %u disable TX%d\r\n", (unsigned int)(i + 1U), (int)status);
        if(status == HAL_OK)
        {
            status = Motor_WaitAck(motors[i]->address, 0xF3U, ack_timeout);
            Log_Printf("Motor %u disable confirm:%d\r\n", (unsigned int)motors[i]->address,(int)status);
        }
        osDelay(10);
    }
    for(;;)
    {
        osDelay(1000);
    }
}

