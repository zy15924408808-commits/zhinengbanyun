#include "app_tasks.h"
#include "board.h"
#include "cmsis_os.h"
#include "log.h"
#include "app_chassis.h"
#include "app_config.h"
#include "app_imu.h"
#include "odometry.h"
#include "app_status.h"
#include <stdbool.h>
#include <string.h>

typedef struct
{
  uint16_t len;
  uint8_t data[64];
}MotorRxChunk_t;

static osMessageQueueId_t motor_rx_queue;
static osMessageQueueId_t motor_ack_queue;          /* 保存电机应答消息的队列 */
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

    /* 创建消息队列 */
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
                /* 将电机的应答消息放到motor_ack_queue队列 */
                if(osMessageQueuePut(motor_ack_queue, &response, 0U, 0U) != osOK)
                {
                    /* 队列满，解析好的回复放不进队列 */
                    Log_Printf("Motor ACK queue put FAIL\r\n");
                }
                if(response.function == X42S_FUNC_READ_POS)
                {
#if APP_LOG_MOTOR_POS
                    /* 收到0x36读位置的回复。addr是电机地址(1,2,3,4)，pos 是累计位置，65536 为一圈 */
                    Log_Printf("Motor POS addr:%u pos:%ld\r\n", (unsigned int)response.address,(long)response.position);
#endif
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

/* ---- 直线里程计测试参数 ---- */
#define MOTOR_ACK_TIMEOUT_MS    200U        /* 等待单条电机应答的上限 */
#define IMU_MAX_AGE_MS          300U        /* IMU 航向超过这个时间没更新就认为失效 */

#define ODOM_PERIOD_MS          20U         /* 里程计更新周期 */
#define ODOM_LOG_EVERY          10U         /* 每10个周期（200ms）打印一次位姿 */

/* 三种测试的运动分量和超时由 ODOM_TEST_MODE 决定 */
#if ODOM_TEST_MODE == ODOM_TEST_MODE_FORWARD
#define ODOM_TEST_CMD_FORWARD   60          /* 前进转速，约250mm/s */
#define ODOM_TEST_CMD_LEFT      0
#define ODOM_TEST_CMD_TURN      0
#define ODOM_TEST_TIMEOUT_MS    8000U       /* 正常约4s走完，超时强制停车 */
#elif ODOM_TEST_MODE == ODOM_TEST_MODE_STRAFE
#define ODOM_TEST_CMD_FORWARD   0
#define ODOM_TEST_CMD_LEFT      60          /* 向左横移，理想速度约250mm/s */
#define ODOM_TEST_CMD_TURN      0
#define ODOM_TEST_TIMEOUT_MS    8000U
#else
#define ODOM_TEST_CMD_FORWARD   0
#define ODOM_TEST_CMD_LEFT      0
#define ODOM_TEST_CMD_TURN      60          /* 原地左转，理想约67度/秒，转一圈约5.4s */
#define ODOM_TEST_TIMEOUT_MS    9000U
#endif
#define ODOM_TEST_TARGET_MM     1000.0f     /* 前进、横移走到这个距离就停 */
#define ODOM_TEST_TARGET_RAD    6.2831853f  /* 原地旋转转到这个角度就停：360° */
#define RAD_TO_DECIDEG          572.957795f /* 弧度换算成0.1度 */

/* 
 * 依次控制四台X42S电机使能或送轴，并且每控制一台电机，都要等待该电机返回应答，确认操作是否成功
 */
static HAL_StatusTypeDef Motor_SetEnableAll(uint8_t enable)
{
    /* 用于保存整个函数最终的执行结果 */
    HAL_StatusTypeDef result = HAL_OK;
    /* 用于保存当前正在操作的那一台电机的执行结果 */
    HAL_StatusTypeDef status;
    uint8_t i;

    for(i = 0U; i < 4U; i++)
    {
        /* 清空应答队列 */
        osMessageQueueReset(motor_ack_queue);
        /* 发送电机使能指令 */
        status = BSP_X42S_Enable(app_chassis.motor[i], enable);
        /* 等待电机应答 */
        if(status == HAL_OK)
        {
            status = Motor_WaitAck(app_chassis.motor[i]->address, X42S_FUNC_ENABLE, MOTOR_ACK_TIMEOUT_MS);
        }
        /* 没收到应答 */
        if(status != HAL_OK)
        {
            Log_Printf("Motor %u %s FAIL:%d\r\n", (unsigned int)app_chassis.motor[i]->address,
                      (enable != 0U) ? "enable" : "disable", (int)status);
            /* 只记录第一次出现的错误 */
            if(result == HAL_OK)
            {
                result = status;
            }
        }
        osDelay(10);
    }
    return result;
}

/* 
 * 依次给四台麦克纳姆轮电机发送目标转速，并等待每台电机返回速度指令应答。如果发生错误，根据stop_on_error决定是否继续发送剩余电机的指令
 * rpm[4]:四台电机的目标转速,stop_on_error:发送错误后是否停止,stop_on_error这个的目的是停车时即使某一台电机发送停车指令失败，也要继续尝试给其他三台电机发送停车指令，尽可能让整个底盘停下来。
 */
static HAL_StatusTypeDef Chassis_SendWheelRpm(const int16_t rpm[4], bool stop_on_error)
{
    /* 保存整个函数最终的执行结果 */
    HAL_StatusTypeDef result = HAL_OK;
    /* 保存当前这台电机的执行结果 */
    HAL_StatusTypeDef status;
    uint8_t i;

    for(i = 0U; i < 4U; i++)
    {
        /* 清空电机应答队列中尚未读取的消息 */
        osMessageQueueReset(motor_ack_queue);
        /* 发送当前电机的目标转速 */
        status = App_Chassis_SetOneWheelRpm(i, rpm[i]);
        /* 等待电机返回速度应答 */
        if(status == HAL_OK)
        {
            status = Motor_WaitAck(app_chassis.motor[i]->address, X42S_FUNC_SPEED, MOTOR_ACK_TIMEOUT_MS);
        }
        if(status != HAL_OK)
        {
            Log_Printf("Motor %u speed FAIL:%d\r\n", (unsigned int)app_chassis.motor[i]->address, (int)status);
            if(result == HAL_OK)
            {
                result = status;
            }
            if(stop_on_error)
            {
                break;
            }
        }
        osDelay(10);
    }
    return result;
}

/* 
 * 依次读取四个电机的当前位置，并将读取结果分别保存到pos[0]~pos[3]中
 */
static HAL_StatusTypeDef Motor_ReadAllPositions(int32_t pos[4])
{
    /* 保存当前电机的位置读取结果 */
    HAL_StatusTypeDef status;
    uint8_t i;

    for(i = 0U; i < 4U; i++)
    {
        /* 读取电机位置 */
        status = Motor_ReadPosition(app_chassis.motor[i], MOTOR_ACK_TIMEOUT_MS, &pos[i]);
        if(status != HAL_OK)
        {
            Log_Printf("Motor %u read pos FAIL:%d\r\n", (unsigned int)app_chassis.motor[i]->address, (int)status);
            return status;
        }
    }
    return HAL_OK;
}

/* 
 * 在机器人停止运动时，再读取一次四个电机的编码器位置和IMU航向角，计算这段时间四个轮子实际移动的距离(编码器计数差值)，然后更新机器人的里程计数据。
 * odom:里程计对象，保存机器人的位置、姿态等信息,last_pose[4]:保存上一次读取的四个电机编码器位置。
 */
static HAL_StatusTypeDef Chassis_OdomStep(Odom_t *odom, int32_t last_pose[4])
{
    /* 保存当前四个电机的编码器位置 */
    int32_t now_pos[4];
    /* 保存四个电机本次运动的计数增量 */
    int32_t delta[4];
    /* 保存IMU当前航向角，单位为弧度 */
    float yaw_rad;
    /* 保存函数执行状态 */
    HAL_StatusTypeDef status;
    uint8_t i;
    /* 读取四个电机当前的编码器位置 */
    status = Motor_ReadAllPositions(now_pos);
    if(status != HAL_OK)
    {
        return status;
    }
    /* 获取IMU航向角 */
    if(!App_Imu_GetYawRad(&yaw_rad, IMU_MAX_AGE_MS))
    {
        Log_Printf("IMU yaw stale\r\n");
        return HAL_ERROR;
    }
    /* 计算四个轮子的编码器增量 */
    for(i = 0U; i < 4U; i++)
    {
        /* 先计算编码器位置变化量 */
        /* 然后统一四个轮子的前进方向 */
        delta[i] = App_Chassis_ToForwardCount(i, Odom_CountDelta(now_pos[i], last_pose[i]));
        /* 更新上一次的位置记录 */
        last_pose[i] = now_pos[i];
    }
    /* 更新里程计 */
    Odom_Update(odom, delta, yaw_rad);
    return HAL_OK;
}

/* 打印位姿：坐标取整到mm，角度单位0.1度，避免在控制任务里格式化浮点数 */
static void Odom_LogPose(const char *tag, const Odom_t *odom)
{
    Log_Printf("Odom %s x:%ld y:%ld th:%ld enc_th:%ld\r\n", tag, 
              (long)odom->pose.x_mm, (long)odom->pose.y_mm,
              (long)(odom->total_theta_rad * RAD_TO_DECIDEG),
              (long)(odom->total_enc_theta_rad * RAD_TO_DECIDEG));
}
/* 测试是否已经达到目标：前进看x、横移看y，原地旋转看IMU累计转角 */
static bool Odom_TestReached(const Odom_t *odom)
{
#if ODOM_TEST_MODE == ODOM_TEST_MODE_FORWARD
    return odom->pose.x_mm >= ODOM_TEST_TARGET_MM;
#elif ODOM_TEST_MODE == ODOM_TEST_MODE_STRAFE
     return odom->pose.y_mm >= ODOM_TEST_TARGET_MM;
#else
    return odom->total_theta_rad >= ODOM_TEST_TARGET_RAD;
#endif
}
/* 把当前位姿和测试结果发布给UI任务，由UI任务画到OLED上 */
static void Odom_Report(App_OdomTestState_t state, const Odom_t *odom, HAL_StatusTypeDef result, uint32_t cycles, uint32_t overruns)
{
    App_OdomReport_t report;

    report.state = state;
    report.x_mm = (int32_t)odom->pose.x_mm;
    report.y_mm = (int32_t)odom->pose.y_mm;
    report.th_decideg = (int32_t)(odom->total_theta_rad * RAD_TO_DECIDEG);
    report.enc_th_decideg = (int32_t)(odom->total_enc_theta_rad * RAD_TO_DECIDEG);
    report.result = (int32_t)result;
    report.cycles = cycles;
    report.overruns = overruns;

    App_Status_PublishOdom(&report);
}
/* 底盘里程计测试控制任务
 * 初始化底盘和里程计,等待电机和IMU准备就绪,获取四个电机的初始位置,控制机器人直线向前运动,
 * 周期性读取电机位置计算机器人实际移动距离,达到目标距离或发生异常时停车,最后输出里程计测试结果
*/
void App_ControlTask(void *argument)
{
    static const Odom_Config_t odom_cfg = {
        CHASSIS_WHEEL_DIAMETER_MM,
        CHASSIS_WHEELBASE_MM / 2.0f,
        CHASSIS_TRACK_MM / 2.0f,
        CHASSIS_COUNTS_PER_REV
    };
    /* 定义机器人初始位姿 */
    const Odom_Pose_t origin = {0.0f, 0.0f, 0.0f};
    /* 四个电机的停车转速 */
    const int16_t zero_rpm[4] = {0, 0, 0, 0};
    /* 保存运动学计算得到的四轮目标转速 */
    int16_t wheel_rpm[4] = {0, 0, 0, 0};
    /* 保存上一次读取的四个电机位置 */
    int32_t last_pos[4] = {0, 0, 0, 0};
    /* 里程计对象，保存配置和机器人位姿等信息 */
    Odom_t odom;
    /* IMU获取的航向角 */
    float yaw_rad = 0.0f;
    /* 测试开始时的系统时间 */
    uint32_t start_tick;
    /* 下一次周期执行的目标时间 */
    uint32_t next_tick;
    /* 里程计更新次数 */
    uint32_t cycles = 0U;
    /* 周期执行超时次数 */
    uint32_t overruns = 0U;
    /* 保存函数执行状态 */
    HAL_StatusTypeDef status = HAL_OK;
    /* 标记是否已经开始运动 */
    bool moving = false;

    (void)argument;
    memset(&odom, 0, sizeof(odom));
    /* 初始化底盘 */
    App_Chassis_Init();

    /* 等待电机上电、IMU 开始出数据 */
    osDelay(2000);
    /* 初始化里程计 */
    if(!Odom_Init(&odom, &odom_cfg))
    {
        Log_Printf("Odom init FAIL\r\n");
        status = HAL_ERROR;
    }

    /* 检查IMU航向角，没有有效航向就不发车 */
    if(status == HAL_OK && !App_Imu_GetYawRad(&yaw_rad, IMU_MAX_AGE_MS))
    {
        Log_Printf("IMU yaw not ready, test aborted\r\n");
        status = HAL_ERROR;
    }
    /* 使能电机 */
    if(status == HAL_OK)
    {
        status = Motor_SetEnableAll(1U);
    }
    /* 读取四个电机当前的编码器位置 */
    if(status == HAL_OK)
    {
        status = Motor_ReadAllPositions(last_pos);
    }
    /* 设置初始位姿并计算四轮转速 */
    if(status == HAL_OK)
    {
        /* 设置里程计初始位姿 */
        Odom_SetPose(&odom, origin, yaw_rad);
        /* 计算四个轮子的目标转速 */
        status = App_Chassis_CalcWheelRpm(ODOM_TEST_CMD_FORWARD, ODOM_TEST_CMD_LEFT, ODOM_TEST_CMD_TURN, wheel_rpm);
    }
    /* 启动机器人 */
    if(status == HAL_OK)
    {
        Log_Printf("Odom test start: mode %d\r\n", (int)ODOM_TEST_MODE);
        moving = true;
        Odom_Report(APP_ODOM_TEST_RUN, &odom, HAL_OK, 0U, 0U);
        /* 将四个轮子的目标转速发送给电机 */
        status = Chassis_SendWheelRpm(wheel_rpm, true);
    }

    /* 固定周期更新里程计：走够距离、超时或通信失败都会退出 */
    start_tick = osKernelGetTickCount();
    next_tick = start_tick;
    while(status == HAL_OK)
    {
        next_tick += ODOM_PERIOD_MS;
        if(osDelayUntil(next_tick) != osOK)
        {
            /* 上一周期耗时超过 ODOM_PERIOD_MS，记一次超时 */
            overruns++;
        }
        /* 更新里程计 */
        status = Chassis_OdomStep(&odom, last_pos);
        if(status != HAL_OK)
        {
            break;
        }
        /* 记录更新次数 */
        cycles++;
        /* 定期打印机器人位姿 */
        if((cycles % ODOM_LOG_EVERY) == 0U)
        {
            Odom_LogPose("run", &odom);
            Odom_Report(APP_ODOM_TEST_RUN, &odom, HAL_OK, cycles, overruns);
        }
        /* 判断是否达到目标距离 */
        if(Odom_TestReached(&odom))
        {
            break;
        }
        /* 判断是否超时 */
        if((osKernelGetTickCount() - start_tick) >= ODOM_TEST_TIMEOUT_MS)
        {
            Log_Printf("Odom test timeout\r\n");
            status = HAL_TIMEOUT;
        }
    }

    /* 不论成功失败都停车：每台都尝试，不因单台失败中断 */
    (void)Chassis_SendWheelRpm(zero_rpm, false);

    if(moving)
    {
        /* 停稳后再更新一次，把刹车滑行的距离也算进去 */
        osDelay(200);
        if(Chassis_OdomStep(&odom, last_pos) == HAL_OK)
        {
            Odom_LogPose("final", &odom);
        }
    }

    Log_Printf("Odom test result:%d cycles:%lu overruns:%lu\r\n",
               (int)status, (unsigned long)cycles, (unsigned long)overruns);
    Odom_Report(APP_ODOM_TEST_DONE, &odom, status, cycles, overruns);
    (void)Motor_SetEnableAll(0U);

    for(;;)
    {
        osDelay(1000);
    }
}
