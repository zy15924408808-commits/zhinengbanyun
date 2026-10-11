/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "board.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t len;
  uint8_t data[64];
}MotorRxChunk_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static osMessageQueueId_t motor_rx_queue;
static volatile uint32_t motor_rx_drop_count = 0U;
/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskControl */
osThreadId_t TaskControlHandle;
const osThreadAttr_t TaskControl_attributes = {
  .name = "TaskControl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UITask */
osThreadId_t UITaskHandle;
const osThreadAttr_t UITask_attributes = {
  .name = "UITask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for MotorRxTask */
osThreadId_t MotorRxTaskHandle;
const osThreadAttr_t MotorRxTask_attributes = {
  .name = "MotorRxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void MotorUartRxCallback(uint8_t *data, uint16_t len);
void StartMotorRxTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartControl(void *argument);
void StartSensor(void *argument);
void StartUI(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    Board_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* 最多暂存8个接收数据块 */
  motor_rx_queue = osMessageQueueNew(8U, sizeof(MotorRxChunk_t), NULL);
  if(motor_rx_queue == NULL)
  {
    Error_Handler();
  }
  else
  {
    BSP_USART_SetRxCallback(motor_uart, MotorUartRxCallback);
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TaskControl */
  TaskControlHandle = osThreadNew(StartControl, NULL, &TaskControl_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensor, NULL, &SensorTask_attributes);

  /* creation of UITask */
  UITaskHandle = osThreadNew(StartUI, NULL, &UITask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    /* creation of MotorRxTask */
  MotorRxTaskHandle = osThreadNew(StartMotorRxTask, NULL, &MotorRxTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControl */
/**
* @brief Function implementing the TaskControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControl */
void StartControl(void *argument)
{
  /* USER CODE BEGIN StartControl */
  BSP_X42S_t motors[4] = {motor1, motor2, motor3, motor4};
  /* 各轮朝车体前进方向滚动时的电机方向 */
  BSP_X42S_Direction_t forward_direction[4] =
  {
    BSP_X42S_CW,
    BSP_X42S_CCW,
    BSP_X42S_CW,
    BSP_X42S_CCW,
  };

  HAL_StatusTypeDef status;
  uint8_t i;
  uint8_t failed;

  /* 等待电机上电启动 */
  osDelay(2000);
  failed = 0U;

  /*先使能四台电机*/
  for(i = 0U; i < 4U; i++)
  {
    status = BSP_X42S_Enable(motors[i], 1U);
    printf("Motor %u enable TX:%d\r\n",(unsigned int)(i + 1U), (int)status);

    if(status != HAL_OK)
    {
      failed = 1U;
      break;
    }
    osDelay(10);
  }
  /*全部使能发送成功后，下发前进速度*/
  if(failed == 0U)
  {
    for(i = 0U; i < 4U; i++)
    {
      status = BSP_X42S_SetSpeed(motors[i], forward_direction[i], 60U, 0U);
      printf("Motor %u speed TX:%d\r\n", (unsigned int)(i + 1), status);

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
    status = BSP_X42S_SetSpeed(motors[i], forward_direction[i], 0U, 0U);
    printf("Motor %u stop TX:%d\r\n", (unsigned int)(i + 1U), (int)status);
    osDelay(10);
  }
  osDelay(10);

  /* 关闭全部电机驱动 */
  for(i = 0U; i < 4U; i++)
  {
    status = BSP_X42S_Enable(motors[i], 0U);
    printf("Motor %u disable TX%d\r\n", (unsigned int)(i + 1U), (int)status);
    osDelay(10);
  }
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartControl */
}

void StartMotorRxTask(void *argument)
{
    MotorRxChunk_t message;
    HAL_StatusTypeDef status;
    uint16_t i;

    /* 任务运行后再启动 DMA，确保调度器和队列已经运行 */
    status = BSP_USART_StartReceiveToIdle(motor_uart);
    printf("Motor RX DMA start:%d\r\n", (int)status);

    for (;;)
    {
        if (osMessageQueueGet(
                motor_rx_queue,
                &message,
                NULL,
                osKernelGetTickFreq()) == osOK)
        {
            printf("Motor RX len:%u data:",
                   (unsigned int)message.len);

            for (i = 0U; i < message.len; i++)
            {
                printf(" %02X", message.data[i]);
            }

            printf("\r\n");
        }
                else
        {
            printf("RX state:0x%02X remaining:%lu error:0x%08lX\r\n",
                   (unsigned int)motor_uart->huart->RxState,
                   (unsigned long)__HAL_DMA_GET_COUNTER(
                       motor_uart->huart->hdmarx),
                   (unsigned long)HAL_UART_GetError(
                       motor_uart->huart));

            printf("RX restart:%d dropped:%lu\r\n",
                   (int)motor_uart->rx_restart_status,
                   (unsigned long)motor_rx_drop_count);
        }
    }
}

/* USER CODE BEGIN Header_StartSensor */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensor */
void StartSensor(void *argument)
{
  /* USER CODE BEGIN StartSensor */
  float qi, qj, qk, qreal;

  BSP_BNO080_EnableRotationVector(imu, 100);
  /* Infinite loop */
  for(;;)
  {
   if(BSP_BNO080_DataAvailable(imu))
   {
       BSP_BNO080_GetQuaternion(imu, &qi, &qj, &qk, &qreal);
       printf("Quaternion: i=%.3f j=%.3f k=%.3f real=%.3f\r\n", qi, qj, qk, qreal);
   }
    osDelay(1000);
  }
  /* USER CODE END StartSensor */
}

/* USER CODE BEGIN Header_StartUI */
/**
* @brief Function implementing the UITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUI */
void StartUI(void *argument)
{
  /* USER CODE BEGIN StartUI */
  /* Infinite loop */
  BSP_OLED_Fill(oled, 0x00);

  BSP_OLED_DrawString(oled, 0U, 0U, "OLED TEST");
  BSP_OLED_DrawString(oled, 0U, 54U, "IMU OK");
  BSP_OLED_DrawString(oled, 2U, 0U, "X42S");

  if (BSP_OLED_Update(oled) != HAL_OK)
  {
    printf("OLED update FAIL\r\n");
  }

    for(;;)
    {
        BSP_LED_On(led1);
        osDelay(1000);
        BSP_LED_Off(led1);
        BSP_LED_On(led2);
        osDelay(1000);
        BSP_LED_Off(led2);
        BSP_LED_On(led3);
        osDelay(1000);
        BSP_LED_Off(led3);
    }
  /* USER CODE END StartUI */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 在接收中断中执行：复制数据并入队 */
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
/* USER CODE END Application */

