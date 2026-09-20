#include "board.h"

/* LED对象 */
static struct bsp_led board_led1;
static struct bsp_led board_led2;
static struct bsp_led board_led3;
BSP_LED_t led1 = &board_led1;
BSP_LED_t led2 = &board_led2;
BSP_LED_t led3 = &board_led3;

/* 串口对象 */
static struct bsp_usart board_debug_uart;
static struct bsp_usart board_motor_uart;
static struct bsp_usart board_camera_uart;
static struct bsp_usart board_gyro_uart;
BSP_USART_t debug_uart  = &board_debug_uart;      /* UART5 调试串口 */
BSP_USART_t motor_uart  = &board_motor_uart;      /* UART4 底盘电机控制 */
BSP_USART_t camera_uart = &board_camera_uart;     /* USART1 摄像头 */
BSP_USART_t gyro_uart   = &board_gyro_uart;       /* USART2 陀螺仪 */

void Board_Init(void)
{
    /* LED对象初始化 */
    BSP_LED_Init(led1, GPIOB, GPIO_PIN_3,LED_ACTIVE_LOW);
    BSP_LED_Init(led2, GPIOB, GPIO_PIN_4,LED_ACTIVE_LOW);
    BSP_LED_Init(led3, GPIOB, GPIO_PIN_5,LED_ACTIVE_LOW);
    
    /* USART对象初始化 */
    BSP_USART_Init(debug_uart,   &huart5);
    BSP_USART_Init(motor_uart,   &huart4);
    BSP_USART_Init(camera_uart,  &huart1);
    BSP_USART_Init(gyro_uart,    &huart2);
}
