#include "board.h"

/* 当前文件内部使用的电机初始化函数 */
static void Board_X42S_init(void);

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
BSP_USART_t debug_uart  = &board_debug_uart;      /* UART5 调试串口 */
BSP_USART_t motor_uart  = &board_motor_uart;      /* UART4 底盘电机控制 */
BSP_USART_t camera_uart = &board_camera_uart;     /* USART1 摄像头 */
/* UART4 DMA 接收缓冲区，四台电机共用 */
static uint8_t motor_rx_buffer[64];

/* I2C2对象 */
static struct bsp_bno080 board_imu;
BSP_BNO080_t imu = &board_imu;

/* SPI对象 */
static struct bsp_oled board_oled;
BSP_OLED_t oled = &board_oled;

/* 电机对象 */
static struct bsp_x42s board_motor1;
static struct bsp_x42s board_motor2;
static struct bsp_x42s board_motor3;
static struct bsp_x42s board_motor4;
BSP_X42S_t motor1 = &board_motor1;
BSP_X42S_t motor2 = &board_motor2;
BSP_X42S_t motor3 = &board_motor3;
BSP_X42S_t motor4 = &board_motor4;

void Board_Init(void)
{
    /* 开启电机供电 */
    BSP_Power_VM_On();

    /* LED初始化函数 */
    BSP_LED_Init(led1, GPIOB, GPIO_PIN_3,LED_ACTIVE_LOW);
    BSP_LED_Init(led2, GPIOB, GPIO_PIN_4,LED_ACTIVE_LOW);
    BSP_LED_Init(led3, GPIOB, GPIO_PIN_5,LED_ACTIVE_LOW);

    /* USART初始化函数 */
    BSP_USART_Init(debug_uart,   &huart5);
    BSP_USART_Init(motor_uart,   &huart4);
    BSP_USART_Init(camera_uart,  &huart1);
    /* 绑定电机串口的接收缓冲区 */
    BSP_USART_SetRxBuffer(motor_uart, motor_rx_buffer, sizeof(motor_rx_buffer));

    /* IMU初始化 */
    if(BSP_BNO080_Init(imu, &hi2c2, 0x4B, GPIOB, GPIO_PIN_0, GPIOA, GPIO_PIN_4))
    {
        printf("IMU init OK\n");
    }
    else
    {
        printf("IMU init FAIL\n");
    }

    /* OLED初始化 */
    if(BSP_OLED_Init(oled, &hspi1, 
                     OLED_CS_GPIO_Port, OLED_CS_Pin,
                     OLED_DC_GPIO_Port, OLED_DC_Pin,
                     OLED_RST_GPIO_Port, OLED_RST_Pin) == HAL_OK)
    {
        printf("OLED init OK\r\n");
    }
    else
    {
        printf("OLED init FAIL\r\n");
    }

    /*四个电机初始化*/
    Board_X42S_init();

}

static void Board_X42S_init(void)
{
    /* 左前轮：地址1 */
    if(BSP_X42S_Init(motor1, motor_uart, 1U) == HAL_OK)
    {
        printf("X42S motor1 init OK\r\n");
    }
    else
    {
        printf("X42S motor1 init FAIL\r\n");
    }

    /* 右前轮：地址2 */
    if(BSP_X42S_Init(motor2, motor_uart, 2U) == HAL_OK)
    {
        printf("X42S motor2 init OK\r\n");
    }
    else
    {
        printf("X42S motor2 init FAIL\r\n");
    }

    /* 左后轮：地址3 */
    if(BSP_X42S_Init(motor3, motor_uart, 3U) == HAL_OK)
    {
        printf("X42S motor3 init OK\r\n");
    }
    else
    {
        printf("X42S motor3 init FAIL\r\n");
    }

    /* 右后轮：地址4 */
    if(BSP_X42S_Init(motor4, motor_uart, 4U) == HAL_OK)
    {
        printf("X42S motor4 init OK\r\n");
    }
    else
    {
        printf("X42S motor4 init FAIL\r\n");
    }
}

