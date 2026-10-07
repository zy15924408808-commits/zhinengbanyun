#ifndef __BSP_BNO080_H__
#define __BSP_BNO080_H__

#include "main.h"
#include <stdbool.h>

#define BNO080_MAX_PACKET_SIZE      128
#define BNO080_READ_CHUNK_SIZE      32

#define BNO080_CHANNEL_COMMAND      0
#define BNO080_CHANNEL_EXECUTABLE   1
#define BNO080_CHANNEL_CONTROL      2
#define BNO080_CHANNEL_REPORTS      3
#define BNO080_CHANNEL_WAKE_REPORTS 4
#define BNO080_CHANNEL_GYRO         5

#define BNO080_REPORTID_ACCELEROMETER 0x01       /* 加速度计ID */
#define BNO080_REPORTID_GYROSCOPE     0x02       /* 陀螺仪ID */
#define BNO080_REPORTID_ROTATION_VECTOR 0x05     /* 旋转向量ID */

#define BNO080_Q_POINT_ACCELEROMETER  8          /* 加速度计Q格式小数点位置 */
#define BNO080_Q_POINT_GYROSCOPE      9          /* 陀螺仪Q格式小数点位置 */
#define BNO080_Q_POINT_ROTATION_VECTOR 14          /* 旋转向量Q格式小数点位置 */

/* BNO080对象 */
struct bsp_bno080
{
    I2C_HandleTypeDef  *hi2c;
    uint8_t             addr;          /* 7位I2C地址：0x4B或0x4A */

    GPIO_TypeDef        *rst_port;
    uint16_t            rst_pin;
    GPIO_TypeDef        *int_port;
    uint16_t            int_pin;

    /* SHTP协议缓冲区 */
    uint8_t             shtp_header[4];
    uint8_t             shtp_data[BNO080_MAX_PACKET_SIZE]; /* 暂存从BNO080读取出来的SHTP数据 */
    uint8_t             sequence_number[6]; /* 每个通道独立的包序列号 */

    volatile uint8_t    data_ready_flag; /* INT中断置位, 任务读取后清零 */
    uint8_t             is_initialized;

    /* 解析后的传感器数据（已转换为浮点） */
    float               quat_i, quat_j, quat_k, quat_real;
    uint8_t             quat_accuracy;
    float               accel_x, accel_y, accel_z;
    uint8_t             accel_accuracy;
    float               gyro_x, gyro_y, gyro_z;
    uint8_t             gyro_accuracy;
};
typedef struct bsp_bno080 *BSP_BNO080_t;

/* 初始化BNO080对象：硬件复位 + 软复位 + 验证通信 */
bool BSP_BNO080_Init(BSP_BNO080_t bno, I2C_HandleTypeDef *hi2c, uint8_t addr,
                      GPIO_TypeDef *rst_port, uint16_t rst_pin,
                      GPIO_TypeDef *int_port, uint16_t int_pin);

/* 在HAL_GPIO_EXTI_Callback里调用，仅置位data_ready_flag，不做I2C操作 */
void BSP_BNO080_INT_Callback(BSP_BNO080_t bno);
/* 检查是否有新数据：若有则读包并解析，返回true；否则返回false */
bool BSP_BNO080_DataAvailable(BSP_BNO080_t bno);

/* 使能对应传感器上报，interval_ms为上报间隔(毫秒) */
void BSP_BNO080_EnableRotationVector(BSP_BNO080_t bno, uint16_t interval_ms);
void BSP_BNO080_EnableAccelerometer(BSP_BNO080_t bno, uint16_t interval_ms);
void BSP_BNO080_EnableGyro(BSP_BNO080_t bno, uint16_t interval_ms);

/* 读取已解析好的传感器数据(浮点值) */
void BSP_BNO080_GetQuaternion(BSP_BNO080_t bno, float *i, float *j, float *k, float *real);
void BSP_BNO080_GetAccel(BSP_BNO080_t bno, float *x, float *y, float *z);
void BSP_BNO080_GetGyro(BSP_BNO080_t bno, float *x, float *y, float *z);

#endif /* __BSP_BNO080_H__ */
