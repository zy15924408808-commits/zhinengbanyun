#include "app_imu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stddef.h>

#define APP_IMU_DEG_TO_RAD  (3.14159265f / 180.f)

/* 传感器任务写、控制任务读：三个变量必须一起读写，用临界区保护 */
static float imu_yaw_rad = 0.0f;        /* 保存IMU测量的角度 */
static uint32_t imu_yaw_tick = 0U;      /* 记录这份数据是什么时候更新的 */
static bool imu_yaw_valid = false;      /* 判断是否已经收到过有效数据 */
/* 发布IMU的航向角数据函数
 * 将IMU测量到的角度转换成弧度，并更新共享数据
 */
void App_Imu_PublishYawDeg(float yaw_deg)
{
    /* 获取当前系统时间 */
    uint32_t now = osKernelGetTickCount();
    /* 进入临界区 */
    /* 有两个任务IMU传感器任务(写数据),底盘控制任务(读数据)都会共享这三个数据，防止两个任务读取到不同时段的数据 */
    taskENTER_CRITICAL();
    /* 角度转换 */
    imu_yaw_rad = yaw_deg * APP_IMU_DEG_TO_RAD;
    /* 更新时间 */
    imu_yaw_tick = now;
    /* 设置标志位 */
    imu_yaw_valid = true;
    /* 退出临界区 */
    taskEXIT_CRITICAL();
}
/* 获取IMU的航向角数据函数
 * 读取IMU保存的航向角，并判断这份数据是否仍然有效
 * yaw_rad:接收航向角的指针，max_age_ms:允许数据存在的最大时间
 */
bool App_Imu_GetYawRad(float *yaw_rad, uint32_t max_age_ms)
{
    float yaw;
    uint32_t tick;
    bool valid;

    if(yaw_rad == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    yaw = imu_yaw_rad;
    tick = imu_yaw_tick;
    valid = imu_yaw_valid;
    taskEXIT_CRITICAL();

    /* 内核 1 tick = 1 ms；数据太旧说明传感器任务停掉了或 IMU 掉线 */
    if(!valid || (osKernelGetTickCount() - tick) > max_age_ms)
    {
        return false;
    }

    *yaw_rad = yaw;
    return true;
}
