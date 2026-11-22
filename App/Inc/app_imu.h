#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdbool.h>
#include <stdint.h>

/* 发布最新航向角（传感器任务调度），输入单位为度 */
void App_Imu_PublishYawDeg(float yaw_deg);

/* 读取最新航向角，单位为弧度：从未发布过或超过 max_age_ms 没有更新时返回 false */
bool App_Imu_GetYawRad(float *yaw_rad, uint32_t max_age_ms);

#endif /* APP_IMU_H */
