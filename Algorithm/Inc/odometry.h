#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* 底盘几何参数，单位 mm */
typedef struct
{
    float wheel_diameter_mm;    /* 麦伦直径 */
    float half_wheelbase_mm;    /* 前后轮轴距的一半 */
    float half_track_mm;        /* 左右轮距的一半 */
    float counts_per_rev;       /* 电机转一圈的编码器计数：Emm 固件为 65536 */
}Odom_Config_t;

/* 场地坐标系下的位姿：逆时针转角为正 */
typedef struct
{
    float x_mm;         /* 机器人在X轴方向的位置 */
    float y_mm;         /* 机器人在Y轴方向的位置 */
    float theta_rad;    /* 机器人当前的旋转角度（朝向） */
}Odom_Pose_t;

/* 里程计状态 */
typedef struct
{
    Odom_Config_t cfg;      /* 保存里程计配置参数 */
    float mm_per_count;     /* 编码器每个计数对应的移动距离 */
    Odom_Pose_t pose;       /* 保存机器人的位置和朝向 */
    float yaw_offset_rad;   /* IMU航向角的偏置 */
    float enc_theta_rad;    /* 仅根据编码器计算的累计航向角，用来和IMU对比、判断打滑 */
}Odom_t;

/* 计算两次编码器读数之差，溢出回绕时也正确 */
int32_t Odom_CountDelta(int32_t now, int32_t prev);

/* 把角度规范到（-pi,pi] */
float Odom_WrapAngle(float rad);

/* 检查参数并初始化：参数不合法返回 false */
bool Odom_Init(Odom_t *odom, const Odom_Config_t *cfg);

/* 设定当前位姿（上电起点、视觉校正都用它）：imu_yaw_rad 为此刻 IMU 读数 */
void Odom_SetPose(Odom_t *odom, Odom_Pose_t pose, float imu_yaw_rad);

/* 用四个轮子的计数增量和当前 IMU 航向更新位姿
 * wheel_delta 顺序：左前、右前、左后、右后：已换算为“车体前进方向为正”
*/
void Odom_Update(Odom_t *odom, const int32_t wheel_delta[4], float imu_yaw_rad);

#endif /* ODOMETRY_H */

