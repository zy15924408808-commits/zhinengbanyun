#include "odometry.h"
#include <math.h>
#include <stddef.h>

#define ODOM_PI 3.14159265358979f
/*
 * 计算两次编码器读数之差函数
 * 计算编码器当前计数值now与上一次计数值prev之间的差值，并且正确处理计数器回绕（溢出后从最大值回到最小值）的情况。
 * int32_t:返回32位有符号整数
 * now:当前读取的编码器计数值，prev:上一次读取的编码器计数值
*/
int32_t Odom_CountDelta(int32_t now, int32_t prev)
{
    /* 先按无符号相减再转回有符号：回绕时结果仍正确，且避免有符号溢出的未定义行为 */
    return (int32_t)((uint32_t)now - (uint32_t)prev);
}
/*
 * 角度归一化函数
 * 负责把角度差归一化，让机器人能够正确判断应该顺时针旋转还是逆时针旋转(得到最短角度差)，以及需要旋转多少角度。
 * float:返回一个浮点数
 * rad:传入的角度，单位是弧度
*/
float Odom_WrapAngle(float rad)
{
    /* 处理超过π的角度 */
    while(rad > ODOM_PI)
    {
        /* 如果角度大于π就不断减2π(一圈)直到小于π */
        rad -= 2.0f * ODOM_PI;
    }
    /* 处理小于等于-π的角度 */
    while (rad <= -ODOM_PI)
    {
        /* 如果角度小于等于-π就不断加2π(一圈)直到大于-π */
        rad += 2.0f * ODOM_PI;
    }
    return rad;
}
/*
 * 里程计模块初始化函数
 * 根据配置参数初始化里程计对象，计算车轮每个编码器计数对应的实际移动距离，并将机器人的初始位置设置为原点。
 * bool:返回true和false
 * odom:里程计对象指针，保存里程计运行时的数据，cfg:里程计配置结构体指针，提供初始化所需的参数
*/
bool Odom_Init(Odom_t *odom, const Odom_Config_t *cfg)
{
    /* 把机器人初始位置定义为坐标原点 */
    Odom_Pose_t origin = {0.0f, 0.0f, 0.0f};

    if(odom == NULL || cfg == NULL)
    {
        return false;
    }
    /* 参数必须为正，否则后面会除以 0 或算出全 0 的位移 */
    if(cfg->wheel_diameter_mm <= 0.0f || cfg->counts_per_rev <= 0 ||
       cfg->half_wheelbase_mm <= 0.0f || cfg->half_track_mm <= 0.0f)
    {
        return false;
    }
    /* 把外部传入的配置结构体完整复制到里程计对象内部。 */
    odom->cfg = *cfg;
    /* 把外部传入的配置结构体完整复制到里程计对象内部。 */
    odom->cfg = *cfg;
    /* 计算车轮每产生一个编码器计数，机器人理论上移动多少毫米。 */
    odom->mm_per_count = ODOM_PI * cfg->wheel_diameter_mm / (float)cfg->counts_per_rev;
    /* 设置机器人初始位姿 */
    Odom_SetPose(odom, origin, 0.0f);
    return true;
}
/*
 * 机器人的位姿设置函数
 * 设置机器人的位姿，同时建立 IMU 航向角与场地坐标系之间的角度偏置。
 * void
 * odom:需要设置位姿的里程计对象，pose:希望设置的机器人位姿，imu_yaw_rad:当前 IMU 测量到的航向角。
*/
void Odom_SetPose(Odom_t *odom, Odom_Pose_t pose, float imu_yaw_rad)
{
    if(odom == NULL)
    {
        return;
    }
    /* 调用角度归一化函数，将机器人航向角限制（-π，π]，避免角度不断累积变大或变小。*/
    pose.theta_rad = Odom_WrapAngle(pose.theta_rad);
    /* 将传入的机器人位姿完整复制到里程计对象中。*/
    odom->pose = pose;
    /* 计算物理世界和传感器测量角度的偏置。 */
    odom->yaw_offset_rad = Odom_WrapAngle(pose.theta_rad - imu_yaw_rad);
    /* 把当前机器人的初始航向角，作为编码器航向角积分的起点。*/
    odom->enc_theta_rad = pose.theta_rad;
}
/*
 * 更新机器人当前的位置和朝向函数
 * 机器人每运动一段时间，就根据四个车轮的编码器变化量和 IMU 测得的航向角，更新机器人当前的位置和朝向。
 * void
 * odom:里程计对象，保存配置和当前位姿，wheel_delta[4]:四个车轮本次更新期间的编码器计数变化量，imu_yaw_rad:IMU当前测量到的航向角，单位为弧度
*/
void Odom_Update(Odom_t *odom, const int32_t wheel_delta[4], float imu_yaw_rad)
{
    float d[4];         /* 四个车轮本次移动的距离，单位mm */
    float dx_body;      /* 机器人自身坐标系下X方向的位移 */
    float dy_body;      /* 机器人自身坐标系下Y方向的位移 */
    float theta_new;    /* 本次更新得到的最新场地航向角 */
    float theta_mid;    /* 本次运动过程中的平均航向角 */
    uint8_t i;

    if(odom == NULL || wheel_delta == NULL)
    {
        return;
    }
    /* 将编码器计数值转化为车轮移动距离 */
    for(i = 0U; i < 4U; i++)
    {
        d[i] = (float)wheel_delta[i] * odom->mm_per_count;
    }

    /* 麦伦正运动学 */
    /* 四个车轮移动的距离相加除以4就是小车朝X轴移动的距离 */
    dx_body = ( d[0] + d[1] + d[2] + d[3]) / 4.0f;
    /* 朝Y轴移动会有两个对角电机编码器是负值，所有要带负号，然后相加除以4就是小车朝Y轴移动的距离 */
    dy_body = (-d[0] + d[1] + d[2] - d[3]) / 4.0f;
    /* 根据编码器计算旋转角度 */
    odom->enc_theta_rad = Odom_WrapAngle(odom->enc_theta_rad + 
                                        (-d[0] + d[1] - d[2] + d[3]) / 
                                        (4.0f * (odom->cfg.half_wheelbase_mm + odom->cfg.half_track_mm)));

    /* 使用IMU计算最新航向角 */
    theta_new = Odom_WrapAngle(imu_yaw_rad + odom->yaw_offset_rad);
    /* 计算本次运动的平均航向角 */
    theta_mid = odom->pose.theta_rad + Odom_WrapAngle(theta_new - odom->pose.theta_rad) / 2.0f;
    /* 更新机器人在场地坐标系中的X、Y */
    odom->pose.x_mm += dx_body * cosf(theta_mid) - dy_body * sinf(theta_mid);
    odom->pose.y_mm += dx_body * sinf(theta_mid) + dy_body * cosf(theta_mid);
    /* 将经过 IMU 偏移修正得到的航向角，保存到当前位姿中。*/
    odom->pose.theta_rad = theta_new;
}



