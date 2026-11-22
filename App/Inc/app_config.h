#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---- 底盘几何参数（2026-11-17 实测），单位 mm -- -- */
/* 有效轮径:外径实测77mm，临时值79.9mm(2026-11-19在实验室地面用1m直线标定)，比赛场地需要重新标定 */

#define CHASSIS_WHEEL_DIAMETER_MM   79.9f       /* 麦轮直径 */
#define CHASSIS_WHEELBASE_MM        206.0f      /* 前后轮轴心距离 */
#define CHASSIS_TRACK_MM            220.0f      /* 左右轮中心距离 */
#define CHASSIS_COUNTS_PER_REV      65536.0f    /* X42S Emm固件一圈的计数 */

/* ---- 距离修正系数：实际距离 / 里程计距离(实验室地面标定，比赛场地需要重新标定) ---- */
#define CHASSIS_SCALE_X             1.0f        /* 前进：残差约0.8%,暂不修正 */
#define CHASSIS_SCALE_Y             0.944f      /* 横移：2026-11-21三次测试，卷尺/OLED=0.944f*/

/* ---- 里程计测试模式：只改 ODOM_TEST_MODE 这一行 ---- */
#define ODOM_TEST_MODE_FORWARD      0           /* 前进1m */
#define ODOM_TEST_MODE_STRAFE       1           /* 向左横移1m */
#define ODOM_TEST_MODE_SPIN         2           /* 原地左转360° */
#define ODOM_TEST_MODE              ODOM_TEST_MODE_STRAFE 

/* ---- 日志开关：1 打开，0 关闭 ---- */
#define APP_LOG_IMU                 0           /* 每次 IMU 变更都打印四元数和航向 */
#define APP_LOG_MOTOR_POS           0           /* 每次收到 0x36 应答都打印位置 */

#endif  /* APP_CONFIG_H */
