#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---- 底盘几何参数（2026-11-17 实测），单位 mm --- */
#define CHASSIS_WHEEL_DIAMETER_MM   77.0f       /* 麦伦直径 */
#define CHASSIS_WHEELBASE_MM        206.0f      /* 前后轮轴心距离 */
#define CHASSIS_TRACK_MM            220.0f      /* 左右轮中心距离 */
#define CHASSIS_COUNTS_PER_REV      65536.0f    /* X42S Emm固件一圈的计数 */

/* ---- 日志开关：1 打开，0 关闭 ---- */
#define APP_LOG_IMU                 0           /* 每次 IMU 变更都打印四元数和航向 */
#define APP_LOG_MOTOR_POS           0           /* 每次收到 0x36 应答都打印位置 */

#endif  /* APP_CONFIG_H */
