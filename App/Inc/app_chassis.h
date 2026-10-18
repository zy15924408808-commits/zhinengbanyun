#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "bsp_x42s.h"
#include <stdint.h>

typedef struct
{
    BSP_X42S_t motor[4];

    /* 没个轮子朝车体前进方向转动时使用的 Emm 方向 */
    BSP_X42S_Direction_t forward_direction[4];
}App_Chassis_t;

/* 全局底盘对象 */
extern App_Chassis_t app_chassis;

/* 绑定四台已经初始化的 X42S */
void App_Chassis_Init(void);

/* 设置四个轮子的带符号转速，正数表示车体前进方向 */
HAL_StatusTypeDef App_Chassis_SetWheelRpm(int16_t left_front_rpm, int16_t right_front_rpm, int16_t left_rear_rpm, int16_t right_rear_rpm);

/* 四轮速度全部设为 0 */
HAL_StatusTypeDef App_Chassis_Stop(void);

/* 设置指定轮子的带符号转速，index: 0左前 1右前 2左后 3右后 */
HAL_StatusTypeDef App_Chassis_SetOneWheelRpm(uint8_t index, int16_t rpm);

#endif /* APP_CHASSIS_H */
