#include "app_chassis.h"
#include "board.h"

App_Chassis_t app_chassis;

static HAL_StatusTypeDef App_Chassis_SetOneWheel(BSP_X42S_t motor, BSP_X42S_Direction_t forward_direction, int16_t rpm)
{
    BSP_X42S_Direction_t direction;
    uint16_t speed_rpm;
    int32_t magnitude;

    if(motor == NULL)
    {
        return HAL_ERROR;
    }

    if(rpm >= 0)
    {
        direction = forward_direction;
        magnitude = rpm;
    }
    else
    {
        direction = (forward_direction == BSP_X42S_CW) ? BSP_X42S_CCW : BSP_X42S_CW;
        magnitude = -(int32_t)rpm;
    }

    if(magnitude > 3000)
    {
        return HAL_ERROR;
    }

    speed_rpm = (uint16_t)magnitude;

    return BSP_X42S_SetSpeed(motor, direction, speed_rpm, 0U);
}

void App_Chassis_Init(void)
{
    app_chassis.motor[0] = motor1;
    app_chassis.motor[1] = motor2;
    app_chassis.motor[2] = motor3;
    app_chassis.motor[3] = motor4;

    /* 左前、右前、左后、右后 */
    app_chassis.forward_direction[0] = BSP_X42S_CW;
    app_chassis.forward_direction[1] = BSP_X42S_CCW;
    app_chassis.forward_direction[2] = BSP_X42S_CW;
    app_chassis.forward_direction[3] = BSP_X42S_CCW;
}

HAL_StatusTypeDef App_Chassis_SetWheelRpm(int16_t left_front_rpm, int16_t right_front_rpm, int16_t left_rear_rpm, int16_t right_rear_rpm)
{
    HAL_StatusTypeDef status;

    status = App_Chassis_SetOneWheel(app_chassis.motor[0], app_chassis.forward_direction[0], left_front_rpm);
    if (status != HAL_OK)
    {
        return status;
    }

    status = App_Chassis_SetOneWheel(app_chassis.motor[1], app_chassis.forward_direction[1], right_front_rpm);
    if (status != HAL_OK)
    {
        return status;
    }
    
    status = App_Chassis_SetOneWheel(app_chassis.motor[2], app_chassis.forward_direction[2], left_rear_rpm);
    if (status != HAL_OK)
    {
        return status;
    }

    return App_Chassis_SetOneWheel(app_chassis.motor[3], app_chassis.forward_direction[3], right_rear_rpm);
}    

HAL_StatusTypeDef App_Chassis_SetOneWheelRpm(uint8_t index, int16_t rpm)
{
    if(index >= 4U)
    {
        return HAL_ERROR;
    }

    return App_Chassis_SetOneWheel(app_chassis.motor[index], app_chassis.forward_direction[index],rpm);
}

HAL_StatusTypeDef App_Chassis_Stop(void)
{
    return App_Chassis_SetWheelRpm(0, 0, 0, 0);
}

HAL_StatusTypeDef App_Chassis_CalcWheelRpm(int16_t forward_rpm, int16_t left_rpm, int16_t turn_rpm, int16_t wheel_rpm[4])
{
    int32_t rpm[4];
    uint8_t i;

    if(wheel_rpm == NULL)
    {
        return HAL_ERROR;
    }

    /* 顺序：左前、右前、左后、右后 */
    rpm[0] = (int32_t)forward_rpm - left_rpm - turn_rpm;
    rpm[1] = (int32_t)forward_rpm + left_rpm + turn_rpm;
    rpm[2] = (int32_t)forward_rpm + left_rpm - turn_rpm;
    rpm[3] = (int32_t)forward_rpm - left_rpm + turn_rpm;

    /* 先检查全部轮速，超限则返回错误 */
    for(i = 0U; i < 4U; i++)
    {
        if(rpm[i] < -3000 || rpm[i] > 3000)
        {
            return  HAL_ERROR;
        }
    }

    /* 全部合法后，再写入输出数组 */
    for(i = 0U; i < 4U; i++)
    {
        wheel_rpm[i] = (int16_t)rpm[i];
    }

    return HAL_OK;
}
