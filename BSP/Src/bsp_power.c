#include "bsp_power.h"

void BSP_Power_VM_On(void)
{
    HAL_GPIO_WritePin(VM_SW_GPIO_Port, VM_SW_Pin, GPIO_PIN_SET);
}

void BSP_Power_VM_Off(void)
{
    HAL_GPIO_WritePin(VM_SW_GPIO_Port, VM_SW_Pin, GPIO_PIN_RESET);
}
