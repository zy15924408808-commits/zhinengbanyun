#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "main.h"
#include <stdbool.h>

/**
 * LED有效电平定义
 */
typedef enum
{
    LED_ACTIVE_LOW  = 0,    /* 低电平点亮 */
    LED_ACTIVE_HIGH = 1     /* 高电平点亮 */
}LED_ActiveLevel_t;

/**
 * LED对象结构体
 */
struct bsp_led
{
    GPIO_TypeDef        *gpio_port;     /* GPIO端口（GPIOA/GPIOB/...） */
    uint16_t            gpio_pin;       /* GPIO引脚（GPIO_PIN_0~15） */
    LED_ActiveLevel_t   active_level;   /* 有效电平 */
    bool                state;          /* 当前状态：true=亮, false=灭 */
};
typedef struct bsp_led *BSP_LED_t;

/* 初始化LED */
void BSP_LED_Init(BSP_LED_t led, GPIO_TypeDef *gpio_port, uint16_t gpio_pin, LED_ActiveLevel_t active_level);
/* 点亮LED */
void BSP_LED_On(BSP_LED_t led);
/* 熄灭LED */
void BSP_LED_Off(BSP_LED_t led);
/* 翻转LED */
void BSP_LED_Toggle(BSP_LED_t led);



    #endif /* __BSP_LED_H__ */
