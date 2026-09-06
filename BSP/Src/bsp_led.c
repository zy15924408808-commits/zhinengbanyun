#include "bsp_led.h"

/* 初始化LED对象 */
void BSP_LED_Init(BSP_LED_t led, GPIO_TypeDef *gpio_port, uint16_t gpio_pin, LED_ActiveLevel_t active_level)
{
    if(led == NULL) return;
    
    led->gpio_port      = gpio_port;
    led->gpio_pin       = gpio_pin;
    led->active_level   = active_level;
    
     BSP_LED_Off(led);
}
/* 点亮LED */
void BSP_LED_On(BSP_LED_t led)
{
    if(led == NULL) return;
    
    GPIO_PinState pin_start;
    
    if(led -> active_level == LED_ACTIVE_LOW)
    {
        pin_start = GPIO_PIN_RESET; /* 低电平点亮 */
    }
    else
    {
        pin_start = GPIO_PIN_SET;   /* 高电平点亮 */
    }
    HAL_GPIO_WritePin(led->gpio_port,led->gpio_pin,pin_start);
}
/* 熄灭LED */
void BSP_LED_Off(BSP_LED_t led)
{
    if(led == NULL) return;
    
    GPIO_PinState pin_start;
    
    if(led -> active_level == LED_ACTIVE_LOW)
    {
        pin_start = GPIO_PIN_SET;   /* 高电平点亮 */
    }
    else
    {
        pin_start = GPIO_PIN_RESET; /* 低电平点亮 */
    }
    HAL_GPIO_WritePin(led->gpio_port,led->gpio_pin,pin_start);
}
/* 翻转LED */
void BSP_LED_Toggle(BSP_LED_t led)
{
    if(led == NULL) return;
    
    HAL_GPIO_TogglePin(led->gpio_port, led->gpio_pin);
}
