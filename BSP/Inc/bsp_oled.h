#ifndef BSP_OLED_H
#define BSP_OLED_H

#include "main.h"

#define BSP_OLED_WIDTH  128U
#define BSP_OLED_HEIGHT 64U
#define BSP_OLED_PAGES  (BSP_OLED_HEIGHT / 8U)

struct bsp_oled
{
    SPI_HandleTypeDef   *hspi;
    GPIO_TypeDef        *cs_port;
    uint16_t            cs_pin;
    GPIO_TypeDef        *dc_port;
    uint16_t            dc_pin;
    GPIO_TypeDef        *rst_port;
    uint16_t            rst_pin;
    uint8_t             framebuffer[BSP_OLED_PAGES][BSP_OLED_WIDTH];
};

typedef struct bsp_oled *BSP_OLED_t;

HAL_StatusTypeDef BSP_OLED_Init(BSP_OLED_t oled, 
                                SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                GPIO_TypeDef *dc_port, uint16_t dc_pin,
                                GPIO_TypeDef *rst_port, uint16_t rst_pin);

void BSP_OLED_Fill(BSP_OLED_t oled, uint8_t pattern);
HAL_StatusTypeDef BSP_OLED_Update(BSP_OLED_t oled);

void BSP_OLED_DrawChar(BSP_OLED_t oled,uint8_t page,uint8_t column,char ch);
void BSP_OLED_DrawString(BSP_OLED_t oled,uint8_t page,uint8_t column,const char *str);

#endif /* BSP_OLED_H */
