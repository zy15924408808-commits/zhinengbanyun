#include "bsp_oled.h"
#include "bsp_oled_font.h"

#define OLED_SPI_TIMEOUT_MS 100U

/* 发送一个命令字节：DC = 0 表示命令 */
static HAL_StatusTypeDef OLED_WriteCommand(BSP_OLED_t oled, uint8_t command)
{
    HAL_StatusTypeDef status;

    if(oled == NULL || oled->hspi == NULL || oled->cs_port == NULL || oled->dc_port == NULL)
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(oled->dc_port, oled->dc_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(oled->cs_port, oled->cs_pin, GPIO_PIN_RESET);

    status = HAL_SPI_Transmit(oled->hspi, &command, 1U, OLED_SPI_TIMEOUT_MS);

    HAL_GPIO_WritePin(oled->cs_port, oled->cs_pin, GPIO_PIN_SET);

    return status;
}

/* 发送一段显示数据：DC = 1 表示数据 */
static HAL_StatusTypeDef OLED_WriteData(BSP_OLED_t oled, const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if(oled == NULL || oled->hspi == NULL || oled->cs_port == NULL || oled->dc_port == NULL || data == NULL || len == 0U)
    {
        return HAL_ERROR;
    }
    
    HAL_GPIO_WritePin(oled->dc_port, oled->dc_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(oled->cs_port, oled->cs_pin, GPIO_PIN_RESET);

    status = HAL_SPI_Transmit(oled->hspi, data, len, OLED_SPI_TIMEOUT_MS);
    
    HAL_GPIO_WritePin(oled->cs_port, oled->cs_pin, GPIO_PIN_SET);

    return status;
}

/* 填充STM32显存 */
void BSP_OLED_Fill(BSP_OLED_t oled, uint8_t pattern)
{
    uint16_t page;
    uint16_t column;

    if(oled == NULL)
    {
        return;
    }

    for(page = 0U; page < BSP_OLED_PAGES; page++)
    {
        for(column = 0U; column < BSP_OLED_WIDTH; column++)
        {
            oled->framebuffer[page][column] = pattern;
        }
    }
}

/* 把显存逐页发送到屏幕 */
HAL_StatusTypeDef BSP_OLED_Update(BSP_OLED_t oled)
{
    HAL_StatusTypeDef status;
    uint16_t page;
    uint16_t index;
    uint8_t address[3];

    if(oled == NULL || oled->hspi == NULL || oled->cs_port == NULL || oled->dc_port ==NULL)
    {
        return HAL_ERROR;
    }

    for(page = 0U; page < BSP_OLED_PAGES; page++)
    {
        address[0] = (uint8_t)(0xB0U + page);
        address[1] = 0x00U;
        address[2] = 0x10U;

        for(index = 0U; index < 3U; index++)
        {
            status = OLED_WriteCommand(oled, address[index]);
            if(status != HAL_OK)
            {
                return status;
            }
        }
        status = OLED_WriteData(oled, oled->framebuffer[page], BSP_OLED_WIDTH);

        if(status != HAL_OK)
        {
            return status;
        }
    }

    return HAL_OK;
}

/* 显示字符 */
void BSP_OLED_DrawChar(BSP_OLED_t oled, uint8_t page, uint8_t column, char ch)
{
    const uint8_t *glyph;
    uint8_t index;
    if(oled == NULL || page >= BSP_OLED_PAGES)
    {
        return;
    }
    if(column > (BSP_OLED_WIDTH - BSP_OLED_FONT_WIDTH - BSP_OLED_FONT_SPACING))
    {
        return;
    }
    glyph = BSP_OLED_Font_GetGlyph(ch);
    for(index = 0U; index < BSP_OLED_FONT_WIDTH; index++)
    {
        oled->framebuffer[page][column + index] = glyph[index];
    }
    oled->framebuffer[page][column + BSP_OLED_FONT_WIDTH] = 0x00U;
}

void BSP_OLED_DrawString(BSP_OLED_t oled, uint8_t page, uint8_t column, const char *str)
{
    if(oled == NULL || str == NULL || page >= BSP_OLED_PAGES)
    {
        return;
    }
    while(*str != '\0')
    {
        if(column > (BSP_OLED_WIDTH - BSP_OLED_FONT_WIDTH - BSP_OLED_FONT_SPACING))
        {
            break;
        }

        BSP_OLED_DrawChar(oled, page, column, *str);

        column += BSP_OLED_FONT_WIDTH + BSP_OLED_FONT_SPACING;
        str++;
    }
}

HAL_StatusTypeDef BSP_OLED_Init(BSP_OLED_t oled, SPI_HandleTypeDef *hspi, 
                                GPIO_TypeDef *cs_port, uint16_t cs_pin, 
                                GPIO_TypeDef *dc_port, uint16_t dc_pin,
                                GPIO_TypeDef *rst_port,uint16_t rst_pin)
{
    static const uint8_t init_commands[] =
    {
        0xAE,
        0xD5,0X80,
        0XA8,0X3F,
        0XD3,0X00,
        0X40,
        0X20,0X02,
        0XA1,
        0XC8,
        0XDA,0X12,
        0X81,0XCF,
        0XD9,0XF1,
        0XDB,0X40,
        0XA4,
        0XA6,
        0X8D,0X14
    };
    HAL_StatusTypeDef status;
    uint16_t index;

    if(oled == NULL || hspi ==NULL || 
       cs_port == NULL || dc_port == NULL || rst_port == NULL ||
       cs_pin ==0U || dc_pin ==0U || rst_pin == 0U)
    {
        return HAL_ERROR;
    }

    oled->hspi = hspi;
    oled->cs_port = cs_port;
    oled->cs_pin = cs_pin;
    oled->dc_port = dc_port;
    oled->dc_pin = dc_pin;
    oled->rst_port = rst_port;
    oled->rst_pin = rst_pin;

    HAL_GPIO_WritePin(oled->cs_port, oled->cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(oled->dc_port, oled->dc_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(oled->rst_port, oled->rst_pin, GPIO_PIN_SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(oled->rst_port, oled->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(20U);
    HAL_GPIO_WritePin(oled->rst_port, oled->rst_pin, GPIO_PIN_SET);
    HAL_Delay(20U);

    for(index = 0U; index < sizeof(init_commands); index++)
    {
        status = OLED_WriteCommand(oled, init_commands[index]);
        if(status != HAL_OK)
        {
            return status;
        }
    }

    BSP_OLED_Fill(oled, 0x00U);
    status = BSP_OLED_Update(oled);
    if(status != HAL_OK)
    {
        return status;
    }
    return OLED_WriteCommand(oled, 0XAFU);
}
