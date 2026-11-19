#ifndef APP_UI_H
#define APP_UI_H

#include "bsp_oled.h"
#include "app_status.h"

/* 把里程计测试结果画面显示到（不刷新屏幕。调用方再BSP_OLED_Update） */
void App_Ui_DrawOdom(BSP_OLED_t oled, const App_OdomReport_t *report);


#endif /* APP_UI_H */
