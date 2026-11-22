#include "app_ui.h"
#include <stdio.h>

/* 每行最多21个字符：128像素/(5像素字宽+1像素距离) */
#define UI_LINE_CHARS 21U

/* 
 * 将一个以 0.1° 为单位的整数角度，格式化成带一位小数的字符串
 * buf:保存最终格式化字符串的缓冲区,size:缓冲区大小，防止字符串写入越界,label:显示的标签,decideg:输入角度，单位是0.1度
 */
static void Ui_FormatDeciDeg(char *buf, size_t size, const char *label, int32_t decideg)
{
    int32_t mag = (decideg < 0) ? -decideg : decideg;

    (void)snprintf(buf, size, "%s:%s%ld.%ld DEG", label, (decideg < 0) ? "-" : "", (long)(mag / 10), (long)(mag % 10));
}

/* HAL_StatusTypeDef 转换屏幕上看得懂的文字 */
static const char *Ui_ResultText(int32_t result)
{
    switch (result)
    {
        case 0:  return "OK";
        case 1:  return "ERROR";
        case 2:  return "BUSY";
        case 3:  return "TIMEOUT";
        default: return "UNKNOWN";
    }
}
/*
 * 根据里程计测试状态，将机器人当前的里程计数据和测试结果显示到OLED屏幕上
 * oled:OLED设备句柄，report:指向里程计状态报告结构体的指针
*/
void App_Ui_DrawOdom(BSP_OLED_t oled, const App_OdomReport_t *report)
{
    static const char *const state_text[] = {"ODOM TEST WAIT", "ODOM TEST RUN", "ODOM TEST DONE"};
    char line[UI_LINE_CHARS +1U];

    if(oled == NULL || report == NULL || report->state > APP_ODOM_TEST_DONE)
    {
        return;
    }
    /* 清屏 */
    BSP_OLED_Fill(oled, 0x00U);
    /* 根据当前状态显示对应文字 */
    BSP_OLED_DrawString(oled, 0U, 0U, state_text[report->state]);

    if(report->state == APP_ODOM_TEST_WAIT)
    {
        return;
    }

    (void)snprintf(line, sizeof(line), "X:%ld MM", (long)report->x_mm);
    BSP_OLED_DrawString(oled, 2U, 0U, line);

    (void)snprintf(line, sizeof(line), "Y:%ld MM", (long)report->y_mm);
    BSP_OLED_DrawString(oled, 3U, 0U, line);
    /* 航向角 */
    Ui_FormatDeciDeg(line, sizeof(line), "TH", (long)report->th_decideg);
    BSP_OLED_DrawString(oled, 4U, 0U, line);
    /* 编码器计算的角度 */
    Ui_FormatDeciDeg(line, sizeof(line), "ENC", (long)report->enc_th_decideg);
    BSP_OLED_DrawString(oled, 5U, 0U, line);

    /* 结果和周期统计只有结束后才有意义 */
    if(report->state == APP_ODOM_TEST_DONE)
    {
        /* 测试结果 */
        (void)snprintf(line, sizeof(line), "RES:%s", Ui_ResultText(report->result));
        BSP_OLED_DrawString(oled, 6U, 0U, line);
        /* 测试周期和超时次数 */
        (void)snprintf(line, sizeof(line), "CYC:%lu OV:%lu", (unsigned long)report->cycles, (unsigned long)report->overruns);
        BSP_OLED_DrawString(oled, 7U, 0U, line);
    }
}
