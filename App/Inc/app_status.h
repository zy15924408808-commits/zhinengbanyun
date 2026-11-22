#ifndef APP_STATUS_H
#define APP_STATUS_H

#include <stdint.h>

/* 里程计直线测试所处的阶段 */
typedef enum
{
    APP_ODOM_TEST_WAIT = 0,     /* 还没开始 */
    APP_ODOM_TEST_RUN,          /* 正在走 */
    APP_ODOM_TEST_DONE          /* 已经结束（成功或失败） */
}App_OdomTestState_t;

/* 控制任务发给UI任务的测试结果，角度单位为0.1度 */
typedef struct
{
    App_OdomTestState_t state;
    int32_t x_mm;
    int32_t y_mm;
    int32_t th_decideg;
    int32_t enc_th_decideg;
    int32_t result;
    uint32_t cycles;
    uint32_t overruns;
}App_OdomReport_t;

/* 发布最新结果（控制任务调用） */
void App_Status_PublishOdom(const App_OdomReport_t *report);

/* 读取最新结果，返回发布序号：序号变了说明有新数据，需要重画 */
uint32_t App_Status_GetOdom(App_OdomReport_t *report);

#endif/* APP_STATUS_H */
