#include "app_status.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

/* 控制任务写、UI任务读：结构体要整体拷贝，用临界区保护 */
static App_OdomReport_t odom_report = {APP_ODOM_TEST_WAIT, 0, 0, 0, 0, 0, 0U, 0U};
static uint32_t odom_report_seq = 0U;
/* 发布里程计状态
 * 将传入的里程计状态赋值到全局共享变量中，并将更新序号加1
*/
void App_Status_PublishOdom(const App_OdomReport_t *report)
{
    if(report == NULL)
    {
        return;
    }
    /* 进入临界区 */
    taskENTER_CRITICAL();
    /* 复制整个结构体到全局变量 */
    odom_report = *report;
    /* 更新序号 */
    odom_report_seq++;
    /* 退出临界区 */
    taskEXIT_CRITICAL();
}
/* 获取里程计状态
 * 将全局共享的里程计状态复制给调用者，并返回当前的数据更新序号
*/
uint32_t App_Status_GetOdom(App_OdomReport_t *report)
{
    /* 暂时保存当前的数据更新序号 */
    uint32_t seq;
    if(report == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    /* 复制结构体 */
    *report = odom_report;
    /* 读取更新序号 */
    seq = odom_report_seq;
    taskEXIT_CRITICAL();

    return seq;
}

