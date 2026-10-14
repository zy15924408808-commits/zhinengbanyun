#ifndef APP_TASKS_H
#define APP_TASKS_H

/* 创建应用队列和自定义接收任务 */
void App_RTOS_Init(void);

/* 控制任务的业务入口 */
void App_ControlTask(void *argument);


#endif /* APP_TASKS_H */
