#ifndef LOG_H
#define LOG_H

#include <stdbool.h>

/* 创建日志互斥锁，在任务运行前调用 */
bool Log_Init(void);

/* 格式化输出一条日志，供任务调用 */
void Log_Printf(const char *format, ...);



#endif /* LOG_H */
