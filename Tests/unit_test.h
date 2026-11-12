#ifndef UNIT_TEST_H
#define UNIT_TEST_H

#include <math.h>
#include <stdio.h>

/* 极简单元测试框架：失败时打印位置并计数，不中断，最后汇总 */
extern int ut_failures;
extern int ut_checks;

#define CHECK(cond)                                                     \
    do {                                                                \
        ut_checks++;                                                    \
        if(!(cond)) {                                                   \
            ut_failures++;                                              \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                               \
    } while(0)

#define CHECK_NEAR(actual, expected, tol)                               \
    do {                                                                \
        double ut_a = (double)(actual);                                 \
        double ut_e = (double)(expected);                               \
        ut_checks++;                                                    \
        if(fabs(ut_a - ut_e) > (double)(tol)) {                         \
            ut_failures++;                                              \
            printf("  FAIL %s:%d: %s = %.6f, expected %.6f\n",          \
                   __FILE__, __LINE__, #actual, ut_a, ut_e);            \
        }                                                               \
    } while(0)

#define RUN_TEST(fn)                                                    \
    do {                                                                \
        printf("%s\n", #fn);                                            \
        fn();                                                           \
    } while(0)

#endif /* UNIT_TEST_H */