#include "unit_test.h"

int ut_failures = 0;
int ut_checks = 0;

void run_x42s_proto_tests(void);
void run_odometry_tests(void);

int main(void)
{
    run_x42s_proto_tests();
    run_odometry_tests();

    printf("\n%d checks, %d failures\n", ut_checks, ut_failures);
    return (ut_failures == 0) ? 0 : 1;
}