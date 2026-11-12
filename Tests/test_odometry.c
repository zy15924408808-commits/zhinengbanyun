#include "unit_test.h"
#include "odometry.h"
#include <stdint.h>

#define PI_F      3.14159265358979f
#define REV       65536
#define TOL_MM    0.01
#define TOL_RAD   1e-5

/* 测试用参数：轮径 80mm，轴距 200mm，轮距 220mm */
static const Odom_Config_t cfg = {80.0f, 100.0f, 110.0f, REV};

static void test_init_rejects_bad_config(void)
{
    Odom_t o;
    Odom_Config_t bad = cfg;

    CHECK(Odom_Init(&o, &cfg));
    CHECK(!Odom_Init(NULL, &cfg));
    CHECK(!Odom_Init(&o, NULL));

    bad.wheel_diameter_mm = 0.0f;
    CHECK(!Odom_Init(&o, &bad));

    bad = cfg;
    bad.counts_per_rev = 0;
    CHECK(!Odom_Init(&o, &bad));
}

static void test_count_delta_wraps(void)
{
    CHECK(Odom_CountDelta(100, 40) == 60);
    CHECK(Odom_CountDelta(-50, 30) == -80);
    /* 计数从 INT32_MAX 附近回绕到 INT32_MIN 附近，实际只前进了 10 */
    CHECK(Odom_CountDelta(INT32_MIN + 5, INT32_MAX - 4) == 10);
}

static void test_wrap_angle(void)
{
    CHECK_NEAR(Odom_WrapAngle(1.5f * PI_F), -0.5f * PI_F, TOL_RAD);
    CHECK_NEAR(Odom_WrapAngle(-PI_F), PI_F, TOL_RAD);
    CHECK_NEAR(Odom_WrapAngle(0.3f), 0.3f, TOL_RAD);
}

static void test_forward_one_rev(void)
{
    Odom_t o;
    const int32_t d[4] = {REV, REV, REV, REV};

    Odom_Init(&o, &cfg);
    Odom_Update(&o, d, 0.0f);

    CHECK_NEAR(o.pose.x_mm, PI_F * 80.0f, TOL_MM);
    CHECK_NEAR(o.pose.y_mm, 0.0f, TOL_MM);
}

static void test_strafe_left_one_rev(void)
{
    Odom_t o;
    /* 与 App_Chassis_CalcWheelRpm(0, left, 0) 的符号一致：{-,+,+,-} */
    const int32_t d[4] = {-REV, REV, REV, -REV};

    Odom_Init(&o, &cfg);
    Odom_Update(&o, d, 0.0f);

    CHECK_NEAR(o.pose.x_mm, 0.0f, TOL_MM);
    CHECK_NEAR(o.pose.y_mm, PI_F * 80.0f, TOL_MM);
}

static void test_forward_while_facing_90deg(void)
{
    Odom_t o;
    const int32_t d[4] = {REV, REV, REV, REV};
    Odom_Pose_t start = {0.0f, 0.0f, PI_F / 2.0f};

    Odom_Init(&o, &cfg);
    Odom_SetPose(&o, start, 0.0f);
    Odom_Update(&o, d, 0.0f);

    /* 车头朝场地 +y，前进应当全部落在 y 上 */
    CHECK_NEAR(o.pose.x_mm, 0.0f, TOL_MM);
    CHECK_NEAR(o.pose.y_mm, PI_F * 80.0f, TOL_MM);
}

static void test_imu_offset_applied(void)
{
    Odom_t o;
    const int32_t still[4] = {0, 0, 0, 0};
    Odom_Pose_t start = {0.0f, 0.0f, 0.0f};

    Odom_Init(&o, &cfg);
    /* 设定位姿时 IMU 读数为 1.0 rad，此后 IMU 不变则场地航向应保持 0 */
    Odom_SetPose(&o, start, 1.0f);
    Odom_Update(&o, still, 1.0f);
    CHECK_NEAR(o.pose.theta_rad, 0.0f, TOL_RAD);

    /* IMU 再逆时针转 0.5 rad，场地航向也转 0.5 rad */
    Odom_Update(&o, still, 1.5f);
    CHECK_NEAR(o.pose.theta_rad, 0.5f, TOL_RAD);
}

static void test_heading_wrap_at_180(void)
{
    Odom_t o;
    const int32_t d[4] = {REV, REV, REV, REV};
    Odom_Pose_t start = {0.0f, 0.0f, 179.0f * PI_F / 180.0f};

    Odom_Init(&o, &cfg);
    Odom_SetPose(&o, start, start.theta_rad);
    /* 航向从 179° 跨到 -179°，平均航向应是 180°，车向 -x 走 */
    Odom_Update(&o, d, -179.0f * PI_F / 180.0f);

    CHECK_NEAR(o.pose.x_mm, -PI_F * 80.0f, 0.05);
    CHECK_NEAR(o.pose.y_mm, 0.0f, 0.05);
}

static void test_square_returns_to_origin(void)
{
    Odom_t o;
    const int32_t fwd[4] = {REV, REV, REV, REV};
    const int32_t still[4] = {0, 0, 0, 0};
    float yaw = 0.0f;
    int side;

    Odom_Init(&o, &cfg);
    for(side = 0; side < 4; side++)
    {
        Odom_Update(&o, fwd, yaw);
        yaw += PI_F / 2.0f;          /* 原地左转 90°，只有 IMU 航向变化 */
        Odom_Update(&o, still, yaw);
    }

    CHECK_NEAR(o.pose.x_mm, 0.0f, 0.05);
    CHECK_NEAR(o.pose.y_mm, 0.0f, 0.05);
    CHECK_NEAR(Odom_WrapAngle(o.pose.theta_rad), 0.0f, 1e-4);
}

static void test_encoder_heading_for_rotation(void)
{
    Odom_t o;
    /* 原地左转：{-,+,-,+}，每轮转 c 个计数 */
    const int32_t c = 10000;
    const int32_t d[4] = {-c, c, -c, c};
    const float expected = (float)c * (PI_F * 80.0f / (float)REV) / (100.0f + 110.0f);

    Odom_Init(&o, &cfg);
    Odom_Update(&o, d, 0.0f);

    CHECK_NEAR(o.enc_theta_rad, expected, TOL_RAD);
    /* 纯旋转不应产生平移 */
    CHECK_NEAR(o.pose.x_mm, 0.0f, TOL_MM);
    CHECK_NEAR(o.pose.y_mm, 0.0f, TOL_MM);
}

void run_odometry_tests(void)
{
    RUN_TEST(test_init_rejects_bad_config);
    RUN_TEST(test_count_delta_wraps);
    RUN_TEST(test_wrap_angle);
    RUN_TEST(test_forward_one_rev);
    RUN_TEST(test_strafe_left_one_rev);
    RUN_TEST(test_forward_while_facing_90deg);
    RUN_TEST(test_imu_offset_applied);
    RUN_TEST(test_heading_wrap_at_180);
    RUN_TEST(test_square_returns_to_origin);
    RUN_TEST(test_encoder_heading_for_rotation);
}