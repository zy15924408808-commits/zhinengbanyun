#include "unit_test.h"
#include "x42s_proto.h"

/* 把一串字节逐个喂给解析器，返回最后一个字节时是否解析出完整应答 */
static bool feed(X42S_Parser_t *p, const uint8_t *bytes, int n, X42S_Response_t *r)
{
    bool hit = false;
    int i;

    for(i = 0; i < n; i++)
    {
        hit = X42S_ParseByte(p, bytes[i], r);
    }
    return hit;
}

static void test_build_frames(void)
{
    uint8_t out[8];

    CHECK(X42S_BuildEnable(1, true, out) == 6);
    CHECK(out[0] == 1 && out[1] == 0xF3 && out[2] == 0xAB && out[3] == 1 && out[5] == 0x6B);

    CHECK(X42S_BuildSpeed(2, 1, 1500, 10, out) == 8);
    CHECK(out[2] == 1 && out[3] == 0x05 && out[4] == 0xDC && out[5] == 10 && out[7] == 0x6B);

    CHECK(X42S_BuildReadPosition(4, out) == 3);
    CHECK(out[0] == 4 && out[1] == 0x36 && out[2] == 0x6B);
}

static void test_build_rejects_bad_args(void)
{
    uint8_t out[8];

    CHECK(X42S_BuildEnable(0, true, out) == 0);          /* 广播地址 */
    CHECK(X42S_BuildSpeed(2, 2, 100, 0, out) == 0);      /* 方向非法 */
    CHECK(X42S_BuildSpeed(2, 0, 3001, 0, out) == 0);     /* 超速 */
    CHECK(X42S_BuildReadPosition(0, out) == 0);
}

static void test_parse_ack(void)
{
    X42S_Parser_t p;
    X42S_Response_t r;
    const uint8_t ack[4] = {2, 0xF6, 0x02, 0x6B};

    X42S_ParserInit(&p);
    CHECK(feed(&p, ack, 4, &r));
    CHECK(r.address == 2 && r.function == 0xF6 && r.status == 0x02);
}

static void test_parse_position_sign(void)
{
    X42S_Parser_t p;
    X42S_Response_t r;
    const uint8_t pos[8] = {4, 0x36, 0x00, 0x00, 0x01, 0x00, 0x00, 0x6B};
    const uint8_t neg[8] = {4, 0x36, 0x01, 0x00, 0x03, 0x0C, 0xD7, 0x6B};  /* -199895 */

    X42S_ParserInit(&p);
    CHECK(feed(&p, pos, 8, &r));
    CHECK(r.position == 65536);

    CHECK(feed(&p, neg, 8, &r));
    CHECK(r.position == -199895);
}

static void test_parse_resync_after_noise(void)
{
    X42S_Parser_t p;
    X42S_Response_t r;
    const uint8_t noisy[6] = {0x11, 0x22, 3, 0xF3, 0x02, 0x6B};

    X42S_ParserInit(&p);
    CHECK(feed(&p, noisy, 6, &r));
    CHECK(r.address == 3 && r.function == 0xF3);
}

static void test_parse_rejects_bad_checksum(void)
{
    X42S_Parser_t p;
    X42S_Response_t r;
    const uint8_t bad[4] = {2, 0xF6, 0x02, 0x00};

    X42S_ParserInit(&p);
    CHECK(!feed(&p, bad, 4, &r));
}

void run_x42s_proto_tests(void)
{
    RUN_TEST(test_build_frames);
    RUN_TEST(test_build_rejects_bad_args);
    RUN_TEST(test_parse_ack);
    RUN_TEST(test_parse_position_sign);
    RUN_TEST(test_parse_resync_after_noise);
    RUN_TEST(test_parse_rejects_bad_checksum);
}