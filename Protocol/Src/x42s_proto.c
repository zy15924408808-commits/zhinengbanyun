#include "x42s_proto.h"
#include <string.h>

uint8_t X42S_BuildEnable(uint8_t address, bool enable, uint8_t out[6])
{
    if(out == NULL || address == 0U)
    {
        return 0U;
    }
    /**/
    out[0] = address;
    out[1] = X42S_FUNC_ENABLE;
    out[2] = 0xABU;
    out[3] = enable ? 1U : 0U;
    out[4] = 0x00U;
    out[5] = X42S_CHECKSUM;
    return 6U;
}

uint8_t X42S_BuildSpeed(uint8_t address, uint8_t direction, uint16_t speed_rpm, uint8_t acceleration, uint8_t out[8])
{
    if(out == NULL || address == 0U || direction > 1U || speed_rpm > 3000U)
    {
        return 0U;
    }
    out[0] = address;
    out[1] = X42S_FUNC_SPEED;
    out[2] = direction;
    out[3] = (uint8_t)(speed_rpm >> 8);
    out[4] = (uint8_t)(speed_rpm & 0xFFU);
    out[5] = acceleration;
    out[6] = 0x00U;
    out[7] = X42S_CHECKSUM;
    return 8U;
}

uint8_t X42S_BuildReadPosition(uint8_t address, uint8_t out[3])
{
    if(out == NULL || address == 0U)
    {
        return 0U;
    }
    out[0] = address;
    out[1] = X42S_FUNC_READ_POS;
    out[2] = X42S_CHECKSUM;
    return 3U;
}

void X42S_ParserInit(X42S_Parser_t *parser)
{
    if(parser == NULL)
    {
        return;
    }
    parser->used = 0U;
}

/* 根据功能码判断这一帧应答应有的总长度；功能码不认识返回 0 */
static uint8_t X42S_ExpectedLength(uint8_t function)
{
    switch (function)
    {
    case X42S_FUNC_ENABLE:
    case X42S_FUNC_SPEED:
        return 4U;
    case X42S_FUNC_READ_POS:
        return 8U;
    default:
        return 0U;
    }
}

/* 丢弃缓冲区最早的一个字节，其余数据整体前移，用于重新对齐帧边界 */
static void X42S_Resync(X42S_Parser_t *parser)
{
    memmove(&parser->buffer[0], &parser->buffer[1], (size_t)(parser->used - 1U));
    parser->used--;
}

bool X42S_ParseByte(X42S_Parser_t *parser, uint8_t byte, X42S_Response_t *response)
{
    uint8_t expected_len;
    uint32_t magnitude;

    if(parser == NULL || response == NULL)
    {
        return false;
    }

    /* 理论上不会发生：防止极端情况下越界写 */
    if(parser->used >= X42S_FRAME_MAX)
    {
        X42S_Resync(parser);
    }

    parser->buffer[parser->used] = byte;
    parser->used++;

    /* 至少要收到地址和功能码，才知道这一帧该有多长 */
    if(parser->used < 2U)
    {
        return false;
    }

    expected_len = X42S_ExpectedLength(parser->buffer[1]);

    /* 第2个字节不是认识的功能码：说明帧没对齐，丢第1个字节重新尝试 */
    if(expected_len == 0U)
    {
        X42S_Resync(parser);
        return false;
    }

    /* 还没收够这一帧的长度，继续等 */
    if(parser->used < expected_len)
    {
        return false;
    }

    /* 地址是广播地址，或者效验码不对：丢第1个字节重新对齐 */
    if(parser->buffer[0] == 0U || parser->buffer[expected_len - 1U] != X42S_CHECKSUM)
    {
        X42S_Resync(parser);
        return false;
    }

    /* 到这里，这一帧是完整且校验通过的 */
    response->address  = parser->buffer[0];
    response->function = parser->buffer[1];

    if(expected_len == 4U)
    {
        /* F3/F6应答：第3字节是状态 */
        response->status   = parser->buffer[2];
        response->position = 0;
    }
    else
    {
        /*0x36 读位置应答：第3字节是符号，第4~7字节是32位幅值，高位在前 */
        magnitude = ((uint32_t)parser->buffer[3] << 24)
                  | ((uint32_t)parser->buffer[4] << 16)
                  | ((uint32_t)parser->buffer[5] << 8)
                  |  (uint32_t)parser->buffer[6];

        response->status   = X42S_STATUS_OK;
        response->position = (parser->buffer[2] != 0U) ? -(int32_t)magnitude : (int32_t)magnitude;
    }

    parser->used = 0U;
    return true;
}
