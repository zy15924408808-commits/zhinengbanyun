#ifndef X42S_PROTO_H
#define X42S_PROTO_H

#include <stdbool.h>
#include <stdint.h>

/* Emm 固件功能码 */
#define X42S_FUNC_ENABLE        0xF3U
#define X42S_FUNC_SPEED         0xF6U
#define X42S_FUNC_READ_POS      0x36U
#define X42S_CHECKSUM           0x6BU

#define X42S_STATUS_OK          0x02U
#define X42S_POS_COUNTS_PER_REV 65536L  /* Emm 固件：65536 = 一圈 */
#define X42S_FRAME_MAX          8U      /* 当前支持的最长应答 */

/* 电机应答 */
typedef struct
{
    uint8_t address;
    uint8_t function;
    uint8_t status;     /* F3/F6 有效：0x02 表示成功 */
    int32_t position;   /* 0x36 有效：带符号的累计位置 */
} X42S_Response_t;

/* 逐字解析器，保存未收齐的数据 */
typedef struct 
{
    uint8_t buffer[X42S_FRAME_MAX];
    uint8_t used;   /* 有效字节数量 */
} X42S_Parser_t;

/* 组帧：成功返回帧长度，参数错误返回 0 */
uint8_t X42S_BuildEnable(uint8_t address, bool enable, uint8_t out[6]);
uint8_t X42S_BuildSpeed(uint8_t address, uint8_t direction, uint16_t speed_rpm, uint8_t acceleration, uint8_t out[8]);
uint8_t X42S_BuildReadPosition(uint8_t address, uint8_t out[3]);

/* 解析：输入一个字节，收到完整应答时返回 true */
void X42S_ParserInit(X42S_Parser_t *parser);
bool X42S_ParseByte(X42S_Parser_t *parser, uint8_t byte, X42S_Response_t *response);


#endif /* X42S_PROTO_H */
