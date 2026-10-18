#include "bsp_bno080.h"
#include <string.h>
#include <math.h>

static bool BNO080_SendPacket(BSP_BNO080_t bno, uint8_t channel, uint8_t data_length);
static bool BNO080_ReceivePacket(BSP_BNO080_t bno);
static void BNO080_SetFeatureCommand(BSP_BNO080_t bno, uint8_t report_id, uint32_t interval_us);

/**
 * 发送一个SHTP包
 * 包结构：[长度LSB][长度MSB][通道号][序列号] + shtp_data里的数据
 * 序列号每个通道独立自增，用于检测丢包/乱序
 */
static bool BNO080_SendPacket(BSP_BNO080_t bno, uint8_t channel, uint8_t data_length)
{
    uint16_t packet_length = data_length + 4; /* 总长度=数据+4字节 */
    uint8_t tx_buf[BNO080_MAX_PACKET_SIZE + 4];

    tx_buf[0] = packet_length & 0xFF;
    tx_buf[1] = (packet_length >> 8) & 0xFF;
    tx_buf[2] = channel;
    tx_buf[3] = bno->sequence_number[channel]++;

    memcpy(&tx_buf[4], bno->shtp_data, data_length);

    /* HAL要求8位地址,7位地址左移1位 */
    if(HAL_I2C_Master_Transmit(bno->hi2c, bno->addr << 1, tx_buf, packet_length, 100) != HAL_OK)
    {
        return false;
    }

    return true;
}

/**
 * 接收一个SHPT包
 * 两次读取BNO080——READ_CHUNK_SIZE(一次I2C通信),
 * 再根据包头里的长度字段，截取真正有效的数据部分
 */
static bool BNO080_ReceivePacket(BSP_BNO080_t bno)
{
    uint8_t rx_buf[BNO080_READ_CHUNK_SIZE]; /* 临时数组 */
    uint16_t packet_length;                 /* 整个SHTP数据包长度 */
    uint16_t remaining;                     /* 还有多少Payload没有读取 */
    uint16_t copied = 0;                    /* 已复制到 bno->shtp_data 里面多少字节 */

    /* 先读取4字节SHTP包头 */
    if(HAL_I2C_Master_Receive(bno->hi2c, bno->addr << 1, bno->shtp_header, 4, 500) != HAL_OK)
    {
        return false;
    }

    /* 从包头提取有效数据长度(包头前两个字节) */
    packet_length = (((uint16_t)bno->shtp_header[1] << 8) | bno->shtp_header[0]) & 0x7FFFU;
    if(packet_length < 4U)
    {
        return false;
    }

    remaining = packet_length - 4U;

    /* 每次读取都包含4字节包头，分多次读取消耗完所有有效载荷 */
    while(remaining > 0U)
    {
        uint16_t request_length = remaining + 4U;   /* 每次读取都包含4字节包头 */
        uint16_t payload_length;                    /* 有效载荷长度 */
        uint16_t copy_length;                       /* 复制长度 */

        if(request_length > sizeof(rx_buf))
        {
            request_length = sizeof(rx_buf);
        }
        /* 将完整数据读取到 rx_buf中 */
        if(HAL_I2C_Master_Receive(bno->hi2c, bno->addr << 1, rx_buf, request_length, 500) != HAL_OK)
        {
            return false;
        }

        payload_length = request_length - 4U;

        if(payload_length > remaining)
        {
            payload_length = remaining;
        }

        /* 复制有效载荷到 bno->shtp_data */
        copy_length = payload_length;

        if(copied >= BNO080_MAX_PACKET_SIZE)
        {
            copy_length = 0U;
        }
        else if(copy_length > (BNO080_MAX_PACKET_SIZE - copied))
        {
            copy_length = BNO080_MAX_PACKET_SIZE - copied;
        }

        if(copy_length > 0U)
        {
            memcpy(&bno->shtp_data[copied], &rx_buf[4], copy_length);
            copied += copy_length;
        }

        remaining -= payload_length;
    }
    return true;
}


bool BSP_BNO080_Init(BSP_BNO080_t bno, I2C_HandleTypeDef *hi2c, uint8_t addr, GPIO_TypeDef *rst_port, uint16_t rst_pin, GPIO_TypeDef *int_port, uint16_t int_pin)
{
    uint8_t i;
    uint32_t start_tick;
    /* 保存硬件资源 */
    bno->hi2c       = hi2c;
    bno->addr       = addr;
    bno->rst_port   = rst_port;
    bno->rst_pin    = rst_pin;
    bno->int_port   = int_port;
    bno->int_pin    = int_pin;

    /* 序列号、标志位清零 */
    for(i = 0; i < 6; i++)
    {
        bno->sequence_number[i] = 0;
    }
    bno->data_ready_flag = 0;
    bno->is_initialized = 0;

    /* 硬件复位：RST拉低保持一段时间，再拉高释放 */
    HAL_GPIO_WritePin(bno->rst_port, bno->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(bno->rst_port, bno->rst_pin, GPIO_PIN_SET);
    HAL_Delay(500);/* 等待BNO080启动完成 */

    /* 清空复位后自动发来的广播包(executable通知 + 初始化响应等) */
    for(i = 0; i < 5; i++)
    {
        if(!BNO080_ReceivePacket(bno))
        {
            break; /* 读不到数据了，说明广播包已清空 */
        }
        HAL_Delay(10);
    }

    /* 发送Product ID请求，验证通信 */
    bno->shtp_data[0] = 0xF9;/* Product ID Request */
    bno->shtp_data[1] = 0x00;
    if(!BNO080_SendPacket(bno, BNO080_CHANNEL_CONTROL, 2))
    {
        return false;
    }
    /* 在有限时间内查找 Product ID 响应 */
    start_tick = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start_tick) < 1000U)
    {
        /* INT 低电平表示传感器有数据待读取 */
        if (HAL_GPIO_ReadPin(bno->int_port, bno->int_pin)== GPIO_PIN_RESET)
        {
            if (BNO080_ReceivePacket(bno))
            {
                uint16_t packet_length;

                packet_length =(((uint16_t)bno->shtp_header[1] << 8) | bno->shtp_header[0]) & 0x7FFFU;

                if (packet_length > 4U && bno->shtp_header[2] == BNO080_CHANNEL_CONTROL && bno->shtp_data[0] == 0xF8)
                {
                    bno->is_initialized = 1U;
                    return true;
                }
            }
        }
        HAL_Delay(5);
    }
    return false;
}
/**
 * 发送Set Feature Command，使能指定传感器按interval_us间隔上报数据
 */
static void BNO080_SetFeatureCommand(BSP_BNO080_t bno, uint8_t report_id, uint32_t interval_us)
{
    memset(bno->shtp_data, 0 ,17);

    bno->shtp_data[0] = 0xFD;         /* Set Feature Command */
    bno->shtp_data[1] = report_id;    /* 目标传感器report ID */
    
    /* 上报间隔(微秒) */
    bno->shtp_data[5] = (uint8_t)(interval_us);
    bno->shtp_data[6] = (uint8_t)(interval_us >> 8);
    bno->shtp_data[7] = (uint8_t)(interval_us >> 16);
    bno->shtp_data[8] = (uint8_t)(interval_us >> 24);

    BNO080_SendPacket(bno, BNO080_CHANNEL_CONTROL, 17);
}

/**
 * 将Q格式定点数转换为浮点数 real = raw * 2(-q_point)
 */
static float BNO080_QToFloat(int32_t raw, uint8_t q_point)
{
    float scale = 1.0f / (float)(1U << q_point);
    return (float)raw * scale;
}

/**
 * 解析已经读入shtp_data的传感器上报数据
 * 固定偏移：[0]=0xFB [1-4]=时间戳 [5]=ReportID [6]=序列号 [7]=status [9,10,12,13,14,15,16]=数据
 */
static void BNO080_ParseInputReport(BSP_BNO080_t bno)
{
    uint8_t report_id;
    int16_t raw1, raw2, raw3, raw4;

    if(bno->shtp_data[0] != 0xFB)
    {
        return; /* 不是传感器上报包，忽略 */
    }

    report_id = bno->shtp_data[5];

    raw1 = (int16_t)(((uint16_t)bno->shtp_data[10] << 8) | bno->shtp_data[9]);
    raw2 = (int16_t)(((uint16_t)bno->shtp_data[12] << 8) | bno->shtp_data[11]);
    raw3 = (int16_t)(((uint16_t)bno->shtp_data[14] << 8) | bno->shtp_data[13]);

    switch (report_id)
    {
        case BNO080_REPORTID_ACCELEROMETER:
            bno->accel_x = BNO080_QToFloat(raw1, BNO080_Q_POINT_ACCELEROMETER);
            bno->accel_y = BNO080_QToFloat(raw2, BNO080_Q_POINT_ACCELEROMETER);
            bno->accel_z = BNO080_QToFloat(raw3, BNO080_Q_POINT_ACCELEROMETER);
            bno->accel_accuracy = bno->shtp_data[7] & 0x03U;
            break;
    
        case BNO080_REPORTID_GYROSCOPE:
            bno->gyro_x = BNO080_QToFloat(raw1, BNO080_Q_POINT_GYROSCOPE);
            bno->gyro_y = BNO080_QToFloat(raw2, BNO080_Q_POINT_GYROSCOPE);
            bno->gyro_z = BNO080_QToFloat(raw3, BNO080_Q_POINT_GYROSCOPE);
            bno->gyro_accuracy = bno->shtp_data[7] & 0x03U;
            break;

        case BNO080_REPORTID_ROTATION_VECTOR:
            raw4 = (int16_t)(((uint16_t)bno->shtp_data[16] << 8) | bno->shtp_data[15]);
            bno->quat_i    = BNO080_QToFloat(raw1, BNO080_Q_POINT_ROTATION_VECTOR);
            bno->quat_j    = BNO080_QToFloat(raw2, BNO080_Q_POINT_ROTATION_VECTOR);
            bno->quat_k    = BNO080_QToFloat(raw3, BNO080_Q_POINT_ROTATION_VECTOR);
            bno->quat_real = BNO080_QToFloat(raw4, BNO080_Q_POINT_ROTATION_VECTOR);
            bno->quat_accuracy = bno->shtp_data[7] & 0x03U;
            break;
    default:
        break;/* 未知report ID, 忽略 */
    }
}

void BSP_BNO080_EnableRotationVector(BSP_BNO080_t bno, uint16_t interval_ms)
{
    BNO080_SetFeatureCommand(bno, BNO080_REPORTID_ROTATION_VECTOR, (uint32_t)interval_ms * 1000U);
}

void BSP_BNO080_EnableAccelerometer(BSP_BNO080_t bno, uint16_t interval_ms)
{
    BNO080_SetFeatureCommand(bno, BNO080_REPORTID_ACCELEROMETER, (uint32_t)interval_ms * 1000U);
}

void BSP_BNO080_EnableGyro(BSP_BNO080_t bno, uint16_t interval_ms)
{
    BNO080_SetFeatureCommand(bno, BNO080_REPORTID_GYROSCOPE, (uint32_t)interval_ms * 1000U);
}
/* 有数据准备好了，通过INT引脚中断将标志位置一 */
void BSP_BNO080_INT_Callback(BSP_BNO080_t bno)
{
    bno->data_ready_flag = 1;
}

bool BSP_BNO080_DataAvailable(BSP_BNO080_t bno)
{
    if(bno->data_ready_flag == 0)
    {
        return false;
    }

    bno->data_ready_flag = 0;

    if(!BNO080_ReceivePacket(bno))
    {
        return false;
    }

    BNO080_ParseInputReport(bno);

    return true;
}

void BSP_BNO080_GetQuaternion(BSP_BNO080_t bno, float *i, float *j, float *k, float *real)
{
    *i    = bno->quat_i;
    *j    = bno->quat_j;
    *k    = bno->quat_k;
    *real = bno->quat_real;
}

void BSP_BNO080_GetAccel(BSP_BNO080_t bno, float *x, float *y, float *z)
{
    *x = bno->accel_x;
    *y = bno->accel_y;
    *z = bno->accel_z;
}

void BSP_BNO080_GetGyro(BSP_BNO080_t bno, float *x, float *y, float *z)
{
    *x = bno->gyro_x;
    *y = bno->gyro_y;
    *z = bno->gyro_z;
}

bool BSP_BNO080_GetYaw(BSP_BNO080_t bno, float *yaw_deg)
{
    float qx;
    float qy;
    float qz;
    float qw;
    float norm_sq;
    float norm;
    float yaw_rad;

    if(bno == NULL || yaw_deg == NULL || bno->is_initialized == 0U)
    {
        return false;
    }

    qx = bno->quat_i;
    qy = bno->quat_j;
    qz = bno->quat_k;
    qw = bno->quat_real;

    norm_sq = qx * qx + qy * qy + qz * qz + qw * qw;

    /* 启动初期四元数为全零，或者数据异常时拒绝计算 */
    if(norm_sq < 0.5f || norm_sq > 1.5f)
    {
        return false;
    }

    /* 归一化，避免模长误差影响角度 */
    norm = sqrtf(norm_sq);
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;

    yaw_rad = atan2f(2.0f * (qw *qz + qx * qy),1.0f - 2.0f * (qy * qy + qz * qz));

    *yaw_deg = yaw_rad * 57.2957795f;

    return true;
}
