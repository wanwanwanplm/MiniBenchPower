/**
 * protocol.c
 * 二进制通信协议实现 (环形缓冲 + 帧解析 + CRC16)
 *
 * 实现自定义二进制帧协议的完整功能：
 *   1. 环形缓冲区 (ISR 安全, 无锁)
 *   2. 帧解析状态机 (对接 0xA5/0x5A 帧边界)
 *   3. CRC16 校验 (CCITT 多项式)
 *   4. 命令派遣 (查表法)
 *   5. 响应帧封装
 */

#include "protocol.h"
#include <string.h>

/*===========================================================================
 * 命令处理器注册表 (全局)
 *===========================================================================*/

static CmdRegistry_t *g_cmd_registry = NULL;
static uint8_t g_cmd_count = 0;

/*===========================================================================
 * 帧解析状态机状态定义
 *===========================================================================*/

typedef enum {
    PARSE_IDLE = 0,         /* 等待 0xA5 帧头 */
    PARSE_GOT_HEADER,       /* 收到 0xA5, 等待命令 */
    PARSE_GOT_CMD,          /* 收到命令, 等待长度 */
    PARSE_GOT_LEN,          /* 收到长度, 等待数据 */
    PARSE_GOT_DATA,         /* 收齐数据, 等待 CRC 高字节 */
    PARSE_GOT_CRC1,         /* 收到 CRC 高字节, 等待 CRC 低字节 */
    PARSE_DONE,             /* 帧解析完成 */
} ParseState_t;

/*===========================================================================
 * 环形缓冲区实现
 *===========================================================================*/

/**
 * 环形缓冲区初始化
 *
 * 内存布局：
 *   buffer[0..255]: 256 字节存储空间
 *   head: 下一次写入的位置 (ISR 更新)
 *   tail: 下一次读取的位置 (任务更新)
 *
 * 空: head == tail
 * 满: (head + 1) % size == tail (浪费一个位置来区分空和满)
 */
void RingBuffer_Init(RingBuffer_t *rb)
{
    if (rb == NULL) return;
    memset(rb->buffer, 0, RING_BUFFER_SIZE);
    rb->head = 0;
    rb->tail = 0;
}

/**
 * 环形缓冲区写入（ISR 安全）
 *
 * 为什么 ISR 安全？
 *   只有 ISR (生产者) 会修改 head。
 *   只有任务 (消费者) 会修改 tail。
 *   没有并发写同一个变量的情况 → 不需要锁。
 *
 *   面试可能问：如果多个 ISR 都写怎么办？
 *   答：那就需要关中断保护。但本项目只有 USART1 ISR 会写。
 */
uint8_t RingBuffer_Write(RingBuffer_t *rb, uint8_t data)
{
    uint16_t next_head;

    if (rb == NULL) return 0;

    next_head = (rb->head + 1) % RING_BUFFER_SIZE;

    /* 检查是否满 */
    if (next_head == rb->tail) {
        /*
         * 缓冲区满 → 丢弃字节
         *
         * 为什么丢弃而不是等待？
         *   在 ISR 中不能阻塞等待。丢弃意味着数据丢失，
         *   但帧的 CRC 校验会检测到并请求重传。
         *   上层协议应保证这一点。
         */
        return 0;
    }

    rb->buffer[rb->head] = data;
    rb->head = next_head;
    return 1;
}

uint8_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data)
{
    if (rb == NULL || data == NULL) return 0;

    /* 检查是否空 */
    if (rb->head == rb->tail) {
        return 0;  /* 缓冲区空 */
    }

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    return 1;
}

uint16_t RingBuffer_Available(RingBuffer_t *rb)
{
    if (rb == NULL) return 0;

    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    } else {
        return RING_BUFFER_SIZE - rb->tail + rb->head;
    }
}

/*===========================================================================
 * CRC16 计算
 *===========================================================================*/

/**
 * CRC16-CCITT 查表法实现
 *
 * 多项式: 0x1021
 * 初始值: 0xFFFF (标准 CCITT)
 *
 * 查表法比逐位计算快 8 倍。
 * 256 个 uint16_t = 512 字节，放在 Flash (.rodata) 中。
 */
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t CRC16_Calculate(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;

    for (i = 0; i < len; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }

    return crc;
}

/*===========================================================================
 * 帧解析状态机
 *===========================================================================*/

/**
 * 从环形缓冲区中解析一帧。
 *
 * 帧格式 (完整):
 *   0xA5  cmd  len  data[0..len-1]  crc_l  crc_h  0x5A
 *   ─────┬──── ─┬─  ──────┬────── ───┬─── ───┬─── ───┬───
 *   帧头  │ 命令  │ 长度    │   数据   │ CRC低  │ CRC高  │ 帧尾
 *
 * 状态机设计：
 *   每个状态读取一个字节，转移到下一个状态。
 *   如果任何字节不符合预期（如期望帧头但收到非 0xA5），回退到 IDLE。
 */
uint8_t Protocol_ParseFrame(RingBuffer_t *rb, CommFrame_t *frame)
{
    static ParseState_t state = PARSE_IDLE;
    static uint8_t cmd;
    static uint8_t data_len;
    static uint8_t data_idx;
    static uint8_t data_buf[FRAME_MAX_DATA_LEN];
    static uint16_t crc_received;
    static uint8_t crc_byte;

    uint8_t byte_val;

    if (rb == NULL || frame == NULL) return 0;

    while (RingBuffer_Available(rb) > 0) {
        RingBuffer_Read(rb, &byte_val);

        switch (state) {

        case PARSE_IDLE:
            /*
             * 等待帧头 0xA5
             * 任何非 0xA5 的字节都被丢弃
             */
            if (byte_val == FRAME_SOF) {
                state = PARSE_GOT_HEADER;
                cmd = 0;
                data_len = 0;
                data_idx = 0;
                crc_received = 0;
            }
            /* else: 丢弃, 继续等 */
            break;

        case PARSE_GOT_HEADER:
            /*
             * 收到帧头 → 下一个字节是命令码
             */
            cmd = byte_val;
            state = PARSE_GOT_CMD;
            break;

        case PARSE_GOT_CMD:
            /*
             * 收到命令 → 下一个字节是数据长度
             *
             * 长度检查：不能超过 FRAME_MAX_DATA_LEN
             */
            data_len = byte_val;
            if (data_len > FRAME_MAX_DATA_LEN) {
                /* 长度非法 → 丢弃整个帧 */
                state = PARSE_IDLE;
                break;
            }
            data_idx = 0;
            if (data_len == 0) {
                /* 无数据 → 直接跳到等 CRC */
                state = PARSE_GOT_DATA;
            } else {
                state = PARSE_GOT_LEN;
            }
            break;

        case PARSE_GOT_LEN:
            /*
             * 收数据字节，直到 data_idx == data_len
             */
            data_buf[data_idx++] = byte_val;
            if (data_idx >= data_len) {
                state = PARSE_GOT_DATA;
            }
            break;

        case PARSE_GOT_DATA:
            /*
             * 收齐数据 → 等 CRC 高字节
             * 注意：CRC 是小端序 (低字节在前, 高字节在后)
             */
            crc_byte = byte_val;
            state = PARSE_GOT_CRC1;
            break;

        case PARSE_GOT_CRC1:
            /*
             *
             * 实际上 CRC 是两个字节：crc_l + crc_h
             * 修正：CRC 低字节已在 GOT_DATA 读取, 高字节在 GOT_CRC1 读取
             * 然后等帧尾 0x5A
             */
            crc_received = byte_val;  /* CRC 高字节 */
            crc_received = (crc_received << 8) | crc_byte;  /* 组合 CRC16 */

            /*
             * 验证 CRC
             */
            {
                uint16_t crc_calc;
                uint8_t crc_buf[FRAME_MAX_DATA_LEN + 2];
                crc_buf[0] = cmd;
                crc_buf[1] = data_len;
                if (data_len > 0) {
                    memcpy(&crc_buf[2], data_buf, data_len);
                }
                crc_calc = CRC16_Calculate(crc_buf, data_len + 2);

                if (crc_calc != crc_received) {
                    /* CRC 不匹配 → 丢弃 */
                    state = PARSE_IDLE;
                    break;
                }
            }

            /* CRC 通过 → 组装帧 */
            frame->command = cmd;
            frame->data_len = data_len;
            if (data_len > 0) {
                memcpy(frame->data, data_buf, data_len);
            }
            frame->crc = crc_received;
            frame->valid = 1;

            /*
             * 不需要单独等 0x5A。如果 CRC 通过，帧直接完成。
             * 如果发送方严格按格式发了 0x5A 帧尾，下一个字节就是 0x5A，
             * 会被状态机在 IDLE 状态下忽略（因为不是 0xA5）。
             *
             * 更严格的实现是等 0x5A 再标记完成。
             * 但考虑到 0x5A 可能被误识别为下一个帧的数据，
             * 我们用 CRC 作为帧结束的最终确认。
             */

            state = PARSE_IDLE;
            return 1;  /* 成功解析一帧 */

        default:
            state = PARSE_IDLE;
            break;
        }
    }

    return 0;  /* 暂无完整帧 */
}

/*===========================================================================
 * 帧封装
 *===========================================================================*/

/**
 * 构造响应帧
 *
 * 输出格式：
 *   [0xA5] [cmd] [len] [data...] [CRC_L] [CRC_H] [0x5A]
 *
 * 注意：响应命令码的 bit7 = 1 (CMD_ACK 标志)
 * 如原命令 0x01 → 响应 0x81
 */
uint16_t Protocol_BuildResponse(const CommFrame_t *response,
                                uint8_t *buffer, uint16_t buf_len)
{
    uint16_t crc;
    uint16_t total_len;
    uint16_t idx = 0;

    if (response == NULL || buffer == NULL) return 0;

    /* 计算总长度 */
    total_len = 1 + 1 + 1 + response->data_len + 2 + 1;  /* SOF+cmd+len+data+crc16+eof */
    if (total_len > buf_len) return 0;  /* 缓冲区不够 */

    /* 帧头 */
    buffer[idx++] = FRAME_SOF;

    /* 命令码 */
    buffer[idx++] = response->command;

    /* 数据长度 */
    buffer[idx++] = response->data_len;

    /* 数据 */
    if (response->data_len > 0) {
        memcpy(&buffer[idx], response->data, response->data_len);
        idx += response->data_len;
    }

    /* CRC16 (小端序: 低字节在前) */
    /*
     * CRC 计算范围：command + len + data (不含 SOF/EOF)
     */
    {
        uint8_t crc_input[FRAME_MAX_DATA_LEN + 2];
        crc_input[0] = response->command;
        crc_input[1] = response->data_len;
        if (response->data_len > 0) {
            memcpy(&crc_input[2], response->data, response->data_len);
        }
        crc = CRC16_Calculate(crc_input, response->data_len + 2);
    }
    buffer[idx++] = crc & 0xFF;          /* CRC 低字节 */
    buffer[idx++] = (crc >> 8) & 0xFF;   /* CRC 高字节 */

    /* 帧尾 */
    buffer[idx++] = FRAME_EOF;

    return idx;  /* 总长度 */
}

/*===========================================================================
 * 命令注册与派遣
 *===========================================================================*/

void Protocol_RegisterHandlers(CmdRegistry_t *registry, uint8_t registry_size)
{
    g_cmd_registry = registry;
    g_cmd_count = registry_size;
}

/**
 * 命令派遣：查表找到对应的处理器并调用
 */
uint8_t Protocol_Dispatch(const CommFrame_t *request, CommFrame_t *response)
{
    uint8_t i;

    if (request == NULL || response == NULL) return 0;
    if (g_cmd_registry == NULL) return 0;

    /* 查找处理器 */
    for (i = 0; i < g_cmd_count; i++) {
        if (g_cmd_registry[i].command == request->command) {
            if (g_cmd_registry[i].handler != NULL) {
                g_cmd_registry[i].handler(request, response);
                return 1;
            }
        }
    }

    /* 未知命令 → 返回 NAK */
    response->command = CMD_ACK | request->command;  /* bit7=1 表示应答 */
    response->data_len = 1;
    response->data[0] = 0x01;  /* 错误码: 未知命令 */
    response->valid = 1;

    return 0;
}
