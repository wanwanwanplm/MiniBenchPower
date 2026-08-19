/**
 * protocol.h
 * 二进制通信协议 (帧解析 + 环形缓冲 + CRC16)
 *
 * 自定义二进制帧协议，比 Modbus 轻量，比 ASCII 高效。
 *
 * 帧格式：
 *   ┌──────┬──────┬────┬──────────┬────────┬──────┐
 *   │ 帧头  │ 命令  │长度│   数据    │ CRC16  │ 帧尾  │
 *   │ 0xA5 │ 1Byte│ 1B │ 0~64 Byte│ 2 Byte │ 0x5A │
 *   └──────┴──────┴────┴──────────┴────────┴──────┘
 *
 * 支持的命令：
 *   0x01: 读取实时数据
 *   0x02: 设定目标电压/电流
 *   0x03: 在线调 PID 参数
 *   0x04: 读取/写入预设值
 *   0x05: 使能/关断输出
 *   0x06: 保存配置到 Flash
 *   0x07: 恢复出厂设置
 *
 * 为什么用二进制帧而不是 ASCII/JSON？
 *   答：(1) 二进制帧更紧凑——同样的信息需要的字节少。
 *     (2) 解析快——不需要字符串比较。
 *     (3) 适合 MCU 资源受限环境（JSON 解析器占很多 Flash）。
 *     (4) 带 CRC16 校验确保数据完整性。
 *
 * CRC16 为什么重要？
 *   答：UART 通信没有硬件纠错。如果传输中一位翻转（EMI 干扰），
 *     导致设定电压从 5V 变成 50V —— 后果严重。
 *     CRC16 能检测绝大多数传输错误（HD=4 保证检测 ≤3 位错误）。
 */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "app_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 常量定义
 *===========================================================================*/

/* 命令码 */
#define CMD_READ_DATA           0x01    /* 读取实时数据 */
#define CMD_SET_V_I             0x02    /* 设定目标电压/电流 */
#define CMD_SET_PID             0x03    /* 在线调 PID 参数 */
#define CMD_PRESET              0x04    /* 读取/写入预设值 */
#define CMD_OUTPUT_CTRL         0x05    /* 使能/关断输出 */
#define CMD_SAVE_CONFIG         0x06    /* 保存配置到 Flash */
#define CMD_FACTORY_RESET       0x07    /* 恢复出厂设置（擦除 EEPROM 页） */
#define CMD_ACK                 0x80    /* 应答 (bit7=1) */

/* 帧边界 */
#define FRAME_SOF               0xA5    /* Start of Frame */
#define FRAME_EOF               0x5A    /* End of Frame */

/*===========================================================================
 * 类型定义
 *===========================================================================*/

/**
 * @brief 环形缓冲区 (Ring Buffer / Circular Buffer)
 *
 * ISR 写入 (生产者), vTaskComm 读取 (消费者)
 *
 * 为什么用环形缓冲区？
 *   1. 无锁设计——单生产者+单消费者不需要互斥锁
 *   2. 固定内存——不需要动态分配
 *   3. O(1) 读写——不需要移动数据
 *
 * 环形缓冲区怎么判断满/空？
 *   答：两种方法：
 *     ① 浪费一个位置：tail + 1 == head → 满; tail == head → 空
 *     ② 用计数器：count == size → 满; count == 0 → 空
 *     本项目用方法①（更常用）。
 */
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;     /* 写入索引 (ISR 操作) */
    volatile uint16_t tail;     /* 读取索引 (任务操作) */
} RingBuffer_t;

/**
 * @brief 通信帧
 */
typedef struct {
    uint8_t command;                    /* 命令码 */
    uint8_t data_len;                   /* 数据长度 */
    uint8_t data[FRAME_MAX_DATA_LEN];   /* 数据 */
    uint16_t crc;                       /* CRC16 */
    uint8_t valid;                      /* 帧有效标志 */
} CommFrame_t;

/**
 * @brief 命令处理函数类型
 *
 * 每个命令的处理函数接收帧数据，返回响应数据。
 */
typedef void (*CmdHandler_t)(const CommFrame_t *request, CommFrame_t *response);

/**
 * @brief 命令处理器注册表
 */
typedef struct {
    uint8_t command;
    CmdHandler_t handler;
} CmdRegistry_t;

/*===========================================================================
 * 环形缓冲区 API
 *===========================================================================*/

/**
 * @brief 初始化环形缓冲区（清零）
 *
 * @param rb  环形缓冲区指针
 */
void RingBuffer_Init(RingBuffer_t *rb);

/**
 * @brief 向环形缓冲区写入一个字节
 *
 * @param rb    环形缓冲区指针
 * @param data  要写入的字节
 * @return uint8_t  1 = 成功, 0 = 缓冲区满
 *
 * 调用时机：USART1 RX ISR (每收到一个字节调用一次)
 */
uint8_t RingBuffer_Write(RingBuffer_t *rb, uint8_t data);

/**
 * @brief 从环形缓冲区读取一个字节
 *
 * @param rb    环形缓冲区指针
 * @param data  输出：读取的字节
 * @return uint8_t  1 = 成功, 0 = 缓冲区空
 */
uint8_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data);

/**
 * @brief 获取环形缓冲区中的可用字节数
 *
 * @param rb  环形缓冲区指针
 * @return uint16_t  可用字节数
 */
uint16_t RingBuffer_Available(RingBuffer_t *rb);

/*===========================================================================
 * 帧解析 API
 *===========================================================================*/

/**
 * @brief 帧解析状态机
 *
 * 状态转换：
 *   STATE_IDLE ──(收到 0xA5)──> STATE_HEADER_RCVD
 *                  ──(收到 0xA5)──> STATE_WAIT_LENGTH
 *                  ──(收到长度)──> STATE_WAIT_DATA
 *                  ──(收齐数据)──> STATE_WAIT_CRC1
 *                  ──(收齐 CRC)──> STATE_WAIT_CRC2 → 校验 → 入队列
 *                  ──(收到 0x5A)──> 帧结束 → 验证 CRC → 成功/失败
 *
 * 调用时机：vTaskComm 检测到环形缓冲区有数据时
 *
 * @param rb      环形缓冲区指针
 * @param frame   输出：解析完成的帧
 * @return uint8_t  1 = 成功解析一帧, 0 = 暂无完整帧
 */
uint8_t Protocol_ParseFrame(RingBuffer_t *rb, CommFrame_t *frame);

/*===========================================================================
 * CRC16 API
 *===========================================================================*/

/**
 * @brief 计算 CRC16 (CCITT 多项式)
 *
 * 多项式: 0x1021 (X^16 + X^12 + X^5 + 1)
 * 初始值: 0xFFFF
 *
 * @param data   数据缓冲区
 * @param len    数据长度
 * @return uint16_t  CRC16 值
 *
 * CRC 和 Checksum 有什么区别？
 *   答：Checksum 只是简单的加法/异或，能检测随机错误但防不了
 *     "两个字节互换"或"两个位同时翻转"等问题。
 *     CRC 基于多项式除法，能检测的内容更全面。
 */
uint16_t CRC16_Calculate(const uint8_t *data, uint16_t len);

/*===========================================================================
 * 帧封装 API
 *===========================================================================*/

/**
 * @brief 构造响应帧
 *
 * @param response  响应帧结构体
 * @param buffer    输出缓冲区 (用于 UART TX DMA)
 * @param buf_len   缓冲区大小
 * @return uint16_t  实际帧长度 (0 = 失败)
 *
 * 帧格式最终封装：
 *   [0xA5] [cmd] [len] [data...] [CRC_L] [CRC_H] [0x5A]
 */
uint16_t Protocol_BuildResponse(const CommFrame_t *response,
                                uint8_t *buffer, uint16_t buf_len);

/*===========================================================================
 * 命令注册 API
 *===========================================================================*/

/**
 * @brief 注册命令处理器
 *
 * @param registry     注册表数组
 * @param registry_size 注册表大小
 *
 * 调用时机：通信任务初始化时
 */
void Protocol_RegisterHandlers(CmdRegistry_t *registry, uint8_t registry_size);

/**
 * @brief 派遣命令到对应的处理器
 *
 * @param request   请求帧
 * @param response  响应帧 (输出)
 * @return uint8_t  1 = 找到处理器并执行, 0 = 未知命令
 */
uint8_t Protocol_Dispatch(const CommFrame_t *request, CommFrame_t *response);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
