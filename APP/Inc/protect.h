/**
 * protect.h
 * 保护逻辑 (过压/过流/过功率/过温检测 + 故障管理)

 *
 * 多层保护架构：
 *   第 0 层 (硬件, 不可绕过):
 *     - LM393 → 2N5551 → 拉低 LM358 PIN3 → P-MOS 截止 (<5μs) [v3.0D]
 *     - 5A 保险丝 (过流熔断)
 *   第 1 层 (软件, 本模块):
 *     - OVP: 过压保护 (28.5V 软件阈值) [v2.0]
 *     - OCP: 过流保护 (3.2A 软件阈值)
 *     - OPP: 过功率保护 (88W 阈值) [v2.0]
 *     - OTP: 过温保护 (80°C 阈值)
 *   第 2 层 (硬件 OVP 通知):
 *     - PA4 检测到 LM393 输出翻高 (上升沿) → 软件拉 PA15=0 (双保险) + 蜂鸣器报警 [v3.0D]
 *
 * 为什么需要多层保护？
 *   答：硬件保护是最快的（<5μs），不依赖软件。
 *     软件可能死机、跑飞、响应延迟（ms 级）。
 *     对于电源产品，过压可能在 ms 级烧坏负载——
 *     所以硬件保护是必须的，软件保护是补充。
 *     硬件和软件两条路径共用同一 LM358 PIN3 关断机制，
 *     一条失效另一条仍能保护——这就是"纵深防御"(Defense in Depth) 思想。
 */

#ifndef __PROTECT_H
#define __PROTECT_H

#include "app_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 类型定义
 *===========================================================================*/

/**
 * @brief 故障记录 (用于诊断)
 */
typedef struct {
    FaultFlag_t type;            /* 故障类型 */
    uint32_t    timestamp_ms;    /* 发生时刻 */
    float       v_out;          /* 故障时的输出电压 */
    float       i_out;          /* 故障时的输出电流 */
    float       v_in;           /* 故障时的输入电压 */
    float       temperature;    /* 故障时的温度 */
} FaultRecord_t;

/**
 * @brief 保护模块上下文
 */
typedef struct {
    FaultFlag_t     active_faults;   /* 当前活跃的故障标志 */
    uint8_t         output_latched_off; /* 输出已锁存关断 */
    uint32_t        fault_timestamp; /* 最近一次故障的时刻 */
    uint8_t         fault_retry_count; /* 故障重试次数 */
    FaultRecord_t   last_fault;     /* 最近一次故障记录 */
} ProtectContext_t;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化保护模块
 *
 * @param ctx  保护上下文指针
 *
 * 调用时机：系统初始化时
 */
void Protect_Init(ProtectContext_t *ctx);

/**
 * @brief 运行保护检查
 *
 * @param ctx       保护上下文
 * @param adc_data  当前 ADC 采样数据
 * @return FaultFlag_t  检测到的故障 (0 = 正常)
 *
 * 检查项目：
 *   1. 过压 (V_out > OVP_THRESHOLD)
 *   2. 过流 (I_out > OCP_THRESHOLD)
 *   3. 过功率 (V_out × I_out > OPP_THRESHOLD)
 *   4. 过温 (Temp > OTP_THRESHOLD)
 *
 * 调用时机：vTaskMonitor 每 500ms 调用一次
 *
 * 返回非零值 → 调用方应：
 *   1. 设置 PA15 = 0 (关断输出)
 *   2. 启动蜂鸣器报警
 *   3. 记录故障
 *   4. 通知上位机
 */
FaultFlag_t Protect_Check(ProtectContext_t *ctx, const ADCData_t *adc_data);

/**
 * @brief 处理硬件 OVP 中断通知 [v2.0 修正]
 *
 * @param ctx  保护上下文
 *
 * 调用时机：EXTI4 ISR (PA4 上升沿) [v3.0D]
 *
 * PA4 = 1 → LM393 输出翻高 → 2N5551 已拉低 LM358 PIN3 (硬件关断)
 * 软件做双保险：
 *   1. PA15=0 → D_en 同样拉低 PIN3 (与硬件路径并联)
 *   2. 设置 FAULT_OVP_HARDWARE 故障标志
 *
 * 注意: EXTI4 优先级 3, 高于 FreeRTOS 阈值, 不能调用 FreeRTOS API
 */
void Protect_HardwareOVP(ProtectContext_t *ctx);

/**
 * @brief 硬件 OVP 中断标志 (由 EXTI4 ISR 置位)
 *
 * EXTI4 ISR 优先级 3 高于 FreeRTOS 阈值, 不能调用 FreeRTOS API,
 * 也不能安全访问 ProtectContext_t (可能正被任务修改)。
 * 因此 ISR 只置位这个 volatile 标志, 由 vTaskMonitor 轮询后
 * 合并进 ProtectContext (置 FAULT_OVP_HARDWARE + 锁存)。
 *
 * 1 = 硬件 OVP 已触发, 0 = 未触发。清零由 Protect_ClearFaults 负责。
 */
extern volatile uint8_t g_hw_ovp_flag;

/**
 * @brief 清除故障（用户手动确认）
 *
 * @param ctx  保护上下文
 *
 * 调用时机：用户按下编码器按键确认故障清除
 *
 * 为什么需要手动确认？
 *   安全第一。如果自动恢复，负载故障（如短路）持续存在，
 *   系统会反复触发保护 → 瞬间输出 → 再次保护 → 反复，
 *   形成振荡。手动确认强制用户检查负载后再恢复。
 */
void Protect_ClearFaults(ProtectContext_t *ctx);

/**
 * @brief 检查是否有活跃故障
 *
 * @param ctx  保护上下文
 * @return uint8_t  1 = 有故障, 0 = 无故障
 */
uint8_t Protect_HasFault(ProtectContext_t *ctx);

/**
 * @brief 获取故障描述字符串 (用于显示)
 *
 * @param fault  故障标志
 * @param buf    输出缓冲区
 * @param buf_size 缓冲区大小
 */
void Protect_GetFaultString(FaultFlag_t fault, char *buf, uint16_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* __PROTECT_H */
