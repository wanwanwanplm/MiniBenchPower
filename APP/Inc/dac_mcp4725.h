/**
 * dac_mcp4725.h
 * MCP4725 12-bit DAC 驱动 (I²C, 双芯片)
 *
 * 驱动两片 MCP4725：
 *   - #1 @ 0x60: 控制 Buck FB 节点 (XL4016 输出电压, 反比)
 *   - #2 @ 0x61: 控制线性 P-MOS 栅极 (IRF9540, 经 LM358 闭环, 正比)
 *
 * 【v2.0 接口】DAC#2 映射：V_out = V_DAC2 × 10 (线性正比)
 *   DAC#2 软件限幅至 DAC2_MAX_CODE=3475 (对应 V_out ≈ 28V)
 *   [问题 #24 修正] 旧注释写 "3636/30V" 属 v1.0 遗留, 已更新为 3475/28V
 *
 * ─────────────────────────────────────────────────────────────────────────
 * 🔧 算法：MCP4725 Fast Write Mode 位打包 (问题 #1 — 最致命 bug)
 *   📐 数学原理：12-bit 码值 code∈[0,4095] 要拆成两个 I²C 数据字节发出。
 *      Fast Write 帧 (地址字节之后, MSB first, 逐 bit):
 *        字节1 = [ 0 | 0 | PD1 | PD0 | D11 | D10 | D9 | D8 ]
 *        字节2 = [ D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0 ]
 *      高 2 bit 固定为 0, 紧接 PD1:PD0 (省电模式, 正常输出=00), 再接数据高 4 位。
 *   💻 代码映射 (见 .c 的 DAC_WriteCode)：
 *        字节1 = MCP4725_CMD_FAST_WRITE | ((code >> 8) & 0x0F)   ← PD=00 + D11:D8
 *        字节2 = code & 0xFF                                      ← D7:D0
 *   ⚠️ 常见坑：v1.0 代码错误地用 (code>>6)&0x3F 和 (code&0x3F)<<2 打包,
 *      把 12 位数据劈成"高6位+低6位<<2", 导致输出电压严重错误 (整条控制链失效)。
 *      本版按数据手册 Figure 6-2 修正为"高4位+低8位"。
 * ─────────────────────────────────────────────────────────────────────────
 */

#ifndef __DAC_MCP4725_H
#define __DAC_MCP4725_H

#include "app_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 硬件常量
 *===========================================================================*/

/*
 * MCP4725 Fast Write 命令位:
 *   字节1 的 bit7:6 恒为 0, bit5:4 = PD1:PD0 (省电选择)。
 *   正常输出模式 PD1:PD0 = 00, 故 Fast Write 命令基值 = 0x00。
 *   (省电模式: 01=1kΩ, 10=100kΩ, 11=500kΩ 下拉到 GND, 本项目不使用)
 */
#define MCP4725_CMD_FAST_WRITE      0x00    /* Fast Write, PD=00 正常模式 */
#define MCP4725_WRITE_TIMEOUT_MS    10      /* I²C 写入超时 (ms) */

/* DAC 满量程码值 (12-bit) */
#define MCP4725_MAX_CODE            4095U   /* 2^12 - 1 */

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 配置 I²C1 外设 (400kHz Fast Mode, PB6/PB7)
 *
 * 为什么需要：把共享句柄 hi2c1 配置为 400kHz Fast Mode 主机模式。
 * 输入：无
 * 输出：无
 * 调用时机：main() 硬件初始化序列中 (GPIO 复用已配好之后)。
 * 副作用：使能 I2C1 时钟并对全局 hi2c1 执行 HAL_I2C_Init。
 *
 * ⚠️ 句柄单一来源 (根治 #7)：hi2c1 由 main.c (CubeMX 层) 定义, 本驱动
 *    只通过 main.h 的 extern 引用同一实例, 绝不再定义 static 副本。
 */
//void DAC_Init(void);

/**
 * @brief 上电清零 —— 向两片 MCP4725 写入安全值
 *
 * 为什么必须清零：MCP4725 上电会从内部 EEPROM 加载上次的码值。若上次关机
 *   时 DAC#2 停在高压, 上电瞬间 LM358 会驱动 P-MOS 输出高压 —— 非常危险。
 * 输入：无
 * 输出：uint8_t 位掩码, 0=全部成功; bit0=DAC1 失败, bit1=DAC2 失败
 * 调用时机：main() 中, 输出使能 (PA15) 之前, 且 DAC_Init 之后。
 * 副作用：DAC#2 写 0 (P-MOS 截止), DAC#1 写 DAC1_MAX_CODE (Buck 输出最低)。
 */
uint8_t DAC_PowerOnZero(void);

/**
 * @brief 设置 DAC#1 (Buck FB 控制) 码值 —— 自动限幅
 *
 * 输入：code —— 目标码值, 内部限幅到 [DAC1_MIN_CODE, DAC1_MAX_CODE]
 * 输出：0=成功, 非 0=I²C 错误
 * 调用时机：PID 任务算出 Buck 目标后。
 * 副作用：一次 I²C Fast Write。
 *
 * 反比映射:
 *   V_buck ≈ 32.5 - 10 × V_DAC1,  V_DAC1 = code/4096 × 3.3V
 *   code ↑ → V_buck ↓
 */
uint8_t DAC1_SetCode(uint16_t code);

/**
 * @brief 设置 DAC#2 (线性栅极控制) 码值 —— 自动限幅
 *
 * 输入：code —— 目标码值, 内部限幅到 [0, DAC2_MAX_CODE]
 * 输出：0=成功, 非 0=I²C 错误
 * 调用时机：PID 任务算出输出目标后。
 * 副作用：一次 I²C Fast Write。
 *
 * 【v2.0】正比映射 (硬件假设: docs/04 第4.2节):
 *   V_out = V_DAC2 × 10 = (code/4096 × 3.3V) × 10
 *   DAC=0      → V_out = 0V   (P-MOS 截止)
 *   DAC=3475   → V_out ≈ 28V  (软件上限)
 */
uint8_t DAC2_SetCode(uint16_t code);

/**
 * @brief 同时设置两路 DAC（PID 控制周期调用）
 *
 * 输入：code1 (DAC#1 Buck FB), code2 (DAC#2 线性栅极), 各自内部限幅
 * 输出：0=全部成功, bit0=DAC1 失败, bit1=DAC2 失败
 * 调用时机：PID 任务每 10ms。
 * 副作用：两次 I²C Fast Write (先 DAC#1 粗调, 后 DAC#2 精调)。
 */
uint8_t DAC_SetBoth(uint16_t code1, uint16_t code2);

/**
 * @brief I²C 总线软件复位（解除 SDA 死锁）
 *
 * 为什么需要：从设备在传输中途掉电可能一直拉低 SDA 导致总线死锁。
 * 输入：无  输出：无
 * 调用时机：I²C 操作超时后 (DAC_WriteCode 内部自动调用)。
 * 副作用：临时把 PB6/PB7 切为 GPIO 发 ≤9 个 SCL 脉冲 + STOP, 再重配 I²C。
 */
void DAC_I2C_SoftwareReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __DAC_MCP4725_H */
