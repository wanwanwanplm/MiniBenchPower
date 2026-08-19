/**
 * eeprom_emulate.h
 * 内部 Flash 模拟 EEPROM (参数存储)
 * STM32F103C8T6 没有硬件 EEPROM, 用内部 Flash 最后一页
 * (0x0800FC00 ~ 0x0800FFFF, 1KB) 模拟。
 *
 * 存储内容: 校准系数、上次用户设定、4 个预设电压、CV/CC 的 PID 参数。
 *
 * F103 没有 EEPROM：
 * 用内部 Flash 模拟。Flash 擦除以页为单位 (1KB), 写入以半字 (16-bit)
 * 为单位; 且只能 1→0, 覆写前必须整页擦除。策略见 .c: 页内轮转追加写记录,
 * 写满整页才擦除 (磨损均衡)。
 * F103 Flash 按半字 (16-bit) 编程, 记录体 sizeof 必须为偶数,
 *   否则 .c 里 count=sizeof/2 会漏掉最后 1 字节。EEPROM_Config_t 通过显式
 *   padding 保证 sizeof 为偶数。又因结构体 (76B) > 旧 RECORD_SIZE-4(60B),
 *   将 EEPROM_RECORD_SIZE 从 64 提升到 128 (每页 1024/128=8 记录, 满足磨损均衡)。
 * ─────────────────────────────────────────────────────────────────────────
 */

#ifndef __EEPROM_EMULATE_H
#define __EEPROM_EMULATE_H

#include "app_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * Flash 页地址定义
 *===========================================================================*/

/*
 * STM32F103C8T6 Flash: 64KB, 页 1KB, 最后一页 0x0800FC00 (页号 63)。
 */
#define FLASH_EEPROM_BASE_ADDR      0x0800FC00
#define FLASH_EEPROM_PAGE_SIZE      1024        /* 1KB */
#define FLASH_EEPROM_PAGE_NUMBER    63          /* 最后一页 (0-indexed) */

/*
 * 磨损均衡: 页内每条记录占 EEPROM_RECORD_SIZE 字节 (含 2B magic + 数据 + padding)。
 * [问题 #4] RECORD_SIZE 64→128: 因 EEPROM_Config_t=76B 超过 64-4。
 * 一页可存 1024/128 = 8 条记录, 写满才擦整页。
 */
#define EEPROM_RECORD_SIZE          128
#define EEPROM_MAX_RECORDS          (FLASH_EEPROM_PAGE_SIZE / EEPROM_RECORD_SIZE)  /* 8 */

/* 记录头部标记 (有效记录) */
#define EEPROM_RECORD_MAGIC         0xAA55

/*===========================================================================
 * 存储数据结构
 *===========================================================================*/

/**
 * @brief EEPROM 存储的配置数据
 *
 * ⚠️ 布局约束 (问题 #4): sizeof 必须为偶数 (半字编程)。
 *   float 天然 4 字节对齐; output_enable (1B) 后显式补 _pad[3], 使后续
 *   preset_voltages 对齐且整体 sizeof = 76 (偶数, 38 个半字)。
 *   实测: 16(cal)+8(set)+1(en)+3(pad)+16(preset)+24(pid)+8(rsv) = 76B。
 */
typedef struct {
    /* 校准参数 (16B) */
    float v_cal_slope;
    float v_cal_offset;
    float i_cal_slope;
    float i_cal_offset;

    /* 用户上次设定 (8B) */
    float v_set;
    float i_set;

    /* 输出使能 + 显式对齐填充 (4B) —— 保证 sizeof 偶数, 见 #4 */
    uint8_t output_enable;
    uint8_t _pad[3];

    /* 预设电压值 4 个 (16B) */
    float preset_voltages[4];

    /* PID 参数 (24B) */
    float pid_cv_kp;
    float pid_cv_ki;
    float pid_cv_kd;
    float pid_cc_kp;
    float pid_cc_ki;
    float pid_cc_kd;

    /* 保留 (8B) */
    uint8_t reserved[8];
} EEPROM_Config_t;

/*
 * 编译期断言 (问题 #3: 替代 C11 _Static_assert):
 *   条件为真 → typedef char[1] (合法); 为假 → char[-1] (编译错误)。
 *   要求: 数据体 ≤ RECORD_SIZE-4 (预留 2B magic + 2B 对齐余量), 且 sizeof 为偶数。
 */
typedef char eeprom_size_le_check[(sizeof(EEPROM_Config_t) <= (EEPROM_RECORD_SIZE - 4)) ? 1 : -1];
typedef char eeprom_size_even_check[((sizeof(EEPROM_Config_t) % 2) == 0) ? 1 : -1];

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化 EEPROM 模拟模块
 * 调用时机: main() 初始化阶段 (FreeRTOS 启动前, Flash 操作耗时)。
 */
void EEPROM_Init(void);

/**
 * @brief 从 Flash 加载最新一条有效配置
 * 输入：config —— 输出缓冲
 * 输出：1=加载成功; 0=首次使用 (已填默认值)
 * 调用时机：上电初始化。
 */
uint8_t EEPROM_LoadConfig(EEPROM_Config_t *config);

/**
 * @brief 保存配置到 Flash (页内追加, 满则擦页重来)
 * 输入：config —— 待保存数据
 * 输出：1=成功, 0=失败
 * 调用时机：用户改校准/预设/PID 后, 或正常关机前。
 * 副作用：解锁 Flash, 半字写入 (count=sizeof/2), 可能触发整页擦除, 最后锁定。
 */
uint8_t EEPROM_SaveConfig(const EEPROM_Config_t *config);

/**
 * @brief 加载出厂默认配置到 config (不访问 Flash)
 */
void EEPROM_LoadDefaults(EEPROM_Config_t *config);

/**
 * @brief 擦除整个 EEPROM 页 (恢复出厂)
 * 输出：1=成功, 0=失败
 */
uint8_t EEPROM_ErasePage(void);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_EMULATE_H */
