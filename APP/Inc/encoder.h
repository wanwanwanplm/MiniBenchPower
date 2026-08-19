/**
 * encoder.h
 * EC11 旋转编码器驱动 (状态机消抖 + 方向解码 + 长按/双击识别)
 *
 * 硬件连接：
 *   A 相 → PB0 (EXTI0, 上下沿中断)
 *   B 相 → PB1 (GPIO Input, 在 ISR 中读电平判方向)
 *   SW  → PB10 (EXTI10, 下降沿中断)
 *
 * 为什么编码器消抖用状态机而不是延时？
 *   答：延时消抖（delay）会阻塞 CPU。状态机消抖不阻塞——
 *     在每次中断中检查时间间隔，状态转换决策，然后立即返回。
 *     这对 RTOS 环境尤其重要——ISR 永远不能长时间阻塞。
 *
 * 如何判断旋转方向？
 *   答：EC11 输出两路正交信号（A 和 B 相），A 超前 B 90°。
 *     在 A 相的下降沿（或上升沿）中断中读取 B 相电平：
 *       B = 0 → A 超前 → 顺时针 (CW)
 *       B = 1 → A 滞后 → 逆时针 (CCW)
 *     这利用了正交编码的原理——两个信号的相对相位决定了方向。
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "app_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 类型定义
 *===========================================================================*/

/**
 * @brief 编码器状态机 (消抖)
 *
 * 状态转换图：
 *
 *   IDLE ──(中断触发)──> PRESS_PENDING
 *                          │
 *                     (消抖定时到?)
 *                     ├─ 是 → CONFIRMED (产生事件)
 *                     └─ 否 → 等待下一个 tick
 *
 * 对于旋转：只需检查两次中断的间隔 > 消抖时间
 * 对于按键：需要跟踪按下时长来区分短按/长按
 */
typedef enum {
    ENC_STATE_IDLE = 0,         /* 空闲，等待事件 */
    ENC_STATE_ROTATING,         /* 正在旋转（消抖中） */
    ENC_STATE_BUTTON_DOWN,      /* 按键按下（等待释放或长按） */
    ENC_STATE_BUTTON_DEBOUNCE,  /* 按键消抖中 */
} EncoderState_t;

/**
 * @brief 编码器全局上下文
 */
typedef struct {
    /* 旋转相关 */
    volatile int32_t  pulse_count;       /* 脉冲计数 (CW=+, CCW=-) */
    volatile uint32_t last_rotate_tick;  /* 上次旋转的时间戳 (ms) */

    /* 按键相关 */
    volatile uint32_t button_down_tick;  /* 按键按下的时刻 (ms) */
    volatile uint8_t  button_pressed;    /* 按键当前状态：0=释放, 1=按下 */
    volatile uint8_t  button_released;   /* 按键已释放标志 */

    /* 双击检测 */
    volatile uint32_t last_click_tick;   /* 上一次点击的时刻 */
    volatile uint8_t  click_count;       /* 点击计数 (用于双击检测) */

    volatile EncoderEvent_t button_event;  /* 待消费的按键事件 (SHORT/LONG/DOUBLE) */
    volatile uint8_t        button_event_ready;  /* 按键事件就绪标志 */

    /* 加速功能 */
    uint32_t          accel_threshold;    /* 加速阈值 (脉冲间隔 ms) */
    uint32_t          accel_multiplier;   /* 加速倍数 */

    /* 状态机 */
    EncoderState_t    state;             /* 当前状态 */
} EncoderContext_t;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化编码器上下文和硬件
 *
 * @param ctx  编码器上下文指针
 *
 * 调用时机：main.c 初始化序列中
 *
 * 硬件配置：
 *   - PB0: GPIO Input, 内部上拉, EXTI0 (上下沿中断)
 *   - PB1: GPIO Input, 内部上拉
 *   - PB10: GPIO Input, 内部上拉, EXTI10 (下降沿中断)
 */
void Encoder_Init(EncoderContext_t *ctx);

/**
 * @brief 获取编码器上下文指针 (供 ISR 使用)
 *
 * @return EncoderContext_t*  全局编码器上下文
 *
 * ISR 需要访问编码器上下文，但不能传参数。
 * 此函数返回全局指针。
 */
EncoderContext_t* Encoder_GetContext(void);

/**
 * @brief 编码器 A 相中断处理 (在 EXTI0 ISR 中调用)
 *
 * 调用时机：EXTI0_IRQHandler (PB0 上下沿中断)
 *
 * 功能：
 *   1. 消抖检查（距上次中断间隔 > 消抖时间？）
 *   2. 读取 PB1 电平判断旋转方向
 *   3. 更新脉冲计数
 *   4. 加速检测（快速旋转时步进增大）
 */
void Encoder_ISR_A(void);

/**
 * @brief 编码器按键中断处理 (在 EXTI15_10 ISR 中调用)
 *
 * 调用时机：EXTI15_10_IRQHandler (PB10 下降沿中断)
 *
 * 功能：
 *   1. 记录按键按下时刻
 *   2. 后续在 UI 任务中判断短按/长按/双击
 */
void Encoder_ISR_Button(void);

/**
 * @brief 编码器定时处理 (在 vTaskUI 中每 100ms 调用)
 *
 * 调用时机：vTaskUI 任务循环中
 *
 * 功能：
 *   1. 检测按键释放
 *   2. 判断短按/长按/双击
 *   3. 生成编码器事件放入 UI 事件队列
 */
void Encoder_Process(EncoderContext_t *ctx);

/**
 * @brief 获取并清除待处理的按键事件 (#14/#15: 只返回按键事件)
 *
 * @param ctx   编码器上下文指针
 * @return EncoderEvent_t  按键事件 (SHORT_PRESS / LONG_PRESS / DOUBLE_CLICK) 或 NONE
 *
 * 注意: 旋转事件不经此函数, 由 Encoder_GetStep() 单独消费 (正负号即方向)。
 *       本函数永不返回 CW/CCW。
 *
 * 调用时机：vTaskUI 消费按键事件时。
 */
EncoderEvent_t Encoder_GetEvent(EncoderContext_t *ctx);

/**
 * @brief 获取旋转步进值（含加速）
 *
 * @param ctx   编码器上下文指针
 * @return int32_t  步进值 (正值=CW, 负值=CCW)
 *
 * 加速逻辑：
 *   旋转速度 > 阈值 → 步进 × 加速倍数
 *   旋转速度 < 阈值 → 步进 = 1
 *
 * 步进值基于脉冲计数计算，读取后清零计数。
 */
int32_t Encoder_GetStep(EncoderContext_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
