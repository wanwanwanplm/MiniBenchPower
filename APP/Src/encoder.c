/**
 * encoder.c
 * EC11 旋转编码器驱动实现
 *
 * 实现功能：
 *   1. 状态机消抖（非阻塞，适合 RTOS 环境）
 *   2. 正交信号方向解码
 *   3. 短按/长按/双击识别
 *   4. 旋转加速功能
 *
 * 状态机消抖
 *   传统的 delay() 消抖在 ISR 中不能用（阻塞 CPU）。
 *   状态机消抖的原理：
 *     - 每次中断进来只记录时间戳
 *     - 在 task 中检查：if(now - last_tick > debounce_time) → 确认事件
 *     - ISR 保持极快（< 1μs），延迟检查在任务中进行
 */

#include "encoder.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"           /* taskENTER_CRITICAL / taskEXIT_CRITICAL */

/*===========================================================================
 * 全局变量
 *===========================================================================*/

static EncoderContext_t g_encoder_ctx;
static EncoderContext_t *p_encoder = NULL;

/*===========================================================================
 * 初始化
 *===========================================================================*/

void Encoder_Init(EncoderContext_t *ctx)
{
    if (ctx == NULL) {
        p_encoder = &g_encoder_ctx;
    } else {
        p_encoder = ctx;
    }

    /* 清零所有状态 */
    p_encoder->pulse_count = 0;
    p_encoder->last_rotate_tick = 0;
    p_encoder->button_down_tick = 0;
    p_encoder->button_pressed = 0;
    p_encoder->button_released = 0;
    p_encoder->last_click_tick = 0;
    p_encoder->click_count = 0;
    p_encoder->button_event = ENC_EVENT_NONE;   /* #14/#15: 独立按键事件槽 */
    p_encoder->button_event_ready = 0;
    p_encoder->accel_threshold = ENCODER_ACCEL_THRESHOLD;
    p_encoder->accel_multiplier = ENCODER_ACCEL_MULTIPLIER;
    p_encoder->state = ENC_STATE_IDLE;

    /*
     * 注意：GPIO 初始化在 main.c 中统一配置。
     * EXTI 中断配置也在 main.c 的 GPIO 初始化部分。
     * 这里只初始化软件状态。
     */

}

EncoderContext_t* Encoder_GetContext(void)
{
    return p_encoder;
}

/*===========================================================================
 * ISR 处理函数
 *===========================================================================*/

/**
 * 编码器 A 相中断 (PB0, EXTI0)
 *
 * 在 EXTI0_IRQHandler 中调用。
 *
 * 方向判定原理：
 *   EC11 编码器输出两路方波（A 和 B），90° 相位差。
 *   顺时针旋转：A 超前 B 90° (A 先变化)
 *   逆时针旋转：B 超前 A 90° (B 先变化)
 *
 *   在 A 相的边沿（上升或下降）：
 *     如果 B = 0 → A 超前 B → 顺时针 (CW)
 *     如果 B = 1 → A 滞后 B → 逆时针 (CCW)
 *
 *   参考：EC11 数据手册的时序图。
 *
 * 为什么上下沿都触发？
 *   EC11 每圈 20 个脉冲（一个周期 = 一个完整的高低变化）。
 *   如果只在下降沿触发 → 20 步/圈。
 *   如果上下沿都触发 → 40 步/圈（4 倍频如果在 B 相也触发可达 80 步/圈）。
 *   本项目用上下沿触发（40 步/圈），手感好且不丢步。
 */
void Encoder_ISR_A(void)
{
    uint32_t now;
    uint8_t  a_level, b_level;

    if (p_encoder == NULL) return;

    now = HAL_GetTick();

    /*
     * Bug 2 修复: 消抖改为"同 1ms 窗内仅处理首边沿"。
     *   机械编码器人手旋转最快 ~3 rev/s = 120 edges/s = 8ms/edge,
     *   远大于 1ms 窗口, 正常旋转不会丢步。
     *   弹跳间隔通常 < 0.5ms, 会在同一 1ms 窗内, 被正确过滤。
     */
    if (now == p_encoder->last_rotate_tick) {
        return;  /* 同 1ms 窗内 → 弹跳, 忽略 */
    }

    /*
     * 正交解码 (Bug 1 修复): 读 A/B 电平, 用 XOR 判断方向。
     *
     *   A B | A^B | 方向
     *   ────┼─────┼─────
     *   0 0 |  0  | CW   (A↓, B=0)
     *   0 1 |  1  | CCW  (A↓, B=1)
     *   1 0 |  1  | CCW  (A↑, B=0)
     *   1 1 |  0  | CW   (A↑, B=1)
     *
     * 规律: A⊕B=0 → CW, A⊕B=1 → CCW
     */
    a_level = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) ? 0 : 1;
    b_level = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) ? 0 : 1;

    if ((a_level ^ b_level) == 0) {
        p_encoder->pulse_count++;   /* CW */
    } else {
        p_encoder->pulse_count--;   /* CCW */
    }

    p_encoder->last_rotate_tick = now;
}

/**
 * 编码器按键中断 (PB10, EXTI10)
 *
 * 在 EXTI15_10_IRQHandler 中调用。
 *
 * 这里只记录"按键按下"的时刻。
 * 实际的短按/长按/双击判断在 Encoder_Process() 中进行。
 *
 * 为什么不在 ISR 中判断长按？
 *   ISR 应该尽量短。长按需要等待 1 秒——在 ISR 中等 1 秒
 *   等于死机 1 秒。正确做法是记录时刻，在任务中检查时间差。
 */
void Encoder_ISR_Button(void)
{
    if (p_encoder == NULL) return;

    /*
     * Bug 4 修复: 按键按下弹跳不再重置计时起点。
     *   首次下降沿 → 记录 button_down_tick
     *   后续弹跳 → 忽略 (button_pressed 已置位)
     *
     * 释放时: Encoder_Process 检测到 PB10 回到高电平后
     *   清零 button_pressed, 为下次按下做准备。
     */
    if (p_encoder->button_pressed) {
        return;  /* 已按下, 弹跳 → 忽略 */
    }

    p_encoder->button_pressed = 1;
    p_encoder->button_down_tick = HAL_GetTick();
    p_encoder->button_released = 0;
}

/*===========================================================================
 * 定时处理 (在 vTaskUI 中调用)
 *===========================================================================*/

/**
 * 编码器事件处理
 *
 * 在每个 UI 周期（100ms）调用一次。
 *
 * 按键逻辑：
 *   1. 如果按键已按下且当前已释放：
 *      a. 按下时长 < 500ms → 短按
 *      b. 按下时长 > 1000ms → 长按
 *   2. 双击检测：两次短按间隔 < 300ms
 *
 * 旋转逻辑：
 *   1. 读取脉冲计数
 *   2. 根据旋转速度决定是否加速
 */
void Encoder_Process(EncoderContext_t *ctx)
{
    uint32_t now;
    uint32_t press_duration;
    uint32_t click_interval;

    if (ctx == NULL) return;

    now = HAL_GetTick();

    /*──────────────────────────────────────────────────
     * 处理旋转事件
     *
     * 旋转事件不在此处处理，由 Encoder_GetStep() 消费。
     * 这里只处理加速逻辑的状态更新。
     *──────────────────────────────────────────────────*/

    /*──────────────────────────────────────────────────
     * 处理按键事件
     *──────────────────────────────────────────────────*/

    if (ctx->button_pressed) {
        /*
         * 按键按下中 → 检查是否释放
         *
         * PB10 读取：内部上拉 + 外部上拉，
         * 按下 = 低电平, 释放 = 高电平
         */
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) {
            /*
             * 按键已释放！
             */
            ctx->button_released = 1;
            ctx->button_pressed = 0;

            press_duration = now - ctx->button_down_tick;

            /*
             * 消抖检查：按下时长 < 20ms → 认为是弹跳 → 忽略
             */
            if (press_duration < BUTTON_DEBOUNCE_MS) {
                /* 弹跳，忽略 */
                ctx->button_released = 0;
                ctx->click_count = 0;
                return;
            }

            /*
             * 判断按下类型
             */
            if (press_duration >= ENCODER_LONG_PRESS_MS) {
                /*
                 * 长按 (> 1000ms) → 输出使能/关断
                 */
                ctx->button_event = ENC_EVENT_LONG_PRESS;
                ctx->button_event_ready = 1;
                ctx->click_count = 0;  /* 长按不参与双击计数 */
            } else {
                /*
                 * 短按 (< 1000ms) → 切换调节位 或 双击检测
                 */

                /*
                 * 双击检测：
                 *   如果距离上次短按 < 300ms → 双击
                 *   否则 → 记录本次短按，等待可能的第二次
                 */
                click_interval = now - ctx->last_click_tick;

                if (ctx->click_count == 1 && click_interval < ENCODER_DOUBLE_CLICK_MS) {
                    /* 双击！ */
                    ctx->button_event = ENC_EVENT_DOUBLE_CLICK;
                    ctx->button_event_ready = 1;
                    ctx->click_count = 0;
                } else {
                    /* 第一次短按（或距离上次太久） */
                    ctx->click_count = 1;
                    ctx->last_click_tick = now;

                    /*
                     * 不立即触发短按事件——等待 300ms 看是否有第二次点击。
                     * 如果 300ms 内有第二次点击 → 双击
                     * 如果 300ms 内无第二次点击 → 确认为短按
                     *
                     * 这个"延迟确认"在下一次 Encoder_Process 调用的
                     * 超时检查中处理。
                     */
                }
            }

            ctx->button_released = 0;
        }
    }

    /*
     * 双击超时检查：
     *   如果 click_count == 1 且距离 last_click_tick > 300ms
     *   → 确认为短按事件
     */
    if (ctx->click_count == 1) {
        if ((now - ctx->last_click_tick) > ENCODER_DOUBLE_CLICK_MS) {
            ctx->button_event = ENC_EVENT_SHORT_PRESS;
            ctx->button_event_ready = 1;
            ctx->click_count = 0;
        }
    }

    /*
     * #14/#15 根治: 旋转事件不再写入任何"事件槽"。
     * 旋转完全由 pulse_count 表达, 经 Encoder_GetStep() 读并清零 (正负号即方向)。
     * 这样 Encoder_Process 绝不会覆盖上面刚判定的按键事件, 旋转和按键彻底解耦。
     * (旧代码此处有一段 "if pulse_count!=0 → pending_event=CW/CCW" 的覆盖逻辑, 已删除。)
     */
}

/*===========================================================================
 * 事件获取
 *===========================================================================*/

EncoderEvent_t Encoder_GetEvent(EncoderContext_t *ctx)
{
    EncoderEvent_t event;

    if (ctx == NULL) return ENC_EVENT_NONE;

    if (!ctx->button_event_ready) {
        return ENC_EVENT_NONE;
    }

    event = ctx->button_event;
    ctx->button_event = ENC_EVENT_NONE;
    ctx->button_event_ready = 0;

    return event;
}

/*===========================================================================
 * 旋转步进值（含加速）
 *===========================================================================*/

/**
 * 获取旋转步进值，含加速功能。
 *
 * 加速逻辑：
 *   如果最近两次旋转间隔 < accel_threshold (50ms)
 *   → 用户转得很快 → 步进 × accel_multiplier (4)
 *
 * 为什么要加速？
 *   从 3.3V 调到 24V，每次 0.01V 需要转 2070 步——手酸。
 *   快速旋转时加大步进（如每次 0.1V），慢速时保持精细调节。
 *   这是人机交互的基本设计——类似鼠标的加速功能。
 *
 * 读取后清零脉冲计数。
 */
int32_t Encoder_GetStep(EncoderContext_t *ctx)
{
    int32_t step;
    int32_t abs_step;

    if (ctx == NULL) return 0;

    /*
     * Bug 3 修复: ISR 与任务共享 pulse_count, 读-清零必须原子。
     *   Cortex-M3 上 32-bit 读写单指令原子, 但读→清零是两步:
     *     step = ctx->pulse_count;   // <-- 若此时 ISR 修改 pulse_count
     *     ctx->pulse_count = 0;      // <-- ISR 的修改被覆盖丢失
     *   用 taskENTER_CRITICAL 关中断保护读-清零窗口 (仅 ~3 条指令)。
     */
    taskENTER_CRITICAL();
    step = ctx->pulse_count;
    ctx->pulse_count = 0;
    taskEXIT_CRITICAL();

    if (step == 0) return 0;

    abs_step = (step > 0) ? step : (-step);

    /*
     * 加速检测：
     *   如果脉冲数在短时间内累加很多 → 用户快速旋转
     *   步进量 = 基础步进 × 加速倍数
     *
     * 简化逻辑：如果累计脉冲 > 加速阈值 → 步进 × 倍数
     * 实际上应该根据 RATE（脉冲/秒）来判断。
     * 这里简化为根据绝对计数值。
     *
     * 步进值含义：
     *   1 步 = 1 个脉冲刻度
     *   在 UI 中映射为：
     *     CV 模式: 1 步 = 0.01V (精细) 或 0.1V (加速)
     *     CC 模式: 1 步 = 0.001A (精细) 或 0.01A (加速)
     */
    if (abs_step > ctx->accel_threshold) {
        step = step * (int32_t)ctx->accel_multiplier;
    }

    return step;
}
