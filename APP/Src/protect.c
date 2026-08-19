/**
 * protect.c
 * 保护逻辑实现
 *
 * 故障处理策略：
 *   - 检测到故障 → 立即关断输出 (PA15 = 0) + 蜂鸣器报警
 *   - 故障锁存 → 需要用户手动确认清除
 *   - 硬件 OVP (LM393) 有独立路径 → 软件只是记录
 */

#include "protect.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/*===========================================================================
 * 全局变量
 *===========================================================================*/

/*
 * protect 模块【不】持有 ctx 的 static 实例。
 * 全局唯一 ProtectContext_t 由 app_state 持有 (AppState_GetProtectCtx())，
 * 所有 protect 函数都操作调用方传入的 ctx 指针。这样保证 Monitor/UI/Comm
 * 三方看到的是同一份故障状态，不会出现"多个上下文各记各的"不一致问题。
 *
 * 硬件 OVP 中断标志
 * 由 EXTI4_IRQHandler 置位 (ISR 优先级高于 FreeRTOS 阈值, 只能操作 volatile),
 * 由 vTaskMonitor 轮询后调用 Protect_HardwareOVP 合并进 ctx。
 * 清零由 Protect_ClearFaults 负责。
 */
volatile uint8_t g_hw_ovp_flag = 0;

/*===========================================================================
 * 初始化
 *===========================================================================*/

void Protect_Init(ProtectContext_t *ctx)
{
    if (ctx == NULL) return;

    /* 直接初始化调用方传入的 ctx (app_state 持有的全局唯一实例) */
    memset(ctx, 0, sizeof(ProtectContext_t));
    ctx->active_faults = FAULT_NONE;
    ctx->output_latched_off = 0;
    ctx->fault_timestamp = 0;
    ctx->fault_retry_count = 0;
}

/*===========================================================================
 * 保护检查
 *===========================================================================*/

/**
 * 逐项检查所有保护阈值。
 *
 * 检查顺序（为什么 OVP 第一个？）：
 *   过压最危险 → 优先检查 → 一旦触发就不需要继续检查其他的。
 *   但实际上所有检查都应该独立进行（可能同时有多个故障），
 *   所以这里不做短路逻辑——所有项都检查。
 */
FaultFlag_t Protect_Check(ProtectContext_t *ctx, const ADCData_t *adc_data)
{
    FaultFlag_t faults = FAULT_NONE;
    float power;

    if (ctx == NULL || adc_data == NULL) return FAULT_NONE;

    /*
     * 如果输出已锁存关断，跳过检查
     * (等待用户手动清除故障)
     */
    if (ctx->output_latched_off) {
        return ctx->active_faults;
    }

    /*──────────────────────────────────────────────────
     * 1. 过压保护 (OVP) — 软件阈值 28.5V [v2.0]
     *
     * 硬件 OVP (LM393→2N5551→LM358 PIN3) 在 ~28.7V 触发 (<5μs)
     * 软件 OVP 稍低 (28.5V) 作为预警和软件关断双保险。
     * 硬件路径和 PA15 软件路径共用 LM358 PIN3 拉低机制。
     *──────────────────────────────────────────────────*/
    if (adc_data->v_out > OVP_THRESHOLD_VOLTAGE) {
        faults |= FAULT_OVP;

        /*
         * OVP 处理：立即关断输出
         * PA15 = 0 → D_en 正偏 → LM358 PIN3 被拉低 → 输出关断
         * 与硬件 OVP 共用同一条 PIN3 拉低路径
         */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        ctx->output_latched_off = 1;
    }

    /*──────────────────────────────────────────────────
     * 2. 过流保护 (OCP) — 软件阈值 3.2A
     *
     * 为什么阈值是 3.2A 而不是 3.0A？
     *   ADC 有 ~2% 的测量误差，3A 读数可能在 2.94~3.06A 之间。
     *   设 3.2A 留了余量，防止误触发。
     *
     * 为什么不用硬件比较器做 OCP？
     *   可以用（如 INA169 的 OUT 接比较器），但本项目用软件。
     *   软件 OCP 响应时间最坏约 10ms (PID 周期)，
     *   对于大多数负载（非精密 IC）可以接受。
     *──────────────────────────────────────────────────*/
    if (adc_data->i_out > OCP_THRESHOLD_CURRENT) {
        faults |= FAULT_OCP;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        ctx->output_latched_off = 1;
    }

    /*──────────────────────────────────────────────────
     * 3. 短路检测 (I > 5A)
     *
     * 短路和过流分开，短路更严重。
     *──────────────────────────────────────────────────*/
    if (adc_data->i_out > 5.0f) {
        faults |= FAULT_SHORT;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        ctx->output_latched_off = 1;
    }

    /*──────────────────────────────────────────────────
     * 4. 过功率保护 (OPP) — 88W 阈值 [v2.0]
     *
     * 最大额定功率 = 28V × 3A = 84W
     * 阈值设 88W 留余量。
     *──────────────────────────────────────────────────*/
    power = adc_data->v_out * adc_data->i_out;
    if (power > OPP_THRESHOLD_POWER) {
        faults |= FAULT_OPP;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        ctx->output_latched_off = 1;
    }

    /*──────────────────────────────────────────────────
     * 5. 过温保护 (OTP) — 80°C 阈值
     *
     * NTC 检测散热片温度。
     * IRF9540 在 6W 耗散下散热片温度可能达到 80°C。
     * 过热保护防止 MOSFET 超过 Tj_max (150°C)。
     *──────────────────────────────────────────────────*/
    if (adc_data->temperature > OTP_THRESHOLD_TEMP) {
        faults |= FAULT_OTP;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        ctx->output_latched_off = 1;
    }

    /*──────────────────────────────────────────────────
     * 6. 记录故障
     *──────────────────────────────────────────────────*/
    if (faults != FAULT_NONE) {
        ctx->active_faults = faults;
        ctx->fault_timestamp = HAL_GetTick();

        /* 记录故障详情 */
        ctx->last_fault.type = faults;
        ctx->last_fault.timestamp_ms = ctx->fault_timestamp;
        ctx->last_fault.v_out = adc_data->v_out;
        ctx->last_fault.i_out = adc_data->i_out;
        ctx->last_fault.v_in = adc_data->v_in;
        ctx->last_fault.temperature = adc_data->temperature;
    }

    return faults;
}

/*===========================================================================
 * 硬件 OVP 处理
 *===========================================================================*/

/**
 * 硬件 OVP 中断处理
 *
 * LM393 输出翻高 → 2N5551 导通 → 拉低 LM358 PIN3 → P-MOS 截止 (硬件路径, <5μs)。
 * 软件这边做双保险：
 *   1. PA15 = 0 → D_en 同样拉低 PIN3 (与硬件路径并联)
 *   2. 设置 FAULT_OVP_HARDWARE 故障标志
 *   3. 禁止重新使能直到用户确认
 *
 * 注意：这个函数在 ISR 中调用（EXTI4_IRQHandler），
 * 应该尽量简短。不做复杂操作。
 */
void Protect_HardwareOVP(ProtectContext_t *ctx)
{
    if (ctx == NULL) return;

    /*
     * PA15 = 0：硬件已通过 LM393→2N5551 拉低了 PIN3，
     * 软件通过 D_en 同样拉低 PIN3，双重保险。
     * [v3.0D] 两条路径共用同一 LM358 PIN3 关断机制
     */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    ctx->active_faults = FAULT_OVP_HARDWARE;
    ctx->output_latched_off = 1;
    ctx->fault_timestamp = HAL_GetTick();
}

/*===========================================================================
 * 故障清除
 *===========================================================================*/

/**
 * 清除所有故障并重新使能输出。
 *
 * 调用前应确保：
 *   1. 用户已确认故障
 *   2. 负载已断开或故障已排除
 *
 * 为什么不是自动恢复？
 *   见头文件注释。简言之：安全 + 防止保护振荡。
 */
void Protect_ClearFaults(ProtectContext_t *ctx)
{
    if (ctx == NULL) return;

    ctx->active_faults = FAULT_NONE;
    ctx->output_latched_off = 0;
    ctx->fault_retry_count = 0;
    g_hw_ovp_flag = 0;              /*一并清除硬件 OVP 标志 */

    /*
     * 不在这里恢复 PA15。
     * 调用者 (vTaskMonitor 或 vTaskUI) 负责重新使能。
     * 这样确保恢复和故障检查之间的时间差是可控的。
     */
}

uint8_t Protect_HasFault(ProtectContext_t *ctx)
{
    if (ctx == NULL) return 0;
    return (ctx->active_faults != FAULT_NONE) ? 1 : 0;
}

/*===========================================================================
 * 故障描述
 *===========================================================================*/

void Protect_GetFaultString(FaultFlag_t fault, char *buf, uint16_t buf_size)
{
    if (buf == NULL || buf_size == 0) return;

    buf[0] = '\0';

    if (fault == FAULT_NONE) {
        strncpy(buf, "OK", buf_size - 1);
        buf[buf_size - 1] = '\0';
        return;
    }

    if (fault & FAULT_OVP) {
        strncat(buf, "OVP ", buf_size - strlen(buf) - 1);
    }
    if (fault & FAULT_OCP) {
        strncat(buf, "OCP ", buf_size - strlen(buf) - 1);
    }
    if (fault & FAULT_OPP) {
        strncat(buf, "OPP ", buf_size - strlen(buf) - 1);
    }
    if (fault & FAULT_OTP) {
        strncat(buf, "OTP ", buf_size - strlen(buf) - 1);
    }
    if (fault & FAULT_SHORT) {
        strncat(buf, "SHORT ", buf_size - strlen(buf) - 1);
    }
    if (fault & FAULT_OVP_HARDWARE) {
        strncat(buf, "OVP-HW ", buf_size - strlen(buf) - 1);
    }
    if (fault & FAULT_I2C_ERROR) {
        strncat(buf, "I2C-ERR ", buf_size - strlen(buf) - 1);
    }
    if (fault & FAULT_ADC_ERROR) {
        strncat(buf, "ADC-ERR ", buf_size - strlen(buf) - 1);
    }
}
