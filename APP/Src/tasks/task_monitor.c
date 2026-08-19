/**
 * task_monitor.c
 * 监控任务实现 (v2.0, 本批次修正)
 *
 * 低频监控任务：硬件 OVP 合并、蜂鸣器报警、输入/温度监测、心跳 LED。
 * 周期：500ms   优先级：0 (最低)
 * 为什么监控优先级最低？
 *   硬件保护 (LM393 OVP <5μs, 保险丝) 是最快防线; PID 的 10ms Protect_Check 是
 *   第二道软件防线。Monitor 只是"善后 + 提示"角色, 被阻塞也不影响安全。
 */

#include "task_monitor.h"
#include "protect.h"
#include "app_state.h"        /* #11: 全局唯一 ctx; #5: AppState_GetADC */
#include "app_config.h"
#include "stm32f1xx_hal.h"

/*===========================================================================
 * 硬件映射
 *===========================================================================*/

#define BUZZER_PORT     GPIOB
#define BUZZER_PIN      GPIO_PIN_11

#define HEARTBEAT_PORT  GPIOC
#define HEARTBEAT_PIN   GPIO_PIN_13

/* 输入电压软告警窗口 (仅提示, 不触发保护) */
#define VIN_UNDER_WARN  10.0f       /* < 10V: 供电不足告警 */
#define VIN_OVER_WARN   34.0f       /* > 34V: 输入过压告警 */


/*===========================================================================
 * 任务主循环
 *===========================================================================*/

void vTaskMonitor(void *argument)
{
    ADCData_t         adc_data;
    ProtectContext_t *ctx;              /* #11: 全局唯一 */
    TickType_t        xLastWakeTime;
    uint8_t           has_fault;

    (void)argument;

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        ctx = AppState_GetProtectCtx();

        /*──────────────────────────────────────────────────
         * Step 1: 硬件 OVP 标志合并 [v3.0D]
         *
         * EXTI4 ISR (优先级高于 FreeRTOS 阈值) 只置位 g_hw_ovp_flag, 不能碰 ctx。
         * 这里轮询后合并进全局唯一 ctx: 置 FAULT_OVP_HARDWARE + 锁存。
         * 放最前面 —— 即使下面无 ADC 数据, 硬件 OVP 的软件锁存也不被推迟。
         *──────────────────────────────────────────────────*/
        if (g_hw_ovp_flag) {
            g_hw_ovp_flag = 0;                  /* 清零, 避免重复处理 */
            Protect_HardwareOVP(ctx);           /* 置 FAULT_OVP_HARDWARE + 锁存 (统一 ctx) */
        }

        /*──────────────────────────────────────────────────
         * Step 2: 读最新实测值 (#5 走 AppState)
         *──────────────────────────────────────────────────*/
        AppState_GetADC(&adc_data);

        /*──────────────────────────────────────────────────
         * Step 3: 蜂鸣器报警
         *
         * #10: 不再 Protect_Check —— 是否有故障直接查全局唯一 ctx。
         *   PID 任务(10ms)已负责实时检测并置位故障, Monitor 只据此驱动蜂鸣。
         *   有故障: 0.5Hz 方波 (每 500ms 周期翻转一次); 无故障: 静音。
         *──────────────────────────────────────────────────*/
        has_fault = Protect_HasFault(ctx);
        if (has_fault) {
            static uint8_t buzzer_toggle = 0;
            buzzer_toggle = (uint8_t)(!buzzer_toggle);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,
                              buzzer_toggle ? GPIO_PIN_SET : GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
        }

        /*──────────────────────────────────────────────────
         * Step 4: 输入电压软告警 (仅提示, 不关断)
         *
         * 输入异常最终会体现在输出异常上 (由 PID 的保护检查兜底), 这里只做
         * 轻量提示钩子。当前实现留空动作 —— 保留判断结构便于后续扩展 (如 LED/日志)。
         *──────────────────────────────────────────────────*/
        if (adc_data.v_in < VIN_UNDER_WARN) {
            /* 欠压告警钩子 (可扩展: 点亮告警 LED / 上报上位机) */
        } else if (adc_data.v_in > VIN_OVER_WARN) {
            /* 过压告警钩子 */
        }

        /*──────────────────────────────────────────────────
         * Step 5: 温度监测
         *   过温保护 (OTP) 已由 PID 的 Protect_Check 覆盖, 这里可扩展温升速率预警。
         *   本项目暂不实现。
         *──────────────────────────────────────────────────*/

        /*──────────────────────────────────────────────────
         * Step 6: 心跳 LED (PC13) —— 0.5Hz 闪烁表示系统存活
         *──────────────────────────────────────────────────*/
        {
            static uint8_t led_toggle = 0;
            led_toggle = (uint8_t)(!led_toggle);
            HAL_GPIO_WritePin(HEARTBEAT_PORT, HEARTBEAT_PIN,
                              led_toggle ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        /*──────────────────────────────────────────────────
         * Step 7: 精确周期延迟
         *──────────────────────────────────────────────────*/
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_MONITOR_PERIOD_MS));
    }
}
