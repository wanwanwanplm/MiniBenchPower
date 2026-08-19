/**
 * task_monitor.h
 * 监控任务 (vTaskMonitor) — 500ms 周期
 *
 * 职责 (本批次收窄后)：
 *   1. 消费 g_hw_ovp_flag → Protect_HardwareOVP (全局唯一 ctx, #11)
 *   2. 蜂鸣器报警 (依据 Protect_HasFault)
 *   3. 输入电压 / 温度监测 (软告警)
 *   4. 心跳 LED (PC13)
 *
 * ─────────────────────────────────────────────────────────────────────────
 *   #10 不再调 Protect_Check —— 保护检查已移到 vTaskPID 的 10ms 周期。
 *       Monitor 只保留 500ms 级别的慢速事务 (硬件 OVP 合并、蜂鸣、监测、心跳)。
 *   #11 删除 static ProtectContext_t g_protect_ctx —— 改用全局唯一
 *       AppState_GetProtectCtx()。不再 Protect_Init (AppState_Init 已做)。
 *   #5  TaskMonitor_Create 去掉 adc_queue 参数, 实测走 AppState_GetADC。
 * ─────────────────────────────────────────────────────────────────────────
 *
 * 周期：500ms   优先级：0 (最低, Idle 级)
 */

#ifndef __TASK_MONITOR_H
#define __TASK_MONITOR_H

#include "app_config.h"
#include "protect.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void vTaskMonitor(void *argument);

/**
 * @brief 创建监控任务
 * 输入：无 (#5: 去掉 adc_queue)
 * 副作用：xTaskCreate; 不再 Protect_Init (全局唯一 ctx 已由 AppState_Init 初始化)。
 * 调用时机：main.c 中, AppState_Init 之后、调度器启动前。
 */
//void TaskMonitor_Create(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_MONITOR_H */
