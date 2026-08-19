/**
 * task_pid.h
 * PID 控制任务 (vTaskPID) — 10ms 周期，核心控制任务
 *
 * 这是整个系统最重要的任务——实现 CV/CC 双环 PID 控制。
 *
 * 职责 (本批次修正后)：
 *   1. AppState_GetADC 读最新反馈 (不再用 adc_queue)          [#12]
 *   2. Protect_Check 每 10ms 先检保护, 有故障则安全关断+复位   [#10]
 *   3. AppState_GetSetting 读设定 (不再裸访问 g_system_setting)[#12]
 *   4. 软启动状态机 + 设定变化检测 → 正确管理 PID 积分         [#8/#13]
 *   5. CV/CC 双环并行 + MIN 选择器 + DAC1/DAC2 映射
 *
 * 周期：10ms (100Hz)   优先级：3 (仅次于 vTaskADC)
 */

#ifndef __TASK_PID_H
#define __TASK_PID_H

#include "app_config.h"
#include "pid.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 全局变量 (供其他任务访问)
 *
 * 保留 g_pid_cv/g_pid_cc/g_current_mode 为 task_pid 拥有的全局:
 *   - UI 通过 TaskPID_GetMode() 读工作模式;
 *   - Comm 通过 TaskPID_UpdateCVParams/CCParams 调参。
 * 但设定值(setting)和 ADC 数据一律走 AppState, 不再有 g_system_setting 裸全局。
 *===========================================================================*/

extern PID_Controller g_pid_cv;         /* 恒压 PID 控制器 */
extern PID_Controller g_pid_cc;         /* 恒流 PID 控制器 */
extern OperatingMode_t g_current_mode;  /* 当前工作模式 (CV/CC) */

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief vTaskPID 任务函数入口
 * @param argument 未使用
 */
void vTaskPID(void *argument);

/**
 * @brief 创建 PID 控制任务
 *
 * 输入：无 (#5: 去掉 adc_queue, 反馈改走 AppState_GetADC)
 * 副作用：
 *   - PID_Init 两个控制器 (增益取 app_config 的 *_DEFAULT 宏);
 *   - TaskPID_LoadFromAppState 从 AppState 同步初始设定 (#32);
 *   - xTaskCreate 创建 vTaskPID。
 * 调用时机：main.c 中, AppState_Init (及 EEPROM 灌值) 之后、调度器启动前。
 */
//void TaskPID_Create(void);

/**
 * @brief 从 AppState 同步初始设定到 PID 层内部记忆 (#32 EEPROM 加载生效)
 *
 * 为什么需要：main.c 读 EEPROM 后调 AppState_SetSetting 灌入上次的 v_set/i_set。
 *   本函数把这些初值同步进 PID 层的"上次设定"记忆 (last_v_set/last_i_set),
 *   使首次使能时软启动 ramp 的目标就是用户上次保存的值, 而非硬编码 5V/1A。
 * 调用时机：TaskPID_Create 内部会调一次; main.c 若在 Create 之后才灌 AppState,
 *   可再显式调用一次刷新。仅在调度器启动前调用是安全的 (无并发)。
 */
void TaskPID_LoadFromAppState(void);

/** @brief 获取当前工作模式 (CV/CC) —— 供 UI 显示 */
OperatingMode_t TaskPID_GetMode(void);

/**
 * @brief 使能输出 (经 AppState 改 output_enable=1, 软启动由 vTaskPID 触发)
 * 副作用：仅 AppState_SetSetting; 不直接改软启动内部状态 (避免跨任务竞态)。
 */
void TaskPID_EnableOutput(void);

/**
 * @brief 关断输出 (经 AppState 改 output_enable=0)
 */
void TaskPID_DisableOutput(void);

/** @brief 在线更新 CV PID 参数 (供 Comm 调参) */
void TaskPID_UpdateCVParams(float kp, float ki, float kd);
/** @brief 在线更新 CC PID 参数 (供 Comm 调参) */
void TaskPID_UpdateCCParams(float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_PID_H */
