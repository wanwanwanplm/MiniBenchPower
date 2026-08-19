/**
 * @file    task_ui.h
 * @brief   UI 管理任务 (vTaskUI) — 100ms 周期
 * @author  software-engineer
 * @date    2026-07-12
 *
 * 职责：
 *   1. TFT 显示刷新 (设定V/I、实际V/I、功率、CV/CC 模式、故障)
 *   2. 编码器旋转/按键分离处理 (#14/#15)
 *   3. 预设按键扫描 (K1~K4)
 *   4. 设定值统一走 AppState (#12), 故障查询用全局唯一 ctx (#11)
 *
 * 周期：400ms   优先级：2 (Normal)
 */

#ifndef __TASK_UI_H
#define __TASK_UI_H

#include "app_config.h"
#include "cmsis_os.h"
#include "queue.h"


#ifdef __cplusplus
extern "C" {
#endif

extern QueueHandle_t g_ui_event_queue;

void vTaskUI(void *argument);
/**
 * @brief 创建 UI 任务及相关资源
 * 输入：无 (#5: 去掉 adc_queue)
 * 副作用：创建 UI 事件队列、Encoder_Init;
 *         不再 Protect_Init (AppState_Init 已初始化全局唯一 ctx, #11)。
 * 调用时机：main.c 中, AppState_Init 之后、调度器启动前。
 */
//void TaskUI_Create(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_UI_H */
