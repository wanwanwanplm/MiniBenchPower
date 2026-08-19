/**
 * task_comm.h
 * 通信任务 (vTaskComm) — 轮询触发型
 *
 * 职责：
 *   1. 轮询 UART RX 环形缓冲区, 解析协议帧
 *   2. 派遣命令到处理器, 构造响应帧
 *   3. 经 USART1 TX DMA 发送 (判 BUSY 重试, #19)
 *   4. 设定/读数统一走 AppState (#12)
 *
 * ─────────────────────────────────────────────────────────────────────────
 * 本任务是"轮询触发型", 不是固定周期任务:
 *   - 环形缓冲有数据 → 尽力连续解析处理 (不睡);
 *   - 环形缓冲为空   → vTaskDelay(COMM_IDLE_POLL_MS) 让出 CPU 再来看。
 *   旧注释同时写"50ms"和"10ms"自相矛盾, 现以实际代码常量 COMM_IDLE_POLL_MS(=10ms)
 *   为准, 表示"空闲时每 10ms 轮询一次 RX 缓冲"。
 * ─────────────────────────────────────────────────────────────────────────
 *
 * 优先级：1 (低于 UI 和控制任务 —— 控制 > 通信, 上位机可容忍 ms 级延迟)
 */

#ifndef __TASK_COMM_H
#define __TASK_COMM_H

#include "app_config.h"
#include "protocol.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

extern RingBuffer_t g_uart_rx_ring;     /* UART RX 环形缓冲区 (USART1 RX ISR 写入) */

void vTaskComm(void *argument);

/**
 * @brief 创建通信任务
 * 输入：无
 * 副作用：RingBuffer_Init、注册命令处理器、xTaskCreate。
 * 调用时机：main.c 中, AppState_Init 之后、调度器启动前。
 */
//void TaskComm_Create(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_COMM_H */
