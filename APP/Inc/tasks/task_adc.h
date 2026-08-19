/**
 * task_adc.h
 * ADC 采样任务 (vTaskADC) — 	TIM3定时触发驱动1KHz
 *
 * 职责：
 *   1. 从 ISR→任务事件队列取"哪半区就绪"(uint8_t 半区索引 0/1)
 *   2. 调 ADC_DMA_ProcessHalfBuffer 做滤波+校准+转换
 *   3. ProcessHalfBuffer 内部已 AppState_UpdateADC 上报, 本任务不再自行发数据队列
 *
 * 优先级：4 (最高应用任务, 仅次于 SysTick / DMA ISR)
 */

#ifndef __TASK_ADC_H
#define __TASK_ADC_H

#include "app_config.h"
#include "adc_dma.h"       /* ADC_Buffer_t / ADC_HALF_BUFFER_SIZE / ADC_EVENT_QUEUE_LEN */
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief vTaskADC 任务函数入口
 *
 * @param argument  任务参数 (未使用)
 *
 * 任务循环流程：
 *   ADC_DMA_Start();
 *   for(;;) {
 *     xQueueReceive(event_queue, &half_index, portMAX_DELAY);  // 等 HT/TC 事件
 *     half_ptr = buffer + half_index * ADC_HALF_BUFFER_SIZE;   // 定位半区
 *     ADC_DMA_ProcessHalfBuffer(half_ptr, &data);              // 滤波+转换+上报
 *   }
 */
void vTaskADC(void *argument);


#ifdef __cplusplus
}
#endif

#endif /* __TASK_ADC_H */
