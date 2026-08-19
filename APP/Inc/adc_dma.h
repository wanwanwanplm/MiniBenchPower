/**
 * adc_dma.h
 * ADC1 DMA 多通道循环采样驱动
 * 实现：
 *   - ADC1 4 通道规则组扫描 (V_OUT, I_OUT, V_IN, TEMP)
 *   - DMA1_Channel1 循环模式 + 双半缓冲 (HT/TC 双回调)
 *   - 采样时间 71.5 cycles（适配 PA0/PA2 高阻抗源）
 *   - 中值 + 均值滤波 (10 点过采样)
 *   - 物理量转换 (ADC 码值 → 电压/电流/温度)
 *
 * ⚠️ 句柄单一来源 (根治 #7)：hadc1 / hdma_adc1 由 main.c 定义, 本驱动经
 *    main.h 的 extern 引用同一实例。绝不再定义 static 副本 —— DMA 中断
 *    (stm32f1xx_it.c 里 HAL_DMA_IRQHandler(&hdma_adc1)) 必须与本驱动操作
 *    的是同一个句柄, 否则回调根本不触发。
 * 【双缓冲竞态】
 *   新实现: 取消共享 active_half; HT 回调固定投递半区索引 0, TC 回调固定投递
 *   半区索引 1, 经"长度=2 的事件队列" (xQueueSendFromISR) 传给任务。每个事件
 *   自带半区号, 队列能同时容纳 HT+TC 两条, 绝不串区、绝不漏处理。
 */

#ifndef __ADC_DMA_H
#define __ADC_DMA_H

#include "app_config.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 常量
 *===========================================================================*/

/*
 * 半缓冲大小 (uint16_t 个数) = 总缓冲的一半 = 4 通道 × 10 样本 = 40。
 * DMA 每填满半个缓冲触发一次 HT/TC 中断。
 */
#define ADC_HALF_BUFFER_SIZE    (ADC_DMA_BUFFER_SIZE)

/* 事件队列深度: 必须 ≥2, 以同时容纳一个 HT 和一个 TC 事件, 防止事件合并 */
#define ADC_EVENT_QUEUE_LEN     2

/*===========================================================================
 * 类型定义
 *===========================================================================*/

/**
 * @brief ADC DMA 双半缓冲结构
 *
 * buffer[]      : DMA 循环填充区, 前半 [0..39], 后半 [40..79] (交织存放 4 通道)。
 * event_queue   : ISR → 任务的事件队列, 元素为 uint8_t 半区索引 (0=前半,1=后半)。
 *                 由任务层创建 (xQueueCreate(ADC_EVENT_QUEUE_LEN,sizeof(uint8_t)))
 *                 并通过 ADC_DMA_SetEventQueue 注册给驱动。
 *
 * 【设计要点】不再有 volatile active_half 共享变量 —— 半区号随事件走, 天然无竞态。
 */
typedef struct {
    uint16_t     buffer[ADC_DMA_BUFFER_SIZE * 2];   /* DMA 循环缓冲 (80 × uint16_t) */
    QueueHandle_t event_queue;                  /* 半区索引事件队列 (ISR→任务) */
} ADC_Buffer_t;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化 ADC1 + DMA1_Channel1
 *
 * 为什么需要：配置全局 hadc1/hdma_adc1 为 4 通道扫描 + 循环 DMA, 并使能 HT/TC 中断。
 * 输入：adc_buffer —— 调用方 (任务层) 提供的缓冲结构 (需长期有效, 通常为 static)
 * 输出：无
 * 调用时机：main() 或任务创建时, 在 ADC_DMA_Start 之前。
 * 副作用：使能 ADC1/DMA1 时钟; 配置全局 hadc1/hdma_adc1; 记录 adc_buffer 指针;
 *         清零 buffer; 使能 DMA_IT_HT / DMA_IT_TC。
 *
 * 注意：PA0~PA3 需在 main.c 中配置为模拟输入模式。
 */
void ADC_DMA_Init(ADC_Buffer_t *adc_buffer);

/**
 * @brief 注册 ISR→任务事件队列
 *
 * 为什么需要：DMA 回调 (ISR) 通过此队列把"哪半区就绪"传给 vTaskADC。
 * 输入：event_queue —— 长度 ≥ ADC_EVENT_QUEUE_LEN、元素为 uint8_t 的队列句柄
 * 输出：无
 * 调用时机：任务层创建队列后、ADC_DMA_Start 之前。
 * 副作用：保存队列句柄到驱动内部。
 *
 * ⚠️ 替代了旧的 ADC_DMA_SetSemaphore (二值信号量方案已因竞态废弃, 见 #16)。
 */
void ADC_DMA_SetEventQueue(QueueHandle_t event_queue);

/**
 * @brief 启动 ADC DMA 连续采样
 * 调用时机：事件队列注册后、vTaskADC 进入循环前。
 * 副作用：HAL_ADC_Start_DMA 启动后台采样, DMA 循环填充 buffer。
 */
void ADC_DMA_Start(void);

/**
 * @brief 停止 ADC DMA 采样
 * 调用时机：进入故障或低功耗时。
 */
void ADC_DMA_Stop(void);

/**
 * @brief 处理指定半区数据 (滤波 + 转换 + 上报)
 *
 * 为什么需要：把某半区 40 个原始样本解交织、滤波、换算为物理量。
 * 输入：half_buffer —— 半区起始地址 (指向 ADC_HALF_BUFFER_SIZE 个 uint16_t)
 *       result      —— 输出: 滤波+校准后的一帧 ADCData_t
 * 输出：*result 被填充; 并调用 AppState_UpdateADC(result) 上报全局最新数据。
 * 调用时机：vTaskADC 从事件队列取到半区索引后, 在任务上下文调用。
 * 副作用：写 *result; 调用 AppState_UpdateADC (含临界区)。
 *
 * ⚠️ 必须在任务上下文调用, 不可从 ISR 调用 —— AppState_UpdateADC 用临界区,
 *    且本函数含浮点/logf 运算, 不适合放中断里 (见 stm32f1xx_it.c ISR 设计原则)。
 *
 * 【数据上报选择】本驱动在此直接调用 AppState_UpdateADC 更新全局
 *   最新 ADC 数据, 替代 v1.0 未定义的裸全局 g_latest_adc_data。因为本函数已在
 *   任务上下文运行, 直接上报最简单可靠; 若下批想改由 task 层上报, 去掉此调用即可。
 */
void ADC_DMA_ProcessHalfBuffer(const uint16_t *half_buffer, ADCData_t *result);

/**
 * @brief ADC 码值 → 输出电压 (V), 含校准与钳位
 * 转换：V_out = adc/4096 × VREF × V_OUT_DIVIDER_RATIO, 再 ×slope+offset
 */
float ADC_ConvertToVoltage(uint16_t adc_value);

/**
 * @brief ADC 码值 → 输出电流 (A), 含校准与钳位
 * 转换：I_out = adc/4096 × VREF (INA169 1V/A), 再 ×slope+offset
 */
float ADC_ConvertToCurrent(uint16_t adc_value);

/**
 * @brief ADC 码值 → 输入电压 (V)
 * 转换：V_in = adc/4096 × VREF × V_IN_DIVIDER_RATIO  [v2.0: 1/7.25]
 */
float ADC_ConvertToInputVoltage(uint16_t adc_value);

/**
 * @brief ADC 码值 → 温度 (°C), NTC B 参数方程
 * 注意：含一次 logf(), 开销评估见 .c 实现注释 (#21)。
 */
float ADC_ConvertToTemperature(uint16_t adc_value);

/** @brief 更新电压校准系数 (slope/offset) */
void ADC_SetVoltageCalibration(float slope, float offset);
/** @brief 更新电流校准系数 (slope/offset) */
void ADC_SetCurrentCalibration(float slope, float offset);
/** @brief 读取当前校准系数 (任一指针可为 NULL) */
void ADC_GetCalibration(float *v_slope, float *v_offset,
                        float *i_slope, float *i_offset);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_DMA_H */
