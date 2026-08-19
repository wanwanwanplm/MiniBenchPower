/**
 * task_adc.c
 * ADC 采样任务实现 — 事件队列驱动 (v2.0)
 *
 * vTaskADC — 最高优先级的应用任务 (优先级 4)。
 *
 * 数据流 (本批次修正后)：
 *   DMA (后台循环填充) ──HT/TC 中断──> ISR 投递"半区索引"到事件队列
 *      → vTaskADC 从事件队列取到 0 或 1
 *      → ADC_DMA_ProcessHalfBuffer(半区指针, &data)  [内部滤波+校准+AppState_UpdateADC]
 *      → 其它任务通过 AppState_GetADC 拿最新一致快照
 * 为什么 ADC 任务优先级最高？
 *   1. PID 计算依赖 ADC 数据 → ADC 延迟 = PID 延迟
 *   2. 保护检查依赖 ADC 数据 → ADC 延迟 = 保护延迟
 *   3. ADC 处理很快 (~50μs)，不会严重占用 CPU
 */

#include "tasks/task_adc.h"
#include "adc_dma.h"
#include "app_state.h"        /* #32: 读校准系数下发 ADC 驱动 */
#include "stm32f1xx_hal.h"

/*===========================================================================
 * 模块级变量
 *===========================================================================*/

/*
 * ADC 双半缓冲结构 (含 DMA 循环缓冲 + 事件队列句柄)。
 * 必须 static 长期有效 —— DMA 后台持续写入本缓冲, 生命周期贯穿整个运行期。
 */
static ADC_Buffer_t   g_adc_buffer;

/*
 * ISR→任务的"半区就绪"事件队列。
 * 元素类型 uint8_t = 半区索引 (0=前半, 1=后半); 长度 ADC_EVENT_QUEUE_LEN(=2)。
 * 由本任务创建, 经 ADC_DMA_SetEventQueue 注册给驱动。
 */
static QueueHandle_t  g_adc_event_queue = NULL;

/*===========================================================================
 * 任务函数
 *===========================================================================*/

void vTaskADC(void *argument)
{
    ADCData_t adc_data;         /* ProcessHalfBuffer 的输出缓冲 (本地, 无并发) */
    uint8_t   half_index;       /* 从事件队列取到的半区索引: 0=前半, 1=后半 */
    uint16_t *half_ptr;         /* 半区起始地址 */
    Calibration_t calib;      /* : 从 AppState 取校准初值 */
	
	
    (void)argument;

	  /*
     * Step 1: 创建事件队列。
     * 长度必须 = ADC_EVENT_QUEUE_LEN(2), 元素 sizeof(uint8_t) —— 与驱动/ISR 约定一致。
     */
    g_adc_event_queue = xQueueCreate(ADC_EVENT_QUEUE_LEN, sizeof(uint8_t));
    if (g_adc_event_queue == NULL) {
        while (1) { /* 队列创建失败: 资源不足, 停机等看门狗复位 */ }
    }

    /*
     * Step 2: 初始化 ADC DMA 硬件, 把双半缓冲交给驱动记录 (驱动只存指针, 不拷贝)。
     */
    ADC_DMA_Init(&g_adc_buffer);

    /*
     * Step 3: 把事件队列注册进驱动 —— ISR 拿到它才能 xQueueSendFromISR 投递半区号。
     * (替代旧的 ADC_DMA_SetSemaphore, 二值信号量方案已因竞态废弃。)
     */
    ADC_DMA_SetEventQueue(g_adc_event_queue);

    /*
     * Step 4 (EEPROM 校准生效): 从 AppState 取校准系数下发给 ADC 驱动。
     */
    AppState_GetCalibration(&calib);
    ADC_SetVoltageCalibration(calib.v_slope, calib.v_offset);
    ADC_SetCurrentCalibration(calib.i_slope, calib.i_offset);

    /*
     * 启动 ADC DMA 连续采样。
     * 一旦启动, DMA 后台循环填充 g_adc_buffer.buffer, 每填满半区触发 HT/TC 中断,
     * ISR 把半区号投进事件队列。必须在进入接收循环前启动。
     */
    ADC_DMA_Start();

    for (;;) {
        /*
         * 阻塞等待"半区就绪"事件。
         *   portMAX_DELAY = 一直等到队列有事件。
         *   队列元素就是半区号, 天然携带"处理哪半区"的信息, 无需再读共享变量。
         */
        if (xQueueReceive(g_adc_event_queue, &half_index, portMAX_DELAY) == pdTRUE) {

            /*
             * 防御性判断: 半区号只能是 0 或 1。异常值直接丢弃, 避免越界访问缓冲。
             */
            if (half_index > 1U) {
                continue;
            }

            /*
             * 定位半区起始地址:
             *   half_index=0 → buffer[0 .. ADC_HALF_BUFFER_SIZE-1]   (前半)
             *   half_index=1 → buffer[ADC_HALF_BUFFER_SIZE .. end]   (后半)
             * ADC_HALF_BUFFER_SIZE = 4 通道 × 10 样本 = 40。
             */
            half_ptr = g_adc_buffer.buffer
                       + ((uint32_t)half_index * ADC_HALF_BUFFER_SIZE);

            /*
             * 处理半区: 解交织 → 中值滤波 → 均值 → 物理量转换 → AppState_UpdateADC 上报。
             * 上报在函数内部完成, 故本任务不再往任何"数据队列"发送。
             */
            ADC_DMA_ProcessHalfBuffer(half_ptr, &adc_data);
        }
    }
}
