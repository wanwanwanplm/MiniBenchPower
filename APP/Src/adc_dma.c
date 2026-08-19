/* 实现 ADC 驱动核心功能：
 *   1. 4 通道规则组 DMA 循环采样 (V_OUT, I_OUT, V_IN, TEMP)
 *   V_IN 监测 Boost 前输入电压 (5~20V), 分压比 1/7.25
 *   2. 中值 + 均值滤波 (抗噪声)
 *   3. 物理量转换 (ADC 码值 → V/A/°C) + 上报 AppState
 *
 * DMA 缓冲布局（交织采样）:
 *   每次 ADC 扫描产生 4 个结果 → 按通道顺序写入 DMA buffer
 *   前半 buffer[0..39], 后半 buffer[40..79], 各含 4ch × 10 样本。
 *
 * ⚠️ 句柄单一来源: hadc1/hdma_adc1 来自 main.c, 经 main.h extern。
 *
 * 🔧 算法：中值+均值复合滤波
 *   📐 数学原理：对每通道 10 个过采样点排序, 去掉 1 个最大 + 1 个最小 (剔除脉冲
 *      噪声), 对中间 8 点取算术平均 (平滑随机噪声)。
 *   💻 代码映射：BubbleSort → 去 sorted[0]/sorted[N-1] → sum/(N-2)。
 *   ⚠️ 常见坑：单纯均值会被单个 4095 尖峰严重拉偏; 单纯中值丢失平滑性。复合更稳。
 */


#include "main.h"             /* extern ADC_HandleTypeDef hadc1; DMA_HandleTypeDef hdma_adc1 */
#include "adc.h"
#include "adc_dma.h"
#include "app_state.h"        /* AppState_UpdateADC —— 全局最新数据上报 */
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include <math.h>             /* logf() —— 温度换算, 从函数内移到此处*/

/*===========================================================================
 * 模块级变量 (不含外设句柄 —— 句柄单一来源在 main.c)
 *===========================================================================*/

static ADC_Buffer_t  *p_adc_buffer = NULL;   /* 任务层提供的缓冲结构指针 */
static QueueHandle_t   adc_event_queue = NULL;  /* ISR→任务 半区事件队列 */

/*
 * 校准系数（默认值, 可由上位机/EEPROM 更新）。存于 RAM。
 */
static float g_v_cal_slope  = V_CAL_SLOPE_DEFAULT;
static float g_v_cal_offset = V_CAL_OFFSET_DEFAULT;
static float g_i_cal_slope  = I_CAL_SLOPE_DEFAULT;
static float g_i_cal_offset = I_CAL_OFFSET_DEFAULT;


void ADC_DMA_Init(ADC_Buffer_t *adc_buffer)
{
    if (adc_buffer == NULL) {
        return;
    }
    p_adc_buffer = adc_buffer;

    /* 清零 DMA 缓冲 (event_queue 由任务层创建, 此处不动) */
    memset(p_adc_buffer->buffer, 0, sizeof(p_adc_buffer->buffer));
}


void ADC_DMA_SetEventQueue(QueueHandle_t event_queue)
{
    adc_event_queue = event_queue;
}


void ADC_DMA_Start(void)
{
    if (p_adc_buffer == NULL) {
        return;
    }
    HAL_ADC_Start_DMA(&hadc1,
                      (uint32_t *)p_adc_buffer->buffer,
                      ADC_DMA_BUFFER_SIZE * 2);
		
		__HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_TC);
}


void ADC_DMA_Stop(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
}

/*===========================================================================
 * DMA 中断回调 (由 stm32f1xx_it.c 的 HAL_DMA_IRQHandler 间接调用)
 * ISR 只做一件事: 把半区号塞进队列 (xQueueSendFromISR), 处理全部留给任务。
 *===========================================================================*/

/**
 * DMA 半传输完成 → 前半区 (buffer[0..39]) 就绪 → 投递索引 0。
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t half_index = 0;   /* 固定: 前半 */

    (void)hadc;

    if (adc_event_queue != NULL) {
        /*
         * 队列深度=2, 即便任务暂时没取, HT 和 TC 两条事件也都能入队,
         * 不会像二值信号量那样把两次事件合并成一次 (根治 #16)。
         */
        xQueueSendFromISR(adc_event_queue, &half_index, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * DMA 传输完成 → 后半区 (buffer[40..79]) 就绪 → 投递索引 1。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t half_index = 1;   /* 固定: 后半 */

    (void)hadc;

    if (adc_event_queue != NULL) {
        xQueueSendFromISR(adc_event_queue, &half_index, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*===========================================================================
 * 数据处理
 *===========================================================================*/

/**
 * 冒泡排序 (10 元素, 用于中值滤波)。
 * 为什么冒泡: 10 元素 O(n²)=100 次比较, 72MHz 下几 μs; 代码简单, 无需链接 qsort。
 */
static void BubbleSort(uint16_t *arr, int n)
{
    int i, j;
    uint16_t temp;
    for (i = 0; i < n - 1; i++) {
        for (j = 0; j < n - 1 - i; j++) {
            if (arr[j] > arr[j + 1]) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void ADC_DMA_ProcessHalfBuffer(const uint16_t *half_buffer, ADCData_t *result)
{
    uint16_t ch_data[ADC_CHANNEL_COUNT][ADC_BUFFER_SIZE];
    uint16_t sorted[ADC_BUFFER_SIZE];
    uint32_t sum;
    int ch, i;
    uint16_t v_out_raw, i_out_raw, v_in_raw, temp_raw;

    if (half_buffer == NULL || result == NULL) {
        return;
    }

    /*
     * Step 1: 解交织。buffer 排列: [CH0 CH1 CH2 CH3][CH0 CH1 ...]
     *   第 i 组第 ch 通道 = half_buffer[i*ADC_CHANNEL_COUNT + ch]
     */
    for (i = 0; i < ADC_BUFFER_SIZE; i++) {
        ch_data[0][i] = half_buffer[i * ADC_CHANNEL_COUNT + 0];  /* V_OUT */
        ch_data[1][i] = half_buffer[i * ADC_CHANNEL_COUNT + 1];  /* I_OUT */
        ch_data[2][i] = half_buffer[i * ADC_CHANNEL_COUNT + 2];  /* V_IN  */
        ch_data[3][i] = half_buffer[i * ADC_CHANNEL_COUNT + 3];  /* TEMP  */
    }

    /*
     * Step 2: 逐通道 排序 → 去极值 → 均值。结果写回 ch_data[ch][0]。
     */
    for (ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
        memcpy(sorted, ch_data[ch], ADC_BUFFER_SIZE * sizeof(uint16_t));
        BubbleSort(sorted, ADC_BUFFER_SIZE);

        sum = 0;
        for (i = 1; i < ADC_BUFFER_SIZE - 1; i++) {   /* 去 sorted[0] 和 sorted[N-1] */
            sum += sorted[i];
        }
        ch_data[ch][0] = (uint16_t)(sum / (ADC_BUFFER_SIZE - 2));
    }

    v_out_raw = ch_data[0][0];
    i_out_raw = ch_data[1][0];
    v_in_raw  = ch_data[2][0];
    temp_raw  = ch_data[3][0];

    /* Step 3: 物理量转换 */
    result->v_out       = ADC_ConvertToVoltage(v_out_raw);
    result->i_out       = ADC_ConvertToCurrent(i_out_raw);
    result->v_in        = ADC_ConvertToInputVoltage(v_in_raw);
    result->temperature = ADC_ConvertToTemperature(temp_raw);
    result->timestamp_ms = HAL_GetTick();

    /*
     * Step 4: 上报全局最新 ADC 数据。
     *   本函数运行在任务上下文, 可安全调用带临界区的 AppState_UpdateADC。
     */
    AppState_UpdateADC(result);
}

/*===========================================================================
 * ADC 码值 → 物理量转换
 *===========================================================================*/

float ADC_ConvertToVoltage(uint16_t adc_value)
{
    float voltage;

    /*
     * V_out = adc/4096 × VREF × 分压比倒数。
     */
    voltage = (float)adc_value * ADC_VREF / (float)ADC_RESOLUTION * V_OUT_DIVIDER_RATIO;

    /* 校准: V = slope×V + offset */
    voltage = voltage * g_v_cal_slope + g_v_cal_offset;

    if (voltage < 0.0f) { voltage = 0.0f; }
    if (voltage > 35.0f) { voltage = 35.0f; }   /* 钳位: 略高于 28V 上限留裕量 */
    return voltage;
}

float ADC_ConvertToCurrent(uint16_t adc_value)
{
    float current;

    /*
     * INA169 + LM358 缓冲 → 1V/A: V_ADC 数值即电流值 (A)。
     * I = adc/4096 × VREF。
     */
    current = (float)adc_value * ADC_VREF / (float)ADC_RESOLUTION;

    current = current * g_i_cal_slope + g_i_cal_offset;

    if (current < 0.0f) { current = 0.0f; }
    if (current > 5.0f) { current = 5.0f; }
    return current;
}

float ADC_ConvertToInputVoltage(uint16_t adc_value)
{
    float voltage;

    /*
     * V_in = adc/4096 × VREF × V_IN_DIVIDER_RATIO。
     * 分压网络 75k+12k → 1/7.25 (监测 Boost 前 5~20V)。
     */
    voltage = (float)adc_value * ADC_VREF / (float)ADC_RESOLUTION * V_IN_DIVIDER_RATIO;

    if (voltage < 0.0f) { voltage = 0.0f; }
    if (voltage > 25.0f) { voltage = 25.0f; }   /* 最大输入 20V, 留余量 */
    return voltage;
}

/**
 * ADC 码值 → 温度 (°C)。NTC 100k (B=3950) + 10k 上拉到 3.3V。
 * 分压反推: R_ntc = R_pull × V_ADC / (VREF - V_ADC)
 * B 参数方程: 1/T = 1/T25 + (1/B)·ln(R_ntc/R25), T25=298.15K, R25=100k
 */
float ADC_ConvertToTemperature(uint16_t adc_value)
{
    /*
     * 温度-ADC 对照 (10k 上拉, ADC=4095×R_ntc/(R_ntc+10k)), 供未来查表法参考:
     *   -10°C:4015  0°C:3969  25°C:3727  50°C:3172  80°C:2176  100°C:1547
     */
    float v_adc, r_ntc, temp_k, temp_c;

    v_adc = (float)adc_value * ADC_VREF / (float)ADC_RESOLUTION;

    /* 防除零: V_ADC 不能贴到 0 或 VREF */
    if (v_adc >= (ADC_VREF - 0.001f)) { v_adc = ADC_VREF - 0.001f; }
    if (v_adc <= 0.001f)              { v_adc = 0.001f; }

    /* NTC 电阻 (单位 Ω): R_pull=10k */
    r_ntc = 10000.0f * v_adc / (ADC_VREF - v_adc);

    /* B 参数方程 (R25=100000Ω) */
    temp_k = 1.0f / (1.0f / 298.15f + (1.0f / 3950.0f) * logf(r_ntc / 100000.0f));
    temp_c = temp_k - 273.15f;

    if (temp_c < -20.0f) { temp_c = -20.0f; }
    if (temp_c > 150.0f) { temp_c = 150.0f; }
    return temp_c;
}

/*===========================================================================
 * 校准系数接口
 *===========================================================================*/

void ADC_SetVoltageCalibration(float slope, float offset)
{
    g_v_cal_slope = slope;
    g_v_cal_offset = offset;
}

void ADC_SetCurrentCalibration(float slope, float offset)
{
    g_i_cal_slope = slope;
    g_i_cal_offset = offset;
}

void ADC_GetCalibration(float *v_slope, float *v_offset,
                        float *i_slope, float *i_offset)
{
    if (v_slope)  { *v_slope  = g_v_cal_slope; }
    if (v_offset) { *v_offset = g_v_cal_offset; }
    if (i_slope)  { *i_slope  = g_i_cal_slope; }
    if (i_offset) { *i_offset = g_i_cal_offset; }
}

