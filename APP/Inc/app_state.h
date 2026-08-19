/**
 * app_state.h
 * 全局状态集中管理层 (跨任务共享状态 + 并发保护)
 * ============================================================================
 * 【这个文件解决什么问题？—— 面试高频，务必理解】
 * ============================================================================
 * v1.0 的代码里，全局状态（设定值、最新 ADC 数据、保护上下文、校准系数）
 * 散落在各个 .c 文件里，用裸露的全局变量互相访问，例如各处直接引用
 * g_latest_adc_data。这带来两个致命问题：
 *
 *   1. 竞态 (Race Condition)：
 *      ADC 任务（1ms）正在写 g_adc_data 的 v_out 字段，PID 任务（10ms）
 *      同时在读同一结构体。ADCData_t 有 5 个字段（20 字节），一次写入不是
 *      原子操作 —— PID 可能读到"新 v_out + 旧 i_out"的撕裂数据(torn read)，
 *      算出错误的功率，误触发 OPP 保护。
 *
 *   2. 无封装：谁都能改全局变量，改坏了无从追查。
 *
 * 本模块的解法 —— "集中管理 + 接口封装 + 强制加锁"：
 *   - 所有跨任务状态收敛为本模块内部的 static 变量，外部无法直接触碰。
 *   - 唯一的访问方式是本文件声明的 Get/Set 接口，每个接口内部都自带
 *     并发保护（互斥锁或临界区）。调用方拿到的永远是一份"一致的快照"。
 *
 * 【架构决策：为什么是"全局状态集中管理"而不是消息队列？】
 *   数控电源的状态是"最新值语义"（PID 只关心最新的 ADC 读数，不关心历史），
 *   用共享内存 + 锁比队列更省内存、延迟更低。F103C8T6 只有 20KB SRAM，
 *   队列缓存历史数据是浪费。（若是"事件语义"如按键事件，才用队列。）
 *
 * ============================================================================
 * 【并发保护策略速查表 —— 本模块的核心取舍】
 * ============================================================================
 *   数据          | 保护方式              | 为什么
 *   --------------|----------------------|---------------------------------
 *   setting       | FreeRTOS 互斥锁 Mutex | 低频访问(UI/Comm 写, PID 读)，
 *   calibration   | (xSemaphoreTake/Give) | 可容忍阻塞，Mutex 带优先级继承
 *   --------------|----------------------|---------------------------------
 *   adc_data      | 临界区                | 高频(1ms 写)，临界区开销极小，
 *                 | (taskENTER_CRITICAL) | 不引入调度延迟
 *   --------------|----------------------|---------------------------------
 *   g_hw_ovp_flag | volatile 裸访问(无锁)  | 由 ISR 置位，ISR 内禁止用任何
 *                 | (定义见 protect.h)     | FreeRTOS API / Mutex
 *
 * 详细取舍理由见 app_state.c 各函数注释。
 */

#ifndef __APP_STATE_H
#define __APP_STATE_H

/* --- MISRA-C: 所有 #include 集中放文件顶部 --- */
#include "app_config.h"     /* SystemSetting_t / ADCData_t / Calibration_t / *_DEFAULT 宏 */
#include "protect.h"        /* ProtectContext_t 已在此完整定义(5 字段)，本模块直接复用，
                               绝不重复定义，避免类型冲突与循环依赖 */
#include "FreeRTOS.h"       /* FreeRTOS 内核 (临界区宏 taskENTER_CRITICAL 等) */
#include "semphr.h"         /* 信号量/互斥锁 API (xSemaphoreCreateMutex 等) */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 对外接口 (全部自带并发保护，外部禁止直接访问内部全局变量)
 *===========================================================================*/

/**
 * @brief 初始化全局状态管理层
 *
 * 为什么需要：系统上电后、任何任务/中断启动前，必须先建立好互斥锁并把
 *            所有共享状态置为安全默认值，否则首个访问者会拿到未初始化数据。
 * 输入：无
 * 输出：无
 * 调用时机：main() 中，在 osKernelStart()/vTaskStartScheduler() 之前调用一次。
 * 副作用：创建 FreeRTOS 互斥锁（占用一个内核对象）；
 *         初始化 setting(v_set/i_set/output_enable=关)、校准(用 *_DEFAULT 宏)、
 *         并调用 Protect_Init 初始化保护上下文。
 * 注意：必须在调度器启动前调用，此时无并发，内部初始化不加锁是安全的。
 */
void AppState_Init(void);

/**
 * @brief 写入系统设定值 (设定电压/电流/输出使能)
 *
 * 为什么需要：UI 任务(旋钮调节)和 Comm 任务(上位机指令)都会修改设定值，
 *            必须串行化写入，防止两个写者互相覆盖。
 * 输入：s —— 调用方准备好的一份完整设定值 (物理单位: v_set[V], i_set[A])
 * 输出：无
 * 调用时机：UI 旋钮改值后 / Comm 收到设定指令后。
 * 副作用：内部整体拷贝 *s 到全局 g_setting，Mutex 保护。
 */
void AppState_SetSetting(const SystemSetting_t *s);

/**
 * @brief 读取系统设定值 (返回一致快照)
 *
 * 为什么需要：PID 任务每周期需要最新目标值，且必须是一致快照
 *            (v_set/i_set/output_enable 同一时刻的值)。
 * 输入：out —— 调用方提供的接收缓冲区
 * 输出：*out 被填入当前设定值的完整拷贝
 * 调用时机：PID 任务每 10ms / UI 刷新显示时。
 * 副作用：Mutex 保护下拷贝，无状态修改。
 */
void AppState_GetSetting(SystemSetting_t *out);

/**
 * @brief 更新最新 ADC 采样数据
 *
 * 为什么需要：替代 v1.0 中未定义、无保护的裸全局变量 g_latest_adc_data。
 *            ADCData_t 是多字段结构，写入非原子，必须整体保护防撕裂读。
 * 输入：d —— ADC 任务滤波+校准后的一帧数据 (v_out[V], i_out[A], v_in[V], temp[°C])
 * 输出：无
 * 调用时机：ADC 任务每 1ms 完成一帧滤波后。
 * 副作用：临界区保护下整体拷贝 *d 到 g_adc_data。
 * 注意：用临界区(非 Mutex)，因为本函数从高频任务调用，且不能从 ISR 调用
 *      Mutex —— 详见 .c 实现内注释。
 */
void AppState_UpdateADC(const ADCData_t *d);

/**
 * @brief 读取最新 ADC 数据 (返回一致快照)
 *
 * 输入：out —— 调用方提供的接收缓冲区
 * 输出：*out 被填入最新一帧 ADC 数据的完整拷贝
 * 调用时机：PID 任务(读反馈)、Monitor 任务(保护检测)、UI(显示实测值)。
 * 副作用：临界区保护下拷贝，无状态修改。
 */
void AppState_GetADC(ADCData_t *out);

/**
 * @brief 获取全局唯一的保护上下文指针
 *
 * 为什么需要：Monitor(检测并写故障)、UI(查询显示故障)、Comm(清故障)三方
 *            必须操作同一个 ProtectContext_t 实例，否则故障状态不一致。
 * 输入：无
 * 输出：指向内部唯一 g_protect_ctx 的指针 (永不为 NULL)
 * 调用时机：需要读写保护状态时。
 * 副作用：仅返回指针，不加锁。
 * ⚠️ 并发约定：本函数只交出指针，不保护指针指向的内容。ctx 内容的并发安全
 *    由"约定：仅 Monitor 任务单线程写 ProtectContext"来保证；UI/Comm 对
 *    ctx 的读/清操作也统一走 Monitor 的单线程流程或 protect 模块的接口。
 *    这样避免了对 ctx 每个字段再套一层锁的开销。
 */
ProtectContext_t* AppState_GetProtectCtx(void);

/**
 * @brief 读取校准参数 (返回一致快照)
 *
 * 输入：out —— 调用方提供的接收缓冲区
 * 输出：*out 被填入当前校准系数 (v_slope/v_offset/i_slope/i_offset)
 * 调用时机：ADC 任务把原始值换算为物理量时。
 * 副作用：Mutex 保护下拷贝。
 */
void AppState_GetCalibration(Calibration_t *out);

/**
 * @brief 写入校准参数
 *
 * 为什么需要：上位机标定后下发新校准系数，需串行化写入。
 * 输入：c —— 新的校准系数
 * 输出：无
 * 调用时机：Comm 收到标定指令 / 出厂标定流程。
 * 副作用：Mutex 保护下整体拷贝 *c 到 g_calib。
 */
void AppState_SetCalibration(const Calibration_t *c);

/**
 * @brief 懒创建互斥锁 (v2.2)
 *
 * 为什么需要：xSemaphoreCreateMutex 在 osKernelStart 前调用会触发
 *   configASSERT → HardFault。因此锁延迟到调度器启动后"懒创建"。
 *
 * 调用时机：taskUI 入口处 (调度器启动后、主循环前)。
 * 副作用：首次调用时创建 g_state_mutex；后续调用是空操作。
 * 并发：仅 taskUI 调用，无竞态。
 */
void AppState_EnsureMutex(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_STATE_H */
