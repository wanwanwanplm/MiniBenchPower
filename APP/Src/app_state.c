/**
 * app_state.c
 * 全局状态集中管理层 —— 实现 (共享状态存储 + 并发保护)
 * ============================================================================
 * 🔧 并发保护：Mutex vs 临界区
 * ============================================================================
 *   📐 原理：多任务/中断访问同一内存 = 竞态。保护手段有两类：
 *
 *   1) FreeRTOS 互斥锁 (Mutex, xSemaphoreCreateMutex)
 *      - 机制：任务拿不到锁时"阻塞挂起"，让出 CPU 给别人跑。
 *      - 优点：不忙等，带"优先级继承"防优先级反转。
 *      - 代价：涉及调度器上下文切换，开销较大 (数十条指令+)。
 *      - ❌ 绝对禁止在 ISR 中使用 (ISR 不能阻塞/挂起)。
 *      - 适用：低频、可容忍短暂阻塞的数据 —— 本模块的 setting、calibration。
 *
 *   2) 临界区 (taskENTER_CRITICAL / taskEXIT_CRITICAL)
 *      - 机制：直接关中断(到 configMAX_SYSCALL_INTERRUPT_PRIORITY 阈值)，
 *              保护期间任何任务/受管中断都无法打断，天然互斥。
 *      - 优点：开销极小(几条指令)，无上下文切换。
 *      - 代价：关中断期间会拉高中断延迟，所以临界区必须"极短"。
 *      - 适用：高频、拷贝极快的数据 —— 本模块的 adc_data (1ms 更新一次)。
 *
 *   💻 本模块选择：
 *      setting/calib  → Mutex   (低频，UI/Comm 写、PID 读，拷贝可容忍阻塞)
 *      adc_data       → 临界区   (高频 1ms，拷贝 20 字节耗时极短，用锁太重)
 *
 *   ⚠️ 常见坑：
 *      - 在 ISR 里调用 xSemaphoreTake → 直接把系统搞挂 (ISR 不能阻塞)。
 *      - 临界区里做耗时操作(如浮点循环) → 中断延迟飙升，实时性崩溃。
 *      - 忘记 Give 锁 → 死锁，其他任务永远拿不到。
 *
 * ============================================================================
 * 🔧 ISR 安全：为什么硬件 OVP 标志只用 volatile 裸变量？
 * ============================================================================
 *   g_hw_ovp_flag 由 EXTI4 ISR (优先级 3，高于 FreeRTOS 阈值) 置位。
 *   该 ISR 内：
 *     - 不能用 Mutex/临界区之外的 FreeRTOS API (优先级超阈值，行为未定义)；
 *     - 只做最少动作：把一个 volatile uint8_t 置 1。
 *   uint8_t 写入在 Cortex-M3 上是单条 STRB 指令，天然原子，无需加锁。
 *   volatile 保证编译器每次都从内存真实读写、不做寄存器缓存优化，
 *   使 Monitor 任务轮询时一定能看到 ISR 刚写的值。
 *   之后由 vTaskMonitor 轮询该标志，再合并进 ProtectContext (置故障+锁存)。
 *   —— 这是嵌入式经典范式："ISR 只置标志，重活留给任务"。
 *
 * ============================================================================
 * 🔧 v2 修正: 互斥锁"懒创建"
 * ============================================================================
 *   问题：xSemaphoreCreateMutex/CreateMutexStatic 在 osKernelStart() 前调用
 *   会触发 configASSERT 崩溃 (HardFault → 白屏)。
 *
 *   解法：
 *     1. AppState_Init() 只初始化数据，不创建锁（此时单线程安全）。
 *     2. Get/Set 内做 if(g_state_mutex) 判空：空 → 直接读写字面，不空 → 锁保护。
 *     3. 调度器启动后由 taskUI 调用 AppState_EnsureMutex() 建锁。
 *     4. 建锁后所有路径自动切换到加锁模式。
 *
 *   安全性论证：
 *     - 建锁前：仅 main() 单线程 + taskUI 单任务 (其他 4 任务未创建)。
 *       taskUI 第一个动作是 EnsureMutex，建锁后才进主循环，
 *       因此不存在"一任务持锁、另一任务无锁越权"的窗口。
 *     - 建锁后：所有 Get/Set 走锁路径，和原设计完全一致。
 */

/* --- MISRA-C: 所有 #include 集中放文件顶部 --- */
#include "app_state.h"      /* 本模块对外接口 + 已间接包含 app_config/protect/FreeRTOS/semphr */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"           /* taskENTER_CRITICAL / taskEXIT_CRITICAL */
#include <string.h>         /* memcpy: 结构体整体拷贝，保证快照一致性 */

/*===========================================================================
 * 内部全局状态 (唯一实例，全部 static —— 外部只能经接口访问)
 *===========================================================================*/

/* 当前系统设定值 (v_set/i_set/output_enable)，Mutex 保护 */
static SystemSetting_t   g_setting;

/* 最新 ADC 采样数据 (滤波+校准后)，临界区保护 —— 替代旧的裸 g_latest_adc_data */
static ADCData_t         g_adc_data;

/* 全局唯一保护上下文，经 AppState_GetProtectCtx 交出指针，
   内容并发安全由"Monitor 单线程访问约定"负责 (见 .h 说明) */
static ProtectContext_t  g_protect_ctx;

/* 校准系数，Mutex 保护 */
static Calibration_t     g_calib;

/*
 * 状态互斥锁：保护 g_setting 与 g_calib。
 * 二者共用一把锁足矣 —— 访问都是低频，锁竞争极低，省一个内核对象。
 * v2: 锁改为"懒创建" (调度器启动前 = NULL, 启动后由 EnsureMutex 赋值)。
 */
static StaticSemaphore_t g_state_mutex_buffer;
static SemaphoreHandle_t g_state_mutex = NULL;

/*
 * 硬件 OVP 中断标志 g_hw_ovp_flag 的定义在 protect.c (归属保护模块)。
 * 本文件通过 protect.h 的 extern 声明使用它, 不重复定义 (避免 L6200E multiply defined)。
 */

/*===========================================================================
 * 初始化 (调度器启动前调用, 单线程安全, 不创建锁)
 *===========================================================================*/

void AppState_Init(void)
{
    /*
     * 【段落：设定值安全默认 —— 上电即安全】
     * 输出默认"关断"(output_enable=0)：绝不允许上电瞬间就带电输出，
     * 必须等用户显式使能。v_set/i_set 给一个保守的低设定值。
     */
    g_setting.v_set         = 0.0f;   /* 默认 0V，用户使能前无输出目标 */
    g_setting.i_set         = 0.1f;   /* 默认限流 0.1A，防误接负载时大电流冲击 */
    g_setting.output_enable = 0U;     /* 关键安全默认：输出关断 */

    /*
     * 【段落：ADC 数据清零】
     * 首帧真实 ADC 到来前，读者拿到全 0 快照(而非随机值)，避免误判。
     */
    (void)memset(&g_adc_data, 0, sizeof(g_adc_data));

    /*
     * 【段落：校准系数装载出厂默认】
     * 用 app_config.h 的 *_DEFAULT 宏(slope=1.0, offset=0.0 = 未校准直通)。
     * 实际标定值稍后可由上位机经 AppState_SetCalibration 覆盖。
     */
    g_calib.v_slope  = V_CAL_SLOPE_DEFAULT;
    g_calib.v_offset = V_CAL_OFFSET_DEFAULT;
    g_calib.i_slope  = I_CAL_SLOPE_DEFAULT;
    g_calib.i_offset = I_CAL_OFFSET_DEFAULT;

    /*
     * 【段落：保护上下文初始化】
     * 委托给 protect 模块自己的初始化函数(单一职责)，清空故障标志/解锁存等。
     */
    Protect_Init(&g_protect_ctx);

    /* 硬件 OVP 标志复位 (与全局定义处初值一致，此处再确保一次) */
    g_hw_ovp_flag = 0U;

    /*
     * 注意：此处【不】创建互斥锁。锁改为"懒创建"——
     * 调度器启动后由 taskUI 调用 AppState_EnsureMutex() 完成。
     * 建锁前 Get/Set 内部 if(g_state_mutex) 判空,
     * 单线程下安全直读直写 (详见文件头 v2.2 修正说明)。
     */
}

/*===========================================================================
 * v2 懒创建互斥锁 (调度器启动后, 第一个任务调用)
 *===========================================================================*/

void AppState_EnsureMutex(void)
{
    if (g_state_mutex == NULL) {
        g_state_mutex = xSemaphoreCreateMutexStatic(&g_state_mutex_buffer);
        configASSERT(g_state_mutex != NULL);
    }
}

/*===========================================================================
 * 设定值 (Mutex 保护 / 建锁前直通)
 *===========================================================================*/

void AppState_SetSetting(const SystemSetting_t *s)
{
    if (s == NULL) {            /* 防御：空指针直接返回，不破坏全局状态 */
        return;
    }
    if (g_state_mutex != NULL) {
        (void)xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    }
    g_setting = *s;             /* 结构体赋值 = 逐字段整体拷贝，天然一致 */
    if (g_state_mutex != NULL) {
        (void)xSemaphoreGive(g_state_mutex);
    }
}

void AppState_GetSetting(SystemSetting_t *out)
{
    if (out == NULL) {
        return;
    }
    if (g_state_mutex != NULL) {
        (void)xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    }
    *out = g_setting;           /* 拷出一致快照，调用方读 out 时不再有并发 */
    if (g_state_mutex != NULL) {
        (void)xSemaphoreGive(g_state_mutex);
    }
}

/*===========================================================================
 * ADC 数据 (临界区保护 —— 高频、轻量)
 *===========================================================================*/

void AppState_UpdateADC(const ADCData_t *d)
{
    if (d == NULL) {
        return;
    }
    /*
     * 用临界区而非 Mutex 的原因：
     *   1) ADC 任务 1ms 一次，频次高，Mutex 上下文切换开销放大明显；
     *   2) 结构体拷贝(约 20 字节)耗时极短，关中断窗口很小，中断延迟可忽略；
     *   3) 若将来需从 ADC-DMA 完成中断里更新，Mutex 根本用不了，
     *      临界区(受管中断版可用 ...FROM_ISR)才是可扩展方向。
     */
    taskENTER_CRITICAL();       /*进入临界区*/
    g_adc_data = *d;            /* 整体拷贝，读者永远拿不到撕裂的半新半旧数据 */
    taskEXIT_CRITICAL();        /*退出临界区*/
}

void AppState_GetADC(ADCData_t *out)
{
    if (out == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    *out = g_adc_data;          /* 拷出一致快照 */
    taskEXIT_CRITICAL();
}

/*===========================================================================
 * 保护上下文 (仅交出指针，内容并发由 Monitor 单线程约定负责)
 *===========================================================================*/

ProtectContext_t* AppState_GetProtectCtx(void)
{
    /*
     * 返回唯一实例地址。不加锁：加锁只能保护"取指针"这一瞬间，
     * 保护不了调用方随后对 ctx 内容的持续访问。真正的一致性靠
     * "仅 Monitor 任务写 ProtectContext" 的单线程访问约定(见 .h)。
     */
    return &g_protect_ctx;
}

/*===========================================================================
 * 校准参数 (Mutex 保护 / 建锁前直通)
 *===========================================================================*/

void AppState_GetCalibration(Calibration_t *out)
{
    if (out == NULL) {
        return;
    }
    if (g_state_mutex != NULL) {
        (void)xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    }
    *out = g_calib;             /* 一致快照 */
    if (g_state_mutex != NULL) {
        (void)xSemaphoreGive(g_state_mutex);
    }
}

void AppState_SetCalibration(const Calibration_t *c)
{
    if (c == NULL) {
        return;
    }
    if (g_state_mutex != NULL) {
        (void)xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    }
    g_calib = *c;
    if (g_state_mutex != NULL) {
        (void)xSemaphoreGive(g_state_mutex);
    }
}
