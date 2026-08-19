/**
 * pid.h
 * 位置式 PID 控制器 + CV/CC MIN 选择器
 *
 * 关键设计点：
 *   1. 为什么用"位置式 PID"而非"增量式 PID"？
 *      → 因为 CV 和 CC 两个 PID 并行运行，MIN 选择器需要比较二者的
 *        绝对输出值。增量式 PID 输出的是 △u，无法直接比较。
 *
 *   2. 为什么 MIN 选择器不需要 if/else 判断 CV/CC 模式？
 *      → 两个 PID 都在持续计算，输出较小的那个"胜出"控制 DAC。
 *        当负载电流 < 设定电流时，CC 环的误差小 → 输出大 → CV 环胜出；
 *        当负载电流接近设定电流时，CC 环输出变小 → CC 环胜出。
 *        模式切换是"自动的、无感的"——没有模式标志位翻转。
 *
 *   3. 为什么不用 PID 的 D 项？
 *      → 电源负载通常是电阻性或缓变负载，不需要微分预测。
 *        D 项对 ADC 采样噪声放大严重，反而导致 DAC 输出抖动。
 */

#ifndef __PID_H
#define __PID_H

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 类型定义
 *===========================================================================*/

/**
 * @brief 位置式 PID 控制器结构体
 *
 * 为什么用结构体封装 PID？
 *   答：一个系统有两个 PID（CV + CC），结构体封装让代码复用，
 *   不重复写两遍 PID 计算逻辑。这也是 OOP 思想在 C 语言中的体现。
 *
 * 内存占用：约 40 bytes / 实例（7 个 float + 1 个 uint8_t），
 * 两个 PID 共占 ~80 bytes RAM，F103C8T6 的 20KB SRAM 完全够用。
 */
typedef struct {
    /* PID 参数（可在线修改） */
    float Kp;               /* 比例系数 */
    float Ki;               /* 积分系数 */
    float Kd;               /* 微分系数 */

    /* PID 内部状态 */
    float setpoint;         /* 设定值 (目标) */
    float last_error;       /* 上一次误差 e(k-1)，用于微分项计算 */
    float integral;         /* 积分累积项 */
    float last_output;      /* 上一次输出 u(k-1) */

    /* 限幅参数 */
    float output_min;       /* 输出下限 */
    float output_max;       /* 输出上限 */
    float integral_limit;   /* 积分项上限 (用于抗积分饱和) */

    /*
     * 积分分离阈值：|error| > this → 关闭积分
     * 防止大偏差时积分累积过多（windup）
     */
    float integral_sep_threshold;

    /* 标志位 */
    uint8_t initialized;    /* 初始化标志 */
} PID_Controller;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化 PID 控制器
 *
 * @param pid       PID 控制器指针
 * @param kp        比例系数
 * @param ki        积分系数
 * @param kd        微分系数
 * @param out_min   输出下限
 * @param out_max   输出上限
 * @param integral_sep_threshold  积分分离阈值
 *
 * 调用时机：系统上电初始化时调用一次（在 FreeRTOS 调度器启动前）
 */
void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float out_min, float out_max,
              float integral_sep_threshold);

/**
 * @brief 更新 PID 设定值
 *
 * @param pid       PID 控制器指针
 * @param setpoint  新的设定值
 *
 * 调用时机：用户旋钮调节或上位机修改设定值时
 * 注意：设定值改变时会自动清除积分累积，防止旧积分影响新目标
 */
void PID_SetSetpoint(PID_Controller *pid, float setpoint);

/**
 * @brief 更新设定值但【不清零积分】—— 供软启动 ramp 每周期调用
 *
 * @param pid       PID 控制器指针
 * @param setpoint  新的设定值
 *
 * 调用时机：软启动 (soft-start) 期间，setpoint 从 0 每 10ms 递增 SOFT_START_STEP
 *          缓慢 ramp 到用户目标值。
 *
 * 【为什么必须区别于 PID_SetSetpoint？】
 *   PID_SetSetpoint 每次都清零积分（换目标就重置记忆，语义正确）。
 *   但软启动 ramp 每个 10ms 周期都要"微调"setpoint（+0.05V），
 *   若每周期都调 PID_SetSetpoint，积分会被反复清零 →
 *   积分项永远累积不起来 → PID 退化成纯比例 (P) 控制 →
 *   稳态存在静差、输出爬升迟缓。
 *   ramp 是"连续微调同一个上升目标"，不是"跳变到新目标"，
 *   所以只更新 setpoint、保留积分累积，控制才连续平滑。
 *
 *   区分原则：
 *     - 用户/上位机把目标从 5V 跳到 12V → PID_SetSetpoint（清积分）
 *     - 软启动 ramp 每周期 setpoint += 0.05V → PID_UpdateSetpointNoReset（不清）
 */
void PID_UpdateSetpointNoReset(PID_Controller *pid, float setpoint);

/**
 * @brief 在线修改 PID 参数
 *
 * @param pid  PID 控制器指针
 * @param kp   新的比例系数
 * @param ki   新的积分系数
 * @param kd   新的微分系数
 *
 * 调用时机：上位机发送调参命令时
 *
 * 如何防止在线改参数导致系统不稳定？
 *   答：(1) 参数范围检查（拒绝负数/过大值）
 *       (2) 改参数时清零积分项，防止新旧参数混合导致输出跳变
 *       (3) 上位机/用户应在轻载或空载时调参
 */
void PID_UpdateParams(PID_Controller *pid, float kp, float ki, float kd);

/**
 * @brief 执行一次 PID 计算
 *
 * @param pid        PID 控制器指针
 * @param feedback   当前反馈值（实际 ADC 读数转换后的物理量）
 * @return float     本次 PID 输出值 (钳位到 [output_min, output_max])
 *
 * 调用时机：vTaskPID 每个控制周期（10ms）调用一次
 *
 * 核心算法（位置式 PID）：
 *   error = setpoint - feedback
 *   P_term = Kp × error
 *   I_term = integral += Ki × error × dt    (dt 隐含在 Ki 中)
 *   D_term = Kd × (error - last_error) / dt (dt 隐含在 Kd 中)
 *   output = P_term + I_term + D_term
 */
float PID_Compute(PID_Controller *pid, float feedback);

/**
 * @brief CV/CC MIN 选择器
 *
 * @param cv_output  CV (恒压) PID 的当前输出
 * @param cc_output  CC (恒流) PID 的当前输出
 * @param mode       输出当前工作模式 (CV or CC)，用于 UI 显示
 * @return float     cv_output 和 cc_output 中较小的值
 *
 * 原理：
 *   - 轻载时：实际电流 << 设定电流，CC 环误差大→输出大
 *             MIN(cv, cc) = cv → 系统工作在 CV 模式
 *   - 重载时：实际电流 ≈ 设定电流，CC 环输出减小
 *             MIN(cv, cc) = cc → 系统工作在 CC 模式
 *   - 切换是无感的——不需要判断语句、不需要标志位翻转
 *
 * 为什么 MIN 选择器不需要 if/else？
 *    答：因为没有"切换"这回事。两个 PID 一直并行运行，较小的那个
 *    输出自然"胜出"。这比 if(I_out > I_set) 切换更平滑——避免了
 *    切换瞬间的 DAC 输出跳变。
 */
float CV_CC_Select(float cv_output, float cc_output, OperatingMode_t *mode);

/**
 * @brief 重置 PID 控制器（清除积分和误差历史）
 *
 * @param pid  PID 控制器指针
 *
 * 调用时机：
 *   - 输出关断时（防止下次使能时积分累积导致过冲）
 *   - 故障保护触发时
 *   - CV/CC 切换时（可选）
 */
void PID_Reset(PID_Controller *pid);

/**
 * @brief 获取 PID 内部状态（用于调试/上位机查询）
 *
 * @param pid        PID 控制器指针
 * @param p_term     输出：比例项值
 * @param i_term     输出：积分项值
 * @param d_term     输出：微分项值
 * @param error      输出：当前误差
 */
void PID_GetState(PID_Controller *pid,
                  float *p_term, float *i_term, float *d_term, float *error);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
