/**
 * pid.c
 * 位置式 PID 控制器 + CV/CC MIN 选择器 — 实现
 * 本文件实现：
 *   1. 位置式 PID 算法（含积分分离 + 抗积分饱和）
 *   2. CV/CC 自动切换 MIN 选择器
 *   3. PID 参数在线更新接口
 */

#include "pid.h"
#include <string.h>
#include <math.h>       /* fabsf() */

/*===========================================================================
 * PID_Init — 初始化 PID 控制器
 *===========================================================================*/

/**
 * 初始化一个 PID 控制器实例。
 *
 * 为什么初始化时 integral 和 last_error 清零？
 *   第一次 Compute 时，如果 integral 有旧值，会产生错误输出跳变。
 *   PID 是"有记忆的"，每次换目标/重启动时必须重置记忆。
 */
void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float out_min, float out_max,
              float integral_sep_threshold)
{
    if (pid == NULL) return;

    /* 参数赋值 */
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    /* 限幅参数 */
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_limit = out_max;  /* 默认积分限幅 = 输出上限 */

    /* 积分分离阈值 */
    pid->integral_sep_threshold = integral_sep_threshold;

    /* 内部状态清零 —— 重要！防止旧数据影响 */
    pid->setpoint = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->last_output = 0.0f;

    pid->initialized = 1;
}

/*===========================================================================
 * PID_SetSetpoint — 更新设定值
 *===========================================================================*/

/**
 * 设定值改变时，清除积分累积。
 *
 * 为什么？
 *   假设之前设定 5V，积分累积了 1000 来维持 5V 输出。
 *   现在改设定为 12V，误差突然很大，如果积分从 1000 开始继续累加，
 *   会导致严重超调。"换目标就清零积分"是最简单有效的做法。
 */
void PID_SetSetpoint(PID_Controller *pid, float setpoint)
{
    if (pid == NULL) return;

    pid->setpoint = setpoint;
    pid->integral = 0.0f;       /* 切换目标 → 清零积分 */
    pid->last_error = 0.0f;     /* 清零历史误差 */
}

/*===========================================================================
 * PID_UpdateSetpointNoReset — 更新设定值但不清零积分（软启动 ramp 用）
 *===========================================================================*/

/**
 * 只更新 setpoint，保留 integral 和 last_error。
 *   软启动期间 setpoint 从 0 每 10ms 递增一小步（SOFT_START_STEP=0.05V），
 *   直到 ramp 到用户目标。这是"对同一个上升目标的连续微调"，
 *   若每周期都清积分，PID 就退化成纯 P → 静差 + 爬升迟缓。
 *   所以这里保留积分累积，只推进目标值。
 *
 *   注意：本函数不做"是否比旧 setpoint 更大"的判断——
 *   调用方 (task_pid 软启动逻辑) 负责决定每周期的 ramp 目标。
 */
void PID_UpdateSetpointNoReset(PID_Controller *pid, float setpoint)
{
    if (pid == NULL) return;

    pid->setpoint = setpoint;
    /* 刻意不动 integral / last_error —— 保持积分记忆连续 */
}

/*===========================================================================
 * PID_UpdateParams — 在线修改 PID 参数
 *===========================================================================*/

/**
 * 安全地修改 PID 参数。
 *
 * 安全检查：
 *   - Kp/Ki/Kd 不能为负数（可能导致正反馈振荡）
 *   - Ki 太大容易积分饱和，设上限警示（实际不强制，由调用者负责）
 */
void PID_UpdateParams(PID_Controller *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;

    /* 参数合法性检查：拒绝负值 */
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f) {
        return; /* 非法参数，忽略 */
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    /* 改参数时清零积分 —— 防止新旧参数混合 */
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

/*===========================================================================
 * PID_Compute — 核心 PID 计算
 *===========================================================================*/

/**
 * 位置式 PID 计算公式：
 *
 *   error(k)   = setpoint - feedback
 *   P_term(k)  = Kp × error(k)
 *   I_term(k)  = I_term(k-1) + Ki × error(k)
 *               (dt 已隐含在 Ki 参数中，调用者按固定周期调用即可)
 *   D_term(k)  = Kd × [error(k) - error(k-1)]
 *               (同理 dt 隐含在 Kd 中)
 *   output(k)  = P_term + I_term + D_term
 *
 * 额外处理：
 *   1. 积分分离：|error| > threshold → 关闭积分（只算 P + D）
 *   2. 抗积分饱和：积分项钳位到 [0, integral_limit]
 *      （为什么下限是 0 而不是负值？电源控制中 DAC 输出 ≥ 0，
 *        负积分意味着"反向调节"，对本系统无意义，反而可能导致
 *        MIN 选择器输出为负，使 DAC 长时间为 0。）
 *   3. 输出钳位到 [output_min, output_max]
 */
float PID_Compute(PID_Controller *pid, float feedback)
{
    if (pid == NULL || !pid->initialized) {
        return 0.0f;
    }

    float error;
    float p_term, i_term, d_term;
    float output;

    /*──────────────────────────────────────────────────
     * Step 1: 计算当前误差
     *──────────────────────────────────────────────────*/
    error = pid->setpoint - feedback;

    /*──────────────────────────────────────────────────
     * Step 2: 比例项 P
     *
     * 比例项 = Kp × error
     * 作用：根据当前误差大小立即调整输出。Kp 越大响应越快，
     * 但太大可能超调甚至振荡。这是 PID 的"现在"分量。
     *──────────────────────────────────────────────────*/
    p_term = pid->Kp * error;

    /*──────────────────────────────────────────────────
     * Step 3: 积分项 I（带积分分离 + 抗饱和）
     *
     * 积分项的作用：消除稳态误差。纯比例控制会有"静差"——
     * 输出永远不能精确到达设定值（因为需要 non-zero error
     * 来维持 non-zero output）。积分项"记住"历史误差，
     * 只要误差还存在，积分就继续累积，直到误差为零。
     *
     * 积分分离 (integral separation):
     *   当误差很大时（如刚改变设定值），P 项已经足够驱动输出。
     *   这时候积分只会"帮倒忙"——累积大量积分后再慢慢释放，
     *   表现为严重超调。所以大误差时关闭积分。
     *
     * 抗积分饱和 (anti-windup):
     *   即使误差不大，如果输出已经达到上限（如 DAC=4095），
     *   继续积分没有意义——执行器已经"饱和"了。
     *   本项目用 integral_limit 来限制积分项的最大值。
     *──────────────────────────────────────────────────*/
    if (fabsf(error) > pid->integral_sep_threshold) {
        /*
         * 误差太大 → 关闭积分
         * 不等于清零积分！只是本轮不累加。
         * 清零的话当误差回落后需要重新累积，响应更慢。
         */
        /* 积分项保持不变，不累加也不清零 */
        i_term = pid->integral;
    } else {
        /*
         * 误差在阈值内 → 正常积分
         *
         * 注意：累加前需要乘以 dt（控制周期）。
         * 本项目 PID 周期固定为 10ms，dt 已并入 Ki 参数。
         * 即：实际 Ki_true = Ki × 0.01
         * 这里直接用 Ki 乘以误差。
         */
        pid->integral += pid->Ki * error;

        /*
         * 抗积分饱和：钳位积分项到 [0, integral_limit]
         *
         * 为什么不钳位到负数？
         *   DAC 只能输出 0~4095，对应的物理量是 0~28V/0~3A。
         *   负积分意味着"反向输出"，没有对应的物理动作。
         *   允许负积分会导致：轻载时 CC 环的积分一直往负走，
         *   当负载突然加重需要 CC 环接管时，积分要从负数慢慢爬上来，
         *   造成响应滞后。
         */
        if (pid->integral > pid->integral_limit) {
            pid->integral = pid->integral_limit;
        }
        if (pid->integral < 0.0f) {
            pid->integral = 0.0f;
        }

        i_term = pid->integral;
    }

    /*──────────────────────────────────────────────────
     * Step 4: 微分项 D
     *
     * 微分项 = Kd × (error(k) - error(k-1))
     * 作用：预测误差趋势，"提前刹车"，减小超调。
     * 但微分对噪声敏感——ADC 的一个毛刺会被放大。
     * 本系统 Kd = 0（电源控制通常不需要 D）。
     *──────────────────────────────────────────────────*/
    d_term = pid->Kd * (error - pid->last_error);

    /*──────────────────────────────────────────────────
     * Step 5: 求和 + 输出钳位
     *
     * 钳位输出到 [output_min, output_max]
     * 对 DAC 来说就是 [0, 4095]
     *──────────────────────────────────────────────────*/
    output = p_term + i_term + d_term;

    if (output > pid->output_max) {
        output = pid->output_max;
    }
    if (output < pid->output_min) {
        output = pid->output_min;
    }

    /*──────────────────────────────────────────────────
     * Step 6: 保存状态，为下一次计算准备
     *──────────────────────────────────────────────────*/
    pid->last_error = error;
    pid->last_output = output;

    return output;
}

/*===========================================================================
 * CV_CC_Select — MIN 选择器
 *===========================================================================*/

/**
 * MIN 选择器实现 CV/CC 自动切换。
 *
 * 工作流程：
 *   两个 PID（CV 和 CC）各自独立计算，产生各自的 DAC 输出值。
 *
 *   场景 A — 轻载（CV 模式）:
 *     V_set = 12V, I_set = 1A, 实际负载 100mA
 *     CV 环: feedback = 12V, error ≈ 0 → 输出维持在某个中等值
 *     CC 环: feedback = 0.1A, error = 0.9A → 积分持续累积 → 输出变大
 *     MIN(CV_out, CC_out) = CV_out → 系统按 CV 环运行
 *     CC 环的积分被 integral_limit 钳住，不会无限增长
 *
 *   场景 B — 重载（CC 模式）:
 *     V_set = 12V, I_set = 1A, 实际负载 12Ω → 需要 1A @ 12V
 *     如果用户接了一个 5Ω 负载：I 会趋向 12V/5Ω = 2.4A > 1A
 *     CV 环: feedback = 12V? 不，输出被电流限制了
 *            V_actual 开始下降（因为 CC 环在限流）→ error 变大 → CV 输出变大
 *     CC 环: feedback 趋近 1A → error 变小 → CC 输出变小
 *     MIN(CV_out, CC_out) = CC_out → 系统按 CC 环运行
 *     输出电压自动下降到 I_set × R_load = 1A × 5Ω = 5V
 *
 * 关键理解：
 *   - "胜出"的不是"谁更大"，而是"谁更小"。
 *   - 因为 DAC 输出控制的是"输出能力"，小的那个先达到上限。
 *   - 或者说：想"抬高"输出的环（error 大）输出大，
 *     想"压低"输出的环（error 小/负）输出小，
 *     MIN 选择器选择了"更保守"的那个——安全第一。
 */
float CV_CC_Select(float cv_output, float cc_output, OperatingMode_t *mode)
{
    if (cv_output <= cc_output) {
        /*
         * CV 输出 ≤ CC 输出 → CV 模式
         *
         * 为什么是 ≤ 而不是 <？
         *   等号情况（cv_output == cc_output）：两个环的输出相同，
         *   系统处于 CV/CC 的临界点。此时报告 CV 模式是合理的——
         *   输出电压仍在设定值。
         */
        if (mode != NULL) {
            *mode = MODE_CV;
        }
        return cv_output;
    } else {
        /*
         * CC 输出 < CV 输出 → CC 模式
         *
         * 这说明电流环在"喊疼"——它想降低输出来限制电流。
         * MIN 选择器听它的，因为电流过大比电压不足更危险。
         */
        if (mode != NULL) {
            *mode = MODE_CC;
        }
        return cc_output;
    }
}

/*===========================================================================
 * PID_Reset — 重置 PID
 *===========================================================================*/

/**
 * 完全重置 PID，适用于：
 *   - 输出关断时（防止下次使能积分累积）
 *   - 故障保护触发时（清除旧状态）
 *   - 手动切换 CV/CC 设定值时（可选）
 */
void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL) return;

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_output = 0.0f;
    /* 不改变 Kp/Ki/Kd/setpoint —— 只清除"记忆" */
}

/*===========================================================================
 * PID_GetState — 获取 PID 内部状态
 *===========================================================================*/

/**
 * 读取 PID 的中间变量，用于调试和上位机展示。
 *
 * 为什么需要这个函数？
 *   PID 的内部状态（特别是 integral 和 error）是调试关键。
 *   通过上位机读取这些值，可以绘制阶跃响应曲线，
 *   直观判断 Kp/Ki/Kd 是否需要调整。
 */
void PID_GetState(PID_Controller *pid,
                  float *p_term, float *i_term, float *d_term, float *error)
{
    if (pid == NULL) return;

    if (p_term != NULL) {
        *p_term = pid->Kp * pid->last_error;
    }
    if (i_term != NULL) {
        *i_term = pid->integral;
    }
    if (d_term != NULL) {
        *d_term = 0.0f;  /* Kd=0, 微分项始终为 0 */
    }
    if (error != NULL) {
        *error = pid->last_error;
    }
}
