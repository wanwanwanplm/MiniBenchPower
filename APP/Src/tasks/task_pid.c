/**
 * task_pid.c
 * PID 控制任务实现 — 系统核心控制环路 (v2.0, 本批次大修)
 * 本文件实现数控电源的核心控制逻辑：
 *   1. CV (恒压) PID + CC (恒流) PID 并行运行 + MIN 选择器自动切换
 *   2. DAC#1 Buck FB 控制 (反比) + DAC#2 线性栅极控制 (正比, 限幅 3475)
 *   3. 软启动状态机 (idle / ramping) + 设定变化检测
 *   4. 每 10ms 先做保护检查, 有故障即安全关断
 *
 * ─────────────────────────────────────────────────────────────────────────
 * 🔧 算法：位置式 PID + 软启动 ramp + 积分管理 (本批次修正核心)
 *   📐 数学原理：
 *        e(k)=setpoint-feedback
 *        u(k)=Kp·e(k) + Ki·Σe·dt + Kd·(e(k)-e(k-1))/dt   (本项目 Kd=0)
 *        积分项 Σe·dt 是"记忆"—— 它记住了历史误差, 负责消除稳态静差。
 *   💻 代码映射：
 *        setpoint ← 软启动 ramp 目标 (ss_v/ss_i) 或用户设定 (last_v_set/last_i_set)
 *        feedback ← AppState_GetADC 的 v_out / i_out
 *        u(k)     → DAC 码值 → DAC2 (线性) / DAC1 (Buck 预稳压)
 *   🎮 调参指南 (增益宏在 app_config.h)：
 *        Kp↑ 响应快, 过大→超调/振荡;  Ki↑ 消静差快, 过大→积分饱和超调;
 *        Kd↑ 抑超调, 但放大 ADC 噪声致 DAC 抖动 (故本项目 Kd=0)。
 *   ⚠️ 常见坑：
 *        · windup(积分饱和): 靠 pid.c 的积分限幅 + 积分分离阈值抑制。
 *        · 本批次 #8 根因: 旧代码每 10ms 无条件 PID_SetSetpoint → 每周期清积分
 *          → 积分永远累不起来 → PID 退化成纯 P → 有稳态静差。本版仅在"真正跳变"
 *          时清积分, ramp 期间用 UpdateSetpointNoReset, 稳态完全不动 setpoint。
 * ─────────────────────────────────────────────────────────────────────────
 *
 * 【积分管理决策表 —— 本文件最需要理解的部分】
 *   场景                              | 对 setpoint 的动作          | 积分
 *   ----------------------------------|----------------------------|--------
 *   使能上升沿(关→开)                 | PID_SetSetpoint(0)          | 清零
 *   软启动 ramp 期间(每 10ms 推进)    | PID_UpdateSetpointNoReset   | 保留累积  #8
 *   稳态(ramp 完成 & 设定未变)        | 什么都不做                  | 持续累积  #8
 *   运行中用户改设定(已使能)          | 重启 ramp, 平滑过渡         | 保留      #13
 *   输出关断 / 保护故障               | PID_Reset                   | 清零
 *
 * DAC 映射公式：
 *   DAC#2 (线性栅极): V_out = V_DAC2 × 10 → code2 = PID 输出直接用, 限幅 3475
 *   DAC#1 (Buck FB) : V_buck = 32.5 - 10×V_DAC1, 取 V_buck=V_out+2V 预稳压反解
 *   硬件假设来源: docs/03-interface-definition.md 第4.1/4.2 节 (v2.0),
 *                hardware/schematic.md 模块 ④/⑧ (v2.0)
 */

#include "task_pid.h"
#include "pid.h"
#include "dac_mcp4725.h"
#include "app_state.h"        /* #12: 设定/ADC/保护上下文统一走 AppState */
#include "protect.h"          /* #10: 每 10ms Protect_Check */
#include "stm32f1xx_hal.h"
#include "main.h"             /* OUT_EN_Pin/OUT_EN_GPIO_Port (输出使能 GPIO) */

/*===========================================================================
 * 全局变量 (task_pid 拥有; 设定值/ADC 不在此, 走 AppState)
 *===========================================================================*/

PID_Controller  g_pid_cv;                   /* 恒压 PID */
PID_Controller  g_pid_cc;                   /* 恒流 PID */
OperatingMode_t g_current_mode = MODE_CV;   /* 当前工作模式 (供 UI 读) */

/*===========================================================================
 * 软启动状态机 (仅 vTaskPID 单线程访问, 故用普通 static, 无需加锁)
 *===========================================================================*/

/* 软启动状态: 是否正在 ramp */
typedef enum {
    SS_IDLE = 0,        /* 空闲: 稳态或未使能, setpoint 不再推进 */
    SS_RAMPING          /* 正在 ramp: 每周期推进 setpoint */
} SoftStartState_t;

static SoftStartState_t g_ss_state = SS_IDLE;
static float g_ss_v = 0.0f;         /* 软启动当前电压目标 (ramp 中间值) */
static float g_ss_i = 0.0f;         /* 软启动当前电流目标 (ramp 中间值) */

/*
 * #8/#13 关键: 记住"上一次同步给 PID 的用户设定值"。
 * 用它和最新 AppState 设定比较, 判断"用户是否真的改了设定"。
 * 只有真的改了才触发相应动作 (重启 ramp), 而不是每周期都动 setpoint。
 */
static float   g_last_v_set = 0.0f;
static float   g_last_i_set = 0.0f;
static uint8_t g_prev_enable = 0;   /* 上一周期的 output_enable, 用于检测使能上升沿 */
static uint8_t g_dac_safe     = 0;   /* DAC 是否已写入安全关断值 (避免 I2C 失败时每周期重试) */

/* 浮点设定值"是否变化"的比较容差 (避免浮点相等判断的精度陷阱) */
#define SETTING_EPSILON     0.0005f

/*===========================================================================
 * 内部辅助
 *===========================================================================*/

/**
 * @brief 判断两个浮点设定是否"真的不同" (超过容差)
 *
 * 为什么不用 a==b？浮点相等比较不可靠 (量化/累加误差), 且 UI 每步 0.01V
 * 的改动必须被识别为"变化", 故用差值绝对值 > 容差判定。不依赖 libm。
 * setting_changed(setting.v_set, g_last_v_set)
 */
static uint8_t setting_changed(float a, float b)
{
    float d = a - b;
    if (d < 0.0f) {
        d = -d;
    }
    return (d > SETTING_EPSILON) ? 1U : 0U;
}

/**
 * @brief 单方向 ramp 推进: 把 *cur 朝 target 迈一步 (步长 SOFT_START_STEP)
 * @return 1 = 已到达 target, 0 = 仍在途中
 * ramp_step(&g_ss_v, g_last_v_set)
 * 支持双向: target 比当前大则上升, 比当前小则下降 (运行中调低设定 #13)。
 */
static uint8_t ramp_step(float *cur, float target)
{
    if (*cur < target) {
        *cur += SOFT_START_STEP;
        if (*cur >= target) { *cur = target; return 1U; }
        return 0U;
    } else if (*cur > target) {
        *cur -= SOFT_START_STEP;
        if (*cur <= target) { *cur = target; return 1U; }
        return 0U;
    }
    return 1U;   /* 已相等 */
}

/**
 * @brief 安全关断输出 (关断 / 故障时调用)
 *
 * DAC#1 写 DAC1_MAX_CODE → Buck 输出最低 (反比映射, 码值越大电压越低);
 * DAC#2 写 0 → 线性 P-MOS 栅极为 0 → 截止 → V_out=0。
 * 同时复位两个 PID 并把软启动状态清回 IDLE, 防止下次使能时积分残留致过冲。
 */
static void pid_safe_shutdown(void)
{
    PID_Reset(&g_pid_cv);
    PID_Reset(&g_pid_cc);
    g_ss_state = SS_IDLE;
    g_ss_v = 0.0f;
    g_ss_i = 0.0f;
    HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_RESET);  /* 硬件关断输出 */
    if (!g_dac_safe) {
        (void)DAC_SetBoth(DAC1_MAX_CODE, 0);   /* Buck 最低 + 线性截止 (仅首次) */
        g_dac_safe = 1;
    }
}


void TaskPID_LoadFromAppState(void)
{
    /*
     * #32: 把 AppState 中的初始设定同步进本层"上次设定"记忆。
     * 这样首次使能时软启动 ramp 的目标就是用户上次保存的值 (来自 EEPROM),
     * 而非硬编码 5V/1A。仅在调度器启动前调用是安全的 (无并发)。
     */
    SystemSetting_t s;
    AppState_GetSetting(&s);
    g_last_v_set = s.v_set;
    g_last_i_set = s.i_set;
    g_prev_enable = s.output_enable;   /* 若上电即使能, 首个周期不会误判为使能上升沿 */
}

OperatingMode_t TaskPID_GetMode(void)
{
    return g_current_mode;
}

/*===========================================================================
 * PID 任务主循环
 *===========================================================================*/

void vTaskPID(void *argument)
{
    ADCData_t         adc_data;
    SystemSetting_t   setting;
    ProtectContext_t *ctx;              /* #11: 全局唯一保护上下文 */
    TickType_t        xLastWakeTime;
    float             cv_output, cc_output, dac_output;
    uint16_t          dac1_code, dac2_code;
    float             v_buck_desired, v_dac1;
    FaultFlag_t       faults;

    (void)argument;
	
    /* 初始化两个 PID 控制器 (增益取 app_config 默认宏; 上位机可后续在线改) */
    PID_Init(&g_pid_cv,
             PID_CV_KP_DEFAULT, PID_CV_KI_DEFAULT, PID_CV_KD_DEFAULT,
             PID_OUTPUT_MIN, PID_OUTPUT_MAX,
             PID_CV_INTEGRAL_SEPARATION_THRESHOLD);

    PID_Init(&g_pid_cc,
             PID_CC_KP_DEFAULT, PID_CC_KI_DEFAULT, PID_CC_KD_DEFAULT,
             PID_OUTPUT_MIN, PID_OUTPUT_MAX,
             PID_CC_INTEGRAL_SEPARATION_THRESHOLD);

    /* #32: 从 AppState 同步初始设定 (main.c 已由 EEPROM 灌入上次的 v_set/i_set) */
    TaskPID_LoadFromAppState();
		
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        /*──────────────────────────────────────────────────
         * Step 1: 读最新 ADC 反馈 (#12 走 AppState, 拿一致快照)
         *   不再用 adc_queue。AppState_GetADC 内部临界区保护, 防撕裂读。
         *──────────────────────────────────────────────────*/
        AppState_GetADC(&adc_data);

        /*──────────────────────────────────────────────────
         * Step 2: 保护检查前移到 10ms (#10)
         *   ctx 为全局唯一实例 (#11): PID 检测、Monitor(硬件OVP)、UI/Comm 查清故障
         *   都操作同一个 ctx。注意 AppState_Init 已调 Protect_Init, 这里不再重复。
         *
         *   把 Protect_Check 从 500ms(Monitor) 移到 10ms(PID) 的理由:
         *     软件保护响应从最坏 500ms 缩短到 10ms —— 过流/过功率能更快关断,
         *     减小对负载的损害窗口 (硬件 OVP <5μs 仍是第一道防线, 这是第二道)。
         *──────────────────────────────────────────────────*/
        ctx = AppState_GetProtectCtx();
        faults = Protect_Check(ctx, &adc_data);
        if (faults != FAULT_NONE || Protect_HasFault(ctx)) {
            /*
             * 有故障 → 立即安全关断, 不做 PID 输出。
             * prev_enable 置 0: 故障期间视作"已关断", 待故障清除且仍使能时
             * 会被重新识别为使能上升沿 → 从 0 软启动, 避免恢复瞬间过冲。
             */
            pid_safe_shutdown();
            g_prev_enable = 0;
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PID_PERIOD_MS));
            continue;
        }

        /*──────────────────────────────────────────────────
         * Step 3: 读设定 (#12 走 AppState, 一致快照)
         *──────────────────────────────────────────────────*/
        AppState_GetSetting(&setting);

        /*──────────────────────────────────────────────────
         * Step 4: 输出关断处理
         *──────────────────────────────────────────────────*/
        if (!setting.output_enable) {
            pid_safe_shutdown();
            /*
             * 同步"上次设定"记忆, 使:
             *   1) 关断期间用户改设定不会在这里累积成"变化", 重新使能时统一从 0 ramp;
             *   2) prev_enable=0 → 下次使能被正确识别为上升沿。
             */
            g_last_v_set = setting.v_set;
            g_last_i_set = setting.i_set;
            g_prev_enable = 0;
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PID_PERIOD_MS));
            continue;
        }

        /*──────────────────────────────────────────────────
         * Step 5: 使能上升沿检测 → 启动软启动 (从 0 ramp)
         *
         * 关→开的瞬间是"真正的跳变": 此时清积分 (PID_SetSetpoint(0)) 并把
         * ramp 目标从 0 开始。这满足 #8 "真正变化才清积分", 也避免使能瞬间浪涌。
         *──────────────────────────────────────────────────*/
        if (!g_prev_enable) {
            g_ss_state = SS_RAMPING;
            g_ss_v = 0.0f;
            g_ss_i = 0.0f;
            g_dac_safe = 0;   /* 重新使能: 下次关断需重新写 DAC 安全值 */
            PID_SetSetpoint(&g_pid_cv, 0.0f);   /* 设 setpoint=0 且清积分 */
            PID_SetSetpoint(&g_pid_cc, 0.0f);
            g_last_v_set = setting.v_set;
            g_last_i_set = setting.i_set;
        } else {
            /*──────────────────────────────────────────────
             * Step 6: 运行中检测用户改设定 (#8 检测 + #13 重启 ramp)
             *
             * 只有"真的变了"才动作 —— 这正是修掉 #8 的关键 (旧代码无条件动)。
             * 运行中改设定: 不瞬间跳变、不清积分, 而是重启 ramp 从当前 ramp 值
             * 平滑过渡到新目标 (#13)。ramp 期间用 UpdateSetpointNoReset 保留积分。
             *──────────────────────────────────────────────*/
            if (setting_changed(setting.v_set, g_last_v_set)) {
                g_last_v_set = setting.v_set;
                g_ss_state = SS_RAMPING;        /* 电压变化 → 重启 ramp (ss_v 保持当前值) */
            }
            if (setting_changed(setting.i_set, g_last_i_set)) {
                g_last_i_set = setting.i_set;
                g_ss_state = SS_RAMPING;        /* 电流变化 → 重启 ramp */
            }
        }

        /*──────────────────────────────────────────────────
         * Step 7: 软启动状态机推进
         *
         * RAMPING: 每周期把 ss_v/ss_i 朝 last_v_set/last_i_set 迈一步, 并用
         *          UpdateSetpointNoReset 更新 PID setpoint —— 不清积分 (#8)。
         *          两路都到达目标 → 切回 IDLE。
         * IDLE   : 不动 setpoint —— 让积分持续累积消除静差 (#8 稳态)。
         *──────────────────────────────────────────────────*/
        if (g_ss_state == SS_RAMPING) {
            uint8_t v_done = ramp_step(&g_ss_v, g_last_v_set);
            uint8_t i_done = ramp_step(&g_ss_i, g_last_i_set);

            PID_UpdateSetpointNoReset(&g_pid_cv, g_ss_v);   /* 推进但不清积分 */
            PID_UpdateSetpointNoReset(&g_pid_cc, g_ss_i);

            if (v_done && i_done) {
                g_ss_state = SS_IDLE;   /* ramp 完成, 进入稳态 */
            }
        }
        /* else: IDLE —— 稳态, 什么都不做, setpoint 已等于用户目标, 积分持续累积 */

        /*──────────────────────────────────────────────────
         * Step 8: 双 PID 并行计算 + MIN 选择器
         *
         * 两个 PID 都在跑, 输出较小者胜出 → 自动 CV/CC 切换, 无标志位翻转。
         * (原理详见 pid.h; MIN 选择器同时回填 g_current_mode 供 UI 显示。)
         *──────────────────────────────────────────────────*/
        cv_output = PID_Compute(&g_pid_cv, adc_data.v_out);
        cc_output = PID_Compute(&g_pid_cc, adc_data.i_out);
        dac_output = CV_CC_Select(cv_output, cc_output, &g_current_mode);

        /*──────────────────────────────────────────────────
         * Step 9: 映射 PID 输出到 DAC 码值 (v2.0 公式, 沿用未改)
         *──────────────────────────────────────────────────*/

        /* DAC#2 (线性栅极): PID 输出已是 [0,4095] 码值, 直接用, 限幅 3475(≈28V) */
        dac2_code = (uint16_t)dac_output;
        if (dac2_code > DAC2_MAX_CODE) {
            dac2_code = DAC2_MAX_CODE;
        }

        /*
         * DAC#1 (Buck FB): 让 Buck 输出 = 当前软启动目标 + 2V 预稳压。
         * 用软启动目标 g_ss_v (而非实际 V_out 或最终 V_set):
         *   - 用 V_out 会"鸡生蛋"死锁 (V_out 被 Buck 限制, Buck 又按 V_out 算)。
         *   - 用 V_set 会让 Buck 一使能就跳到最终电压, 上电瞬间大电流冲击 → 复位。
         *   g_ss_v 是软启动 ramp 的当前目标 (使能时从 0 缓慢爬升), 让 Buck 跟着
         *   ramp 逐步升压, 既避免死锁 (g_ss_v 领先 V_out), 又避免上电冲击。
         *   V_buck = 32.5 - 10×V_DAC1  → V_DAC1 = (BUCK_VBUCK_MAX - V_buck)/BUCK_DAC_SLOPE
         */
        v_buck_desired = g_ss_v + BUCK_LINEAR_DROP;
        if (v_buck_desired < 3.0f) {
            v_buck_desired = 3.0f;                          /* Buck 最低约需 3V */
        }
        if (v_buck_desired > (V_OUT_MAX + BUCK_LINEAR_DROP)) {
            v_buck_desired = V_OUT_MAX + BUCK_LINEAR_DROP;  /* 上限 28+2=30V */
        }

        v_dac1 = (BUCK_VBUCK_MAX - v_buck_desired) / BUCK_DAC_SLOPE;
        if (v_dac1 < 0.0f)  { v_dac1 = 0.0f; }
        if (v_dac1 > 3.3f)  { v_dac1 = 3.3f; }

        dac1_code = (uint16_t)(v_dac1 / ADC_VREF * (float)ADC_RESOLUTION);
        if (dac1_code < DAC1_MIN_CODE) { dac1_code = DAC1_MIN_CODE; }
        if (dac1_code > DAC1_MAX_CODE) { dac1_code = DAC1_MAX_CODE; }

        /*──────────────────────────────────────────────────
         * Step 10: 写 DAC (先 DAC#1 Buck 粗调, 后 DAC#2 线性精调)
         *   DAC_SetBoth 返回位掩码: bit0=DAC1 失败, bit1=DAC2 失败。
         *   I²C 故障不在此展开处理 —— 记录留给上层, 保护逻辑另有覆盖。
         *──────────────────────────────────────────────────*/
        (void)DAC_SetBoth(dac1_code, dac2_code);

        /* 记录本周期使能态, 供下周期检测使能上升沿 */
        g_prev_enable = 1;

        /*──────────────────────────────────────────────────
         * Step 11: 精确周期延迟到下一个 10ms 边界
         *──────────────────────────────────────────────────*/
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PID_PERIOD_MS));
    }
}

/*===========================================================================
 * 对外接口 (供 UI / 通信任务调用)
 *===========================================================================*/

/**
 * @brief 使能输出
 *
 * output_enable 的唯一真相在 AppState (#12/一致性要求): 这里读改写 AppState,
 * 不直接改软启动内部状态 —— 软启动由 vTaskPID 检测"使能上升沿"后自行触发,
 * 避免 UI/Comm 任务与 PID 任务并发写软启动状态导致竞态。
 */
void TaskPID_EnableOutput(void)
{
    SystemSetting_t s;
    AppState_GetSetting(&s);
    s.output_enable = 1;
    AppState_SetSetting(&s);
    HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);   /* 硬件使能输出 */
}

/**
 * @brief 关断输出 (经 AppState)
 */
void TaskPID_DisableOutput(void)
{
    SystemSetting_t s;
    AppState_GetSetting(&s);
    s.output_enable = 0;
    AppState_SetSetting(&s);
    HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_RESET);  /* 硬件关断输出 */
}

/** @brief 在线更新 CV PID 参数 (Comm 调参) */
void TaskPID_UpdateCVParams(float kp, float ki, float kd)
{
    PID_UpdateParams(&g_pid_cv, kp, ki, kd);
}

/** @brief 在线更新 CC PID 参数 (Comm 调参) */
void TaskPID_UpdateCCParams(float kp, float ki, float kd)
{
    PID_UpdateParams(&g_pid_cc, kp, ki, kd);
}
