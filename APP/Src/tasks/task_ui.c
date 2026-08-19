/**
 * task_ui.c
 * UI 管理任务实现 (v2.0, 本批次修正)
 * 人机交互任务：TFT 显示 + 编码器 (旋转/按键分离) + 预设按键。
 * 周期：100ms (10Hz, 人眼舒适)   优先级：2 (Normal)
 *
 * 为什么 UI 优先级低于 PID？
 *   UI 的 100ms 周期远慢于 PID 的 10ms。若 UI 高于 PID, UI 刷 TFT 时会阻塞
 *   PID 计算 → 电压波动。UI 必须给控制环路让路。
 */

#include "task_ui.h"
#include "task_pid.h"
#include "tft_st7735.h"
#include "encoder.h"
#include "protect.h"
#include "eeprom_emulate.h"
#include "app_state.h"        /* #11/#12: 统一状态入口 */
#include "app_config.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/*===========================================================================
 * 模块级变量
 *===========================================================================*/

QueueHandle_t g_ui_event_queue = NULL;
static EncoderContext_t g_encoder;

/* 当前旋钮调节目标: 电压 or 电流 (短按切换) */
static uint8_t g_encoder_adjust_target = ADJ_VOLTAGE;

/* UI 层设定步进 (每个脉冲对应的物理增量) */
#define UI_STEP_V_FINE      0.01f       /* 精细: 1 脉冲 = 0.01V */
#define UI_STEP_I_FINE      0.001f      /* 精细: 1 脉冲 = 0.001A */

/*===========================================================================
 * 任务主循环
 *===========================================================================*/

void vTaskUI(void *argument)
{
    ADCData_t         adc_data;
    SystemSetting_t   setting;          /* #12: 本地副本, 读改写后 Set 回 AppState */
    ProtectContext_t *ctx;              /* #11: 全局唯一保护上下文 */
    TickType_t        xLastWakeTime;
    EncoderEvent_t    enc_event;
    int32_t           encoder_step;
    uint8_t           fault;
    float             power;
    uint8_t           setting_dirty;    /* 本周期设定是否被改动, 减少无谓 Set */

    (void)argument;

    /*调度器启动后懒创建互斥锁 (必须最先, 在任何 AppState 访问之前) */
    AppState_EnsureMutex();

    /* UI 事件队列 (保留供未来扩展; 当前编码器事件直接消费, 不入此队列) */
    g_ui_event_queue = xQueueCreate(QUEUE_UI_EVENT_LEN, sizeof(EncoderEvent_t));
    if (g_ui_event_queue == NULL) {
        while (1) { /* 队列创建失败: 停机 */ }
    }

    /* 初始化编码器软件状态 (GPIO/EXTI 在 main.c 配置) */
    Encoder_Init(&g_encoder);
		
    /* 启动画面 (main 已完成 TFT 硬件初始化与 InitSequence) */
    TFT_Backlight(1);
    TFT_ShowSplashScreen();
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* 清屏一次，擦掉启动画面的旧像素 (主界面按增量刷新，不再全屏清) */
    TFT_FillScreen(TFT_COLOR_BLACK);

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        /*──────────────────────────────────────────────────
         * Step 1: 读最新实测值 (#12 走 AppState) + 当前设定快照
         *──────────────────────────────────────────────────*/
        AppState_GetADC(&adc_data);
        AppState_GetSetting(&setting);
        ctx = AppState_GetProtectCtx();
        setting_dirty = 0;

        /*──────────────────────────────────────────────────
         * Step 2: 编码器处理 —— 旋转与按键严格分离 (#14/#15)
         *
         * 【消费顺序规避 encoder.c 事件覆盖 bug —— 见摘要"是否需改 encoder"结论】
         *   encoder.c 的 Encoder_Process() 末尾: 若 pulse_count!=0 会把 pending_event
         *   覆写成 CW/CCW, 从而冲掉同一次 Process 里刚判定出的按键事件 (SHORT/LONG/
         *   DOUBLE)。这是 encoder.c 用"单个 pending_event 槽位混装旋转+按键"的设计缺陷。
         *
         *   本 UI 层的规避手法 (不改 encoder.c 也能可靠工作):
         *     (a) 先 Encoder_GetStep() —— 它直接读并清零 pulse_count, 与 pending_event
         *         完全独立, 旋转永不丢。
         *     (b) 再 Encoder_Process() —— 此刻 pulse_count 已被清零, Process 末尾的
         *         "pulse_count!=0 → 覆写 CW/CCW"分支不会触发, 按键事件得以保留。
         *     (c) 最后 Encoder_GetEvent() —— 取到的必是纯按键事件 (SHORT/LONG/DOUBLE),
         *         不会被旋转污染。
         *   旋转方向由 GetStep 的正负号表达, 我们本就不依赖 pending_event 里的 CW/CCW,
         *   因此丢弃 CW/CCW 语义无损失。
         *──────────────────────────────────────────────────*/

        /* (a) 旋转: 先取步进 (读并清零 pulse_count) */
        encoder_step = Encoder_GetStep(&g_encoder);
        if (encoder_step != 0) {
            /* 加速判定: |step| 超阈值 → 步进×加速倍数 (Encoder_GetStep 已含加速, 这里映射物理量) */
            if (g_encoder_adjust_target == ADJ_VOLTAGE) {
                float step_v = (float)encoder_step * UI_STEP_V_FINE;
                setting.v_set += step_v;
                if (setting.v_set < 0.0f)       { setting.v_set = 0.0f; }
                if (setting.v_set > V_OUT_MAX)  { setting.v_set = V_OUT_MAX; }  /* 上限 28V (#与硬件一致) */
                setting_dirty = 1;
            } else {
                float step_i = (float)encoder_step * UI_STEP_I_FINE;
                setting.i_set += step_i;
                if (setting.i_set < 0.0f)                 { setting.i_set = 0.0f; }
                if (setting.i_set > OCP_THRESHOLD_CURRENT) { setting.i_set = OCP_THRESHOLD_CURRENT; }
                setting_dirty = 1;
            }
        }

        /* (b) 推进编码器状态机 (此时 pulse_count 已清零, 不会覆写按键事件) */
        Encoder_Process(&g_encoder);

        /* (c) 按键: 取纯按键事件 */
        enc_event = Encoder_GetEvent(&g_encoder);
        switch (enc_event) {
        case ENC_EVENT_SHORT_PRESS:
            /* 短按: 切换调节目标 (电压 ↔ 电流) */
            g_encoder_adjust_target = (g_encoder_adjust_target == ADJ_VOLTAGE)
                                      ? (uint8_t)ADJ_CURRENT : (uint8_t)ADJ_VOLTAGE;
            break;

        case ENC_EVENT_LONG_PRESS:
            /*
             * 长按: 使能/关断输出。
             * output_enable 唯一真相在 AppState —— 用 TaskPID_EnableOutput/DisableOutput
             * (它们内部经 AppState 改状态, 软启动由 PID 任务按使能上升沿自动触发)。
             *
             * 关断输出时自动保存当前设定值到 Flash (EEPROM_SaveConfig)。
             * 这样下次上电恢复上次的电压/电流设定。
             */
            if (setting.output_enable) {
                TaskPID_DisableOutput();

                /* --- 关断时保存配置到 Flash --- */
                {
                    EEPROM_Config_t cfg;
                    Calibration_t   cal;
                    SystemSetting_t latest;

                    AppState_GetSetting(&latest);
                    AppState_GetCalibration(&cal);

                    memset(&cfg, 0, sizeof(EEPROM_Config_t));
                    cfg.v_set = latest.v_set;
                    cfg.i_set = latest.i_set;
                    cfg.output_enable = 0;

                    cfg.v_cal_slope  = cal.v_slope;
                    cfg.v_cal_offset = cal.v_offset;
                    cfg.i_cal_slope  = cal.i_slope;
                    cfg.i_cal_offset = cal.i_offset;

                    cfg.preset_voltages[0] = PRESET_VOLTAGE_1;
                    cfg.preset_voltages[1] = PRESET_VOLTAGE_2;
                    cfg.preset_voltages[2] = PRESET_VOLTAGE_3;
                    cfg.preset_voltages[3] = PRESET_VOLTAGE_4;

                    cfg.pid_cv_kp = PID_CV_KP_DEFAULT;
                    cfg.pid_cv_ki = PID_CV_KI_DEFAULT;
                    cfg.pid_cv_kd = PID_CV_KD_DEFAULT;
                    cfg.pid_cc_kp = PID_CC_KP_DEFAULT;
                    cfg.pid_cc_ki = PID_CC_KI_DEFAULT;
                    cfg.pid_cc_kd = PID_CC_KD_DEFAULT;

                    EEPROM_SaveConfig(&cfg);
                }
            } else {
                /* 有未清除故障时禁止使能 (安全第一) —— 查全局唯一 ctx (#11) */
                if (!Protect_HasFault(ctx)) {
                    TaskPID_EnableOutput();
                }
            }
            /*
             * 注意: 此处不再手动改本地 setting.output_enable, 也不 Set 回去 ——
             * 使能态已由 TaskPID_Enable/DisableOutput 写入 AppState, 下一周期
             * AppState_GetSetting 会读到最新值, 避免与 PID 侧写冲突 (#12 一致性)。
             */
            break;

        case ENC_EVENT_DOUBLE_CLICK:
            /* 双击: 清除故障 (查/清都用全局唯一 ctx, #11) */
            if (Protect_HasFault(ctx)) {
                Protect_ClearFaults(ctx);
            }
            break;

        default:
            break;
        }

        /*──────────────────────────────────────────────────
         * Step 3: 预设按键扫描 K1~K4 (PB12~PB15, 按下=低)
         *   命中则改本地 setting.v_set 并标脏, 统一在 Step 4 一次性 Set 回 (#12)。
         *──────────────────────────────────────────────────*/
        {
            static uint32_t last_key_tick = 0;
            uint32_t now = HAL_GetTick();

            if ((now - last_key_tick) > BUTTON_DEBOUNCE_MS) {
                if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
                    setting.v_set = PRESET_VOLTAGE_1;  setting_dirty = 1;  last_key_tick = now;
                } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) {
                    setting.v_set = PRESET_VOLTAGE_2;  setting_dirty = 1;  last_key_tick = now;
                } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) {
                    setting.v_set = PRESET_VOLTAGE_3;  setting_dirty = 1;  last_key_tick = now;
                } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) {
                    setting.v_set = PRESET_VOLTAGE_4;  setting_dirty = 1;  last_key_tick = now;
                }
            }
        }

        /*──────────────────────────────────────────────────
         * Step 4: 若本周期改了 v_set/i_set, 一次性写回 AppState (#12)
         *
         * 读改写模式: 前面对本地副本 setting 的所有修改, 到这里统一 Set。
         * 注意本地副本的 output_enable 可能已过期 (长按已直接经 AppState 改)。
         * 为避免用过期 output_enable 覆盖 PID 侧的新值, 这里重新 Get 一次拿最新
         * 使能态, 只把 v_set/i_set 合并进去再 Set —— 精确写, 不误伤使能位。
         *──────────────────────────────────────────────────*/
        if (setting_dirty) {
            SystemSetting_t latest;
            AppState_GetSetting(&latest);       /* 拿最新使能态 */
            latest.v_set = setting.v_set;        /* 只覆盖被 UI 改动的字段 */
            latest.i_set = setting.i_set;
            AppState_SetSetting(&latest);
            setting = latest;                    /* 本地副本同步, 供下面显示 */
        }

        /*──────────────────────────────────────────────────
         * Step 5: 计算功率 + 故障标志, 刷新 TFT
         *
         * v2.1: 有故障时覆盖全屏故障详情 (TFT_ShowFault),
         *       无故障时正常显示主界面 (TFT_DrawMainScreen)。
         *──────────────────────────────────────────────────*/
        power = adc_data.v_out * adc_data.i_out;
        fault = Protect_HasFault(ctx) ? 1U : 0U;

        if (fault) {
            /* 全屏故障详情: 逐条列出 OVP/OCP/OPP/OTP/SHORT/OVP-HW */
            TFT_ShowFault((uint8_t)ctx->active_faults);
        } else {
            TFT_DrawMainScreen(
                setting.v_set,
                adc_data.v_out,
                setting.i_set,
                adc_data.i_out,
                power,
					      (uint8_t)TaskPID_GetMode(),
                setting.output_enable,
                fault
            );
        }
        
        /*──────────────────────────────────────────────────
         * Step 6: 精确周期延迟
         *──────────────────────────────────────────────────*/
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_UI_PERIOD_MS));
    }
}
