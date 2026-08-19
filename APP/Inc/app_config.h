/**
 * app_config.h
 * 迷你数控电源 - 全局配置宏定义 (v2.0 Boost+Buck+线性架构)
 * 本文件包含所有可调参数：PID 默认值、校准系数、保护阈值、
 * DAC 限幅值、预设电压值。所有参数集中管理，方便调试和在线修改。
 *   - V_out 上限 30V → 28V, OVP 软件阈值 30.5V → 28.5V
 *   - V_IN 监测点改为 Boost 前, 分压比 1/12 → 1/7.25
 *   - DAC#1 映射更新: V_buck = 32.5 - 10×V_DAC1 (受 33V Boost 输入限制)
 *   - DAC#2 限幅: 3636 → 3475 (对应 V_out 上限 28V)
 *   - PB4 新增 Boost EN 控制 (GPIO Output, 高=使能)
 */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*===========================================================================
 * 1. 系统基础参数
 *===========================================================================*/

/* MCU 主频 */
#define SYSTEM_CLOCK_HZ         72000000UL      /* 72MHz (8MHz HSE ×9 PLL) */

/* ADC 参考电压 */
#define ADC_VREF                3.30f           /* VDDA = 3.3V (经过 LC 滤波) */
#define ADC_RESOLUTION          4096            /* 12-bit ADC */

/*===========================================================================
 * 2. PID 控制参数
 *===========================================================================*/

/*
 * PID 周期 = 10ms (vTaskPID 任务周期)
 *
 * 调参原则（面试可能问）：
 *   先 Kp 后 Ki 最后 Kd
 *   Kp：增大 → 响应快但可能超调/振荡
 *   Ki：消除稳态误差，但太大会引起积分饱和
 *   Kd：抑制超调，但对噪声敏感（此处 Kd=0 是因为电源负载通常变化不快）
 */

/* --- 恒压环 (CV) PID 参数 --- */
#define PID_CV_KP_DEFAULT       109.0f          /* 比例增益 (码值/V, ≈1/植物增益0.00806) */
#define PID_CV_KI_DEFAULT       4.5f            /* 积分增益 (码值/V/周期) */
#define PID_CV_KD_DEFAULT       0.00f           /* 微分增益 (电源控制通常不用 D) */

/* --- 恒流环 (CC) PID 参数 --- */
#define PID_CC_KP_DEFAULT       120.0f          /* 比例增益 (起始值, 需带载标定) */
#define PID_CC_KI_DEFAULT       50.0f           /* 积分增益: 非限流时快速饱和(≈0.1s)让CV环胜出 */
#define PID_CC_KD_DEFAULT       0.00f           /* 微分增益 */

/*
 * 积分分离阈值：误差超过此值时关闭积分项，防止大偏差下积分累积过多
 * 导致"积分饱和"(integral windup)，表现为超调严重、恢复缓慢。
 */
#define PID_CV_INTEGRAL_SEPARATION_THRESHOLD   1000.0f /* 已禁用(原3V): MIN选择器需要非限流环积分饱和 */
#define PID_CC_INTEGRAL_SEPARATION_THRESHOLD   1000.0f /* 已禁用(原0.5A): 同上, 分离会锁死CC环致误切CC */

/* PID 输出限幅 (DAC 12-bit 范围) */
#define PID_OUTPUT_MIN          0.0f            /* 最小输出 = DAC 码值 0 */
#define PID_OUTPUT_MAX          4095.0f         /* 最大输出 = DAC 码值 4095 */

/*===========================================================================
 * 3. DAC 配置
 *===========================================================================*/

/* MCP4725 I²C 地址 (7-bit) */
#define DAC1_ADDR               0x60            /* Buck FB 控制 (A0 = GND) */
#define DAC2_ADDR               0x61            /* 线性栅极控制 (A0 = VCC) */

/*
 * DAC#2 软件限幅
 *
 * P-MOS 方案下 V_out = V_DAC2 × 10 (LM358 闭环跟随, 线性正比)
 *   V_out 上限 = 28V  → V_DAC2 = 2.80V
 *   对应 DAC 码值 = 2.80V / 3.30V × 4096 ≈ 3477
 *   考虑 ADC 采样误差和过冲余量，限幅至 3475
 *
 *   [DAC2_MAX_CODE=3636, 对应 V_out≈30V]
 */
#define DAC2_MAX_CODE           3475            /* 对应 V_out ≈ 28V */

/* DAC#1 限幅 (Buck FB 控制，反比关系，0 = 最大输出) */
#define DAC1_MIN_CODE           100             /* 最小值不为 0，避免 Buck 输出过高 */
#define DAC1_MAX_CODE           4000            /* 最大值限制，避免 Buck 完全关断 */

/*===========================================================================
 * 3.5 v2.0 电压映射常量 (Boost 前级 + Buck + 线性 三级架构)
 *===========================================================================*/

#define BUCK_VIN_FIXED          33.0f           /* Buck 输入固定 33V (Boost 输出) */

/*
 * V_buck 最大值 (DAC#1=0 时的理论 V_buck)
 * 公式: V_buck = 32.5 - 10×V_DAC1, 当 V_DAC=0 时 V_buck≈32.5V
 * 实际受 33V 输入限制, V_buck 最高约 32V
 */
#define BUCK_VBUCK_MAX          32.5f           /* V_buck 最大 (V), 受 33V Boost 输入限制 */

/*
 * DAC#1 映射斜率: V_buck 每 V_DAC1 变化量
 * V_buck = BUCK_VBUCK_MAX - BUCK_DAC_SLOPE × V_DAC1
 * 斜率由 R_top/R_inj 电阻比决定 = 16.5k/1.65k = 10
 */
#define BUCK_DAC_SLOPE          10.0f           /* V_buck 对 V_DAC1 斜率 (反比) */

/*
 * 线性级最小压降 (P-MOS IRF9540 + NPN 电平转换)
 * Buck 输出需至少比目标 V_out 高此值, 使 P-MOS 不完全导通, 有调节空间
 */
#define BUCK_LINEAR_DROP        2.0f            /* 线性级最小压降 (V) */

#define V_OUT_MAX               28.0f           /* V_out 上限 (V) */

/*===========================================================================
 * 4. 保护阈值
 *===========================================================================*/

#define OVP_THRESHOLD_VOLTAGE   28.5f           /* 过压阈值 (V) */

/*
 * 过流保护 (OCP) — 软件阈值
 * 额定最大 3A，软件阈值设 3.2A 留余量
 */
#define OCP_THRESHOLD_CURRENT   3.2f            /* 过流阈值 (A) */

/*
 * 过功率保护 (OPP)
 * 最大输出功率 = 28V × 3A = 84W，阈值设 88W 留余量
 */
#define OPP_THRESHOLD_POWER     88.0f           /* 过功率阈值 (W) */

/*
 * 过温保护 (OTP)
 * NTC 检测散热片温度，> 80°C 触发保护
 */
#define OTP_THRESHOLD_TEMP      80.0f           /* 过温阈值 (摄氏度) */

/*===========================================================================
 * 5. 校准系数 (默认值，可通过上位机修改后存入 Flash)
 *===========================================================================*/

/*
 * 电压校准：V_actual = V_CAL_SLOPE × V_measured + V_CAL_OFFSET
 * 默认 slope=1.0, offset=0.0（未校准）
 * 实际使用前用万用表测量后计算填入
 */
#define V_CAL_SLOPE_DEFAULT     1.096f
#define V_CAL_OFFSET_DEFAULT    0.00f

/*
 * 电流校准：I_actual = I_CAL_SLOPE × I_measured + I_CAL_OFFSET
 * 默认 slope=1.0, offset=0.0（未校准）
 */
#define I_CAL_SLOPE_DEFAULT     1.00f
#define I_CAL_OFFSET_DEFAULT    0.00f

/*===========================================================================
 * 6. ADC 采样参数
 *===========================================================================*/

/*
 * 采样时间必须 ≥ 71.5 cycles
 * 原因：PA0/PA2 的等效源阻抗约 9kΩ（分压电阻 Thevenin 等效）
 *      需要长采样时间让内部采样电容充分充电
 *
 * 硬件假设来源：docs/04-hardware-interface.md 第 2.1/2.3 节
 */
#define ADC_SAMPLE_TIME         ADC_SAMPLETIME_71CYCLES_5

/*
 * DMA 双缓冲大小：每个通道采集 ADC_BUFFER_SIZE 个样本后触发处理
 * 4 通道 × 10 样本 × 2 缓冲区 = 80 个 uint16_t
 */
#define ADC_BUFFER_SIZE         10              /* 每个通道每次缓冲的样本数 */
#define ADC_CHANNEL_COUNT       4               /* V_OUT, I_OUT, V_IN, TEMP */
#define ADC_DMA_BUFFER_SIZE     (ADC_CHANNEL_COUNT * ADC_BUFFER_SIZE)

/*
 * 滤波配置：中值滤波窗口 + 均值滤波点数
 */
#define ADC_MEDIAN_WINDOW       5               /* 中值滤波窗口大小 (奇数) */
#define ADC_AVERAGE_COUNT       10              /* 均值滤波点数 */

/*
 * V_IN_SENSE 分压比 [v2.0: 从 1/12 改为 1/7.25]
 * 分压网络: R_top=75k, R_bot=12k → 分压比 = 12/(75+12) = 1/7.25
 * V_in = ADC × 3.3 / 4096 × 7.25
 * 硬件假设来源：docs/03-interface-definition.md 第 2.4 节 (v2.0)
 *              hardware/schematic.md 模块 ② (v2.0)
 */
#define V_IN_DIVIDER_RATIO      7.25f           /* V_in 分压比倒数 (75k+12k → 1/7.25) */

/*
 * V_OUT_SENSE 分压比
 * 来源：问题 #26 —— 原来该系数 10 硬编码在 adc.c 里，现集中到此处统一管理，
 *       避免魔法数字散落各处（与本文件"配置集中管理"原则一致）。
 * 分压网络对 V_out 采样: 分压比 = 1/10，故还原公式:
 *   V_out = ADC × 3.3 / 4096 × 10.0
 */
#define V_OUT_DIVIDER_RATIO     10.0f           /* V_out 分压比倒数 (实测标定: 端子≈1.11×SET_V, 原10偏小) */

/*===========================================================================
 * 7. 通信协议参数
 *===========================================================================*/

#define UART_BAUDRATE           115200          /* 波特率 */
#define RING_BUFFER_SIZE        256             /* 环形缓冲区大小 (字节) */
#define RX_FRAME_TIMEOUT_MS     50              /* 帧接收超时 (ms) */

/* 帧格式定义 */
#define FRAME_HEADER            0xA5            /* 帧头 */
#define FRAME_FOOTER            0x5A            /* 帧尾 */
#define FRAME_MAX_DATA_LEN      64              /* 单帧最大数据长度 */

/*===========================================================================
 * 8. 预设电压值 (单位: V)
 *
 *   为什么用 #define 而不是 const？
 *   答：#define 在预处理阶段直接替换，不占 RAM。对于 F103C8T6
 *   只有 20KB SRAM 的 MCU，常量放 Flash(.rodata) 或直接宏定义
 *   都比放 RAM 好。如果预设值需要在线修改并存 Flash，
 *   则应该用变量存于 Flash 模拟 EEPROM 中。
 *===========================================================================*/

#define PRESET_VOLTAGE_1        3.3f            /* 预设 3.3V */
#define PRESET_VOLTAGE_2        5.0f            /* 预设 5V */
#define PRESET_VOLTAGE_3        12.0f           /* 预设 12V */
#define PRESET_VOLTAGE_4        24.0f           /* 预设 24V */

/*===========================================================================
 * 9. 编码器参数
 *===========================================================================*/

#define ENCODER_PULSE_PER_REV   20              /* EC11 每圈脉冲数 */
#define ENCODER_ACCEL_THRESHOLD 50              /* 加速阈值 (脉冲/秒) */
#define ENCODER_ACCEL_MULTIPLIER 4              /* 加速倍数 */
#define ENCODER_LONG_PRESS_MS   1000            /* 长按时间阈值 (ms) */
#define ENCODER_DOUBLE_CLICK_MS 300             /* 双击间隔阈值 (ms) */
#define BUTTON_DEBOUNCE_MS      20              /* 按键消抖时间 (ms) */

/*===========================================================================
 * 10. 软启动参数
 *===========================================================================*/

/*
 * 软启动：输出使能后，设定值从 0 缓慢 ramp 到目标值
 * 避免 Buck 启动浪涌和输出电压过冲
 */
#define SOFT_START_STEP         0.05f           /* 每 10ms 步进量 (V 或 A) */
#define SOFT_START_PERIOD_MS    10              /* 步进周期 (ms) */

/*===========================================================================
 * 11. 保护恢复策略
 *===========================================================================*/

/*
 * 故障锁存：保护触发后锁存故障状态，需要用户手动确认清除
 * 不允许自动恢复 —— 安全第一
 */
#define FAULT_RETRY_COUNT_MAX   3               /* 最大重试次数 */
#define FAULT_CLEAR_TIMEOUT_MS  5000            /* 故障自动清除超时 (ms) */

/*===========================================================================
 * 12. 任务调度参数
 *===========================================================================*/

/* 任务周期 */
#define TASK_ADC_PERIOD_MS      1               /* ADC 采样任务周期 */
#define TASK_PID_PERIOD_MS      10              /* PID 控制任务周期 */
#define TASK_UI_PERIOD_MS       400             /* UI 刷新任务周期 */
#define TASK_MONITOR_PERIOD_MS  500             /* 监控任务周期 */

/* 任务优先级 (数字越大优先级越高, FreeRTOS 中 0~7) */
#define PRIO_ADC_TASK           4               /* ADC 采样 - 高频高优先级 */
#define PRIO_PID_TASK           3               /* PID 控制 - 高优先级 */
#define PRIO_UI_TASK            2               /* UI 刷新 - 普通优先级 */
#define PRIO_COMM_TASK          1               /* 通信处理 - 普通优先级 */
#define PRIO_MONITOR_TASK       0               /* 监控 - 最低优先级 (Idle) */

/* 任务栈大小 (单位: word, 1 word = 4 bytes on Cortex-M3) */
#define STACK_ADC_TASK          256
#define STACK_PID_TASK          512             /* PID 浮点运算需要较大栈 */
#define STACK_UI_TASK           512             /* TFT 操作需要较大栈 */
#define STACK_COMM_TASK         384
#define STACK_MONITOR_TASK      256

/* 队列长度 */
#define QUEUE_ADC_DATA_LEN      16              /* ADC 数据队列 */
#define QUEUE_COMMAND_LEN       8               /* 通信命令队列 */
#define QUEUE_UI_EVENT_LEN      16              /* UI 事件队列 */

/*===========================================================================
 * 13. 调试开关
 *===========================================================================*/

/* 设为 1 启用串口调试输出（printf 重定向到 USART1） */
#define DEBUG_PRINTF_ENABLE     0

/* 设为 1 启用 PID 调试输出（每次 PID 计算后输出中间变量） */
#define PID_DEBUG_ENABLE        0

/*===========================================================================
 * 14. 类型定义与枚举
 *===========================================================================*/

/* 工作模式 */
typedef enum {
    MODE_CV = 0,            /* 恒压模式 (Constant Voltage) */
    MODE_CC = 1             /* 恒流模式 (Constant Current) */
} OperatingMode_t;

/* 故障类型 (位掩码，可同时存在多个故障) */
typedef enum {
    FAULT_NONE          = 0x00,
    FAULT_OVP           = 0x01,     /* 过压保护 */
    FAULT_OCP           = 0x02,     /* 过流保护 */
    FAULT_OPP           = 0x04,     /* 过功率保护 */
    FAULT_OTP           = 0x08,     /* 过温保护 */
    FAULT_SHORT         = 0x10,     /* 短路保护 (电流 > 5A) */
    FAULT_OVP_HARDWARE  = 0x20,     /* 硬件 OVP 触发 (LM393) */
    FAULT_I2C_ERROR     = 0x40,     /* I²C 通信故障 */
    FAULT_ADC_ERROR     = 0x80      /* ADC 异常 (长时间无数据) */
} FaultFlag_t;

/* 编码器事件类型 */
typedef enum {
    ENC_EVENT_NONE      = 0,
    ENC_EVENT_CW        = 1,        /* 顺时针旋转 */
    ENC_EVENT_CCW       = 2,        /* 逆时针旋转 */
    ENC_EVENT_SHORT_PRESS  = 3,     /* 短按 */
    ENC_EVENT_LONG_PRESS    = 4,     /* 长按 */
    ENC_EVENT_DOUBLE_CLICK  = 5      /* 双击 */
} EncoderEvent_t;

/* UI 调节目标 */
typedef enum {
    ADJ_VOLTAGE = 0,        /* 正在调节电压 */
    ADJ_CURRENT = 1         /* 正在调节电流 */
} AdjustTarget_t;

/*===========================================================================
 * 15. 系统状态结构体
 *===========================================================================*/

/**
 * @brief ADC 采样数据（滤波后）
 *
 * 由 vTaskADC 产生，通过队列发送给 vTaskPID 和 vTaskMonitor
 */
typedef struct {
    float v_out;            /* 输出电压 (V)，已校准 */
    float i_out;            /* 输出电流 (A)，已校准 */
    float v_in;             /* 输入电压 (V) */
    float temperature;      /* 散热片温度 (摄氏度) */
    uint32_t timestamp_ms;  /* 采样时间戳 (系统毫秒) */
} ADCData_t;

/**
 * @brief 系统设定值
 *
 * 由 vTaskUI 或 vTaskComm 修改，被 vTaskPID 读取
 */
typedef struct {
    float v_set;            /* 设定电压 (V) */
    float i_set;            /* 设定电流 (A) */
    uint8_t output_enable;  /* 输出使能标志：0=关断, 1=使能 */
} SystemSetting_t;

/**
 * @brief PID 参数 (可在线修改)
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;   /* 积分项限幅值 (基于 MIN 选择器输出上限) */
} PIDParams_t;

/**
 * @brief 校准参数
 */
typedef struct {
    float v_slope;
    float v_offset;
    float i_slope;
    float i_offset;
} Calibration_t;

/**
 * @brief 系统全局状态
 *
 * 各任务通过此结构体共享系统状态（需配合互斥锁或临界区保护）
 */
typedef struct {
    ADCData_t       adc_data;           /* 最新 ADC 采样数据 */
    SystemSetting_t setting;            /* 当前设定值 */
    OperatingMode_t mode;               /* 当前工作模式 (CV/CC) */
    FaultFlag_t     fault_flags;        /* 当前故障标志 */
    float           power_out;          /* 当前输出功率 (W) = V_out × I_out */
    uint32_t        sys_tick_ms;        /* 系统运行时间 (ms) */
    uint8_t         output_enabled;     /* 输出当前是否使能 */
    uint8_t         soft_start_active;  /* 软启动进行中标志 */
} SystemState_t;

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H */
