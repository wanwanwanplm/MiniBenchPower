/**
 * task_comm.c
 * 通信任务实现 (v2.0, 本批次修正)
 *
 * 处理上位机通信：帧解析 → 命令派遣 → 响应发送。
 *
 * 数据流：
 *   USART1 RX ISR → 环形缓冲区 → vTaskComm 轮询解析帧
 *   → 命令处理 (读写 AppState) → 响应帧 → USART1 TX DMA (判 BUSY 重试)
 */

#include "task_comm.h"
#include "task_pid.h"
#include "protocol.h"
#include "app_state.h"        /* #12: 设定/ADC 统一入口 */
#include "eeprom_emulate.h"
#include "app_config.h"
#include "pid.h"              /* PID_GetState 声明 */
#include "main.h"             /* huart1 单一来源 */
#include "stm32f1xx_hal.h"
#include <string.h>

/*===========================================================================
 * 常量
 *===========================================================================*/

#define COMM_IDLE_POLL_MS       10      /* #33: 空闲轮询周期 (RX 缓冲为空时休眠) */
#define COMM_TX_RETRY_MAX       3       /* #19: TX DMA BUSY 最大重试次数 */
#define COMM_TX_RETRY_DELAY_MS  2       /* #19: 每次重试间的小延迟 */

/*===========================================================================
 * 全局/模块级变量
 *===========================================================================*/

RingBuffer_t g_uart_rx_ring;            /* UART RX 环形缓冲区 (供 ISR extern 引用) */
static QueueHandle_t g_cmd_queue = NULL;/* 命令队列 (保留, 供未来 ISR→任务扩展) */

/*===========================================================================
 * 命令处理器声明
 *===========================================================================*/

static void CmdHandler_ReadData(const CommFrame_t *req, CommFrame_t *resp);
static void CmdHandler_SetVI(const CommFrame_t *req, CommFrame_t *resp);
static void CmdHandler_SetPID(const CommFrame_t *req, CommFrame_t *resp);
static void CmdHandler_Preset(const CommFrame_t *req, CommFrame_t *resp);
static void CmdHandler_OutputCtrl(const CommFrame_t *req, CommFrame_t *resp);
static void CmdHandler_SaveConfig(const CommFrame_t *req, CommFrame_t *resp);
static void CmdHandler_FactoryReset(const CommFrame_t *req, CommFrame_t *resp);

static CmdRegistry_t cmd_registry[] = {
    {CMD_READ_DATA,     CmdHandler_ReadData},
    {CMD_SET_V_I,       CmdHandler_SetVI},
    {CMD_SET_PID,       CmdHandler_SetPID},
    {CMD_PRESET,        CmdHandler_Preset},
    {CMD_OUTPUT_CTRL,   CmdHandler_OutputCtrl},
    {CMD_SAVE_CONFIG,   CmdHandler_SaveConfig},
    {CMD_FACTORY_RESET, CmdHandler_FactoryReset},
};


/*===========================================================================
 * TX 辅助 (#19)
 *===========================================================================*/

/**
 * @brief 经 USART1 TX DMA 发送, 判 BUSY 有限次重试
 *
 * 为什么必须判 BUSY？
 *   上一帧的 TX DMA 可能仍在进行 (发送慢于产生), 此时 HAL_UART_Transmit_DMA
 *   返回 HAL_BUSY。旧代码"无脑发"→ 直接丢弃本帧且不自知。这里改为:
 *   BUSY → 短延迟后重试, 最多 COMM_TX_RETRY_MAX 次; 仍忙则放弃本帧 (不阻塞任务)。
 * @return 1=已提交发送, 0=多次重试仍忙, 放弃
 */
static uint8_t comm_tx_dma(const uint8_t *buf, uint16_t len)
{
    uint8_t retry;
    for (retry = 0; retry < COMM_TX_RETRY_MAX; retry++) {
        HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len);
        if (st == HAL_OK) {
            return 1U;                  /* 成功提交给 DMA */
        }
        if (st != HAL_BUSY) {
            return 0U;                  /* 非 BUSY 的真错误, 不重试 */
        }
        /* BUSY: 让出一小段时间等上一帧发完再试 */
        vTaskDelay(pdMS_TO_TICKS(COMM_TX_RETRY_DELAY_MS));
    }
    return 0U;                          /* 重试耗尽仍忙 → 放弃本帧 */
}

/*===========================================================================
 * 任务主循环 (#33: 轮询触发型)
 *===========================================================================*/

void vTaskComm(void *argument)
{
    CommFrame_t request;
    CommFrame_t response;
    uint8_t     tx_buffer[FRAME_MAX_DATA_LEN + 7];  /* SOF+cmd+len+data+CRC(2)+EOF */
    uint16_t    tx_len;

    (void)argument;

    RingBuffer_Init(&g_uart_rx_ring);

    /* 使能 USART1 RXNE 中断 (ISR 里把字节写入环形缓冲区) */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    g_cmd_queue = xQueueCreate(QUEUE_COMMAND_LEN, sizeof(CommFrame_t));
    if (g_cmd_queue == NULL) {
        while (1) { /* 队列创建失败: 停机 */ }
    }

    Protocol_RegisterHandlers(cmd_registry,
                              (uint8_t)(sizeof(cmd_registry) / sizeof(CmdRegistry_t)));

    for (;;) {
        if (RingBuffer_Available(&g_uart_rx_ring) > 0U) {
            /* 有数据: 尝试解析一帧 */
            if (Protocol_ParseFrame(&g_uart_rx_ring, &request)) {
                memset(&response, 0, sizeof(CommFrame_t));

                if (Protocol_Dispatch(&request, &response)) {
                    /* 命令已由处理器填充 response */
                    tx_len = Protocol_BuildResponse(&response, tx_buffer,
                                                     (uint16_t)sizeof(tx_buffer));
                    if (tx_len > 0U) {
                        (void)comm_tx_dma(tx_buffer, tx_len);   /* #19 判 BUSY 重试 */
                    }
                }
            }
            /* 解析后立即回到循环顶: 缓冲可能还有下一帧, 不睡, 尽快清空 */
        } else {
            /* #33: RX 空 → 休眠 COMM_IDLE_POLL_MS 再轮询, 让出 CPU 给控制任务 */
            vTaskDelay(pdMS_TO_TICKS(COMM_IDLE_POLL_MS));
        }
    }
}

/*===========================================================================
 * 命令处理器实现 (#12: 全部走 AppState)
 *===========================================================================*/

/**
 * CMD_READ_DATA (0x01): 读取实时数据
 * 响应: v_out(4)+i_out(4)+v_in(4)+temp(4)+v_set(4)+i_set(4)
 *       + mode(1)+out_en(1)+power(4)+fault(1)
 *       + cv_p(4)+cv_i(4)+cv_d(4)+cv_error(4)   
 *       + cc_p(4)+cc_i(4)+cc_d(4)+cc_error(4)   
 *       = 59 bytes
 */
static void CmdHandler_ReadData(const CommFrame_t *req, CommFrame_t *resp)
{
    ADCData_t         adc;
    SystemSetting_t   set;
    ProtectContext_t *ctx;
    uint8_t          *p = resp->data;
    float             power;
    uint8_t           fault_byte;

    (void)req;

    /* #12: 实测值 + 设定值均从 AppState 取一致快照 (替代裸 g_latest_adc_data) */
    AppState_GetADC(&adc);
    AppState_GetSetting(&set);
    ctx = AppState_GetProtectCtx();

    resp->command = (uint8_t)(CMD_READ_DATA | CMD_ACK);   /* 0x81 */

    memcpy(p, &adc.v_out, 4);       p += 4;
    memcpy(p, &adc.i_out, 4);       p += 4;
    memcpy(p, &adc.v_in, 4);        p += 4;
    memcpy(p, &adc.temperature, 4); p += 4;
    memcpy(p, &set.v_set, 4);       p += 4;
    memcpy(p, &set.i_set, 4);       p += 4;
    *p++ = (uint8_t)TaskPID_GetMode();
    *p++ = set.output_enable;

    power = adc.v_out * adc.i_out;
    memcpy(p, &power, 4);           p += 4;

    /* fault: 上报当前活跃故障位掩码 (来自全局唯一 ctx) */
    fault_byte = (uint8_t)ctx->active_faults;
    *p++ = fault_byte;

    /*PID 内部状态 (8 个 float, 共 32 字节) ─── */
    {
        float p_term, i_term, d_term, error;

        /* CV 环 P/I/D/error */
        PID_GetState(&g_pid_cv, &p_term, &i_term, &d_term, &error);
        memcpy(p, &p_term, 4);  p += 4;
        memcpy(p, &i_term, 4);  p += 4;
        memcpy(p, &d_term, 4);  p += 4;
        memcpy(p, &error,  4);  p += 4;

        /* CC 环 P/I/D/error */
        PID_GetState(&g_pid_cc, &p_term, &i_term, &d_term, &error);
        memcpy(p, &p_term, 4);  p += 4;
        memcpy(p, &i_term, 4);  p += 4;
        memcpy(p, &d_term, 4);  p += 4;
        memcpy(p, &error,  4);  p += 4;
    }

    resp->data_len = (uint8_t)(p - resp->data);
}

/**
 * CMD_SET_V_I (0x02): 设定目标电压/电流
 * 请求: v_set(4B) + i_set(4B) = 8B   响应: 1B (0=OK,1=超限,2=长度错)
 */
static void CmdHandler_SetVI(const CommFrame_t *req, CommFrame_t *resp)
{
    float v_set, i_set;
    SystemSetting_t s;

    resp->command = (uint8_t)(CMD_SET_V_I | CMD_ACK);
    resp->data_len = 1;

    if (req->data_len < 8) {
        resp->data[0] = 2;      /* 数据长度错误 */
        return;
    }

    memcpy(&v_set, req->data, 4);
    memcpy(&i_set, req->data + 4, 4);

    /* 范围检查 (上限与硬件/app_config 一致: V_OUT_MAX=28V, OCP=3.2A) */
    if (v_set < 0.0f || v_set > V_OUT_MAX ||
        i_set < 0.0f || i_set > OCP_THRESHOLD_CURRENT) {
        resp->data[0] = 1;      /* 参数超限 */
        return;
    }

    /* #12: 读改写 AppState —— 只改 v_set/i_set, 保留最新 output_enable */
    AppState_GetSetting(&s);
    s.v_set = v_set;
    s.i_set = i_set;
    AppState_SetSetting(&s);

    resp->data[0] = 0;          /* OK */
}

/**
 * CMD_SET_PID (0x03): 在线调 PID 参数
 * 请求: loop(1B:0=CV,1=CC) + kp(4B)+ki(4B)+kd(4B) = 13B   响应: 1B
 */
static void CmdHandler_SetPID(const CommFrame_t *req, CommFrame_t *resp)
{
    uint8_t loop_sel;
    float kp, ki, kd;

    resp->command = (uint8_t)(CMD_SET_PID | CMD_ACK);
    resp->data_len = 1;

    if (req->data_len < 13) {
        resp->data[0] = 2;
        return;
    }

    loop_sel = req->data[0];
    memcpy(&kp, req->data + 1, 4);
    memcpy(&ki, req->data + 5, 4);
    memcpy(&kd, req->data + 9, 4);

    if (loop_sel == 0) {
        TaskPID_UpdateCVParams(kp, ki, kd);
    } else if (loop_sel == 1) {
        TaskPID_UpdateCCParams(kp, ki, kd);
    } else {
        resp->data[0] = 3;      /* 无效环选择 */
        return;
    }

    resp->data[0] = 0;
}

/**
 * CMD_PRESET (0x04): 读取预设值
 * 请求: op(1B) + idx(1B, 0~3)   响应: value(4B)
 */
static void CmdHandler_Preset(const CommFrame_t *req, CommFrame_t *resp)
{
    uint8_t idx;
    /* 预设值来自 app_config 宏 (只读; 若需写入预设, 由下批 EEPROM 命令扩展) */
    const float preset_vals[4] = {PRESET_VOLTAGE_1, PRESET_VOLTAGE_2,
                                   PRESET_VOLTAGE_3, PRESET_VOLTAGE_4};

    resp->command = (uint8_t)(CMD_PRESET | CMD_ACK);

    if (req->data_len < 2) {
        resp->data[0] = 2;
        resp->data_len = 1;
        return;
    }

    idx = req->data[1];
    if (idx >= 4) {
        resp->data[0] = 3;
        resp->data_len = 1;
        return;
    }

    memcpy(resp->data, &preset_vals[idx], 4);
    resp->data_len = 4;
}

/**
 * CMD_OUTPUT_CTRL (0x05): 使能/关断输出
 * 请求: 1B (0=关断,1=使能)   响应: 1B
 */
static void CmdHandler_OutputCtrl(const CommFrame_t *req, CommFrame_t *resp)
{
    resp->command = (uint8_t)(CMD_OUTPUT_CTRL | CMD_ACK);
    resp->data_len = 1;

    if (req->data_len < 1) {
        resp->data[0] = 2;
        return;
    }

    /*
     * 一致性: 走 TaskPID 使能/关断接口 (内部经 AppState 改 output_enable),
     * 软启动由 PID 任务按使能上升沿自动触发。不裸改全局。
     */
    if (req->data[0]) {
        TaskPID_EnableOutput();
    } else {
        TaskPID_DisableOutput();
    }

    resp->data[0] = 0;
}

/**
 * CMD_SAVE_CONFIG (0x06): 保存当前配置到 Flash
 * 请求: 无数据   响应: 1B (0=OK, 1=失败)
 *
 * 从 AppState 读取当前设定值 + 校准参数, 填入 EEPROM_Config_t,
 * 调用 EEPROM_SaveConfig 写入内部 Flash。
 * PID 参数写回默认值 (运行时不改 PID 则保持不变)。
 */
static void CmdHandler_SaveConfig(const CommFrame_t *req, CommFrame_t *resp)
{
    EEPROM_Config_t cfg;
    SystemSetting_t s;
    Calibration_t   cal;

    (void)req;

    resp->command = (uint8_t)(CMD_SAVE_CONFIG | CMD_ACK);
    resp->data_len = 1;

    AppState_GetSetting(&s);
    AppState_GetCalibration(&cal);

    /* 填充 EEPROM 结构体: 先清零 (覆盖 padding/reserved), 再逐字段赋值 */
    memset(&cfg, 0, sizeof(EEPROM_Config_t));

    cfg.v_set = s.v_set;
    cfg.i_set = s.i_set;
    cfg.output_enable = 0;  /* 安全: 下次上电总是关断状态 */

    cfg.v_cal_slope  = cal.v_slope;
    cfg.v_cal_offset = cal.v_offset;
    cfg.i_cal_slope  = cal.i_slope;
    cfg.i_cal_offset = cal.i_offset;

    /* 预设值保持默认 (暂不支持在线改预设) */
    cfg.preset_voltages[0] = PRESET_VOLTAGE_1;
    cfg.preset_voltages[1] = PRESET_VOLTAGE_2;
    cfg.preset_voltages[2] = PRESET_VOLTAGE_3;
    cfg.preset_voltages[3] = PRESET_VOLTAGE_4;

    /* PID 参数保持默认 (运行时调参通过 CMD_SET_PID, 不持久化) */
    cfg.pid_cv_kp = PID_CV_KP_DEFAULT;
    cfg.pid_cv_ki = PID_CV_KI_DEFAULT;
    cfg.pid_cv_kd = PID_CV_KD_DEFAULT;
    cfg.pid_cc_kp = PID_CC_KP_DEFAULT;
    cfg.pid_cc_ki = PID_CC_KI_DEFAULT;
    cfg.pid_cc_kd = PID_CC_KD_DEFAULT;

    resp->data[0] = EEPROM_SaveConfig(&cfg) ? 0 : 1;
}

/**
 * CMD_FACTORY_RESET (0x07): 擦除 EEPROM 页 + 系统复位
 * 请求: 无数据   响应: 1B (0=OK, 1=擦除失败)
 *
 * 擦除整页 Flash 后软件复位 MCU, 重启后 EEPROM_LoadConfig 返回 0
 * (首次使用), 自动填入出厂默认值。
 */
static void CmdHandler_FactoryReset(const CommFrame_t *req, CommFrame_t *resp)
{
    (void)req;

    resp->command = (uint8_t)(CMD_FACTORY_RESET | CMD_ACK);
    resp->data_len = 1;

    if (EEPROM_ErasePage()) {
        resp->data[0] = 0;  /* OK */
        /*
         * 擦除成功后延时 50ms 让 UART TX DMA 发完响应帧, 然后软件复位。
         * 复位后 MCU 重新上电初始化, EEPROM_LoadConfig 检测到空页
         * → 填入出厂默认值 (5V/1A, 输出关断)。
         */
        vTaskDelay(pdMS_TO_TICKS(50));
        HAL_NVIC_SystemReset();
        /* 不会执行到这里 */
    } else {
        resp->data[0] = 1;  /* 擦除失败 */
    }
}
