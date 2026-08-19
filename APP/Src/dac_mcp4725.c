/**
 * dac_mcp4725.c
 * MCP4725 12-bit DAC I²C 驱动实现
 *
 * 驱动两片 MCP4725 12-bit DAC：
 *   - #1 @ 0x60 (A0=GND): 控制 Buck FB (反比, code↑→V_buck↓)
 *   - #2 @ 0x61 (A0=VCC): 控制线性栅极 (正比, V_out = V_DAC2 × 10)
 *
 * I²C 总线：PB6(SCL) + PB7(SDA), 400kHz Fast Mode
 * 写模式：Fast Write Mode (2 数据字节), ~68μs/次
 *
 * ⚠️ 句柄单一来源：
 *   本文件不再定义 static I2C_HandleTypeDef hi2c1。所有 I²C 操作都作用于
 *   main.c 定义、main.h 用 extern 导出的全局 hi2c1。若各处各留一份 static
 *   句柄副本, 会出现"配置了这份、操作的却是那份"的诡异 bug (根治 #7 类问题)。
 */

#include "main.h"             /* extern I2C_HandleTypeDef hi2c1 (句柄单一来源) */
#include "i2c.h"
#include "dac_mcp4725.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/*===========================================================================
 * 地址常量
 *===========================================================================*/

/* MCP4725 写地址 = 7-bit 地址 << 1 (HAL 库 DevAddress 要求左移 1 位) */
#define DAC1_WRITE_ADDR     (DAC1_ADDR << 1)   /* 0x60<<1 = 0xC0 */
#define DAC2_WRITE_ADDR     (DAC2_ADDR << 1)   /* 0x61<<1 = 0xC2 */

/* I²C 软复位节流: 上次软复位时间戳, 避免 I²C 故障时每周期都做 ~20ms 软复位 */
static uint32_t g_last_i2c_reset_tick = 0;

/*===========================================================================
 * 底层写入函数
 *===========================================================================*/

/**
 * @brief 向指定地址的 MCP4725 写入 12-bit 码值 (Fast Write Mode)
 *
 * 为什么需要：DAC1/DAC2/SetBoth 的公共底层, 集中处理位打包 + 错误恢复。
 * 输入：dev_addr —— 设备写地址 (7-bit<<1); code —— 12-bit 码值 [0,4095]
 * 输出：0=成功, 1=I²C 超时/错误 (并已触发软复位)
 * 调用时机：DACx_SetCode / DAC_SetBoth / DAC_PowerOnZero 内部。
 * 副作用：一次 I²C 主机发送; 失败时调用 DAC_I2C_SoftwareReset。
 *
 * 🔧 位打包 (问题 #1 核心修正), 依据 MCP4725 datasheet Figure 6-2:
 *   字节1 = [ 0 0 PD1 PD0 D11 D10 D9 D8 ]  正常模式 PD=00
 *   字节2 = [ D7 D6 D5 D4 D3 D2 D1 D0 ]
 */
static uint8_t DAC_WriteCode(uint16_t dev_addr, uint16_t code)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;

    /* 安全限幅到 12-bit 满量程 */
    if (code > MCP4725_MAX_CODE) {
        code = MCP4725_MAX_CODE;
    }

    /*
     * 字节1: PD1:PD0=00 (正常) + 数据高 4 位 D11:D8
     *   (code >> 8) 得到 D11:D8 (值域 0x0..0xF), & 0x0F 防越界,
     *   | 0x00 (MCP4725_CMD_FAST_WRITE) 明确高 4 位/PD 位为 0。
     */
    data[0] = (uint8_t)(MCP4725_CMD_FAST_WRITE | ((code >> 8) & 0x0F));

    /* 字节2: 数据低 8 位 D7:D0 */
    data[1] = (uint8_t)(code & 0xFF);

    /*
     * HAL_I2C_Master_Transmit 自动处理 START/地址字节/STOP。
     * Size=2: 只发这两个数据字节 (Fast Write 无需先写配置寄存器)。
     */
    status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr, data, 2,
                                     MCP4725_WRITE_TIMEOUT_MS);

    if (status != HAL_OK) {
        /*
         * 失败可能原因: SDA/SCL 焊接、地址错误 (A0 电平)、总线死锁、上拉缺失。
         * 处理: 软复位 I²C 总线 (节流到每 1s 最多一次, 避免连续故障时每周期
         *       都执行 ~20ms 的 HAL_Delay 软复位, 拖死 PID 任务)。
         */
        uint32_t now = HAL_GetTick();
        if (now - g_last_i2c_reset_tick >= 1000U) {
            DAC_I2C_SoftwareReset();
            g_last_i2c_reset_tick = now;
        }
        return 1;
    }

    return 0;
}

/*===========================================================================
 * 公开 API
 *===========================================================================*/

uint8_t DAC_PowerOnZero(void)
{
    uint8_t result = 0;

    /*
     * Step 1: DAC#2 写 0 → V_DAC2=0 → P-MOS 截止 → 输出端子无电压 (最重要)。
     *   先关最终输出级, 确保即使 Buck 有电压也传不到输出端子。
     */
    if (DAC_WriteCode(DAC2_WRITE_ADDR, 0) != 0) {
        result |= 0x02;  /* bit1: DAC#2 失败 */
    }

    /*
     * Step 2: DAC#1 写 DAC1_MAX_CODE → V_DAC1 最大 → V_buck 最低 (反比)。
     *   减小上电后 Buck 母线残压, 降低后续软启动冲击。
     *   [问题修正] 原来写魔法数 4000, 现改用 DAC1_MAX_CODE 宏 (=4000), 语义清晰。
     */
    if (DAC_WriteCode(DAC1_WRITE_ADDR, DAC1_MAX_CODE) != 0) {
        result |= 0x01;  /* bit0: DAC#1 失败 */
    }

    return result;
}

uint8_t DAC1_SetCode(uint16_t code)
{
    /* Buck FB 控制限幅: 不小于 MIN (防 Buck 输出过高), 不大于 MAX (防完全关断) */
    if (code < DAC1_MIN_CODE) {
        code = DAC1_MIN_CODE;
    }
    if (code > DAC1_MAX_CODE) {
        code = DAC1_MAX_CODE;
    }

    return DAC_WriteCode(DAC1_WRITE_ADDR, code);
}

uint8_t DAC2_SetCode(uint16_t code)
{
    /*
     * 【v2.0】线性栅极控制限幅至 DAC2_MAX_CODE=3475 (对应 V_out ≈ 28V)。
     * 硬件假设来源：docs/04-hardware-interface.md 第 4.2 节 (v2.0)
     */
    if (code > DAC2_MAX_CODE) {
        code = DAC2_MAX_CODE;
    }

    return DAC_WriteCode(DAC2_WRITE_ADDR, code);
}

uint8_t DAC_SetBoth(uint16_t code1, uint16_t code2)
{
    uint8_t result = 0;

    /*
     * 写入顺序: 先 DAC#1 (Buck 粗调, 大电容响应慢), 后 DAC#2 (线性精调)。
     * 两次写间隔 ~68μs, Buck 电压尚未变化, 顺序影响小, 但保持以明确意图。
     */
    if (DAC1_SetCode(code1) != 0) {
        result |= 0x01;
    }
    if (DAC2_SetCode(code2) != 0) {
        result |= 0x02;
    }

    return result;
}

/*===========================================================================
 * I²C 软件复位 (解除 SDA 死锁)
 *===========================================================================*/

/**
 * I²C 总线死锁恢复。
 *
 * 原理：从设备在传输中途掉电/复位时可能一直拉低 SDA。发送 ≤9 个 SCL 脉冲
 *   让从设备完成当前字节 (8 数据位 + 1 ACK 位) 后释放 SDA, 再补一个 STOP。
 *
 * ⚠️ 阻塞函数 (含 HAL_Delay): 仅在初始化或错误恢复路径调用, 不在高频任务里用。
 */
void DAC_I2C_SoftwareReset(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    int i;

    /* Step 1: 复位 I²C1 外设, 清除内部状态机 */
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    /* Step 2: 把 SCL/SDA 临时切为开漏 GPIO 输出 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;   /* PB6=SCL, PB7=SDA */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Step 3: 释放 SDA (拉高) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    /* Step 4: 最多 9 个 SCL 脉冲, SDA 被释放即退出 */
    for (i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  /* SCL low */
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    /* SCL high */
        HAL_Delay(1);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
            break;  /* SDA 已释放 */
        }
    }

    /* Step 5: 发 STOP 条件 (SCL 高时 SDA 由低变高) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    /* Step 6: 重新初始化 I²C1 (GPIO 复用模式也在此恢复) */
    MX_I2C1_Init();
}
