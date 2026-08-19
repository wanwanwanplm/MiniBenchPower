/**
 * tft_st7735.h
 * 1.8" TFT ST7735S SPI 显示驱动
 *
 * 硬件连接：
 *   SCK  → PA5 (SPI1_SCK)
 *   MOSI → PA7 (SPI1_MOSI)
 *   CS   → PA8 (GPIO Output, 软件片选)
 *   DC   → PA11 (GPIO Output, 数据/命令选择)
 *   RST  → PA12 (GPIO Output, 复位)
 *   BL   → PB3 (GPIO Output, 背光)
 *
 * SPI Mode 3 (CPOL=1, CPHA=1), 18MHz
 *
 * 为什么 SPI 用 Mode 3 (CPOL=1, CPHA=1)?
 *   答：ST7735 的数据手册规定了 SPI 模式要求。
 *     CPOL=1: 时钟空闲时高电平
 *     CPHA=1: 数据在第二个边沿（下降沿）采样
 *     不能随便改——主从设备必须用同一模式。
 *
 * TFT 初始化为什么需要 120ms 延迟？
 *   答：ST7735 上电后内部电源电路（charge pump）需要稳定时间。
 *     如果立即发初始化命令，电源不稳定可能导致初始化失败。
 *     120ms 是数据手册推荐的上电等待时间。
 *     这个延迟在 main.c 中完成，不在这里阻塞。
 */

#ifndef __TFT_ST7735_H
#define __TFT_ST7735_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 颜色定义 (RGB565)
 *===========================================================================*/

#define TFT_COLOR_BLACK         0x0000
#define TFT_COLOR_WHITE         0xFFFF
#define TFT_COLOR_RED           0xF800
#define TFT_COLOR_GREEN         0x07E0
#define TFT_COLOR_BLUE          0x001F
#define TFT_COLOR_YELLOW        0xFFE0
#define TFT_COLOR_CYAN          0x07FF
#define TFT_COLOR_MAGENTA       0xF81F
#define TFT_COLOR_ORANGE        0xFC00
#define TFT_COLOR_GRAY          0x8410
#define TFT_COLOR_DARK_GRAY     0x4208

/*===========================================================================
 * 显示方向
 *===========================================================================*/

#define TFT_WIDTH   160
#define TFT_HEIGHT  128

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化 TFT 硬件 (SPI1 + GPIO)
 *
 * 调用时机：main.c 硬件初始化序列中 (GPIO 初始化之后)
 *
 * 配置：
 *   - SPI1: Mode 3, 8-bit, MSB First, 18MHz (72MHz/4)
 *   - CS/DC/RST/BL: GPIO Output
 *   - 背光默认关闭
 */
void TFT_Init(void);

/**
 * @brief 发送 TFT 初始化命令序列
 *
 * 调用时机：TFT 硬件复位后延时 120ms，然后调用此函数
 *
 * 包含：
 *   - 软件复位
 *   - 帧率设置
 *   - 显示方向 (竖屏 128×160)
 *   - Gamma 校正
 *   - 显示开启
 *
 * 初始化序列为什么要这么长？
 *   答：ST7735 有几十个寄存器需要配置，包括：
 *     帧率、显示窗口、色阶反转、Gamma曲线、电源设置等。
 *     大部分寄存器用默认值即可工作，但 Gamma 和电源设置
 *     必须按数据手册配置才能正确显示颜色。
 *     初始化序列本质是一组寄存器写入命令。
 */
void TFT_SendInitSequence(void);

/**
 * @brief 硬件复位 TFT
 *
 * 时序：RST 低 > 10ms → RST 高 → 等待 120ms
 *
 * 调用时机：系统上电初始化时
 *
 * 注意：此函数会阻塞 ~120ms，只能在初始化阶段调用。
 */
void TFT_HardwareReset(void);

/**
 * @brief 设置背光亮度
 *
 * @param enable  1 = 开, 0 = 关
 *
 * 如果使用 PWM 调光，此函数改为设置 PWM 占空比。
 * 当前简化版本仅支持开关控制。
 */
void TFT_Backlight(uint8_t enable);

/**
 * @brief 填充整个屏幕为单一颜色
 *
 * @param color  16-bit RGB565 颜色值
 */
void TFT_FillScreen(uint16_t color);

/**
 * @brief 填充指定矩形区域
 *
 * @param x      起始 x 坐标 [0, 127]
 * @param y      起始 y 坐标 [0, 159]
 * @param width  宽度
 * @param height 高度
 * @param color  16-bit RGB565 颜色值
 */
void TFT_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

/**
 * @brief 在指定位置显示字符串
 *
 * @param x      起始 x 坐标
 * @param y      起始 y 坐标
 * @param str    字符串 (ASCII)
 * @param color  文字颜色 (RGB565)
 * @param bg     背景颜色 (RGB565)
 * @param size   字体大小倍数 (1=6×8, 2=12×16, ...)
 *
 * 使用 5×7 像素的基础字体 (标准 ASCII)。
 */
void TFT_DrawString(uint16_t x, uint16_t y, const char *str,
                    uint16_t color, uint16_t bg, uint8_t size);

/**
 * @brief 显示数码管风格的大号数字
 *
 * @param x         起始 x 坐标
 * @param y         起始 y 坐标
 * @param value     要显示的浮点数值
 * @param decimals  小数位数 (0=整数, 1=1位小数, 2=2位小数, 3=3位小数)
 * @param color     文字颜色
 * @param bg        背景颜色
 *
 * 数码管风格：用大号字体分段显示，模拟 7 段数码管效果。
 * 电压显示用 2 位小数 (如 12.00V)
 * 电流显示用 3 位小数 (如 1.500A)
 *
 * 为什么用"数码管风格"而不是普通字体？
 *   答：电源面板上的数码管风格是行业惯例，直观、在 128×160 小屏上也清晰。
 *     这对嵌入式面试也是加分项——展示了对 UI 设计的思考。
 */
void TFT_DrawDigitLarge(uint16_t x, uint16_t y, float value, uint8_t decimals,
                        uint16_t color, uint16_t bg);

/**
 * @brief 显示 CV/CC 模式指示
 *
 * @param mode  0 = CV (恒压), 1 = CC (恒流)
 *
 * CV 模式用绿色大字显示 "CV"
 * CC 模式用红色大字显示 "CC"
 */
void TFT_ShowMode(uint8_t mode);

/**
 * @brief 绘制仪表盘主界面
 *
 * 显示内容：
 *   第 1 行: 设定电压值 (大写, 蓝色)
 *   第 2 行: 实际电压值 (大写, 白色)
 *   第 3 行: 设定电流值 (大写, 蓝色)
 *   第 4 行: 实际电流值 (大写, 白色)
 *   第 5 行: 输出功率 + CV/CC 模式
 *   底部: 状态栏 (输出 ON/OFF, 故障指示)
 *
 * 调用时机：vTaskUI 每 100ms 刷新一次
 */
void TFT_DrawMainScreen(float v_set, float v_act,
                        float i_set, float i_act,
                        float power, uint8_t mode,
                        uint8_t output_enabled, uint8_t fault);

/**
 * @brief 在屏幕上显示故障信息
 *
 * @param fault_flags  故障标志 (FaultFlag_t 的位掩码)
 */
void TFT_ShowFault(uint8_t fault_flags);

/**
 * @brief 清屏并显示启动画面
 *
 * 调用时机：系统上电初始化完成后，显示项目名称和版本。
 */
void TFT_ShowSplashScreen(void);

#ifdef __cplusplus
}
#endif

#endif /* __TFT_ST7735_H */
