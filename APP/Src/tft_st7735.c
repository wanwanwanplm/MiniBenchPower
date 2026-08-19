/**
 * tft_st7735.c
 * 1.8" TFT ST7735S SPI 显示驱动实现
 *
 * 本文件实现：
 *   1. ST7735S SPI 通信底层 (命令/数据发送)
 *   2. 初始化序列
 *   3. 基础绘图 (填充、字符、大号数字)
 *   4. 主界面 (电源仪表盘)
 *
 * SPI 底层：SPI1 Mode 3 (CPOL=1,CPHA=1), 18MHz, 仅 MOSI 写。
 *   CS/DC/RST/BL 用软件 GPIO 控制。
 *
 * ⚠️ 句柄单一来源 (根治 #7): 删除原 static SPI_HandleTypeDef hspi1,
 *    改用 main.c 定义、main.h extern 的全局 hspi1。
 */

#include "main.h"             /* extern SPI_HandleTypeDef hspi1 (句柄单一来源) */
#include "tft_st7735.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/*===========================================================================
 * GPIO 引脚定义
 *===========================================================================*/

#define TFT_CS_LOW()    HAL_GPIO_WritePin(TFT_CS_PORT, TFT_CS_PIN, GPIO_PIN_RESET)
#define TFT_CS_HIGH()   HAL_GPIO_WritePin(TFT_CS_PORT, TFT_CS_PIN, GPIO_PIN_SET)
#define TFT_DC_CMD()    HAL_GPIO_WritePin(TFT_DC_PORT, TFT_DC_PIN, GPIO_PIN_RESET)  /* DC=0: 命令 */
#define TFT_DC_DATA()   HAL_GPIO_WritePin(TFT_DC_PORT, TFT_DC_PIN, GPIO_PIN_SET)    /* DC=1: 数据 */

/*===========================================================================
 * ST7735S 命令定义
 *===========================================================================*/

#define ST7735_NOP      0x00
#define ST7735_SWRESET  0x01
#define ST7735_SLPOUT   0x11
#define ST7735_NORON    0x13
#define ST7735_INVOFF   0x20
#define ST7735_INVON    0x21
#define ST7735_DISPON   0x29
#define ST7735_CASET    0x2A
#define ST7735_RASET    0x2B
#define ST7735_RAMWR    0x2C
#define ST7735_MADCTL   0x36
#define ST7735_COLMOD   0x3A
#define ST7735_FRMCTR1  0xB1
#define ST7735_FRMCTR2  0xB2
#define ST7735_FRMCTR3  0xB3
#define ST7735_INVCTR   0xB4
#define ST7735_PWCTR1   0xC0
#define ST7735_PWCTR2   0xC1
#define ST7735_PWCTR3   0xC2
#define ST7735_PWCTR4   0xC3
#define ST7735_PWCTR5   0xC4
#define ST7735_VMCTR1   0xC5
#define ST7735_GAMCTRP1 0xE0
#define ST7735_GAMCTRN1 0xE1

/*===========================================================================
 * 底层 SPI 通信 (全部作用于全局 hspi1)
 *===========================================================================*/

static void TFT_WriteCommand(uint8_t cmd)
{
    TFT_CS_LOW();
    TFT_DC_CMD();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    TFT_CS_HIGH();
}

static void TFT_WriteData(uint8_t data)
{
    TFT_CS_LOW();
    TFT_DC_DATA();
    HAL_SPI_Transmit(&hspi1, &data, 1, 10);
    TFT_CS_HIGH();
}

static void TFT_WriteDataMulti(uint8_t *data, uint16_t len)
{
    TFT_CS_LOW();
    TFT_DC_DATA();
    HAL_SPI_Transmit(&hspi1, data, len, 100);
    TFT_CS_HIGH();
}

/**
 * 设置显存写入窗口 (之后 RAMWR 都写入此矩形)。
 */
static void TFT_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    TFT_WriteCommand(ST7735_CASET);         /* 列地址 */
    data[0] = (uint8_t)((x0 >> 8) & 0xFF);
    data[1] = (uint8_t)(x0 & 0xFF);
    data[2] = (uint8_t)((x1 >> 8) & 0xFF);
    data[3] = (uint8_t)(x1 & 0xFF);
    TFT_WriteDataMulti(data, 4);

    TFT_WriteCommand(ST7735_RASET);         /* 行地址 */
    data[0] = (uint8_t)((y0 >> 8) & 0xFF);
    data[1] = (uint8_t)(y0 & 0xFF);
    data[2] = (uint8_t)((y1 >> 8) & 0xFF);
    data[3] = (uint8_t)(y1 & 0xFF);
    TFT_WriteDataMulti(data, 4);
}

/*===========================================================================
 * 初始化
 *===========================================================================*/

void TFT_Init(void)
{
    TFT_CS_HIGH();   /* 确保 CS 初始为高 (未选中) */
}

void TFT_HardwareReset(void)
{
    /* RST 低 > 10ms → 高 → 等 120ms (ST7735 内部电源稳定) */
    HAL_GPIO_WritePin(TFT_RST_PORT, TFT_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(15);
    HAL_GPIO_WritePin(TFT_RST_PORT, TFT_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);
}

void TFT_SendInitSequence(void)
{
    /*
     * ST7735S 初始化序列: 复位 → 退睡眠 → 帧率 → 电源/VCOM → 显示方向
     * → 颜色格式 (RGB565) → Gamma → 开显示。寄存器值来自 ST7735S 数据手册。
     */
    TFT_WriteCommand(ST7735_SWRESET);
    HAL_Delay(150);

    TFT_WriteCommand(ST7735_SLPOUT);
    HAL_Delay(10);

    TFT_WriteCommand(ST7735_FRMCTR1);
    { uint8_t d[] = {0x01, 0x2C, 0x2D}; TFT_WriteDataMulti(d, 3); }
    TFT_WriteCommand(ST7735_FRMCTR2);
    { uint8_t d[] = {0x01, 0x2C, 0x2D}; TFT_WriteDataMulti(d, 3); }
    TFT_WriteCommand(ST7735_FRMCTR3);
    { uint8_t d[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D}; TFT_WriteDataMulti(d, 6); }

    TFT_WriteCommand(ST7735_INVCTR);
    TFT_WriteData(0x07);

    TFT_WriteCommand(ST7735_PWCTR1);
    { uint8_t d[] = {0xA2, 0x02, 0x84}; TFT_WriteDataMulti(d, 3); }
    TFT_WriteCommand(ST7735_PWCTR2);
    TFT_WriteData(0xC5);
    TFT_WriteCommand(ST7735_PWCTR3);
    { uint8_t d[] = {0x0A, 0x00}; TFT_WriteDataMulti(d, 2); }
    TFT_WriteCommand(ST7735_PWCTR4);
    { uint8_t d[] = {0x8A, 0x2A}; TFT_WriteDataMulti(d, 2); }
    TFT_WriteCommand(ST7735_PWCTR5);
    { uint8_t d[] = {0x8A, 0xEE}; TFT_WriteDataMulti(d, 2); }
    TFT_WriteCommand(ST7735_VMCTR1);
    TFT_WriteData(0x0E);

    TFT_WriteCommand(ST7735_INVOFF);

    /* 显示方向: 横屏 160×128 (MV=1 交换 XY), RGB 顺序 */
    TFT_WriteCommand(ST7735_MADCTL);
    TFT_WriteData(0x60);

    /* 颜色格式: 16-bit/pixel (RGB565) */
    TFT_WriteCommand(ST7735_COLMOD);
    TFT_WriteData(0x05);

    TFT_WriteCommand(ST7735_GAMCTRP1);
    { uint8_t d[] = {0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
                     0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10};
      TFT_WriteDataMulti(d, 16); }
    TFT_WriteCommand(ST7735_GAMCTRN1);
    { uint8_t d[] = {0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                     0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10};
      TFT_WriteDataMulti(d, 16); }

    TFT_WriteCommand(ST7735_NORON);
    HAL_Delay(10);
    TFT_WriteCommand(ST7735_DISPON);
    HAL_Delay(10);
}

void TFT_Backlight(uint8_t enable)
{
    HAL_GPIO_WritePin(TFT_BL_PORT, TFT_BL_PIN,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*===========================================================================
 * 基础绘图函数
 *===========================================================================*/

void TFT_FillScreen(uint16_t color)
{
    TFT_FillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, color);
}

/**
 * 实现: 设窗口 → RAMWR → 循环写同一 16-bit 颜色。18MHz 下全屏约 18ms。
 */
void TFT_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    uint32_t total_pixels;
    uint32_t remaining;
    uint8_t  color_bytes[2];

    if (width == 0 || height == 0) {
        return;
    }
    if (x + width > TFT_WIDTH || y + height > TFT_HEIGHT) {
        return;   /* 越界保护 */
    }

    TFT_SetAddrWindow(x, y, x + width - 1, y + height - 1);
    TFT_WriteCommand(ST7735_RAMWR);

    TFT_CS_LOW();
    TFT_DC_DATA();

    color_bytes[0] = (uint8_t)((color >> 8) & 0xFF);   /* 高字节 */
    color_bytes[1] = (uint8_t)(color & 0xFF);          /* 低字节 */

    {
#define SPI_CHUNK_PIXELS 100
        uint8_t chunk[SPI_CHUNK_PIXELS * 2];
        uint32_t i;

        /* 预填充颜色缓冲区 */
        for (i = 0; i < SPI_CHUNK_PIXELS; i++) {
            chunk[i * 2]     = color_bytes[0];
            chunk[i * 2 + 1] = color_bytes[1];
        }

        total_pixels = (uint32_t)width * (uint32_t)height;
        remaining = total_pixels;
        while (remaining > 0) {
            uint32_t batch = (remaining > SPI_CHUNK_PIXELS)
                           ? SPI_CHUNK_PIXELS : remaining;
            HAL_SPI_Transmit(&hspi1, chunk, (uint16_t)(batch * 2), 100);
            remaining -= batch;
        }
    }

    TFT_CS_HIGH();
}

/*===========================================================================
 * 字符显示 (5×7 基础字体)
 *===========================================================================*/

/*
 * ASCII 5×7 点阵 (完整可打印字符集 0x20~0x7A)。每字符 5 列, bit0=顶部。
 * 未定义字符默认零填充 (显示为空白), 不失灵。
 */
static const uint8_t font_5x7[][5] = {
    /* ── 符号 (0x20~0x2F) ── */
    [0x20] = {0x00, 0x00, 0x00, 0x00, 0x00},  /* space */
    [0x21] = {0x00, 0x00, 0x5F, 0x00, 0x00},  /* ! */
    [0x22] = {0x00, 0x07, 0x00, 0x07, 0x00},  /* " */
    [0x23] = {0x14, 0x7F, 0x14, 0x7F, 0x14},  /* # */
    [0x24] = {0x24, 0x2A, 0x7F, 0x2A, 0x12},  /* $ */
    [0x25] = {0x23, 0x13, 0x08, 0x64, 0x62},  /* % */
    [0x26] = {0x36, 0x49, 0x55, 0x22, 0x50},  /* & */
    [0x27] = {0x00, 0x05, 0x03, 0x00, 0x00},  /* ' */
    [0x28] = {0x00, 0x1C, 0x22, 0x41, 0x00},  /* ( */
    [0x29] = {0x00, 0x41, 0x22, 0x1C, 0x00},  /* ) */
    [0x2A] = {0x08, 0x2A, 0x1C, 0x2A, 0x08},  /* * */
    [0x2B] = {0x08, 0x08, 0x3E, 0x08, 0x08},  /* + */
    [0x2C] = {0x00, 0x50, 0x30, 0x00, 0x00},  /* , */
    [0x2D] = {0x08, 0x08, 0x08, 0x08, 0x08},  /* - */
    [0x2E] = {0x00, 0x60, 0x60, 0x00, 0x00},  /* . */
    [0x2F] = {0x20, 0x10, 0x08, 0x04, 0x02},  /* / */
    /* ── 数字 0~9 (0x30~0x39) ── */
    ['0']  = {0x3E, 0x51, 0x49, 0x45, 0x3E},
    ['1']  = {0x00, 0x42, 0x7F, 0x40, 0x00},
    ['2']  = {0x62, 0x51, 0x49, 0x49, 0x46},
    ['3']  = {0x22, 0x41, 0x49, 0x49, 0x36},
    ['4']  = {0x18, 0x14, 0x12, 0x7F, 0x10},
    ['5']  = {0x27, 0x45, 0x45, 0x45, 0x39},
    ['6']  = {0x3E, 0x49, 0x49, 0x49, 0x32},
    ['7']  = {0x01, 0x71, 0x09, 0x05, 0x03},
    ['8']  = {0x36, 0x49, 0x49, 0x49, 0x36},
    ['9']  = {0x26, 0x49, 0x49, 0x49, 0x3E},
    /* ── 符号 (0x3A~0x40) ── */
    [0x3A] = {0x00, 0x36, 0x36, 0x00, 0x00},  /* : */
    [0x3B] = {0x00, 0x56, 0x36, 0x00, 0x00},  /* ; */
    [0x3C] = {0x00, 0x08, 0x14, 0x22, 0x41},  /* < */
    [0x3D] = {0x14, 0x14, 0x14, 0x14, 0x14},  /* = */
    [0x3E] = {0x41, 0x22, 0x14, 0x08, 0x00},  /* > */
    [0x3F] = {0x02, 0x01, 0x51, 0x09, 0x06},  /* ? */
    [0x40] = {0x32, 0x49, 0x79, 0x41, 0x3E},  /* @ */
    /* ── 大写字母 A~Z (0x41~0x5A) ── */
    ['A']  = {0x7E, 0x09, 0x09, 0x09, 0x7E},
    ['B']  = {0x7F, 0x49, 0x49, 0x49, 0x36},
    ['C']  = {0x3E, 0x41, 0x41, 0x41, 0x22},
    ['D']  = {0x7F, 0x41, 0x41, 0x22, 0x1C},
    ['E']  = {0x7F, 0x49, 0x49, 0x49, 0x41},
    ['F']  = {0x7F, 0x09, 0x09, 0x09, 0x01},
    ['G']  = {0x3E, 0x41, 0x49, 0x49, 0x7A},
    ['H']  = {0x7F, 0x08, 0x08, 0x08, 0x7F},
    ['I']  = {0x00, 0x41, 0x7F, 0x41, 0x00},
    ['J']  = {0x20, 0x40, 0x41, 0x3F, 0x01},
    ['K']  = {0x7F, 0x08, 0x14, 0x22, 0x41},
    ['L']  = {0x7F, 0x40, 0x40, 0x40, 0x40},
    ['M']  = {0x7F, 0x02, 0x04, 0x02, 0x7F},
    ['N']  = {0x7F, 0x04, 0x08, 0x10, 0x7F},
    ['O']  = {0x3E, 0x41, 0x41, 0x41, 0x3E},
    ['P']  = {0x7F, 0x09, 0x09, 0x09, 0x06},
    ['Q']  = {0x3E, 0x41, 0x51, 0x21, 0x5E},
    ['R']  = {0x7F, 0x09, 0x19, 0x29, 0x46},
    ['S']  = {0x26, 0x49, 0x49, 0x49, 0x32},
    ['T']  = {0x01, 0x01, 0x7F, 0x01, 0x01},
    ['U']  = {0x3F, 0x40, 0x40, 0x40, 0x3F},
    ['V']  = {0x03, 0x0C, 0x30, 0x0C, 0x03},
    ['W']  = {0x3F, 0x40, 0x38, 0x40, 0x3F},
    ['X']  = {0x63, 0x14, 0x08, 0x14, 0x63},
    ['Y']  = {0x07, 0x08, 0x70, 0x08, 0x07},
    ['Z']  = {0x61, 0x51, 0x49, 0x45, 0x43},
    /* ── 小写字母 a~z (0x61~0x7A) ── */
    ['a']  = {0x20, 0x54, 0x54, 0x54, 0x78},
    ['b']  = {0x7F, 0x48, 0x44, 0x44, 0x38},
    ['c']  = {0x38, 0x44, 0x44, 0x44, 0x20},
    ['d']  = {0x38, 0x44, 0x44, 0x48, 0x7F},
    ['e']  = {0x38, 0x54, 0x54, 0x54, 0x18},
    ['f']  = {0x08, 0x7E, 0x09, 0x01, 0x02},
    ['g']  = {0x0C, 0x52, 0x52, 0x52, 0x3E},
    ['h']  = {0x7F, 0x08, 0x04, 0x04, 0x78},
    ['i']  = {0x00, 0x44, 0x7D, 0x40, 0x00},
    ['j']  = {0x20, 0x40, 0x44, 0x3D, 0x00},
    ['k']  = {0x00, 0x7F, 0x10, 0x28, 0x44},
    ['l']  = {0x00, 0x41, 0x7F, 0x40, 0x00},
    ['m']  = {0x7C, 0x04, 0x18, 0x04, 0x78},
    ['n']  = {0x7C, 0x08, 0x04, 0x04, 0x78},
    ['o']  = {0x38, 0x44, 0x44, 0x44, 0x38},
    ['p']  = {0x7C, 0x14, 0x14, 0x14, 0x08},
    ['q']  = {0x08, 0x14, 0x14, 0x18, 0x7C},
    ['r']  = {0x7C, 0x08, 0x04, 0x04, 0x08},
    ['s']  = {0x48, 0x54, 0x54, 0x54, 0x20},
    ['t']  = {0x04, 0x3F, 0x44, 0x40, 0x20},
    ['u']  = {0x3C, 0x40, 0x40, 0x20, 0x7C},
    ['v']  = {0x1C, 0x20, 0x40, 0x20, 0x1C},
    ['w']  = {0x3C, 0x40, 0x30, 0x40, 0x3C},
    ['x']  = {0x44, 0x28, 0x10, 0x28, 0x44},
    ['y']  = {0x0C, 0x50, 0x50, 0x50, 0x3C},
    ['z']  = {0x44, 0x64, 0x54, 0x4C, 0x44},
    /* ── 冗余索引 (兼容旧代码中  ' '  引用) ── */
    [' ']  = {0x00, 0x00, 0x00, 0x00, 0x00},
};

static void TFT_DrawChar(uint16_t x, uint16_t y, char c,
                         uint16_t color, uint16_t bg, uint8_t size)
{
    uint8_t col, row;
    uint8_t idx;

    if ((uint8_t)c < ' ' || (uint8_t)c >= (uint8_t)(sizeof(font_5x7) / 5)) {
        idx = ' ';   /* 不可打印 / 超出字库 → 空格 */
    } else {
        idx = (uint8_t)c;
    }

    for (col = 0; col < 5; col++) {
        uint8_t line = font_5x7[idx][col];
        for (row = 0; row < 8; row++) {
            if (line & (1 << row)) {
                TFT_FillRect(x + col * size, y + row * size, size, size, color);
            } else {
                TFT_FillRect(x + col * size, y + row * size, size, size, bg);
            }
        }
    }
    /* 字符间距 1 列 */
    TFT_FillRect(x + 5 * size, y, size, 8 * size, bg);
}

void TFT_DrawString(uint16_t x, uint16_t y, const char *str,
                    uint16_t color, uint16_t bg, uint8_t size)
{
    while (*str) {
        TFT_DrawChar(x, y, *str, color, bg, size);
        x += 6 * size;   /* 5 列字符 + 1 列间距 */
        str++;
    }
}

/*===========================================================================
 * 大号数字 (放大字体近似数码管)
 *===========================================================================*/

void TFT_DrawDigitLarge(uint16_t x, uint16_t y, float value, uint8_t decimals,
                        uint16_t color, uint16_t bg)
{
    char str_buf[16];

    if (decimals == 0) {
        snprintf(str_buf, sizeof(str_buf), "%.0f", value);
    } else if (decimals == 1) {
        snprintf(str_buf, sizeof(str_buf), "%.1f", value);
    } else if (decimals == 2) {
        snprintf(str_buf, sizeof(str_buf), "%.2f", value);
    } else {
        snprintf(str_buf, sizeof(str_buf), "%.3f", value);
    }

    /* size=3 (每字符 18×24 像素), 128 宽屏可放 ~6 字符 */
    TFT_DrawString(x, y, str_buf, color, bg, 3);
}

void TFT_ShowMode(uint8_t mode)
{
	  TFT_FillRect(100, 112, 40, 8, TFT_COLOR_BLACK);
    if (mode == 0) {
			TFT_DrawString(70, 112, "MODE: CV", TFT_COLOR_GREEN, TFT_COLOR_BLACK, 1);
    } else {
			TFT_DrawString(70, 112, "MODE: CC", TFT_COLOR_RED, TFT_COLOR_BLACK, 1);
    }
}

/*===========================================================================
 * 主界面 (仪表盘)
 *===========================================================================*/

void TFT_DrawMainScreen(float v_set, float v_act,
                        float i_set, float i_act,
                        float power, uint8_t mode,
                        uint8_t output_enabled, uint8_t fault)
{
    char buf[20];

    /* 第 1 行 (y=0..23): 设定电压 (蓝) */
    TFT_FillRect(36, 0, 90, 24, TFT_COLOR_BLACK);  /* 先清数字区(最长5字符×18px=90px) */
    TFT_DrawString(0, 8, "SET V:", TFT_COLOR_BLUE, TFT_COLOR_BLACK, 1);
    TFT_DrawDigitLarge(36, 0, v_set, 2, TFT_COLOR_BLUE, TFT_COLOR_BLACK);
    TFT_DrawString(126, 8, "V", TFT_COLOR_BLUE, TFT_COLOR_BLACK, 1);

    /* 第 2 行 (y=28..51): 实际电压 (白) */
    TFT_FillRect(36, 28, 90, 24, TFT_COLOR_BLACK);
    TFT_DrawString(0, 36, "ACT V:", TFT_COLOR_WHITE, TFT_COLOR_BLACK, 1);
    TFT_DrawDigitLarge(36, 28, v_act, 2, TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    TFT_DrawString(126, 36, "V", TFT_COLOR_WHITE, TFT_COLOR_BLACK, 1);

    /* 第 3 行 (y=56..79): 设定电流 (蓝) */
    TFT_FillRect(36, 56, 90, 24, TFT_COLOR_BLACK);
    TFT_DrawString(0, 64, "SET I:", TFT_COLOR_BLUE, TFT_COLOR_BLACK, 1);
    TFT_DrawDigitLarge(36, 56, i_set, 3, TFT_COLOR_BLUE, TFT_COLOR_BLACK);
    TFT_DrawString(126, 64, "A", TFT_COLOR_BLUE, TFT_COLOR_BLACK, 1);

    /* 第 4 行 (y=84..107): 实际电流 (白) */
    TFT_FillRect(36, 84, 90, 24, TFT_COLOR_BLACK);
    TFT_DrawString(0, 92, "ACT I:", TFT_COLOR_WHITE, TFT_COLOR_BLACK, 1);
    TFT_DrawDigitLarge(36, 84, i_act, 3, TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    TFT_DrawString(126, 92, "A", TFT_COLOR_WHITE, TFT_COLOR_BLACK, 1);

    /* 第 5 行 (y=112..119): 功率 + CV/CC 模式 */
    TFT_FillRect(21, 112, 40, 8, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "PWR: %.1fW", power);
    TFT_DrawString(0, 112, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK, 1);
    TFT_ShowMode(mode);

    /* 第 6 行 (y=120..127): 输出状态 + 故障指示 */
    TFT_FillRect(22, 120, 40, 8, TFT_COLOR_BLACK);
    if (output_enabled) {
        TFT_DrawString(0, 120, "OUT: ON ", TFT_COLOR_GREEN, TFT_COLOR_BLACK, 1);
    } else {
        TFT_DrawString(0, 120, "OUT: OFF", TFT_COLOR_RED, TFT_COLOR_BLACK, 1);
    }
    TFT_FillRect(120, 120, 40, 8, TFT_COLOR_BLACK);
    if (fault) {
			TFT_DrawString(70, 120, "PROCESS: FAULT!", TFT_COLOR_RED, TFT_COLOR_BLACK, 1);
    } else {
			TFT_DrawString(70, 120, "PROCESS: OK!", TFT_COLOR_GREEN, TFT_COLOR_BLACK, 1);
    }
}

void TFT_ShowFault(uint8_t fault_flags)
{
    if (fault_flags == 0) {
        return;
    }

    TFT_FillScreen(TFT_COLOR_BLACK);
    TFT_DrawString(10, 10, "FAULT:", TFT_COLOR_RED, TFT_COLOR_BLACK, 2);

    if (fault_flags & 0x01) TFT_DrawString(10, 40,  "OVP",    TFT_COLOR_RED, TFT_COLOR_BLACK, 1);
    if (fault_flags & 0x02) TFT_DrawString(10, 55,  "OCP",    TFT_COLOR_RED, TFT_COLOR_BLACK, 1);
    if (fault_flags & 0x04) TFT_DrawString(10, 70,  "OPP",    TFT_COLOR_RED, TFT_COLOR_BLACK, 1);
    if (fault_flags & 0x08) TFT_DrawString(10, 85,  "OTP",    TFT_COLOR_RED, TFT_COLOR_BLACK, 1);
    if (fault_flags & 0x10) TFT_DrawString(10, 100, "SHORT",  TFT_COLOR_RED, TFT_COLOR_BLACK, 1);
    if (fault_flags & 0x20) TFT_DrawString(10, 115, "OVP-HW", TFT_COLOR_RED, TFT_COLOR_BLACK, 1);

    TFT_DrawString(10, 140, "Press to clear", TFT_COLOR_YELLOW, TFT_COLOR_BLACK, 1);
}

void TFT_ShowSplashScreen(void)
{
    TFT_FillScreen(TFT_COLOR_BLACK);

    TFT_DrawString(0, 10, "MiniBench", TFT_COLOR_CYAN, TFT_COLOR_BLACK, 2);
    TFT_DrawString(0, 30, "Power", TFT_COLOR_CYAN, TFT_COLOR_BLACK, 2);
    TFT_DrawString(70, 37, "v2.0", TFT_COLOR_WHITE, TFT_COLOR_BLACK, 1);
    TFT_DrawString(0, 85, "CV/CC CNC PSU", TFT_COLOR_YELLOW, TFT_COLOR_BLACK, 1);
    TFT_DrawString(0, 110, "0-28V 0-3A 84W", TFT_COLOR_GREEN, TFT_COLOR_BLACK, 1);
}
