/**
 * eeprom_emulate.c
 * 内部 Flash 模拟 EEPROM 实现
 *
 * 用 STM32F103C8T6 的最后一页 Flash (0x0800FC00, 1KB) 存储配置参数。
 *
 * 记录格式 (每条 EEPROM_RECORD_SIZE=128 字节):
 *   [ magic(2B, 0xAA55) ][ EEPROM_Config_t(76B) ][ 未用填充至 128B ]
 *
 * 查找最新记录: 从页首扫描, 取最后一条 magic=0xAA55 的记录。
 * 写新记录: 追加到最后一条之后; 页满则擦整页从头写 (磨损均衡)。
 *
 * ⚠️ 半字编程: F103 Flash 只能按 16-bit 写。数据体 sizeof 已在
 *    头文件保证为偶数, 故 count=sizeof/2 的循环能写完整个结构体, 不漏字节。
 */

#include "eeprom_emulate.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include <string.h>

/*===========================================================================
 * 模块级变量
 *===========================================================================*/

static uint8_t g_eeprom_initialized = 0;

/*===========================================================================
 * 初始化
 *===========================================================================*/

void EEPROM_Init(void)
{
    /*
     * Flash 无需特殊初始化, HAL_FLASH_Unlock 在每次写时调用。
     * 这里仅标记模块就绪 (SaveConfig 会检查此标志)。
     */
    g_eeprom_initialized = 1;
}

/*===========================================================================
 * 查找最新记录
 *===========================================================================*/

/**
 * 扫描页内记录, 返回最后一条有效记录索引; 无则返回 -1。
 *
 * magic==0xAA55 → 有效, 记录位置继续找更新的;
 * magic==0xFFFF → 空槽 (擦除态), 后面都是空的 → 停止。
 */
static int16_t EEPROM_FindLastRecord(void)
{
    uint32_t addr;
    uint16_t magic;
    int16_t last_valid = -1;
    int i;

    for (i = 0; i < EEPROM_MAX_RECORDS; i++) {
        addr = FLASH_EEPROM_BASE_ADDR + (uint32_t)i * EEPROM_RECORD_SIZE;
        magic = *(volatile uint16_t *)addr;

        if (magic == EEPROM_RECORD_MAGIC) {
            last_valid = (int16_t)i;
        } else if (magic == 0xFFFF) {
            break;  /* 空槽, 后面都空 */
        }
        /* 其他值 → 损坏记录, 跳过 */
    }
    return last_valid;
}

/*===========================================================================
 * 加载配置
 *===========================================================================*/

uint8_t EEPROM_LoadConfig(EEPROM_Config_t *config)
{
    int16_t record_idx;
    uint32_t addr;

    if (config == NULL) {
        return 0;
    }

    record_idx = EEPROM_FindLastRecord();
    if (record_idx < 0) {
        /* 无有效记录 → 首次使用, 返回默认值 */
        EEPROM_LoadDefaults(config);
        return 0;
    }

    /* 记录地址 + 2 跳过 magic */
    addr = FLASH_EEPROM_BASE_ADDR
           + (uint32_t)record_idx * EEPROM_RECORD_SIZE
           + 2;

    memcpy(config, (void *)addr, sizeof(EEPROM_Config_t));
    return 1;
}

/*===========================================================================
 * 保存配置
 *===========================================================================*/

uint8_t EEPROM_SaveConfig(const EEPROM_Config_t *config)
{
    int16_t last_record;
    uint32_t write_addr;
    HAL_StatusTypeDef status;

    if (config == NULL || !g_eeprom_initialized) {
        return 0;
    }

    HAL_FLASH_Unlock();

    last_record = EEPROM_FindLastRecord();

    /*
     * 页满 (最后一条已是末槽) → 擦整页, 从第 0 槽重写。
     */
    if (last_record >= (EEPROM_MAX_RECORDS - 1)) {
        FLASH_EraseInitTypeDef erase_init;
        uint32_t page_error;

        erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
        erase_init.PageAddress = FLASH_EEPROM_BASE_ADDR;
        erase_init.NbPages = 1;

        status = HAL_FLASHEx_Erase(&erase_init, &page_error);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return 0;
        }
        last_record = -1;   /* 回到页首 */
    }

    /* 下一条记录地址 */
    write_addr = FLASH_EEPROM_BASE_ADDR
                 + (uint32_t)(last_record + 1) * EEPROM_RECORD_SIZE;

    /*
     * Step 1: 写 magic (半字)。目标地址须为擦除态 (0xFFFF) 才能写。
     */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                               write_addr,
                               EEPROM_RECORD_MAGIC);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return 0;
    }

    /*
     * Step 2: 半字循环写入结构体。
     *   count = sizeof/2: 因 sizeof 已保证为偶数 (头文件编译期断言), 无漏字节。
     */
    {
        const uint16_t *src = (const uint16_t *)config;
        uint16_t count = (uint16_t)(sizeof(EEPROM_Config_t) / 2);
        uint32_t addr = write_addr + 2;   /* 跳过 magic */
        uint16_t j;

        for (j = 0; j < count; j++) {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                       addr + (uint32_t)j * 2,
                                       src[j]);
            if (status != HAL_OK) {
                HAL_FLASH_Lock();
                return 0;
            }
        }
    }

    HAL_FLASH_Lock();
    return 1;
}

/*===========================================================================
 * 默认配置
 *===========================================================================*/

void EEPROM_LoadDefaults(EEPROM_Config_t *config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(EEPROM_Config_t));

    config->v_cal_slope  = V_CAL_SLOPE_DEFAULT;
    config->v_cal_offset = V_CAL_OFFSET_DEFAULT;
    config->i_cal_slope  = I_CAL_SLOPE_DEFAULT;
    config->i_cal_offset = I_CAL_OFFSET_DEFAULT;

    config->v_set = 5.0f;            /* 默认 5V (安全) */
    config->i_set = 1.0f;            /* 默认 1A */
    config->output_enable = 0;       /* 默认关断 */

    config->preset_voltages[0] = PRESET_VOLTAGE_1;   /* 3.3V */
    config->preset_voltages[1] = PRESET_VOLTAGE_2;   /* 5V   */
    config->preset_voltages[2] = PRESET_VOLTAGE_3;   /* 12V  */
    config->preset_voltages[3] = PRESET_VOLTAGE_4;   /* 24V  */

    config->pid_cv_kp = PID_CV_KP_DEFAULT;
    config->pid_cv_ki = PID_CV_KI_DEFAULT;
    config->pid_cv_kd = PID_CV_KD_DEFAULT;
    config->pid_cc_kp = PID_CC_KP_DEFAULT;
    config->pid_cc_ki = PID_CC_KI_DEFAULT;
    config->pid_cc_kd = PID_CC_KD_DEFAULT;
}

uint8_t EEPROM_ErasePage(void)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    HAL_StatusTypeDef status;

    HAL_FLASH_Unlock();

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = FLASH_EEPROM_BASE_ADDR;
    erase_init.NbPages = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();
    return (status == HAL_OK) ? 1 : 0;
}
