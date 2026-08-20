# 迷你数控电源（Mini Bench Power Supply）
基于 STM32F103C8T6 + FreeRTOS 的数控可调直流电源，采用 Boost + Buck + 线性
三级功率架构，输出 0~28V / 0~3A / 84W。
覆盖 MCU 外设、RTOS 任务调度、PID 闭环控制、ADC/DMA/I²C、以及电源硬件设计、四层PCB设计。
视频：https://www.bilibili.com/video/BV1yC8H67E52/?spm_id_from=333.1387.homepage.video_card.click
## 硬件规格
| 项目 | 参数 |
|------|------|
| 输出电压 | 0 ~ 28 V |
| 输出电流 | 0 ~ 3 A |
| 最大功率 | 84 W |
| 输入电压 | 5 ~ 20 V DC |
| 主控 | STM32F103C8T6（Cortex-M3, 72MHz）|
| 显示 | 1.8" TFT LCD（ST7735）|
| 操作 | EC11 旋转编码器 |
| 通信 | USART1（115200，自定义帧协议）|

## 系统架构
三级功率拓扑，前级粗调、后级精调：
DC_IN ─ Boost(XL6019,固定33V) ─ Buck(XL4016,预稳压) ─ Linear(IRF9540,精调) ─ 输出
- Boost 前级：开环固定升压到 33V，给后级提供稳定母线
- Buck 中间级：由 DAC#1 注入 FB 控制，预稳压到「目标电压 + 2V」左右，降低线性级功耗
- 线性后级：P-MOS + LM358 模拟闭环，精调并滤除开关纹波

## 软件架构
- RTOS：FreeRTOS，5 个任务（taskADC 1kHz / taskPID 10ms / taskUI 400ms / taskComm 轮询(10ms)/ taskMonitor 500ms）
- 控制算法：两个位置式 PID 并行（CV + CC），MIN 选择器自动无感切换 CV/CC
- 软启动、多重保护（OVP/OCP/OPP/OTP）

#数据流
- ADC DMA 半传输/传输完成中断
- vTaskADC: 滤波 + 物理量换算 → AppState_UpdateADC()
- - vTaskPID:   AppState_GetADC() → PID 计算 → DAC_SetBoth()
- - vTaskUI:    AppState_GetADC() → TFT_DrawMainScreen()
- - vTaskComm:  AppState_GetADC() → CMD_READ_DATA 响应
- - vTaskMonitor: AppState_GetADC() → 输入/温度软告警
- 设定值流（反向）:
- - vTaskUI / vTaskComm → AppState_SetSetting() → vTaskPID 读取

#通信协议帧

| Byte 0 | Byte 1 | Byte 2 | Byte 3..N | Byte N+1..N+2 | Byte N+3 |
|--------|--------|--------|-----------|---------------|----------|
| 0xA5 | 命令码 | 数据长度 | 数据 | CRC16 (LE) | 0x5A |

支持命令：读数据(0x01) / 设 VI(0x02) / 调 PID(0x03) / 预设(0x04) / 输出控制(0x05) / 保存配置(0x06) / 恢复出厂(0x07)

#PID 控制详述

两个 PID 控制器（CV 恒压 / CC 恒流）并行运行，MIN 选择器取较小输出值驱动 DAC#2，实现无缝自动切换：

- 位置式 PID：`u(k) = Kp·e(k) + Ki·Σe·dt + Kd·(e(k)-e(k-1))/dt`
- 积分分离：|error| > 阈值时关闭积分，防止大偏差下积分饱和
- 抗饱和：积分项限幅至 [0, integral_limit]，防止 windup
- 软启动：使能后设定值从 0 以 0.05V/10ms ramp 到目标，抑制启动浪涌
- 设定变化检测：运行中改设定时重启 ramp 但不清积分，实现平滑过渡
## 目录结构
（APP / User / Libraries / FreeRTOS / Project / Doc / .epro2 / .doc）

## 功能特性
0~28V 连续可调、恒压/恒流自动切换、软启动、多重保护、串口上位机控制
<img width="4096" height="3072" alt="8094dd5de36f2e7a44771eaa5f32de42" src="https://github.com/user-attachments/assets/395863d6-c66f-489b-9bee-f6454c72f91b" />
<img width="4341" height="2860" alt="aedecb75b73e665c6b1f76f29211a069" src="https://github.com/user-attachments/assets/fb3fe498-ae71-402f-8bdd-362ee19dfde9" />
<img width="1547" height="1414" alt="dcd6b78248e38cdc11989b7403251a19" src="https://github.com/user-attachments/assets/4e594a31-04b5-4567-9a1f-a59bf7ea4f65" />
