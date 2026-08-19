# 迷你数控电源（Mini Bench Power Supply）

基于 **STM32F103C8T6 + FreeRTOS** 的数控可调直流电源，采用 **Boost + Buck + 线性** 三级功率架构，输出 **0~28V / 0~3A / 84W**。

这是一个用于学习嵌入式开发的项目，覆盖 MCU 外设、RTOS 任务调度、PID 闭环控制、ADC/DMA/I²C、以及电源硬件设计。

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

```
DC_IN ── Boost(XL6019, 固定33V) ── Buck(XL4016, 预稳压) ── Linear(IRF9540, 精调) ── 输出
```

- **Boost 前级**：开环固定升压到 33V，给后级提供稳定母线
- **Buck 中间级**：由 DAC#1 注入 FB 控制，把母线预稳压到「目标电压 + 2V」，降低线性级功耗
- **线性后级**：P-MOS（IRF9540）+ LM358 模拟闭环，精调并滤除开关纹波

## 软件架构

- **RTOS**：FreeRTOS，5 个任务
  - `taskADC` — 1kHz 采样（TIM3 触发 + ADC DMA 双缓冲）
  - `taskPID` — 10ms 控制环（CV/CC 双 PID + MIN 选择器）
  - `taskUI` — 屏幕显示 + 编码器交互
  - `taskComm` — 串口协议解析
  - `taskMonitor` — 保护检测

- **控制算法**：两个位置式 PID 并行运行（恒压环 CV + 恒流环 CC），MIN 选择器自动、无感切换 CV/CC，无需模式标志位
- **软启动**：输出使能后设定值从 0 缓慢爬升，避免上电浪涌
- **保护**：过压 / 过流 / 过功率 / 过温，软件 + 硬件双重保护

## 目录结构

```
├── APP/            # 应用层：PID、ADC 驱动、任务、串口协议、状态管理
│   ├── Inc/
│   └── Src/tasks/
├── User/           # CubeMX 生成代码 + main（外设初始化、时钟配置）
├── Libraries/      # CMSIS + STM32F1 HAL 库
├── FreeRTOS/       # FreeRTOS 内核源码
├── Project/        # Keil MDK5 工程文件
├── Doc/            # 文档
├── SKDY_Project_*.epro2   # 立创EDA Pro 硬件工程（原理图 + PCB）
└── 数控电源项目介绍.doc
```

## 编译与烧录

1. 安装 Keil MDK5（ARM Compiler V5 / AC5）
2. 打开 `Project/sdky.uvprojx`
3. `F7` 编译，用 ST-Link 通过 SWD 烧录

## 功能特性

- 0~28V 连续可调（编码器 + 预设档位 3.3V/5V/12V/24V）
- 恒压 / 恒流自动切换
- 软启动、上电无浪涌
- 多重保护（OVP/OCP/OPP/OTP）
- 串口上位机：读数据、设电压电流、输出开关、保存配置、恢复出厂

## 许可

个人学习项目，仅供学习交流使用。
