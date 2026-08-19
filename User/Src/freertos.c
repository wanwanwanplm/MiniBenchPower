/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "tasks/task_adc.h"
#include "tasks/task_pid.h"
#include "tasks/task_monitor.h"
#include "tasks/task_comm.h"
#include "tasks/task_ui.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for taskADC */
osThreadId_t taskADCHandle;
const osThreadAttr_t taskADC_attributes = {
  .name = "taskADC",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for taskPID */
osThreadId_t taskPIDHandle;
const osThreadAttr_t taskPID_attributes = {
  .name = "taskPID",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for taskUI */
osThreadId_t taskUIHandle;
const osThreadAttr_t taskUI_attributes = {
  .name = "taskUI",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for taskComm */
osThreadId_t taskCommHandle;
const osThreadAttr_t taskComm_attributes = {
  .name = "taskComm",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for taskMonitor */
osThreadId_t taskMonitorHandle;
const osThreadAttr_t taskMonitor_attributes = {
  .name = "taskMonitor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void TaskADC_Entry(void *argument);
void TaskPID_Entry(void *argument);
void TaskUI_Entry(void *argument);
void TaskComm_Entry(void *argument);
void TaskMonitor_Entry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */


void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
/**
 * @brief Idle 任务内存分配 (静态分配)
 *
 * 调用者: FreeRTOS 内核自动调用 (当 configSUPPORT_STATIC_ALLOCATION=1 时)。
 * 不提供此函数且 configSUPPORT_STATIC_ALLOCATION=1 → 链接报错或 Idle 创建失败。
 * 本函数不依赖动态堆, Idle 栈用 static 数组, 固定 configMINIMAL_STACK_SIZE words。
 * 若 FreeRTOSConfig.h 中 configSUPPORT_STATIC_ALLOCATION=0,
 * 此函数完全不会被调用 —— FreeRTOS 改用 pvPortMalloc 从堆里分配 Idle 栈。
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/**
 * @brief 定时器任务内存分配 (软件定时器守护任务的静态内存)
 *
 * [批次5 修正 #6] 原实现函数体为空, 三个出参指针未赋值 → 若 configUSE_TIMERS=1
 * 且用静态分配, FreeRTOS 会解引用野指针 → HardFault。这里正确填充静态内存。
 * 仿照 Idle 钩子, 用静态 TCB + 栈 (Timer 栈用 configTIMER_TASK_STACK_DEPTH)。
 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t  uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

/**
 * @brief 栈溢出钩子
 *
 * 当 FreeRTOS 检测到任务栈溢出时调用此函数。
 * 栈溢出是嵌入式系统最常见的 bug 之一。
 *
 * 【面试可能问】如何检测和调试栈溢出？
 *   答：
 *   1. 使能 configCHECK_FOR_STACK_OVERFLOW (设为 1 或 2)
 *   2. 实现此钩子函数（设断点/记录日志）
 *   3. 使用 uxTaskGetStackHighWaterMark 监控栈使用峰值
 *   4. 常见原因：
 *      a. 栈大小设太小
 *      b. 递归调用过深
 *      c. 局部变量太大（大数组放栈上）
 *      d. sprintf 等不检查边界的函数
 *      e. 浮点运算：Cortex-M3 无 FPU，浮点用软件模拟栈可能很大
 *
 * 本项目 PID 计算用 float（软件浮点），所以 PID 任务栈设了 512 words。
 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    /*
     * 栈溢出 → 系统严重错误
     * 处理：
     *   1. 关断输出（安全第一）
     *   2. LED 快闪指示错误
     *   3. 无限循环（等待调试器介入）
     */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    while (1) {
        /* LED 快闪模式 */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        for (volatile uint32_t i = 0; i < 500000; i++);
    }
		/* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}

/**
 * @brief 内存分配失败钩子
 *
 * FreeRTOS pvPortMalloc 返回 NULL 时调用。
 * 常见原因：configTOTAL_HEAP_SIZE 太小。
 */
void vApplicationMallocFailedHook(void)
{
    /*
     * 内存分配失败 → 严重错误
     * 关断输出 + 死循环
     */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        for (volatile uint32_t i = 0; i < 200000; i++);
    }
}


/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of taskADC */
  taskADCHandle = osThreadNew(TaskADC_Entry, NULL, &taskADC_attributes);

  /* creation of taskPID */
  taskPIDHandle = osThreadNew(TaskPID_Entry, NULL, &taskPID_attributes);

  /* creation of taskUI */
  taskUIHandle = osThreadNew(TaskUI_Entry, NULL, &taskUI_attributes);

  /* creation of taskComm */
  taskCommHandle = osThreadNew(TaskComm_Entry, NULL, &taskComm_attributes);

  /* creation of taskMonitor */
  taskMonitorHandle = osThreadNew(TaskMonitor_Entry, NULL, &taskMonitor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_TaskADC_Entry */
/**
  * @brief  Function implementing the taskADC thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TaskADC_Entry */
void TaskADC_Entry(void *argument)
{
  /* USER CODE BEGIN TaskADC_Entry */
  /* Infinite loop */
  for(;;)
  {
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    vTaskADC(argument);
  }
  /* USER CODE END TaskADC_Entry */
}

/* USER CODE BEGIN Header_TaskPID_Entry */
/**
* @brief Function implementing the taskPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskPID_Entry */
void TaskPID_Entry(void *argument)
{
  /* USER CODE BEGIN TaskPID_Entry */
  /* Infinite loop */
  for(;;)
  {
    vTaskPID(argument);
  }
  /* USER CODE END TaskPID_Entry */
}

/* USER CODE BEGIN Header_TaskUI_Entry */
/**
* @brief Function implementing the taskUI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskUI_Entry */
void TaskUI_Entry(void *argument)
{
  /* USER CODE BEGIN TaskUI_Entry */
  /* Infinite loop */
  for(;;)
  {
    vTaskUI(argument);
  }
  /* USER CODE END TaskUI_Entry */
}

/* USER CODE BEGIN Header_TaskComm_Entry */
/**
* @brief Function implementing the taskComm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskComm_Entry */
void TaskComm_Entry(void *argument)
{
  /* USER CODE BEGIN TaskComm_Entry */
  /* Infinite loop */
  for(;;)
  {
    vTaskComm(argument);
  }
  /* USER CODE END TaskComm_Entry */
}

/* USER CODE BEGIN Header_TaskMonitor_Entry */
/**
* @brief Function implementing the taskMonitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskMonitor_Entry */
void TaskMonitor_Entry(void *argument)
{
  /* USER CODE BEGIN TaskMonitor_Entry */
  /* Infinite loop */
  for(;;)
  {
    vTaskMonitor(argument);
  }
  /* USER CODE END TaskMonitor_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

