/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_usart1_tx;  /* USART1 TX DMA (DMA1_Channel4), task_comm 用 */
extern TIM_HandleTypeDef htim4;           /* HAL 时基定时器 (与 FreeRTOS SysTick 分离) */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HEARTBEAT_PIN   GPIO_PIN_13
#define HEARTBEAT_PORT  GPIOC
#define OVP_FLAG_Pin GPIO_PIN_4
#define OVP_FLAG_GPIO_Port GPIOA
#define OVP_FLAG_EXTI_IRQn EXTI4_IRQn
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOB
#define ENC_A_EXTI_IRQn EXTI0_IRQn
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define ENC_SW_Pin GPIO_PIN_10
#define ENC_SW_GPIO_Port GPIOB
#define ENC_SW_EXTI_IRQn EXTI15_10_IRQn
#define BUZZER_PIN GPIO_PIN_11
#define BUZZER_PORT GPIOB
#define KEY_3V3_Pin GPIO_PIN_12
#define KEY_3V3_GPIO_Port GPIOB
#define KEY_5V_Pin GPIO_PIN_13
#define KEY_5V_GPIO_Port GPIOB
#define KEY_12V_Pin GPIO_PIN_14
#define KEY_12V_GPIO_Port GPIOB
#define KEY_24V_Pin GPIO_PIN_15
#define KEY_24V_GPIO_Port GPIOB
#define OUT_EN_Pin GPIO_PIN_15
#define OUT_EN_GPIO_Port GPIOA
#define TFT_CS_PORT GPIOA
#define TFT_CS_PIN GPIO_PIN_8
#define TFT_DC_PORT GPIOA
#define TFT_DC_PIN GPIO_PIN_11
#define TFT_RST_PORT GPIOA
#define TFT_RST_PIN GPIO_PIN_12
#define TFT_BL_PORT GPIOB
#define TFT_BL_PIN GPIO_PIN_3

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
