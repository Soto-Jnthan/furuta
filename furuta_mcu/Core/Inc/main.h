/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "x_nucleo_ihmxx.h"
#include "stspin220.h"
#include "x_nucleo_ihm06a1_stm32l4xx.h"
#include "stm32l4xx_ll_usart.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MotorErrHandler(uint16_t error);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AS5600_IN_Pin GPIO_PIN_0
#define AS5600_IN_GPIO_Port GPIOA
#define STS_REF_Pin GPIO_PIN_7
#define STS_REF_GPIO_Port GPIOC
#define STS_DIR_M4_Pin GPIO_PIN_8
#define STS_DIR_M4_GPIO_Port GPIOA
#define STS_RST_Pin GPIO_PIN_9
#define STS_RST_GPIO_Port GPIOA
#define STS_EN_AND_FAULT_Pin GPIO_PIN_10
#define STS_EN_AND_FAULT_GPIO_Port GPIOA
#define STS_EN_AND_FAULT_EXTI_IRQn EXTI15_10_IRQn
#define STS_STCK_M3_Pin GPIO_PIN_3
#define STS_STCK_M3_GPIO_Port GPIOB
#define STS_M1_Pin GPIO_PIN_4
#define STS_M1_GPIO_Port GPIOB
#define STS_M2_Pin GPIO_PIN_6
#define STS_M2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define __IS_HAL_UART_TX_BUSY(__HANDLE__) ((__HANDLE__)->gState == HAL_UART_STATE_BUSY_TX)
#define SWAP(A, B) do{__typeof__(A) C = A; A = B; B = C;}while(0)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
