/**
 ******************************************************************************
 * @file    main.h
 * @author  J.Soto
 * @version V1.2
 * @date    July 28th, 2025
 * @brief   This file contains the common defines of the application.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "x_nucleo_ihmxx.h"
#include "stspin220.h"
#include "x_nucleo_ihm06a1_stm32l4xx.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Private defines -----------------------------------------------------------*/
#define USART_BAUD_RATE 1000000
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

/* Private macros -------------------------------------------------------------*/
#define UART_RX_DMA_HEAD(__HANDLE__) ((__HANDLE__)->RxXferSize - (__HANDLE__)->hdmarx->Instance->CNDTR)
#define IS_UART_TX_BUSY(__HANDLE__) ((__HANDLE__)->gState == HAL_UART_STATE_BUSY_TX)

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
