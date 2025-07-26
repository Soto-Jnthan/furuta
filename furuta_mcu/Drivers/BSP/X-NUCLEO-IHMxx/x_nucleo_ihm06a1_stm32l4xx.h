/**
 ******************************************************************************
 * @file    x_nucleo_ihm06a1_stm32f4xx.h
 * @author  IPC Rennes (Modified by J.Soto)
 * @version V1.4.0
 * @date    May 30th, 2018
 * @brief   Header for BSP driver for x-nucleo-ihm06a1 Nucleo extension board
 *  (based on STSPIN220)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef X_NUCLEO_IHM06A1_STM32L4XX_H
#define X_NUCLEO_IHM06A1_STM32L4XX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_nucleo.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IHM06A1_STM32L4XX
 * @{
 */

/* Exported Constants --------------------------------------------------------*/

/** @defgroup IHM06A1_Exported_Constants IHM06A1 Exported Constants
 * @{
 */

/******************************************************************************/
/* USE_STM32L4XX_NUCLEO                                                       */
/******************************************************************************/

/** @defgroup Constants_For_STM32L4XX_NUCLEO Constants For STM32L4XX NUCLEO
 * @{
 */

/* Fault reporting------------------------------------------------------------*/

// Interrupt line used for EN FAULT (FLAG)
#define BSP_MOTOR_CONTROL_BOARD_IRQn_EN_AND_FAULT (EXTI15_10_IRQn)

// Flag interrupt priority
#define BSP_MOTOR_CONTROL_BOARD_PRIORITY_EN_AND_FAULT (4)

/* reference voltage REF generation ------------------------------------------*/

// Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_PWM_REF (TIM3)

// Channel Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_CHAN_PWM_REF (TIM_CHANNEL_2)

// HAL Active Channel Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_PWM_REF (HAL_TIM_ACTIVE_CHANNEL_2)

// Timer Clock Enable for REF
#define __BSP_MOTOR_CONTROL_BOARD_CLCK_ENABLE_PWM_REF() __TIM3_CLK_ENABLE()

// Timer Clock Disable for REF
#define __BSP_MOTOR_CONTROL_BOARD_CLCK_DISABLE_PWM_REF() __TIM3_CLK_DISABLE()

// REF GPIO alternate function
#define BSP_MOTOR_CONTROL_BOARD_AF_PWM_REF (GPIO_AF2_TIM3)

/* step clock ----------------------------------------------------------------*/

// Timer used for step clock
#define BSP_MOTOR_CONTROL_BOARD_TIM_STCK (TIM2)

// Timer output for step clock
#define BSP_MOTOR_CONTROL_BOARD_OUTPUT_TIM_STCK (TIMER_MAIN_OUTPUT)

// Channel Timer used for step clock
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK (TIM_CHANNEL_2)

// HAL Active Channel Timer used for step clock
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIM_STCK (HAL_TIM_ACTIVE_CHANNEL_2)

// Timer Clock Enable for step clock
#define __BSP_MOTOR_CONTROL_BOARD_CLCK_ENABLE_TIM_STCK() __TIM2_CLK_ENABLE()

// Timer Clock Disable for step clock
#define __BSP_MOTOR_CONTROL_BOARD_CLCK_DISABLE_TIM_STCK() __TIM2_CLK_DISABLE()

// Step clock global interrupt
#define BSP_MOTOR_CONTROL_BOARD_IRQn_TIM_STCK (TIM2_IRQn)

// Step clock global interrupt priority
#define BSP_MOTOR_CONTROL_BOARD_PRIORITY_TIM_STCK (BSP_MOTOR_CONTROL_BOARD_PRIORITY_EN_AND_FAULT + 1)

// step clock GPIO alternate function
#define BSP_MOTOR_CONTROL_BOARD_AF_TIM_STCK (GPIO_AF1_TIM2)

/**
 * @}
 */

/******************************************************************************/
/* Independent plateform definitions                                          */
/******************************************************************************/

/** @defgroup Constants_For_All_Nucleo_Platforms Constants For All Nucleo Platforms
 * @{
 */

// Timer with a main output
#define TIMER_MAIN_OUTPUT (0)
// Timer with a complementary output
#define TIMER_COMPLEMENTARY_OUTPUT (1)
// Timer without output
#define TIMER_NO_OUTPUT (2)

// GPIO Pin used for the STSPIN220 stby reset pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_STBY_RESET (GPIO_PIN_9)
// GPIO port used for the STSPIN220 reset pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_STBY_RESET (GPIOA)

// GPIO Pin used for the STSPIN220 en fault pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT (GPIO_PIN_10)
// GPIO port used for the STSPIN220 en fault pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_EN_AND_FAULT (GPIOA)

// GPIO Pin used for the STSPIN220 ref pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_PWM_REF (GPIO_PIN_7)
// GPIO Port used for the STSPIN220 ref pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_PWM_REF (GPIOC)

// GPIO Pin used for the STSPIN220 mode1 pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_MODE1 (GPIO_PIN_4)
// GPIO port used for the STSPIN220 mode1 pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_MODE1 (GPIOB)

// GPIO Pin used for the STSPIN220 mode2 pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_MODE2 (GPIO_PIN_6)
// GPIO port used for the STSPIN220 mode2 pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_MODE2 (GPIOB)

// GPIO Pin used for the STSPIN220 step clock pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK_MODE3 (GPIO_PIN_3)
// #define BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK_MODE3 (GPIO_PIN_10)
// GPIO Port used for the STSPIN220 step clock pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK_MODE3 (GPIOB)
// #define BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK_MODE3 (GPIOB)

// GPIO Pin used for the STSPIN220 direction pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_DIR_MODE4 (GPIO_PIN_8)
// #define BSP_MOTOR_CONTROL_BOARD_PIN_DIR_MODE4 (GPIO_PIN_5)
// GPIO port used for the STSPIN220 direction pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_DIR_MODE4 (GPIOA)
// #define BSP_MOTOR_CONTROL_BOARD_PORT_DIR_MODE4 (GPIOB)

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

#ifdef __cplusplus
}
#endif

#endif /* X_NUCLEO_IHM06A1_STM32L4XX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
