/**
 ******************************************************************************
 * @file    stm32l4xx_it.c
 * @author  J.Soto
 * @version V1.2
 * @date    July 28th, 2025
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 */

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef hTimerStepClock;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	while (1);
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	Error_Handler();
	while (1);
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	Error_Handler();
	while (1);
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	Error_Handler();
	while (1);
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	Error_Handler();
	while (1);
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{

}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{

}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{

}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	HAL_IncTick();
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 channel6 global interrupt.
 */
void DMA1_Channel6_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
 * @brief This function handles DMA1 channel7 global interrupt.
 */
void DMA1_Channel7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

/**
 * @brief This function handles ADC1 and ADC2 interrupts.
 */
void ADC1_2_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&hadc1);
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&hTimerStepClock);
}

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT);
	HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
}
