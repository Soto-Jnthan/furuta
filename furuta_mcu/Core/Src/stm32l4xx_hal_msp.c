/**
 ******************************************************************************
 * @file    stm32l4xx_hal_msp.h
 * @author  J.Soto
 * @version V1.2
 * @date    July 28th, 2025
 * @brief   This file provides code for the MSP Initialization
 *          and de-Initialization codes.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void)
{
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	/* System interrupt init*/
}

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	if (hadc->Instance == ADC1) {
		/** Initializes the peripherals clock
		 */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
		PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
		PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
		PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
		PeriphClkInit.PLLSAI1.PLLSAI1N = 40;
		PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
		PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
		PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
		PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
			Error_Handler();

		/* Peripheral clock enable */
		__HAL_RCC_ADC_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**ADC1 GPIO Configuration
         PA0     ------> ADC1_IN5
		 */
		GPIO_InitStruct.Pin = AS5600_IN_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(AS5600_IN_GPIO_Port, &GPIO_InitStruct);

		/* ADC1 interrupt Init */
		HAL_NVIC_SetPriority(ADC1_2_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
	}
}

/**
 * @brief ADC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1) {
		/* Peripheral clock disable */
		__HAL_RCC_ADC_CLK_DISABLE();

		/**ADC1 GPIO Configuration
         PA0     ------> ADC1_IN5
		 */
		HAL_GPIO_DeInit(AS5600_IN_GPIO_Port, AS5600_IN_Pin);

		/* ADC1 interrupt DeInit */
		HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
	}
}

/**
 * @brief TIM_OC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_oc: TIM_OC handle pointer
 * @retval None
 */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* htim_oc)
{
	if (htim_oc->Instance == TIM2) {
		GPIO_InitTypeDef GPIO_InitStruct;

		/* Peripheral clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();
		/* TIM2 interrupt Init */
		HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);

		GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK_MODE3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AF_TIM_STCK;
		HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK_MODE3, &GPIO_InitStruct);
	}
}

/**
 * @brief TIM_PWM MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_pwm: TIM_PWM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
	if (htim_pwm->Instance == TIM3) {
		GPIO_InitTypeDef GPIO_InitStruct;

		/* Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();

		GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_PWM_REF;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AF_PWM_REF;
		HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_PWM_REF, &GPIO_InitStruct);
	}
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (htim->Instance == TIM2) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**TIM2 GPIO Configuration
         PB3 (JTDO-TRACESWO)     ------> TIM2_CH2
		 */
		GPIO_InitStruct.Pin = STS_STCK_M3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(STS_STCK_M3_GPIO_Port, &GPIO_InitStruct);
	} else if (htim->Instance == TIM3) {
		__HAL_RCC_GPIOC_CLK_ENABLE();
		/**TIM3 GPIO Configuration
         PC7     ------> TIM3_CH2
		 */
		GPIO_InitStruct.Pin = STS_REF_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(STS_REF_GPIO_Port, &GPIO_InitStruct);
	}
}

/**
 * @brief TIM_OC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_oc: TIM_OC handle pointer
 * @retval None
 */
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef* htim_oc)
{
	if (htim_oc->Instance == TIM2) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();

		/* TIM2 interrupt DeInit */
		HAL_NVIC_DisableIRQ(TIM2_IRQn);

		HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK_MODE3,\
				BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK_MODE3);
	}
}

/**
 * @brief TIM_PWM MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_pwm: TIM_PWM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
	if (htim_pwm->Instance == TIM3) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM3_CLK_DISABLE();
		HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PORT_PWM_REF,\
				BSP_MOTOR_CONTROL_BOARD_PIN_PWM_REF);
	}
}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	if (huart->Instance == USART2) {
		/** Initializes the peripherals clock
		 */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
		PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
			Error_Handler();

		/* Peripheral clock enable */
		__HAL_RCC_USART2_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**USART2 GPIO Configuration
         PA2     ------> USART2_TX
         PA3     ------> USART2_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USART2 DMA Init */
		/* USART2_RX Init */
		hdma_usart2_rx.Instance = DMA1_Channel6;
		hdma_usart2_rx.Init.Request = DMA_REQUEST_2;
		hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
		hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
		if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
			Error_Handler();

		__HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

		/* USART2_TX Init */
		hdma_usart2_tx.Instance = DMA1_Channel7;
		hdma_usart2_tx.Init.Request = DMA_REQUEST_2;
		hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart2_tx.Init.Mode = DMA_NORMAL;
		hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
		if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
			Error_Handler();

		__HAL_LINKDMA(huart,hdmatx,hdma_usart2_tx);

		/* USART2 interrupt Init */
		HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
	}
}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
	if (huart->Instance == USART2) {
		/* Peripheral clock disable */
		__HAL_RCC_USART2_CLK_DISABLE();

		/**USART2 GPIO Configuration
         PA2     ------> USART2_TX
         PA3     ------> USART2_RX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

		/* USART2 DMA DeInit */
		HAL_DMA_DeInit(huart->hdmarx);
		HAL_DMA_DeInit(huart->hdmatx);

		/* USART2 interrupt DeInit */
		HAL_NVIC_DisableIRQ(USART2_IRQn);
	}
}