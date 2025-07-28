/**
 ******************************************************************************
 * @file    main.c
 * @author  J.Soto
 * @version V1.2
 * @date    July 28th, 2025
 * @brief   Furuta Pendulum Controller Application.
 ******************************************************************************
 */

/* Private includes ----------------------------------------------------------*/
#include <math.h>
#include <stdlib.h>
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
typedef union {
	enum /*: uint32_t*/ {
		RESET_CMD = 0x7FC00000, // IEEE754 float for resetting pendulum (e.g. NaN + {int:23})
		START_TX_CMD,
		STOP_TX_CMD,
	} cmd;
	float action;
} pendulum_cmd_t;

/* Private define ------------------------------------------------------------*/
#define CMD_QUEUE_SIZE  1500           // Size of the commands' queue (2 ≤ {value} ≤ 16383)
#define TX_BURST_SIZE   10             // Number of TX packets to be sent after receiving a command
#define ENCDR_BITS      12             // Resolution in bits of the angular encoder
#define ADC_OFFSET      1995           // Angle reading should be zero when the pendulum is upright
#define MOVILITY_RANGE  360.0          // degrees, range: ±{value}
#define STEP_ANGLE      1.8            // degrees
#define STEP_MODE       STEP_MODE_1_16 // motorStepMode_t type
#define RESET_MAX_SPEED 400.0          // pulse/s
#define RESET_ACC_DEC   75.0           // pulse/s^2
#define RESET_COUNTDOWN 30             // Seconds to soft-reset in ErrorHandler (0 to disable it)

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Control Flags */
static volatile bool episode_done;
static volatile bool ack_sent;
static volatile bool break_enabled;

/* USART Communication */
static volatile pendulum_cmd_t cmd_queue[CMD_QUEUE_SIZE];
static volatile unsigned q_tail;
static volatile struct __attribute__((packed)) {
	bool done:1;                // [0 — 1]
	int32_t ph:ENCDR_BITS;      // [-2048 — 2047]
	int32_t th:32-1-ENCDR_BITS; // [-262144 — 262143]
} tx_buff;
static volatile unsigned tx_burst_cnt;

/* Stepper Motor */
static const uint8_t motor_id = 0;
static volatile errorTypes_t motor_last_err;
static const Stspin220_Init_t motor_init_params = {
		8000,            // Acceleration rate in pulse/s2 (must be greater than 0)
		8000,            // Deceleration rate in pulse/s2 (must be greater than 0)
		RESET_MAX_SPEED, // Running speed in pulse/s (8 pulse/s < Maximum speed ≤ 10 000 pulse/s)
		8,               // Minimum speed in pulse/s (8 pulse/s ≤ Minimum speed < 10 000 pulse/s)
		95,              // Acceleration current torque in % (from 0 to 100)
		95,              // Deceleration current torque in % (from 0 to 100)
		95,              // Running current torque in % (from 0 to 100)
		95,              // Holding current torque in % (from 0 to 100)
		false,           // Torque boost speed enable
		325,             // Torque boost speed threshold in fullstep/s
		STEP_MODE,       // Step mode via enum motorStepMode_t
		HOLD_MODE,       // Stop mode via enum motorStopMode_t
		100000           // REF frequency (Hz)
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void init_pendulum(void);
static void update_pendulum(float action);
static void reset_pendulum(void);
static void stop_if_off_limits(void);
static void execute_cmd(volatile const pendulum_cmd_t *cmdptr);
static void motor_failure_handler(void);
static void motor_err_handler(uint16_t error);
static void button_handler(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	init_pendulum();

	while (1) {
		/* Execute instructions as they are queued by the USART's RX DMA Channel */
		if (q_tail != UART_RX_DMA_HEAD(&huart2) / sizeof(*cmd_queue)) {
			execute_cmd(&cmd_queue[q_tail++]);
			if (q_tail >= CMD_QUEUE_SIZE)
				q_tail = 0;
		}
		stop_if_off_limits();
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
		Error_Handler();

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		Error_Handler();

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
		Error_Handler();
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc1.Init.OversamplingMode = ENABLE;
	hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_128;
	hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_7;
	hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
		Error_Handler();

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
		Error_Handler();

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		Error_Handler();
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = USART_BAUD_RATE;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_2;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	if (HAL_UART_Init(&huart2) != HAL_OK)
		Error_Handler();
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, STS_DIR_M4_Pin|STS_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, STS_M1_Pin|STS_M2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC4
                           PC5 PC6 PC8 PC9
                           PC10 PC11 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
			|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
			|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9
			|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PH0 PH1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : PA1 PA4 PA5 PA6
                           PA7 PA11 PA12 PA13
                           PA14 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
			|GPIO_PIN_7|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
			|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB5 PB7 PB8
                           PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
			|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
			|GPIO_PIN_15|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8
			|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : STS_DIR_M4_Pin STS_RST_Pin */
	GPIO_InitStruct.Pin = STS_DIR_M4_Pin|STS_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : STS_EN_AND_FAULT_Pin */
	GPIO_InitStruct.Pin = STS_EN_AND_FAULT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(STS_EN_AND_FAULT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : STS_M1_Pin STS_M2_Pin */
	GPIO_InitStruct.Pin = STS_M1_Pin|STS_M2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief Initialization of the pendulum's drivers and peripherals
 * @param None
 * @retval None
 */
static void init_pendulum(void)
{
	/* Initialize NUCLEO Board Components */
	BSP_LED_Init(LED2);
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
	/* Initialize Stepper Motor Driver */
	if (!BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, 1))
		motor_err_handler(STSPIN220_ERROR_INIT);
	BSP_MotorControl_AttachErrorHandler(motor_err_handler);
	BSP_MotorControl_AttachFlagInterrupt(motor_failure_handler);
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, &motor_init_params);
	/* Initialize UART Reception */
	if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)cmd_queue, sizeof(cmd_queue)) != HAL_OK)
		Error_Handler();
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_TC | DMA_IT_HT);
	/* Initialize ADC for angle readings */
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
		Error_Handler();
}

/**
 * @brief Update maximum speed and direction of the stepper motor
 * @param action Signed stepping rate to be applied to the motor
 * @retval None
 */
static void update_pendulum(float action)
{
	const float speed = fabsf(action);
	const motorDir_t dir = action >= 0 ? FORWARD : BACKWARD;
	if (speed < BSP_MotorControl_GetMinSpeed(motor_id)) {
		BSP_MotorControl_HardStop(motor_id);
	} else {
		if (!BSP_MotorControl_SetMaxSpeed(motor_id, speed))
			BSP_MotorControl_ErrorHandler(STSPIN220_ERROR_SET_MAX_SPEED);
		if (BSP_MotorControl_GetDeviceState(motor_id) >= INACTIVE)
			BSP_MotorControl_Run(motor_id, dir);
		else
			BSP_MotorControl_HardSetDirection(motor_id, dir);
	}
}

/**
 * @brief Reset pendulum's position back to home
 * @param None
 * @retval None
 * @note Stalls the system if break_enabled flag has been set
 */
static void reset_pendulum(void)
{
	episode_done = true;
	ack_sent = !ADC_IS_ENABLE(&hadc1);
	/* Stop the motor if currently moving */
	if (BSP_MotorControl_SoftStop(motor_id))
		BSP_MotorControl_WaitWhileActive(motor_id);
	/* Set parameters used for resetting the pendulum */
	if (!BSP_MotorControl_SetMaxSpeed(motor_id, RESET_MAX_SPEED))
		BSP_MotorControl_ErrorHandler(STSPIN220_ERROR_SET_MAX_SPEED);
	if (!BSP_MotorControl_SetAcceleration(motor_id, RESET_ACC_DEC))
		BSP_MotorControl_ErrorHandler(STSPIN220_ERROR_SET_ACCELERATION);
	if (!BSP_MotorControl_SetDeceleration(motor_id, RESET_ACC_DEC))
		BSP_MotorControl_ErrorHandler(STSPIN220_ERROR_SET_DECELERATION);
	/* Bring the pendulum back to initial position and disable it */
	BSP_MotorControl_GoHome(motor_id);
	BSP_MotorControl_WaitWhileActive(motor_id);
	BSP_MotorControl_CmdHardHiZ(motor_id);
	/* Set parameters back to initial configuration */
	if (!BSP_MotorControl_SetAcceleration(motor_id, motor_init_params.acceleration))
		BSP_MotorControl_ErrorHandler(STSPIN220_ERROR_SET_ACCELERATION);
	if (!BSP_MotorControl_SetDeceleration(motor_id, motor_init_params.deceleration))
		BSP_MotorControl_ErrorHandler(STSPIN220_ERROR_SET_DECELERATION);
	while (!ack_sent); // Wait for "episode_done == true" to be sent if ADC enabled
	while (break_enabled); // Pause if the button has been pressed
	episode_done = false;
	q_tail = UART_RX_DMA_HEAD(&huart2) / sizeof(*cmd_queue);
}

/**
 * @brief Stop the stepper motor if it has gone beyond MOVILITY_RANGE
 * @param None
 * @retval None
 * @note Sets the episode_done flag if the stop is executed
 */
static void stop_if_off_limits(void)
{
	static const uint32_t max_steps = (uint32_t)(MOVILITY_RANGE * (1 << STEP_MODE) / STEP_ANGLE + 0.5);
	if (abs(BSP_MotorControl_GetPosition(motor_id)) >= max_steps) {
		episode_done = true;
		BSP_MotorControl_HardStop(motor_id);
	}
}

/**
 * @brief Handles incoming commands by interpreting a pendulum_cmd_t pointer
 * @param cmdptr Pointer to pendulum_cmd_t representing the command received
 * @retval None
 * @note Defaults to interpreting the command as an action if it does not match known control codes
 */
static void execute_cmd(volatile const pendulum_cmd_t *cmdptr)
{
	if (cmdptr != nullptr) {
		switch (cmdptr->cmd) {
		case RESET_CMD:
			reset_pendulum();
			break;
		case START_TX_CMD:
			if (!ADC_IS_ENABLE(&hadc1) && HAL_ADC_Start_IT(&hadc1) != HAL_OK)
				Error_Handler();
			break;
		case STOP_TX_CMD:
			if (HAL_ADC_Stop_IT(&hadc1) != HAL_OK)
				Error_Handler();
			break;
		default:
			if (!episode_done)
				update_pendulum(cmdptr->action);
			break;
		}
		tx_burst_cnt = 0;
	}
}

/**
 * @brief Handler when EN pin is forced low by a failure
 * @param None
 * @retval None
 * @note Suddenly changing the direction of the motor may trigger this event
 */
static void motor_failure_handler(void)
{
	/* Enforce RL policies that avoid motor failure */
	episode_done = true;
	BSP_MotorControl_HardStop(motor_id);
}

/**
 * @brief Handler for errors raised by the Stepper Motor API
 * @param error Value of error raised
 * @retval None
 */
static void motor_err_handler(uint16_t error)
{
	/* Backup error number */
	motor_last_err = error;
	Error_Handler();
}

/**
 * @brief Button handler for the user to pause the training
 * @param None
 * @retval None
 * @note Holding the button stalls execution of RL commands and motor events
 */
static void button_handler(void)
{
	static const uint32_t debounce_delay = 5; // Milliseconds
	HAL_Delay(debounce_delay);
	while (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET);
	break_enabled = !break_enabled;
}

/* HAL NVIQ Callback Functions */

/**
 * @brief [Priority 2] Mediate errors raised by the UART API
 * @param huart Pointer to the UART handle that triggered the interrupt
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance) {
		if (HAL_UART_Abort(&huart2) != HAL_OK)
			Error_Handler();
		if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)cmd_queue, sizeof(cmd_queue)) != HAL_OK)
			Error_Handler();
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_TC | DMA_IT_HT);
		q_tail = UART_RX_DMA_HEAD(&huart2) / sizeof(*cmd_queue);
	}
}

/**
 * @brief [Priority 3] Send the state variables with every scan of the ADC API
 * @param hadc Pointer to ADC handle that triggered the interrupt
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	static const unsigned ph_mod = 1U << ENCDR_BITS; // Encoder's 2^n modulo
	if (hadc->Instance == hadc1.Instance) {
		uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
		if ((episode_done || tx_burst_cnt < TX_BURST_SIZE) && !IS_UART_TX_BUSY(&huart2)) {
			tx_buff.done = ack_sent = episode_done;
			tx_buff.ph = (ph_mod + adc_val - ADC_OFFSET) % ph_mod - ph_mod / 2; // Zero-centering
			tx_buff.th = BSP_MotorControl_GetPosition(motor_id);
			if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&tx_buff, sizeof(tx_buff)) != HAL_OK)
				Error_Handler();
			__HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);
			if (!episode_done)
				tx_burst_cnt++;
		}
	}
}

/**
 * @brief [Priority 4] External Line Callback
 * @param GPIO_Pin Pin number that triggered the interrupt
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT)
		BSP_MotorControl_FlagInterruptHandler();
	if (GPIO_Pin == KEY_BUTTON_PIN)
		button_handler();
}

/**
 * @brief  [Priority 5] Output Compare callback of Stepper Motor's driver
 * @param  htim Pointer to TIM handle that triggered the interrupt
 * @retval None
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIM_STCK &&
			htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIM_STCK)
		BSP_MotorControl_StepClockHandler(motor_id);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	static const uint32_t blink_delay = 500; // Milliseconds
	static const uint32_t reset_cnt = 1000 * RESET_COUNTDOWN / blink_delay;
	episode_done = true;
	execute_cmd(&(pendulum_cmd_t){START_TX_CMD});
	BSP_MotorControl_CmdResetDevice(motor_id); // Low-consumption mode
	for (uint32_t i = 0; !reset_cnt || i < reset_cnt; i++) {
		BSP_LED_Toggle(LED2);
		HAL_Delay(blink_delay);
	}
	NVIC_SystemReset();
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
