/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RESET_CHAR 	('*') // Character used to start the reset sequence
#define START_TX_CHAR 	('#') // Character used to start the state transmission
#define STOP_TX_CHAR 	('$') // Character used to stop the state transmission
#define USART_SOS 	('A') // Start of string
#define USART_EOS 	('\r') // End of string

#define TxBUFFER_SIZE 	(25) // Size of buffer for serial transmission (Minimum 16)
#define RxBUFFER_SIZE 	(128) // Size of buffer for serial reception (Minimum 10)
#define ACT_QUEUE_SIZE 	(1000) // Size of action-execution queue (Minimum 2. Even more to avoid overrun)

#define ADC_STATIC_ERROR (1995) //Angle reading should be zero when the pendulum is upright

#define MTR_MOVILITY_ANGLE (2 * 360) // degrees
#define MTR_STEP_ANGLE (1.8) // degrees
#define MTR_STEP_MODE (STEP_MODE_1_8)
#define MTR_MAX_STEPS ((uint32_t)(MTR_MOVILITY_ANGLE * (1U << MTR_STEP_MODE) / MTR_STEP_ANGLE))
#define MTR_RESET_MAX_SPEED (250) // pulse/s
#define MTR_RESET_ACC_DEC (75)// pulse/s^2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
static volatile bool episode_done = TRUE;
static volatile bool reset_requested = TRUE;
static volatile bool tx_enabled = FALSE;
static volatile bool ACK_sent = FALSE;
static volatile bool break_enabled = FALSE;

static uint8_t DMA_TxBuffer[TxBUFFER_SIZE];
static uint8_t DMA_RxBuffer[RxBUFFER_SIZE];
static uint8_t DATA_RxBuffer[RxBUFFER_SIZE];

static float action_queue[ACT_QUEUE_SIZE];
static volatile uint32_t q_head;
static volatile uint32_t q_tail;

static volatile errorTypes_t gLastError;

static Stspin220_Init_t initDeviceParameters =
{
		8000,			//Acceleration rate in pulse/s2 (must be greater than 0)
		8000,			//Deceleration rate in pulse/s2 (must be greater than 0)
		MTR_RESET_MAX_SPEED,	//Running speed in pulse/s (8 pulse/s < Maximum speed <= 10 000 pulse/s )
		8,			//Minimum speed in pulse/s (8 pulse/s <= Minimum speed < 10 000 pulse/s)
		95,			//Acceleration current torque in % (from 0 to 100)
		95,			//Deceleration current torque in % (from 0 to 100)
		95,              	//Running current torque in % (from 0 to 100)
		95,              	//Holding current torque in % (from 0 to 100)
		TRUE,            	//Torque boost speed enable
		325,             	//Torque boost speed threshold in fullstep/s
		MTR_STEP_MODE,  	//Step mode via enum motorStepMode_t
		HIZ_MODE,        	//Automatic HIZ STOP
		100000           	//REF frequency (Hz)
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static inline void execute_rx_cmd(const char *restrict cmd);
static inline size_t state_itoa(char *restrict s, int ph, int th, bool ep);
static inline void init_pendulum(void);
static inline void update_pendulum(float action);
static inline void reset_pendulum(void);
static inline void stop_if_off_limits(void);
static void MotorFailureHandler(void);
void ButtonHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	init_pendulum();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/********************************************************************************************/
		// MAIN THREAD: EXECUTE INSTRUCTIONS AS THEY ARE PLACED IN THE QUEUE BY THE UARTEx Callback  /
		/********************************************************************************************/
		if (reset_requested)
		{
			reset_pendulum();
			reset_requested = FALSE;
		}
		while (q_tail != q_head)
		{
			update_pendulum(action_queue[q_tail++]);
			if (q_tail == ACT_QUEUE_SIZE)
			{
				q_tail = 0;
			}
		}
		stop_if_off_limits();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
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
  {
    Error_Handler();
  }

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
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

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
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_128;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_7;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
	__HAL_UART_DISABLE(&huart2);
	LL_USART_ConfigNodeAddress(huart2.Instance, LL_USART_ADDRESS_DETECT_7B, USART_EOS);
	LL_USART_EnableIT_CM(huart2.Instance);
	__HAL_UART_ENABLE(&huart2);
  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/* USER CODE BEGIN 4 */

// NVIRQ CALLBACKS

/**
  * @brief [Priority 3] Store and execute commands as they are received by the UART
  * @param huart Pointer to the handler of the UART that triggered the interrupt
  * @param Size Length in bytes of the data received
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	static uint32_t old_pos;
	if (huart->Instance == huart2.Instance)
	{
		uint32_t RxSize;
		if (Size > old_pos)
		{ 	//Copy the received data to the user's buffer
			RxSize = Size - old_pos;
			memcpy(DATA_RxBuffer, &DMA_RxBuffer[old_pos], sizeof(*DATA_RxBuffer) * RxSize);
		}
		else
		{   //DMA_RxBuffer went full circle
			RxSize = RxBUFFER_SIZE - old_pos;
			memcpy(DATA_RxBuffer, &DMA_RxBuffer[old_pos], sizeof(*DATA_RxBuffer) * RxSize);
			memcpy(&DATA_RxBuffer[RxSize], DMA_RxBuffer, sizeof(*DATA_RxBuffer) * Size);
			RxSize += Size;
		}
		old_pos = Size;
		if (RxSize > 1)
		{
			DATA_RxBuffer[RxSize - 1] = '\0';
			execute_rx_cmd((char*) DATA_RxBuffer);
		}
	}
}

/**
  * @brief [Priority 3] Mediate errors raised by the UART API
  * @param huart Pointer to the handler of the UART that triggered the interrupt
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		if (HAL_UART_Abort(&huart2) != HAL_OK)
			Error_Handler();
		HAL_UART_Receive_DMA(&huart2, DMA_RxBuffer, RxBUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_TC | DMA_IT_HT);
	}
}

/**
  * @brief [Priority 4] Send the state variables with every scan of the ADC API
  * @note Characters are only sent if the tx_enabled or the episode_done flags are set
  * @param hadc Pointer to the handler of the ADC that triggered the interrupt
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == hadc1.Instance)
	{
		int phi = ((4096 + HAL_ADC_GetValue(&hadc1) - ADC_STATIC_ERROR) & 4095U) - 2048; //Adjust offset and symmetry
		int theta = BSP_MotorControl_GetPosition(0); //Steps
		if ((tx_enabled || episode_done) && !__IS_HAL_UART_TX_BUSY(&huart2))
		{
			size_t len = state_itoa((char*) DMA_TxBuffer, phi, theta, episode_done);
			HAL_UART_Transmit_DMA(&huart2, DMA_TxBuffer, len);
			__HAL_DMA_DISABLE_IT(huart2.hdmatx, DMA_IT_HT);
			ACK_sent = episode_done; //'episode_done == 1' sent at least once
		}
	}
}

// GENERAL-PURPOSE FUNCTIONS

/**
  * @brief Execute instruction received from the serial port
  * @note By default, unrecognised commands will be stored in the action_queue
  * @param cmd Pointer to string representing the command received
  * @retval None
  */
static inline void execute_rx_cmd(const char *restrict cmd)
{
	switch (*cmd)
	{
	case RESET_CHAR:
		reset_requested = TRUE;
		break;
	case START_TX_CHAR:
		tx_enabled = TRUE;
		break;
	case STOP_TX_CHAR:
		tx_enabled = FALSE;
		break;
	default:
		action_queue[q_head++] = atof(cmd);
		if (q_head == ACT_QUEUE_SIZE)
		{
			q_head = 0;
		}
		break;
	}
}

/**
  * @brief Join and format the state variables for serial transmission
  * @param s Pointer to space where the array of characters will be placed
  * @param ph PHI angle read from the sensor
  * @param th THETA angle read from the steps counter
  * @param ep episode_done global flag
  * @retval Size in bytes of the stored string
  */
static inline size_t state_itoa(char *restrict s, int ph, int th, bool ep)
{
	size_t len, i = 0;
	inline void rev_itoa(int a)
	{
		bool sign = a < 0;
		if (sign)
			a = -a; //'a' should not be equal to INT_MIN
		do {
			s[i++] = a % 10 + '0';
		} while (a /= 10);
		if (sign)
			s[i++] = '-';
	}

	s[i++] = '\n';
	s[i++] = '\r';

	s[i++] = ep + '0'; //episode_done (Never greater than 1)
	s[i++] = ' ';

	rev_itoa(th); //theta
	s[i++] = ' ';
	rev_itoa(ph); //phi

	s[i] = USART_SOS;

	len = i + 1;
	s[len] = '\0';

	for (size_t j = 0; i > j; j++, i--)
	{
		SWAP(s[i], s[j]);
	}

	return len;
}

/**
  * @brief Initialization of the pendulum's drivers and peripherals
  * @param None
  * @retval None
  */
static inline void init_pendulum(void)
{
	// STEPPER MOTOR DRIVER INITIALIZATION
	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, 1);
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, &initDeviceParameters);
	BSP_MotorControl_AttachFlagInterrupt(MotorFailureHandler);
	BSP_MotorControl_AttachErrorHandler(MotorErrHandler);
	// NUCLEO BOARD COMPONENTS INITIALIZATION
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
	BSP_LED_Init(LED2);
	// ANGULAR ENCODER INITIALIZATION
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_IT(&hadc1);
	// DATA RECEPTION INITIALIZATION
	HAL_UART_Receive_DMA(&huart2, DMA_RxBuffer, RxBUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_TC | DMA_IT_HT);
}

/**
  * @brief Update maximum speed and direction of the stepper motor
  * @param action Signed stepping rate to be applied to the motor
  * @retval None
  */
static inline void update_pendulum(float action)
{
	float speed = fabs(action);
	motorDir_t dir = action > 0 ? FORWARD : BACKWARD;
	if (speed < STSPIN220_MIN_STCK_FREQ)
	{
		BSP_MotorControl_SetStopMode(0, HOLD_MODE);
		BSP_MotorControl_HardStop(0);
	}
	else
	{
		BSP_MotorControl_SetMaxSpeed(0, speed);
		if (BSP_MotorControl_GetDeviceState(0) >= INACTIVE)
		{
			BSP_MotorControl_Run(0, dir);
		}
		else if (dir != BSP_MotorControl_GetDirection(0))
		{
			BSP_MotorControl_HardSetDirection(0, dir);
		}
	}
}

/**
  * @brief Reset pendulum's position back to home
  * @note Stalls the system if break_enabled flag has been set
  * @param None
  * @retval None
  */
static inline void reset_pendulum(void)
{
	episode_done = TRUE;
	BSP_MotorControl_SetStopMode(0, HIZ_MODE);
	BSP_MotorControl_HardStop(0);
	BSP_MotorControl_WaitWhileActive(0);
	BSP_MotorControl_SetAcceleration(0, MTR_RESET_ACC_DEC);
	BSP_MotorControl_SetDeceleration(0, MTR_RESET_ACC_DEC);
	BSP_MotorControl_SetMaxSpeed(0, MTR_RESET_MAX_SPEED);
	BSP_MotorControl_GoHome(0);
	BSP_MotorControl_WaitWhileActive(0);
	BSP_MotorControl_CmdDisable(0);
	BSP_MotorControl_SetAcceleration(0, initDeviceParameters.acceleration);
	BSP_MotorControl_SetDeceleration(0, initDeviceParameters.deceleration);
	while (!ACK_sent);
	while (break_enabled);
	episode_done = FALSE;
}

/**
  * @brief Stop the stepper motor if it has gone beyond the MTR_MAX_STEPS range
  * @note Sets the episode_flag if the stop is executed
  * @param None
  * @retval None
  */
static inline void stop_if_off_limits(void)
{
	if (!episode_done && abs(BSP_MotorControl_GetPosition(0)) >= MTR_MAX_STEPS / 2)
	{
		episode_done = TRUE;
		BSP_MotorControl_SetStopMode(0, HIZ_MODE);
		BSP_MotorControl_HardStop(0);
		BSP_MotorControl_WaitWhileActive(0);
	}
}

// EVENT HANDLERS

/**
  * @brief Handler when EN pin is forced low by a failure
  * @param None
  * @retval None
  */
static void MotorFailureHandler(void)
{
	//Error_Handler();
}

/**
  * @brief Handler for errors raised by the Stepper Motor API
  * @param error Value of error raised
  * @retval None
  */
void MotorErrHandler(uint16_t error)
{
	/* Backup error number */
	gLastError = error;
	Error_Handler();
}

/**
  * @brief Button handler for pausing the training indefinitely by the user
  * @param None
  * @retval None
  */
void ButtonHandler(void)
{
	HAL_Delay(10);
	while (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET);
	break_enabled = !break_enabled;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	episode_done = TRUE;
	BSP_MotorControl_CmdDisable(0);
	while (1)
	{
		BSP_LED_Toggle(LED2);
		HAL_Delay(500);
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
