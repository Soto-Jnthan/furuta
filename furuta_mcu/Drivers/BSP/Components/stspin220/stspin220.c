/**
 ******************************************************************************
 * @file    stspin220.c
 * @author  IPC Rennes (Modified by J.Soto)
 * @version V1.4.0
 * @date    May 30th, 2018
 * @brief   STSPIN220 driver (fully integrated microstepping motor driver)
 * @note    (C) COPYRIGHT 2018 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "stspin220.h"

/** @addtogroup BSP
 * @{
 */

/** @defgroup STSPIN220 STSPIN220
 * @{
 */

/* Private constants ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/** @defgroup Stspin220_Private_Variables Stspin220 Private Variables
 * @{
 */

// Function pointer to flag interrupt call back
void (*flagInterruptCallback)(void) = nullptr;
// Function pointer to error handler call back
void (*errorHandlerCallback)(uint16_t) = nullptr;
static uint16_t stspin220DriverInstance;
static uint8_t stspin220NumberOfDevices;
static volatile bool toggleOdd;

// STSPIN220 Device Paramaters structure
deviceParams_t devicePrm;

/**
 * @}
 */

/* Private Types  ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @defgroup Stspin220_Private_Functions Stspin220 Private Functions
 * @{
 */
static void Stspin220_ApplySpeed(uint8_t pwmId, float newSpeed);
static void Stspin220_SetParamsInactive(void);
static void Stspin220_ApplyTorque(uint8_t deviceId, motorTorqueMode_t torqueMode);
static void Stspin220_ComputeSpeedProfile(uint8_t deviceId, uint32_t nbSteps);
static void Stspin220_FlagInterruptHandler(void);
static void Stspin220_SetDeviceParamsOtherValues(void);
static void Stspin220_SetDeviceParamsToGivenValues(const Stspin220_Init_t *initDevicePrm);
static void Stspin220_SetDeviceParamsToPredefinedValues(void);
static bool Stspin220_SetStepModeWithoutReset(uint8_t deviceId, motorStepMode_t stepMode);
static void Stspin220_StartMovement(uint8_t deviceId);
static void Stspin220_StepClockHandler(uint8_t deviceId);
/**
 * @}
 */

/** @defgroup Stspin220_Exported_Variables Stspin220 Exported Variables
 * @{
 */

// STSPIN220 motor driver functions pointer structure
const motorDrv_t stspin220Drv = {
		Stspin220_Init,					   // void (*Init)(const void*);
		Stspin220_ReadId,				   // uint16_t (*ReadID)(void);
		Stspin220_AttachErrorHandler,	   // void (*AttachErrorHandler)(void (*callback)(uint16_t));
		Stspin220_AttachFlagInterrupt,	   // void (*AttachFlagInterrupt)(void (*callback)(void));
		nullptr,						   // void (*AttachBusyInterrupt)(void (*callback)(void));
		Stspin220_FlagInterruptHandler,	   // void (*FlagInterruptHandler)(void);
		Stspin220_GetAcceleration,		   // float (*GetAcceleration)(uint8_t);
		Stspin220_GetCurrentSpeed,		   // float (*GetCurrentSpeed)(uint8_t);
		Stspin220_GetDeceleration,		   // float (*GetDeceleration)(uint8_t);
		Stspin220_GetDeviceState,		   // motorState_t(*GetDeviceState)(uint8_t);
		Stspin220_GetFwVersion,			   // uint32_t (*GetFwVersion)(void);
		Stspin220_GetMark,				   // int32_t (*GetMark)(uint8_t);
		Stspin220_GetMaxSpeed,			   // float (*GetMaxSpeed)(uint8_t);
		Stspin220_GetMinSpeed,			   // float (*GetMinSpeed)(uint8_t);
		Stspin220_GetPosition,			   // int32_t (*GetPosition)(uint8_t);
		Stspin220_GoHome,				   // void (*GoHome)(uint8_t);
		Stspin220_GoMark,				   // void (*GoMark)(uint8_t);
		Stspin220_GoTo,					   // void (*GoTo)(uint8_t, int32_t);
		Stspin220_HardStop,				   // void (*HardStop)(uint8_t);
		Stspin220_Move,					   // void (*Move)(uint8_t, motorDir_t, uint32_t );
		nullptr,						   // void (*ResetAllDevices)(void);
		Stspin220_Run,					   // void (*Run)(uint8_t, motorDir_t);
		Stspin220_SetAcceleration,		   // bool(*SetAcceleration)(uint8_t ,float );
		Stspin220_SetDeceleration,		   // bool(*SetDeceleration)(uint8_t , float );
		Stspin220_SetHome,				   // void (*SetHome)(uint8_t, int32_t);
		Stspin220_SetMark,				   // void (*SetMark)(uint8_t,int32_t);
		Stspin220_SetMaxSpeed,			   // bool (*SetMaxSpeed)(uint8_t, float );
		Stspin220_SetMinSpeed,			   // bool (*SetMinSpeed)(uint8_t, float );
		Stspin220_SoftStop,				   // bool (*SoftStop)(uint8_t);
		Stspin220_StepClockHandler,		   // void (*StepClockHandler)(uint8_t deviceId);
		Stspin220_WaitWhileActive,		   // void (*WaitWhileActive)(uint8_t);
		Stspin220_Disable,				   // void (*CmdDisable)(uint8_t);
		Stspin220_Enable,				   // void (*CmdEnable)(uint8_t);
		nullptr,						   // uint32_t (*CmdGetParam)(uint8_t, uint32_t);
		nullptr,						   // uint16_t (*CmdGetStatus)(uint8_t);
		nullptr,						   // void (*CmdNop)(uint8_t);
		nullptr,						   // void (*CmdSetParam)(uint8_t, uint32_t, uint32_t);
		nullptr,						   // uint16_t (*ReadStatusRegister)(uint8_t);
		nullptr,						   // void (*ReleaseReset)(void);
		nullptr,						   // void (*Reset)(void);
		Stspin220_SetStepMode,			   // bool (*SelectStepMode)(uint8_t deviceId, motorStepMode_t);
		Stspin220_SetDirection,			   // void (*SetDirection)(uint8_t, motorDir_t);
		Stspin220_HardSetDirection,		   // void (*HardSetDirection)(uint8_t, motorDir_t);
		Stspin220_GoToDir,				   // void (*CmdGoToDir)(uint8_t, motorDir_t, int32_t);
		nullptr,						   // uint8_t (*CheckBusyHw)(void);
		Stspin220_CheckStatusHw,		   // bool (*CheckStatusHw)(void);
		nullptr,						   // void (*CmdGoUntil)(uint8_t, motorAction_t, motorDir_t, uint32_t);
		Stspin220_HardHiZ,				   // void (*CmdHardHiZ)(uint8_t);
		nullptr,						   // void (*CmdReleaseSw)(uint8_t, motorAction_t, motorDir_t);
		Stspin220_PutDeviceInStandby,	   // void (*CmdResetDevice)(uint8_t);
		nullptr,						   // void (*CmdResetPos)(uint8_t);
		nullptr,						   // void (*CmdRun)(uint8_t, motorDir_t, uint32_t);
		nullptr,						   // void (*CmdSoftHiZ)(uint8_t);
		nullptr,						   // void (*CmdStepClock)(uint8_t, motorDir_t);
		nullptr,						   // void (*FetchAndClearAllStatus)(void);
		nullptr,						   // uint16_t (*GetFetchedStatus)(uint8_t);
		Stspin220_GetNbDevices,			   // uint8_t (*GetNbDevices)(void);
		nullptr,						   // bool (*IsDeviceBusy)(uint8_t);
		nullptr,						   // void (*SendQueuedCommands)(void);
		nullptr,						   // void (*QueueCommands)(uint8_t, uint8_t, int32_t);
		nullptr,						   // void (*WaitForAllDevicesNotBusy)(void);
		Stspin220_ErrorHandler,			   // void (*ErrorHandler)(uint16_t);
		nullptr,						   // void (*BusyInterruptHandler)(void);
		nullptr,						   // void (*CmdSoftStop)(uint8_t);
		nullptr,						   // void (*StartStepClock)(uint16_t);
		nullptr,						   // void (*StopStepClock)(void);
		nullptr,						   // void (*SetDualFullBridgeConfig)(uint8_t);
		Stspin220_VrefPwmGetFreq,		   // uint32_t (*GetBridgeInputPwmFreq)(uint8_t);
		Stspin220_VrefPwmSetFreq,		   // void (*SetBridgeInputPwmFreq)(uint8_t, uint32_t);
		Stspin220_SetStopMode,			   // void (*SetStopMode)(uint8_t, motorStopMode_t);
		Stspin220_GetStopMode,			   // motorStopMode_t (*GetStopMode)(uint8_t);
		nullptr,						   // void (*SetDecayMode)(uint8_t, motorDecayMode_t);
		nullptr,						   // motorDecayMode_t (*GetDecayMode)(uint8_t);
		Stspin220_GetStepMode,			   // motorStepMode_t (*GetStepMode)(uint8_t);
		Stspin220_GetDirection,			   // motorDir_t (*GetDirection)(uint8_t);
		Stspin220_ExitDeviceFromStandby,   // void (*ExitDeviceFromReset)(uint8_t);
		Stspin220_SetTorque,			   // void (*SetTorque)(uint8_t, motorTorqueMode_t, uint8_t);
		Stspin220_GetTorque,			   // uint8_t (*GetTorque)(uint8_t, motorTorqueMode_t);
		nullptr,						   // void (*SetRefFreq)(uint8_t, uint32_t);
		nullptr,						   // uint32_t (*GetRefFreq)(uint8_t);
		nullptr,						   // void (*SetRefDc)(uint8_t, uint8_t);
		nullptr,						   // uint8_t (*GetRefDc)(uint8_t);
		Stspin220_SetNbDevices,			   // bool (*SetNbDevices)(uint8_t);
		nullptr,						   // bool (*SetAnalogValue)(uint8_t, uint32_t, float);
		nullptr,						   // float (*GetAnalogValue)(uint8_t, uint32_t);
		Stspin220_SetTorqueBoostEnable,	   // void (*SetTorqueBoostEnable) (uint8_t, bool);
		Stspin220_GetTorqueBoostEnable,	   // bool (*GetTorqueBoostEnable) (uint8_t);
		Stspin220_SetTorqueBoostThreshold, // void (*SetTorqueBoostThreshold) (uint8_t, float);
		Stspin220_GetTorqueBoostThreshold, // float (*GetTorqueBoostThreshold) (uint8_t);
		nullptr							   // uint8_t (*GetDualFullBridgeConfig) (void);
};

/**
 * @}
 */

/** @defgroup STSPIN220_Library_Functions STSPIN220 Library Functions
 * @{
 */

/**
 * @brief Return motor handle (pointer to the STSPIN220 motor driver structure)
 * @retval Pointer to the motorDrv_t structure
 */
const motorDrv_t *Stspin220_GetMotorHandle(void)
{
	return &stspin220Drv;
}

/**
 * @brief Start the STSPIN220 library
 * @param[in] pInit pointer to the initialization data
 * @retval None
 */
void Stspin220_Init(const void *pInit)
{
	stspin220DriverInstance++;

	/* Initialise the GPIOs */
	Stspin220_Board_GpioInit();

	/* Initialise the timer used for the step clock and
	the PWM for the reference voltage generation */
	Stspin220_Board_TimStckInit();
	Stspin220_Board_PwmRefInit();

	if (pInit == nullptr)
		/* Set all context variables to the predefined values */
		/* from stspin220_target_config.h */
		Stspin220_SetDeviceParamsToPredefinedValues();
	else
		Stspin220_SetDeviceParamsToGivenValues((const Stspin220_Init_t *)pInit);
}

/**
 * @brief Read id
 * @retval Id of the stspin220 Driver Instance
 */
uint16_t Stspin220_ReadId(void)
{
	return stspin220DriverInstance;
}

/**
 * @brief Attach a user callback to the error Handler.
 * The call back will be then called each time the library
 * detects an error
 * @param[in] callback Name of the callback to attach
 * to the error Hanlder
 * @retval None
 */
void Stspin220_AttachErrorHandler(void (*callback)(uint16_t))
{
	errorHandlerCallback = callback;
}

/**
 * @brief Attach a user callback to the flag Interrupt
 * The call back will be then called each time the status
 * flag pin will be pulled down due to the occurrence of
 * a programmed alarms ( OCD, thermal pre-warning or
 * shutdown, UVLO, wrong command, non-performable command)
 * @param[in] callback Name of the callback to attach
 * to the Flag Interrupt
 * @retval None
 */
void Stspin220_AttachFlagInterrupt(void (*callback)(void))
{
	flagInterruptCallback = callback;
}

/**
 * @brief Check if STSPIN220 has a fault by reading EN pin position.
 * @retval True if STSPIN220 has EN pin down, otherwise false
 */
bool Stspin220_CheckStatusHw(void)
{
	return !Stspin220_Board_EN_AND_FAULT_PIN_GetState();
}

/**
 * @brief Disable the power bridges (leave the output bridges HiZ)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 */
void Stspin220_Disable(uint8_t deviceId)
{
	UNUSED(deviceId);
	Stspin220_Board_Disable();
}

/**
 * @brief Enable the power bridges
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 */
void Stspin220_Enable(uint8_t deviceId)
{
	UNUSED(deviceId);
	Stspin220_Board_Enable();
}

/**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 */
void Stspin220_ErrorHandler(uint16_t error)
{
	if (errorHandlerCallback != nullptr)
		errorHandlerCallback(error);
	else
		while (1); // Infinite loop
}

/**
 * @brief Exit STSPIN220 device from standby (low power consumption)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 */
void Stspin220_ExitDeviceFromStandby(uint8_t deviceId)
{
	const int16_t sequencerPosition = devicePrm.sequencerPosition;

	/* Exit standby and set step mode */
	Stspin220_SetStepModeWithoutReset(deviceId, devicePrm.stepMode);

	if (devicePrm.sequencerPosition != 0) {
		/* Set direction to FORWARD to ensure the HW sequencer is increased at */
		/* each step clock rising edge */
		Stspin220_SetDirection(deviceId, FORWARD);
		/* Going out of standby */
		devicePrm.motionState = STANDBYTOINACTIVE;
		/* Initialize the step clock timer */
		Stspin220_Board_TimStckInit();
		/* Program the step clock */
		Stspin220_Board_TimStckCompareInit();
		Stspin220_Board_TimStckSetFreq(STSPIN220_MAX_STCK_FREQ);
		toggleOdd = false;
		Stspin220_Board_TimStckStart();
		while (devicePrm.sequencerPosition != 0);
		while (toggleOdd);
		devicePrm.sequencerPosition = sequencerPosition;
	}

	devicePrm.motionState = INACTIVE;
}

/**
 * @brief Return the acceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval Acceleration in pps^2
 */
float Stspin220_GetAcceleration(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.acceleration;
}

/**
 * @brief Return the current speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval Speed in pps
 */
float Stspin220_GetCurrentSpeed(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.speed;
}

/**
 * @brief Return the deceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval Deceleration in pps^2
 */
float Stspin220_GetDeceleration(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.deceleration;
}

/**
 * @brief Return the device state
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval State (ACCELERATING, DECELERATING, STEADY or INACTIVE)
 */
motorState_t Stspin220_GetDeviceState(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.motionState;
}

/**
 * @brief Get the motor current direction
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval direction
 */
motorDir_t Stspin220_GetDirection(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.direction;
}

/**
 * @brief Return the FW version of the library
 * @retval Stspin220_FW_VERSION
 */
uint32_t Stspin220_GetFwVersion(void)
{
	return STSPIN220_FW_VERSION;
}

/**
 * @brief Return the mark position of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval mark position value
 */
int32_t Stspin220_GetMark(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.markPosition;
}

/**
 * @brief Return the max speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval maxSpeed in pps
 */
float Stspin220_GetMaxSpeed(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.maxSpeed;
}

/**
 * @brief Return the min speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval minSpeed in pps
 */
float Stspin220_GetMinSpeed(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.minSpeed;
}

/**
 * @brief  Returns the number of devices
 * @retval number of devices
 */
uint8_t Stspin220_GetNbDevices(void)
{
	return stspin220NumberOfDevices;
}

/**
 * @brief Return the current position value of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval current position value
 */
int32_t Stspin220_GetPosition(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.currentPosition;
}

/**
 * @brief Get the motor step mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval step mode
 */
motorStepMode_t Stspin220_GetStepMode(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.stepMode;
}

/**
 * @brief Get the selected stop mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval the selected stop mode
 */
motorStopMode_t Stspin220_GetStopMode(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.stopMode;
}

/**
 * @brief Get the torque of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] torqueMode torque mode
 * @retval the torqueValue in % (from 0 to 100)
 * @note
 */
uint8_t Stspin220_GetTorque(uint8_t deviceId, motorTorqueMode_t torqueMode)
{
	UNUSED(deviceId);
	uint8_t torqueValue = 0;
	switch (torqueMode) {
	case ACC_TORQUE:
		torqueValue = devicePrm.accelTorque;
		break;
	case DEC_TORQUE:
		torqueValue = devicePrm.decelTorque;
		break;
	case RUN_TORQUE:
		torqueValue = devicePrm.runTorque;
		break;
	case HOLD_TORQUE:
		torqueValue = devicePrm.holdTorque;
		break;
	case CURRENT_TORQUE:
		torqueValue = devicePrm.currentTorque;
		break;
	default:
		break;
	}
	return torqueValue;
}

/**
 * @brief Get the torque boost feature status
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval true if enabled, false if disabled
 */
bool Stspin220_GetTorqueBoostEnable(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.torqueBoostEnable;
}

/**
 * @brief Get the torque boost threshold
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval the torque boost threshold above which the step mode is
 * changed to full step
 */
float Stspin220_GetTorqueBoostThreshold(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.torqueBoostSpeedThreshold;
}

/**
 * @brief Request the motor to move to the home position (ABS_POSITION = 0)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 */
void Stspin220_GoHome(uint8_t deviceId)
{
	Stspin220_GoTo(deviceId, 0);
}

/**
 * @brief Request the motor to move to the mark position
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 */
void Stspin220_GoMark(uint8_t deviceId)
{
	Stspin220_GoTo(deviceId, devicePrm.markPosition);
}

/**
 * @brief Request the motor to move to the specified position
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] targetPosition absolute position in steps
 * @retval None
 * @note The position is at the resolution corresponding to the
 * selected step mode.
 * STEP_MODE_FULL                   : step
 * STEP_MODE_HALF                   : 1/2 step
 * STEP_MODE_1_4                    : 1/4 step
 * STEP_MODE_1_8                    : 1/8 step
 * STEP_MODE_1_16                   : 1/16 step
 * STEP_MODE_1_32                   : 1/32 step
 * STEP_MODE_1_64                   : 1/64 step
 * STEP_MODE_1_128                  : 1/128 step
 * STEP_MODE_1_256                  : 1/256 step
 */
void Stspin220_GoTo(uint8_t deviceId, int32_t targetPosition)
{
	motorDir_t direction;

	/* Exit from standby if needed */
	if (devicePrm.motionState == STANDBY)
		Stspin220_ExitDeviceFromStandby(deviceId);
	/* Deactivate motor if needed */
	else if (devicePrm.motionState != INACTIVE)
		Stspin220_HardHiZ(deviceId);

	if (targetPosition > devicePrm.currentPosition) {
		devicePrm.stepsToTake = (uint32_t)(targetPosition - devicePrm.currentPosition);
		if (devicePrm.stepsToTake < (STSPIN220_POSITION_RANGE >> 1)) {
			direction = FORWARD;
		} else {
			direction = BACKWARD;
			devicePrm.stepsToTake = STSPIN220_POSITION_RANGE - devicePrm.stepsToTake;
		}
	} else {
		devicePrm.stepsToTake = (uint32_t)(devicePrm.currentPosition - targetPosition);
		if (devicePrm.stepsToTake < (STSPIN220_POSITION_RANGE >> 1)) {
			direction = BACKWARD;
		} else {
			direction = FORWARD;
			devicePrm.stepsToTake = STSPIN220_POSITION_RANGE - devicePrm.stepsToTake;
		}
	}
	if (devicePrm.stepsToTake != 0) {
		devicePrm.commandExecuted = MOVE_CMD;

		/* Direction setup */
		Stspin220_SetDirection(deviceId, direction);

		Stspin220_ComputeSpeedProfile(deviceId, devicePrm.stepsToTake);

		/* Motor activation */
		Stspin220_StartMovement(deviceId);
	}
}

/**
 * @brief Move the motor to the absolute position
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] targetPosition 32 bit signed value position
 * @retval None
 * @note The position is at the resolution corresponding to the
 * selected step mode.
 * STEP_MODE_FULL                   : step
 * STEP_MODE_HALF                   : 1/2 step
 * STEP_MODE_1_4                    : 1/4 step
 * STEP_MODE_1_8                    : 1/8 step
 * STEP_MODE_1_16                   : 1/16 step
 * STEP_MODE_1_32                   : 1/32 step
 * STEP_MODE_1_64                   : 1/64 step
 * STEP_MODE_1_128                  : 1/128 step
 * STEP_MODE_1_256                  : 1/256 step
 */
void Stspin220_GoToDir(uint8_t deviceId, motorDir_t direction, int32_t targetPosition)
{
	/* Exit from standby if needed */
	if (devicePrm.motionState == STANDBY)
		Stspin220_ExitDeviceFromStandby(deviceId);
	/* Deactivate motor if needed */
	else if (devicePrm.motionState != INACTIVE)
		Stspin220_HardHiZ(deviceId);

	if (direction != BACKWARD) {
		if (targetPosition > devicePrm.currentPosition)
			devicePrm.stepsToTake = (uint32_t)(targetPosition - devicePrm.currentPosition);
		else
			devicePrm.stepsToTake = STSPIN220_POSITION_RANGE + (uint32_t)(targetPosition - devicePrm.currentPosition);
	}
	else {
		if (targetPosition > devicePrm.currentPosition)
			devicePrm.stepsToTake = STSPIN220_POSITION_RANGE + (uint32_t)(devicePrm.currentPosition - targetPosition);
		else
			devicePrm.stepsToTake = (uint32_t)(devicePrm.currentPosition - targetPosition);
	}

	if (devicePrm.stepsToTake != 0) {
		devicePrm.commandExecuted = MOVE_CMD;

		/* Direction setup */
		Stspin220_SetDirection(deviceId, direction);

		Stspin220_ComputeSpeedProfile(deviceId, devicePrm.stepsToTake);

		/* Motor activation */
		Stspin220_StartMovement(deviceId);
	}
}

/**
 * @brief  Immediately stop the motor and disable the power bridge
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 */
void Stspin220_HardHiZ(uint8_t deviceId)
{
	Stspin220_SetParamsInactive();

	/* Let the PWM REF and bridges enabled at least for DISABLE_DELAY time */
	/* after the last step clock rising edge triggering the last step */
	Stspin220_Board_Delay(DISABLE_DELAY);

	/* Set reference voltage to 0 */
	Stspin220_SetTorque(deviceId, CURRENT_TORQUE, 0);

	/* Disable power bridges */
	Stspin220_Board_Disable();
}

/**
 * @brief  Immediately stop the motor
 * and either set holding torque when stop mode is HOLD_MODE,
 * or call Stspin220_HardHiz function when stop mode is HIZ_MODE,
 * or call Stspin220_PutDeviceInStandby function when stop mode is STANDBY_MODE
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 */
void Stspin220_HardStop(uint8_t deviceId)
{
	if (devicePrm.stopMode == HOLD_MODE) {
		Stspin220_SetParamsInactive();
		/* Set holding torque */
		Stspin220_ApplyTorque(deviceId, HOLD_TORQUE);
	} else if (devicePrm.stopMode == HIZ_MODE) {
		Stspin220_HardHiZ(deviceId);
	} else {
		Stspin220_PutDeviceInStandby(deviceId);
	}
}

/**
 * @brief  Moves the motor of the specified number of steps
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] stepCount Number of steps to perform
 * @retval None
 */
void Stspin220_Move(uint8_t deviceId, motorDir_t direction, uint32_t stepCount)
{
	/* Exit from standby if needed */
	if (devicePrm.motionState == STANDBY)
		Stspin220_ExitDeviceFromStandby(deviceId);

	/* Deactivate motor if needed */
	else if (devicePrm.motionState != INACTIVE)
		Stspin220_HardHiZ(deviceId);

	if (stepCount != 0) {
		devicePrm.stepsToTake = stepCount;
		devicePrm.commandExecuted = MOVE_CMD;

		/* Direction setup */
		Stspin220_SetDirection(deviceId, direction);

		Stspin220_ComputeSpeedProfile(deviceId, stepCount);

		/* Motor activation */
		Stspin220_StartMovement(deviceId);
	}
}

/**
 * @brief Put STSPIN220 device in standby (low power consumption)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 */
void Stspin220_PutDeviceInStandby(uint8_t deviceId)
{
	/* Stop movement */
	Stspin220_HardHiZ(deviceId);

	/* Enter standby */
	Stspin220_Board_Reset();

	devicePrm.motionState = STANDBY;
}

/**
 * @brief  Runs the motor. It will accelerate from the min
 * speed up to the max speed by using the device acceleration.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 */
void Stspin220_Run(uint8_t deviceId, motorDir_t direction)
{
	if (devicePrm.commandExecuted != RUN_CMD) {
		/* Exit from standby if needed */
		if (devicePrm.motionState == STANDBY)
			Stspin220_ExitDeviceFromStandby(deviceId);
		/* Deactivate motor if needed */
		else if (devicePrm.motionState != INACTIVE)
			Stspin220_HardHiZ(deviceId);

		/* Direction setup */
		Stspin220_SetDirection(deviceId, direction);
		devicePrm.commandExecuted = RUN_CMD;
		/* Motor activation */
		Stspin220_StartMovement(deviceId);
	} else {
		Stspin220_SetDirection(deviceId, direction);
	}
}

/**
 * @brief  Changes the acceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] newAcc New acceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed if the device is executing
 * a MOVE or GOTO command (but it can be used during a RUN command)
 */
bool Stspin220_SetAcceleration(uint8_t deviceId, float newAcc)
{
	UNUSED(deviceId);
	if (newAcc && (devicePrm.motionState >= INACTIVE || devicePrm.commandExecuted & RUN_CMD)) {
		devicePrm.acceleration = newAcc;
		return  true;
	}
	return false;
}

/**
 * @brief  Changes the deceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] newDec New deceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed if the device is executing
 * a MOVE or GOTO command (but it can be used during a RUN command)
 */
bool Stspin220_SetDeceleration(uint8_t deviceId, float newDec)
{
	UNUSED(deviceId);
	if (newDec && (devicePrm.motionState >= INACTIVE || devicePrm.commandExecuted & RUN_CMD)) {
		devicePrm.deceleration = newDec;
		return true;
	}
	return false;
}

/**
 * @brief  Specifies the direction
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is applied if the device
 * is in INACTIVE or STANDBY state or if the device is
 * executing a run command
 * @retval None
 */
void Stspin220_SetDirection(uint8_t deviceId, motorDir_t dir)
{
	UNUSED(deviceId);
	if (devicePrm.motionState == INACTIVE || devicePrm.motionState == STANDBY) {
		devicePrm.direction = dir;
		Stspin220_Board_SetDirectionGpio(dir);
	}
	else if (devicePrm.commandExecuted & RUN_CMD && dir != devicePrm.direction) {
		devicePrm.commandExecuted |= STSPIN220_DIR_CHANGE_BIT_MASK;
	}
}

/**
 * @brief  Specifies the direction
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] dir FORWARD or BACKWARD
 * @retval None
 */
void Stspin220_HardSetDirection(uint8_t deviceId, motorDir_t dir)
{
	UNUSED(deviceId);
	if (devicePrm.direction != dir) {
		devicePrm.direction = dir;
		Stspin220_Board_SetDirectionGpio(dir);
	}
}

/**
 * @brief  Set home position
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] homePosition home absolute position in steps
 * @retval None
 * @note The position is at the resolution corresponding to the
 * selected step mode.
 * STEP_MODE_FULL                   : step
 * STEP_MODE_HALF                   : 1/2 step
 * STEP_MODE_1_4                    : 1/4 step
 * STEP_MODE_1_8                    : 1/8 step
 * STEP_MODE_1_16                   : 1/16 step
 * STEP_MODE_1_32                   : 1/32 step
 * STEP_MODE_1_64                   : 1/64 step
 * STEP_MODE_1_128                  : 1/128 step
 * STEP_MODE_1_256                  : 1/256 step
 */
void Stspin220_SetHome(uint8_t deviceId, int32_t homePosition)
{
	UNUSED(deviceId);
	devicePrm.currentPosition -= homePosition;
}

/**
 * @brief  Set current position to be the Mark position
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] markPosition mark absolute position in steps
 * @retval None
 * @note The position is at the resolution corresponding to the
 * selected step mode.
 * STEP_MODE_FULL                   : step
 * STEP_MODE_HALF                   : 1/2 step
 * STEP_MODE_1_4                    : 1/4 step
 * STEP_MODE_1_8                    : 1/8 step
 * STEP_MODE_1_16                   : 1/16 step
 * STEP_MODE_1_32                   : 1/32 step
 * STEP_MODE_1_64                   : 1/64 step
 * STEP_MODE_1_128                  : 1/128 step
 * STEP_MODE_1_256                  : 1/256 step
 */
void Stspin220_SetMark(uint8_t deviceId, int32_t markPosition)
{
	UNUSED(deviceId);
	devicePrm.markPosition = markPosition;
}

/**
 * @brief  Changes the max speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] newMaxSpeed New max speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing
 * a MOVE or GOTO command (but it can be used during a RUN command).
 */
bool Stspin220_SetMaxSpeed(uint8_t deviceId, float newMaxSpeed)
{
	UNUSED(deviceId);
	if (newMaxSpeed >= STSPIN220_MIN_STCK_FREQ &&
	   (newMaxSpeed <= STSPIN220_MAX_STCK_FREQ ||
	    (devicePrm.torqueBoostEnable && newMaxSpeed / (1 << devicePrm.stepMode) <= STSPIN220_MAX_STCK_FREQ)) &&
	   devicePrm.minSpeed <= newMaxSpeed &&
	   (devicePrm.motionState >= INACTIVE || devicePrm.commandExecuted & RUN_CMD))
	{
		devicePrm.maxSpeed = newMaxSpeed;
		return true;
	}
	return false;
}

/**
 * @brief  Changes the min speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] newMinSpeed New min speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing
 * a MOVE or GOTO command (but it can be used during a RUN command).
 */
bool Stspin220_SetMinSpeed(uint8_t deviceId, float newMinSpeed)
{
	UNUSED(deviceId);
	if (newMinSpeed >= STSPIN220_MIN_STCK_FREQ &&
		newMinSpeed <= STSPIN220_MAX_STCK_FREQ &&
		newMinSpeed <= devicePrm.maxSpeed &&
		(devicePrm.motionState >= INACTIVE || devicePrm.commandExecuted & RUN_CMD))
	{
		devicePrm.minSpeed = newMinSpeed;
		return true;
	}
	return false;
}

/**
 * @brief  Sets the number of devices to be used
 * @param[in] nbDevices (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval true if successfull, false if failure, attempt to set a number of
 * devices greater than MAX_NUMBER_OF_DEVICES
 */
bool Stspin220_SetNbDevices(uint8_t nbDevices)
{
	if (nbDevices <= MAX_NUMBER_OF_DEVICES) {
		stspin220NumberOfDevices = nbDevices;
		stspin220DriverInstance = 0;
		return true;
	}
	return false;
}

/**
 * @brief Set the stepping mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] stepMode from full step to 1/256 microstep
 * as specified in enum motorStepMode_t
 * @retval true if the command is successfully executed, else false
 */
bool Stspin220_SetStepMode(uint8_t deviceId, motorStepMode_t stepMode)
{
	/* Eventually deactivate motor */
	if (devicePrm.motionState != INACTIVE && devicePrm.motionState != STANDBY)
		Stspin220_HardHiZ(deviceId);

	/* Enter standby */
	Stspin220_Board_Reset();

	/* Reset the microstepping sequencer position */
	devicePrm.sequencerPosition = 0;

	/* Reset current and mark positions */
	devicePrm.currentPosition = 0;
	devicePrm.markPosition = 0;

	/* Set the step mode */
	return Stspin220_SetStepModeWithoutReset(deviceId, stepMode);
}

/**
 * @brief Select the mode to stop the motor.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] stopMode HOLD_MODE to let power bridge enabled
 * @retval None
 */
void Stspin220_SetStopMode(uint8_t deviceId, motorStopMode_t stopMode)
{
	UNUSED(deviceId);
	devicePrm.stopMode = stopMode;
}

/**
 * @brief Set the torque of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] torqueMode Torque mode as specified in enum motorTorqueMode_t
 * @param[in] torqueValue in % (from 0 to 100)
 * @retval None
 * @note
 */
void Stspin220_SetTorque(uint8_t deviceId, motorTorqueMode_t torqueMode, uint8_t torqueValue)
{
	UNUSED(deviceId);
	devicePrm.updateTorque = true;
	if (torqueValue > 100)
		torqueValue = 100;
	switch (torqueMode) {
	case ACC_TORQUE:
		devicePrm.accelTorque = torqueValue;
		break;
	case DEC_TORQUE:
		devicePrm.decelTorque = torqueValue;
		break;
	case RUN_TORQUE:
		devicePrm.runTorque = torqueValue;
		break;
	case HOLD_TORQUE:
		devicePrm.holdTorque = torqueValue;
		if (devicePrm.motionState != INACTIVE)
			break;
		// fall through
	case CURRENT_TORQUE:
		devicePrm.currentTorque = torqueValue;
		Stspin220_Board_PwmRefSetFreqAndDutyCycle(devicePrm.refPwmFreq, torqueValue);
		// fall through
	default:
		devicePrm.updateTorque = false;
		break; // ignore error
	}
}

/**
 * @brief Enable or disable the torque boost feature
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] enable true to enable torque boost, false to disable
 * @retval None
 */
void Stspin220_SetTorqueBoostEnable(uint8_t deviceId, bool enable)
{
	UNUSED(deviceId);
	devicePrm.torqueBoostEnable = enable;
}

/**
 * @brief Set the torque boost threshold
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] speedThreshold speed threshold above which the step mode is
 * changed to full step
 * @retval None
 */
void Stspin220_SetTorqueBoostThreshold(uint8_t deviceId, float speedThreshold)
{
	UNUSED(deviceId);
	devicePrm.torqueBoostSpeedThreshold = speedThreshold;
}

/**
 * @brief  Stops the motor by using the device deceleration
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed if the device is in INACTIVE,
 * STANDBYTOINACTIVE or STANDBY state.
 */
bool Stspin220_SoftStop(uint8_t deviceId)
{
	UNUSED(deviceId);
	if (devicePrm.motionState < INACTIVE) {
		devicePrm.commandExecuted |= STSPIN220_SOFT_STOP_BIT_MASK;
		return true;
	}
	return false;
}

/**
 * @brief Get the frequency of REF PWM of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval the frequency of REF PWM in Hz
 * @note
 */
uint32_t Stspin220_VrefPwmGetFreq(uint8_t deviceId)
{
	UNUSED(deviceId);
	return devicePrm.refPwmFreq;
}

/**
 * @brief Set the frequency of REF PWM of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] newFreq in Hz
 * @retval None
 * @note
 */
void Stspin220_VrefPwmSetFreq(uint8_t deviceId, uint32_t newFreq)
{
	UNUSED(deviceId);
	devicePrm.refPwmFreq = newFreq;
	Stspin220_Board_PwmRefSetFreqAndDutyCycle(newFreq, devicePrm.currentTorque);
}

/**
 * @brief  Locks until the device state becomes Inactive
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 */
void Stspin220_WaitWhileActive(uint8_t deviceId)
{
	UNUSED(deviceId);
	while (devicePrm.motionState < INACTIVE || toggleOdd);
}

/**
 * @}
 */

/** @addtogroup Stspin220_Private_Functions
 * @{
 */
/**
 * @brief Updates the current speed of the device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] newSpeed in pps
 * @retval None
 */
static void Stspin220_ApplySpeed(uint8_t deviceId, float newSpeed)
{
	UNUSED(deviceId);
	if (devicePrm.torqueBoostEnable) {
		if (devicePrm.stepMode > STEP_MODE_1_256)
			Stspin220_ErrorHandler(STSPIN220_ERROR_APPLY_SPEED);
		if ((devicePrm.commandExecuted & STSPIN220_MOVE_BIT_MASK) != MOVE_CMD ||
				devicePrm.stepsToTake - devicePrm.relativePos >= (1U << devicePrm.stepModeLatched)) {
			if (devicePrm.stepMode != STEP_MODE_FULL &&
				newSpeed / (1 << devicePrm.stepModeLatched) > devicePrm.torqueBoostSpeedThreshold &&
				(devicePrm.sequencerPosition & 0xFF) == 0X80)
			{
				Stspin220_Board_SetFullStep();
				devicePrm.stepMode = STEP_MODE_FULL;
				newSpeed /= 1 << devicePrm.stepModeLatched;
			}
		} else if (newSpeed <= devicePrm.torqueBoostSpeedThreshold && devicePrm.stepModeLatched != STEP_MODE_FULL) {
			Stspin220_Board_UnsetFullStep();
			devicePrm.stepMode = devicePrm.stepModeLatched;
			newSpeed *= 1 << devicePrm.stepModeLatched;
		}
	} else if (devicePrm.stepMode != devicePrm.stepModeLatched) {
		// torqueBoostEnable has just been disabled
		Stspin220_Board_UnsetFullStep();
		devicePrm.stepMode = devicePrm.stepModeLatched;
		newSpeed *= 1 << devicePrm.stepModeLatched;
	}

	if (newSpeed < STSPIN220_MIN_STCK_FREQ)
		newSpeed = STSPIN220_MIN_STCK_FREQ;

	if (newSpeed > STSPIN220_MAX_STCK_FREQ)
		newSpeed = STSPIN220_MAX_STCK_FREQ;

	devicePrm.speed = newSpeed;
	Stspin220_Board_TimStckSetFreq(newSpeed);
}

/**
 * @brief  Set parameters of the device to inactive before entering a stop mode
 * @retval None
 */
static void Stspin220_SetParamsInactive(void)
{
	/* Set inactive state */
	devicePrm.motionState = INACTIVE;

	/* Disable step clock */
	Stspin220_Board_TimStckStop(&toggleOdd);

	/* Comeback to nominal step mode */
	if (devicePrm.stepModeLatched != devicePrm.stepMode) {
		Stspin220_Board_UnsetFullStep();
		devicePrm.stepMode = devicePrm.stepModeLatched;
	}

	devicePrm.commandExecuted = NO_CMD;
	devicePrm.stepsToTake = 0;
	devicePrm.speed = 0;
}

/**
 * @brief Apply the set torque to the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] torqueMode torque mode
 * @retval None
 * @note
 */
static void Stspin220_ApplyTorque(uint8_t deviceId, motorTorqueMode_t torqueMode)
{
	UNUSED(deviceId);
	uint8_t torqueValue = 0;
	devicePrm.updateTorque = false;
	switch (torqueMode) {
	case ACC_TORQUE:
		devicePrm.currentTorque = devicePrm.accelTorque;
		break;
	case DEC_TORQUE:
		devicePrm.currentTorque = devicePrm.decelTorque;
		break;
	case RUN_TORQUE:
		devicePrm.currentTorque = devicePrm.runTorque;
		break;
	case HOLD_TORQUE:
		devicePrm.currentTorque = devicePrm.holdTorque;
		break;
	case CURRENT_TORQUE:
		break;
	default:
		return; // ignore error
	}
	torqueValue = devicePrm.currentTorque;
	Stspin220_Board_PwmRefSetFreqAndDutyCycle(devicePrm.refPwmFreq, torqueValue);
}

/**
 * @brief  Computes the speed profile according to the number of steps to move
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] nbSteps number of steps to perform
 * @retval None
 * @note Using the acceleration and deceleration of the device,
 * this function determines the duration in steps of the acceleration,
 * steady and deceleration phases.
 * If the total number of steps to perform is big enough, a trapezoidal move
 * is performed (i.e. there is a steady phase where the motor runs at the maximum
 * speed.
 * Else, a triangular move is performed (no steady phase: the maximum speed is never
 * reached.
 */
static void Stspin220_ComputeSpeedProfile(uint8_t deviceId, uint32_t nbSteps)
{
	UNUSED(deviceId);
	const double acc = devicePrm.acceleration;
	const double dec = devicePrm.deceleration;
	const double kntcFctr = ((double)devicePrm.maxSpeed + devicePrm.minSpeed) *
							((double)devicePrm.maxSpeed - devicePrm.minSpeed) / 2.0;

	uint32_t reqAccSteps = (uint32_t)(kntcFctr / acc + 0.5);
	uint32_t reqDecSteps = (uint32_t)(kntcFctr / dec + 0.5);

	if (reqAccSteps + reqDecSteps > nbSteps) {
		/* Triangular move  */
		reqDecSteps = (uint32_t)(nbSteps * dec / (acc + dec) + 0.5);
		devicePrm.endAccPos = reqDecSteps != 0 ? reqDecSteps - 1 : 0;
		devicePrm.startDecPos = reqDecSteps;
	} else {
		/* Trapezoidal move */
		/* accelerating phase to endAccPos */
		/* steady phase from  endAccPos to startDecPos */
		/* decelerating from startDecPos to stepsToTake*/
		devicePrm.endAccPos = reqAccSteps;
		devicePrm.startDecPos = nbSteps - reqDecSteps - 1;
	}
}

/**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @retval None
 */
static void Stspin220_FlagInterruptHandler(void)
{
	if (flagInterruptCallback != nullptr)
		flagInterruptCallback();
}

/**
 * @brief  Set the parameters of the device whose values are not defined in
 * stspin220_target_config.h
 * @retval None
 */
static void Stspin220_SetDeviceParamsOtherValues(void)
{
	devicePrm.currentPosition = 0;
	devicePrm.sequencerPosition = 0;
	devicePrm.endAccPos = 0;
	devicePrm.relativePos = 0;
	devicePrm.startDecPos = 0;
	devicePrm.stepsToTake = 0;
	devicePrm.updateTorque = false;
	devicePrm.speed = 0;
	devicePrm.commandExecuted = NO_CMD;
	devicePrm.direction = FORWARD;
	if ((devicePrm.torqueBoostEnable && devicePrm.torqueBoostSpeedThreshold > STSPIN220_MAX_STCK_FREQ) ||
		devicePrm.minSpeed > devicePrm.maxSpeed)
		Stspin220_ErrorHandler(STSPIN220_ERROR_INIT);
}

/**
 * @brief  Set the parameters of the device to values of initDevicePrm structure
 * @param pInitDevicePrm structure containing values to initialize the device
 * parameters
 * @retval None
 */
static void Stspin220_SetDeviceParamsToGivenValues(const Stspin220_Init_t *pInitDevicePrm)
{
	static const uint8_t deviceId = 0;
	devicePrm.motionState = STANDBY;

	if (!Stspin220_SetAcceleration(deviceId, pInitDevicePrm->acceleration))
		Stspin220_ErrorHandler(STSPIN220_ERROR_SET_ACCELERATION);
	if (!Stspin220_SetDeceleration(deviceId, pInitDevicePrm->deceleration))
		Stspin220_ErrorHandler(STSPIN220_ERROR_SET_DECELERATION);
	if (!Stspin220_SetMaxSpeed(deviceId, pInitDevicePrm->maxSpeed))
		Stspin220_ErrorHandler(STSPIN220_ERROR_SET_MAX_SPEED);
	if (!Stspin220_SetMinSpeed(deviceId, pInitDevicePrm->minSpeed))
		Stspin220_ErrorHandler(STSPIN220_ERROR_SET_MIN_SPEED);

	Stspin220_VrefPwmSetFreq(deviceId, pInitDevicePrm->vrefPwmFreq);
	Stspin220_SetTorque(deviceId, ACC_TORQUE, pInitDevicePrm->accelTorque);
	Stspin220_SetTorque(deviceId, DEC_TORQUE, pInitDevicePrm->decelTorque);
	Stspin220_SetTorque(deviceId, RUN_TORQUE, pInitDevicePrm->runTorque);
	Stspin220_SetTorque(deviceId, HOLD_TORQUE, pInitDevicePrm->holdTorque);
	devicePrm.torqueBoostEnable = pInitDevicePrm->torqueBoostEnable;
	devicePrm.torqueBoostSpeedThreshold = pInitDevicePrm->torqueBoostSpeedThreshold;
	Stspin220_SetStopMode(deviceId, pInitDevicePrm->stopMode);

	Stspin220_SetDeviceParamsOtherValues();

	/* Set predefined step mode */
	/* Standby-reset deactivation included to latch the MODEX inputs */
	if (!Stspin220_SetStepMode(deviceId, pInitDevicePrm->stepMode))
		Stspin220_ErrorHandler(STSPIN220_ERROR_SET_STEP_MODE);
}

/**
 * @brief  Sets the parameters of the device to predefined values
 * from stspin220_target_config.h
 * @retval None
 */
static void Stspin220_SetDeviceParamsToPredefinedValues(void)
{
	static const uint8_t deviceId = 0;
	devicePrm.motionState = STANDBY;

	if (!Stspin220_SetAcceleration(deviceId, STSPIN220_CONF_PARAM_ACC))
		Stspin220_ErrorHandler(STSPIN220_ERROR_SET_ACCELERATION);
	if (!Stspin220_SetDeceleration(deviceId, STSPIN220_CONF_PARAM_DEC))
		Stspin220_ErrorHandler(STSPIN220_ERROR_SET_DECELERATION);
	if (!Stspin220_SetMaxSpeed(deviceId, STSPIN220_CONF_PARAM_RUNNING_SPEED))
		Stspin220_ErrorHandler(STSPIN220_ERROR_SET_MAX_SPEED);
	if (!Stspin220_SetMinSpeed(deviceId, STSPIN220_CONF_PARAM_MIN_SPEED))
		Stspin220_ErrorHandler(STSPIN220_ERROR_SET_MIN_SPEED);

	Stspin220_VrefPwmSetFreq(deviceId, STSPIN220_CONF_PARAM_REF_PWM_FREQUENCY);
	Stspin220_SetTorque(deviceId, ACC_TORQUE, STSPIN220_CONF_PARAM_ACC_TORQUE);
	Stspin220_SetTorque(deviceId, DEC_TORQUE, STSPIN220_CONF_PARAM_DEC_TORQUE);
	Stspin220_SetTorque(deviceId, RUN_TORQUE, STSPIN220_CONF_PARAM_RUNNING_TORQUE);
	Stspin220_SetTorque(deviceId, HOLD_TORQUE, STSPIN220_CONF_PARAM_HOLDING_TORQUE);
	devicePrm.torqueBoostEnable = STSPIN220_CONF_PARAM_TORQUE_BOOST_EN;
	devicePrm.torqueBoostSpeedThreshold = STSPIN220_CONF_PARAM_TORQUE_BOOST_TH;
	Stspin220_SetStopMode(deviceId, STSPIN220_CONF_PARAM_AUTO_HIZ_STOP);

	Stspin220_SetDeviceParamsOtherValues();

	/* Set predefined step mode */
	/* Standby-reset deactivation included to latch the MODEX inputs */
	if (!Stspin220_SetStepMode(deviceId, STSPIN220_CONF_PARAM_STEP_MODE))
		Stspin220_ErrorHandler(STSPIN220_ERROR_SET_STEP_MODE);
}

/**
 * @brief Set the stepping mode without reset
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] stepMode from full step to 1/256 microstep
 * as specified in enum motorStepMode_t
 * @retval true if the command is successfully executed, else false
 */
static bool Stspin220_SetStepModeWithoutReset(uint8_t deviceId, motorStepMode_t stepMode)
{
	UNUSED(deviceId);

	/* Set the mode pins to the levels corresponding to the selected step mode */
	switch (stepMode) {
	case STEP_MODE_FULL:
		Stspin220_Board_SetFullStep();
		break;
	case STEP_MODE_HALF:
		Stspin220_Board_SetModePins(1, 0, 1, 0);
		break;
	case STEP_MODE_1_4:
		Stspin220_Board_SetModePins(0, 1, 0, 1);
		break;
	case STEP_MODE_1_8:
		Stspin220_Board_SetModePins(1, 1, 1, 0);
		break;
	case STEP_MODE_1_16:
		Stspin220_Board_SetModePins(1, 1, 1, 1);
		break;
	case STEP_MODE_1_32:
		Stspin220_Board_SetModePins(0, 0, 0, 1);
		break;
	case STEP_MODE_1_64:
		Stspin220_Board_SetModePins(1, 1, 0, 1);
		break;
	case STEP_MODE_1_128:
		Stspin220_Board_SetModePins(0, 0, 1, 0);
		break;
	case STEP_MODE_1_256:
		Stspin220_Board_SetModePins(1, 1, 0, 0);
		break;
	default:
		return false;
	}

	/* Store step mode */
	devicePrm.stepMode = stepMode;
	devicePrm.stepModeLatched = stepMode;

	/* Wait */
	Stspin220_Board_Delay(SELECT_STEP_MODE_DELAY);

	/* Exit standby, selected step mode is latched */
	Stspin220_Board_ReleaseReset();

	/* Let a delay after reset release and step mode latching*/
	Stspin220_Board_Delay(AFTER_STANDBY_EXIT_DEAD_TIME);

	/* If full step mode is not selected, do not keep MODE1 = MODE2 = 0 */
	/* after the device quit the standby condition */
	if (stepMode != STEP_MODE_FULL)
		Stspin220_Board_UnsetFullStep();

	return true;
}

/**
 * @brief Initialises the bridge parameters to start the movement
 * and enable the power bridge
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 */
static void Stspin220_StartMovement(uint8_t deviceId)
{
	/* Enable STSPIN220 powerstage */
	Stspin220_Enable(deviceId);
	toggleOdd = false;
	devicePrm.relativePos = 0;
	if (devicePrm.endAccPos == 0 && devicePrm.commandExecuted != RUN_CMD) {
		devicePrm.motionState = DECELERATING;
		Stspin220_ApplyTorque(deviceId, DEC_TORQUE);
	}
	else {
		devicePrm.motionState = ACCELERATING;
		Stspin220_ApplyTorque(deviceId, ACC_TORQUE);
	}
	Stspin220_Board_PwmRefStart();
	/* Initialize the step clock timer */
	Stspin220_Board_TimStckInit();
	/* Program the step clock */
	Stspin220_Board_TimStckCompareInit();
	Stspin220_ApplySpeed(deviceId, devicePrm.minSpeed);
	Stspin220_Board_TimStckStart();
}

/**
 * @brief  Handles the device state machine at each pulse
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 * @note Must only be called by the timer ISR
 */
static void Stspin220_StepClockHandler(uint8_t deviceId)
{
	float speed = devicePrm.speed;
	const int16_t sequencerStep = 1U << (STEP_MODE_1_256 - devicePrm.stepMode);
	const uint16_t boostFctr = 1U << (devicePrm.stepModeLatched - devicePrm.stepMode);

	toggleOdd = !toggleOdd; // toggleOdd == 0: Step completed; ToggleOdd = 1: Middle of step

	if (devicePrm.motionState == STANDBYTOINACTIVE) {
		if (toggleOdd)
			devicePrm.sequencerPosition -= sequencerStep;
		if (!toggleOdd && devicePrm.sequencerPosition == 0)
			Stspin220_Board_TimStckStop(&toggleOdd);
		else
			Stspin220_Board_TimStckSetFreq(STSPIN220_MAX_STCK_FREQ);
		return;
	}

	if (!toggleOdd) {
		/* Increment the relative position */
		devicePrm.relativePos += boostFctr;
		/* Increment the current position */
		if (devicePrm.direction != BACKWARD) {
			devicePrm.currentPosition += boostFctr;
			devicePrm.sequencerPosition += sequencerStep;
			if (devicePrm.sequencerPosition >= SEQUENCER_MAX_VALUE + 1)
				devicePrm.sequencerPosition -= SEQUENCER_MAX_VALUE + 1;
		} else {
			devicePrm.currentPosition -= boostFctr;
			devicePrm.sequencerPosition -= sequencerStep;
			if (devicePrm.sequencerPosition < 0)
				devicePrm.sequencerPosition += SEQUENCER_MAX_VALUE + 1;
		}

		switch (devicePrm.motionState) {
		case ACCELERATING:
			if (devicePrm.commandExecuted & (STSPIN220_SOFT_STOP_BIT_MASK | STSPIN220_DIR_CHANGE_BIT_MASK) ||
				(devicePrm.commandExecuted == MOVE_CMD && devicePrm.relativePos >= devicePrm.startDecPos))
			{
				devicePrm.motionState = DECELERATING;
				/* Apply decelerating torque */
				Stspin220_ApplyTorque(deviceId, DEC_TORQUE);
			} else if (speed >= devicePrm.maxSpeed / boostFctr ||
					   (devicePrm.commandExecuted == MOVE_CMD && devicePrm.relativePos >= devicePrm.endAccPos))
			{
				devicePrm.motionState = STEADY;
				/* Apply running torque */
				Stspin220_ApplyTorque(deviceId, RUN_TORQUE);
			} else {
				/* Go on accelerating */
				if (speed == 0.0) // Cannot happen, but just in case
					speed = STSPIN220_MIN_STCK_FREQ;
				speed += devicePrm.acceleration / (speed * boostFctr);
				if (speed > devicePrm.maxSpeed / boostFctr)
					speed = devicePrm.maxSpeed / boostFctr;
				if (devicePrm.updateTorque)
					/* Apply new accelerating torque */
					Stspin220_ApplyTorque(deviceId, ACC_TORQUE);
			}
			break;
		case STEADY:
			if (devicePrm.updateTorque)
				/* Apply new running torque */
				Stspin220_ApplyTorque(deviceId, RUN_TORQUE);
			if (devicePrm.commandExecuted & (STSPIN220_SOFT_STOP_BIT_MASK | STSPIN220_DIR_CHANGE_BIT_MASK) ||
				(devicePrm.commandExecuted == MOVE_CMD && devicePrm.relativePos >= devicePrm.startDecPos) ||
				(devicePrm.commandExecuted == RUN_CMD && speed > devicePrm.maxSpeed / boostFctr))
			{
				devicePrm.motionState = DECELERATING;
				/* Apply decelerating torque */
				Stspin220_ApplyTorque(deviceId, DEC_TORQUE);
			} else if (devicePrm.commandExecuted == RUN_CMD && speed < devicePrm.maxSpeed / boostFctr) {
				devicePrm.motionState = ACCELERATING;
				/* Apply accelerating torque */
				Stspin220_ApplyTorque(deviceId, ACC_TORQUE);
			}
			break;
		case DECELERATING:
			if ((devicePrm.commandExecuted & (STSPIN220_SOFT_STOP_BIT_MASK | STSPIN220_DIR_CHANGE_BIT_MASK) &&
				speed <= devicePrm.minSpeed / boostFctr) ||
				(devicePrm.commandExecuted == MOVE_CMD && devicePrm.relativePos >= devicePrm.stepsToTake))
			{
				/* Motion process complete */
				if (devicePrm.commandExecuted & STSPIN220_DIR_CHANGE_BIT_MASK) {
					devicePrm.commandExecuted &= ~STSPIN220_DIR_CHANGE_BIT_MASK;
					if (devicePrm.direction == FORWARD)
						devicePrm.direction = BACKWARD;
					else
						devicePrm.direction = FORWARD;
					Stspin220_Board_SetDirectionGpio(devicePrm.direction);
					if (!(devicePrm.commandExecuted & STSPIN220_SOFT_STOP_BIT_MASK)) {
						devicePrm.motionState = ACCELERATING;
						/* Apply accelerating torque */
						Stspin220_ApplyTorque(deviceId, ACC_TORQUE);
						break;
					}
				}
				Stspin220_HardStop(deviceId);
			} else if (devicePrm.commandExecuted == RUN_CMD && speed <= devicePrm.maxSpeed / boostFctr) {
				devicePrm.motionState = STEADY;
				/* Apply running torque */
				Stspin220_ApplyTorque(deviceId, RUN_TORQUE);
			} else {
				/* Go on decelerating */
				if (speed == 0.0) // Cannot happen, but just in case
					speed = STSPIN220_MIN_STCK_FREQ;
				speed -= devicePrm.deceleration / (speed * boostFctr);
				if (speed < devicePrm.minSpeed / boostFctr)
					speed = devicePrm.minSpeed / boostFctr;
				if (devicePrm.updateTorque)
					/* Apply new decelerating torque */
					Stspin220_ApplyTorque(deviceId, DEC_TORQUE);
			}
			break;
		default:
			break;
		}
	}

	if (devicePrm.motionState < INACTIVE)
		Stspin220_ApplySpeed(deviceId, speed);
	else
		Stspin220_Board_TimStckStop(&toggleOdd);
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
