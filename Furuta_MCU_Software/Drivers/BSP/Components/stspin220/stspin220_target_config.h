/**************************************************************************//**
  * @file    stspin220_target_config.h
  * @author  IPC Rennes
  * @version V1.4.0
  * @date    May 30th, 2018
  * @brief   Predefines values for the STSPIN220 registers
  * and for the devices parameters
  * @note    (C) COPYRIGHT 2018 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STSPIN220_TARGET_CONFIG_H
#define __STSPIN220_TARGET_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup STSPIN220
  * @{
  */   

/** @addtogroup Stspin220_Exported_Constants
  * @{
  */   
   
/** @defgroup Predefined_Stspin220_Registers_Values Predefined Stspin220 Registers Values
  * @{
  */   
   
/// The maximum number of devices
#define MAX_NUMBER_OF_DEVICES                                 (1)

/************************ Speed Profile  *******************************/

/// Acceleration rate in pulse/s2 (must be greater than 0)
#define STSPIN220_CONF_PARAM_ACC                               (480)
   
/// Deceleration rate in pulse/s2 (must be greater than 0)
#define STSPIN220_CONF_PARAM_DEC                               (480)
   
/// Running speed in pulse/s (8 pulse/s < Maximum speed <= 10 000 pulse/s )
#define STSPIN220_CONF_PARAM_RUNNING_SPEED                     (1600)
   
/// Minimum speed in pulse/s (8 pulse/s <= Minimum speed < 10 000 pulse/s)
#define STSPIN220_CONF_PARAM_MIN_SPEED                         (400)

/************************ Torque  *******************************/

/// Acceleration torque in % (from 0 to 100)
#define STSPIN220_CONF_PARAM_ACC_TORQUE                        (20)

/// Deceleration torque in % (from 0 to 100)
#define STSPIN220_CONF_PARAM_DEC_TORQUE                        (15)

/// Running torque in % (from 0 to 100)
#define STSPIN220_CONF_PARAM_RUNNING_TORQUE                    (10)

/// Holding torque in % (from 0 to 100)
#define STSPIN220_CONF_PARAM_HOLDING_TORQUE                    (25)

/// Torque boost speed enable
#define STSPIN220_CONF_PARAM_TORQUE_BOOST_EN                   (TRUE)

/// Torque boost speed threshold in fullstep/s
#define STSPIN220_CONF_PARAM_TORQUE_BOOST_TH                   (200)
    
/******************************* Others ***************************************/

/// Step mode selection settings
#define STSPIN220_CONF_PARAM_STEP_MODE                         (STEP_MODE_1_32)
   
/// Automatic HIZ STOP
#define STSPIN220_CONF_PARAM_AUTO_HIZ_STOP                     (HOLD_MODE)

/// REF PWM frequency (Hz)
#define STSPIN220_CONF_PARAM_REF_PWM_FREQUENCY                 (100000)

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

#endif /* __STSPIN220_TARGET_CONFIG_H */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
