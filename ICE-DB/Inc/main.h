/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define Map_Output_Pin 				GPIO_PIN_13
#define Map_Output_GPIO_Port		GPIOC
#define DRS_Switch_Pin 				GPIO_PIN_14
#define DRS_Switch_GPIO_Port 		GPIOC
#define Active_State_Pin 			GPIO_PIN_15
#define Active_State_GPIO_Port 		GPIOC
#define Up_Shift_Pin 				GPIO_PIN_2
#define Up_Shift_GPIO_Port 			GPIOB
#define Down_Shift_Pin 				GPIO_PIN_11
#define Down_Shift_GPIO_Port 		GPIOB
#define MS_Map_Pin 					GPIO_PIN_8
#define MS_Map_GPIO_Port 			GPIOA
#define Open_Wing_Pin 				GPIO_PIN_11
#define Open_Wing_GPIO_Port 		GPIOA
#define Close_Wing_Pin 				GPIO_PIN_12
#define Close_Wing_GPIO_Port 		GPIOA
#define DRS_Enable_Pin 				GPIO_PIN_7
#define DRS_Enable_GPIO_Port 		GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
