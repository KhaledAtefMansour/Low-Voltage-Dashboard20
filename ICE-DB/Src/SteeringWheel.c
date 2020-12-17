/*
 * SteeringWheel.c
 *
 *  Created on: Mar 20, 2020
 *      Author: Atef Mohammed
 */

#include "SteeringWheel.h"


uint8_t Steering_Wheel_Switches(void) {
	uint8_t Switches[8] = {

	HAL_GPIO_ReadPin(GPIOA, Active_State_Pin),		//active state switch read

	HAL_GPIO_ReadPin(GPIOA, Open_Wing_Pin),			//open wing switch read

	HAL_GPIO_ReadPin(GPIOA, Close_Wing_Pin), 		//close wing switch read

	HAL_GPIO_ReadPin(GPIOA, MS_Map_Pin), 			//MS map switch read

	HAL_GPIO_ReadPin(GPIOC, DRS_Switch_Pin), 		//DRS switch read

	HAL_GPIO_ReadPin(GPIOC, DRS_Enable_Pin), 		//DRS enable switch read

	HAL_GPIO_ReadPin(GPIOB, Up_Shift_Pin), 			//Up Shift switch read

	HAL_GPIO_ReadPin(GPIOB, Down_Shift_Pin)			//Down Shift switch read
			};
	uint8_t Switches_State =0 ;
	for (int i = 0; i < 8; i++) {

		Switches_State = Switches_State | Switches[i];
		Switches_State = Switches_State << 1;
	}
	return Switches_State;
}

void MSMapOut(void) {
	if (HAL_GPIO_ReadPin(MS_Map_GPIO_Port, MS_Map_Pin) == 1) {
		HAL_GPIO_WritePin(Map_Output_GPIO_Port, Map_Output_Pin, 1); //Map set
		HAL_Delay(500);
		HAL_GPIO_WritePin(Map_Output_GPIO_Port, Map_Output_Pin, 0); //Map Reset
		HAL_Delay(1500);
	}
}
