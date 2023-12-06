/*
 * test_switches.c
 *
 *  Created on: Dec 4, 2023
 *      Author: Love Mitteregger
 */
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "test_switches.h"
#include "test_led.h"

void trafficSwitch_Test_NS() {
	// Read the state of the switches
	if(	HAL_GPIO_ReadPin(TL4_Car_GPIO_Port, TL4_Car_Pin) == 1
			|| HAL_GPIO_ReadPin(TL2_Car_GPIO_Port, TL2_Car_Pin) == 1 )
	{
		// If high, activate lights by calling the relevant traffic LED function
		trafficLED_Test_NS();
	} else {
		// Else, turn off lights by setting them to low
		HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 0);
		HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 1);

	}
}

void trafficSwitch_Test_EW() {
	if(	HAL_GPIO_ReadPin(TL1_Car_GPIO_Port, TL1_Car_Pin) == 1
	    || HAL_GPIO_ReadPin(TL3_Car_GPIO_Port, TL3_Car_Pin) == 1 )

	{
		// If high, activate lights by calling the relevant traffic LED function
		trafficLED_Test_EW();
	} else {
		// Turn off lights by setting them to low
		HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 0);
		HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 1);
	}
}

void pedestrianSwitch_Test_N() {
	// Read the state of the switches
	if ( HAL_GPIO_ReadPin(PL2_Switch_GPIO_Port, PL2_Switch_Pin) == 0 )
	{
		// If high, activate lights by calling the relevant pedestrian LED function
		pedestrianLED_Test_N();
	} else {
		// Turn off lights by setting them to low
		HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 0);
		HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 1);
	}
}

void pedestrianSwitch_Test_W() {
	// Read the state of the switches
	if ( HAL_GPIO_ReadPin(PL1_Switch_GPIO_Port, PL1_Switch_Pin) == 0 )
	{
		// If high, activate lights by calling the relevant pedestrian LED function
		pedestrianLED_Test_W();
	} else {
		// Turn off lights by setting them to low
		HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 0);
		HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 1);
	}
}


