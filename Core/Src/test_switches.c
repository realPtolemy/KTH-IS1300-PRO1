/*
 * test_switches.c
 *
 *  Created on: Dec 4, 2023
 *      Author: Love Mitteregger
 */
#include "pro1_funct.h"
#include "pro1_test.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"

extern volatile uint8_t statusVehicle_N;
extern volatile uint8_t statusVehicle_S;
extern volatile uint8_t statusVehicle_E;
extern volatile uint8_t statusVehicle_W;

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


uint8_t checkTraffic() {
	statusVehicle_N = HAL_GPIO_ReadPin(TL4_Car_GPIO_Port, TL4_Car_Pin);
	statusVehicle_S = HAL_GPIO_ReadPin(TL2_Car_GPIO_Port, TL2_Car_Pin);
	statusVehicle_E = HAL_GPIO_ReadPin(TL3_Car_GPIO_Port, TL3_Car_Pin);
	statusVehicle_W = HAL_GPIO_ReadPin(TL1_Car_GPIO_Port, TL1_Car_Pin);

	if(	statusVehicle_N == 1
	 || statusVehicle_S == 1
	 || statusVehicle_E == 1
	 || statusVehicle_W == 1 )
	{
		return 1;
	} else {
		return 0;
	}
};
