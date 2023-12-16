/*
 * functions_input.c
 *
 *  Created on: Dec 15, 2023
 *      Author: Love Mitteregger
 */
#include "pro1_funct.h"
#include "spi.h"
#include "gpio.h"
#include "FreeRTOS.h"

extern volatile uint8_t statusVehicle_N;
extern volatile uint8_t statusVehicle_S;
extern volatile uint8_t statusVehicle_E;
extern volatile uint8_t statusVehicle_W;

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
