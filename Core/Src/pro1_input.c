/************************************************************************************
 *	pro1_input.c																	*
 ************************************************************************************
 *  Created on: Dec 15, 2023														*
 *      Author: Love Mitteregger													*
 ************************************************************************************
 *	MIT License																		*
 *																					*
 *	Copyright (c) 2023 Love Mitteregger												*
 *																					*
 *	Permission is hereby granted, free of charge, to any person obtaining a copy	*
 *	of this software and associated documentation files (the "Software"), to deal	*
 *	in the Software without restriction, including without limitation the rights	*
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell		*
 *	copies of the Software, and to permit persons to whom the Software is			*
 *	furnished to do so, subject to the following conditions:						*
 *																					*
 *	The above copyright notice and this permission notice shall be included in all	*
 *	copies or substantial portions of the Software.									*
 *																					*
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR		*
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,		*
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE		*
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER			*
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,	*
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE	*
 *	SOFTWARE.																		*
 *																					*
 ************************************************************************************/
#include "pro1_funct.h"
#include "spi.h"
#include "gpio.h"
#include "FreeRTOS.h"

// Declaration of vehicle status variables, volatile to ensure check when called
extern volatile uint8_t statusVehicle_N;
extern volatile uint8_t statusVehicle_S;
extern volatile uint8_t statusVehicle_E;
extern volatile uint8_t statusVehicle_W;

/* EXTERNAL INTERRUPT ISR FLAGS FOR BUTTONS
 * -------------------------------------------------------------------------------- *
 * The current input status of the buttons on the traffic board is set by the		*
 * xxx ISR, see file xxx.c for details.												*
 * -------------------------------------------------------------------------------- *
 */

/* POLLING FUNCTION FOR SWITCHES
 * -------------------------------------------------------------------------------- *
 * Sample the status of the event driven configured switches when called.			*
 * Each switch represents a vehicle on the traffic board.							*
 * 																					*
 * TL1_Car = SW1 = Hardware identifier for vehicle switch 1, WEST vehicle switch.	*
 * TL2_Car = SW2 = Hardware identifier for vehicle switch 2, SOUTH vehicle switch.	*
 * TL3_Car = SW3 = Hardware identifier for vehicle switch 3, EAST vehicle switch.	*
 * TL4_Car = SW4 = Hardware identifier for vehicle switch 4, NORTH vehicle switch.	*
 * -------------------------------------------------------------------------------- *
 */
uint8_t checkTraffic() {
	// Polling calls
	statusVehicle_N = HAL_GPIO_ReadPin(TL4_Car_GPIO_Port, TL4_Car_Pin);
	statusVehicle_S = HAL_GPIO_ReadPin(TL2_Car_GPIO_Port, TL2_Car_Pin);
	statusVehicle_E = HAL_GPIO_ReadPin(TL3_Car_GPIO_Port, TL3_Car_Pin);
	statusVehicle_W = HAL_GPIO_ReadPin(TL1_Car_GPIO_Port, TL1_Car_Pin);

	// If any vehicle switch is high, then there is active traffic to be considered
	if(	   statusVehicle_N == 1
		|| statusVehicle_S == 1
		|| statusVehicle_E == 1
		|| statusVehicle_W == 1	)
	{
		return 1;	// RReturn indication of existing active traffic
	} else {
		return 0;	// Return indication of no active traffic
	}
};
