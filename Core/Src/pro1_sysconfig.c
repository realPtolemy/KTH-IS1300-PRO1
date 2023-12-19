/************************************************************************************
 *	pro1_sysconfig.c																*
 ************************************************************************************
 *  Created on: Dec 8, 2023															*
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

/* SHIFT REGISTER BIT SIMULATION ARRAY
 * -------------------------------------------------------------------------------- *
 * Each element represents the bits stored in each respective shift register.		*
 * 3 elements, one per shift register, initialized to zero.							*
 *																					*
 * Used to store and push bits to the serially connected 74HC595 shift registers.	*
 *																					*
 * REG[3] = { U1, U2, U3 };															*
 * U1 = Hardware identifier for register 1, SOUTH Register							*
 * U2 = Hardware identifier for register 2, WEST Register							*
 * U3 = Hardware identifier for register 3, EAST Register							*
 * -------------------------------------------------------------------------------- *
 */
uint8_t REG[3] = {0x00, 0x00, 0x00};

// Set all traffic and pedestrian LEDs to their initial stage at startup.
void system_init(){
	// Initialize the enable pin and reset pin of the shift register
	HAL_GPIO_WritePin(IC595_Enable_GPIO_Port, IC595_Enable_Pin, 0);
	HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 1);

	// Set sim-array bits high for the LEDs that should be on at initialization
	REG[2] = 0b100001;  // U3 - Bits for TL4 and TL3 - NORTH TL Green & EAST TL Red
	REG[1] = 0b001100;  // U2 - Bits for TL2 and PL2 - SOUTH TL Green & NORTH PL Red
	REG[0] = 0b010001;  // U1 - Bits for TL1 and PL1 - WEST TL Red & WEST PL Green

	// Stage and latch shift registers by calling setReg
	setReg();

	// Quick booting delay to confirm that no sync issues occurs...
	HAL_Delay(1000);
}

