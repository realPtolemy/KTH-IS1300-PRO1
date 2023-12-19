/************************************************************************************
 *	pro1_lights.c																	*
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
#include "FreeRTOS.h"

// Instantiate the shift register array
extern uint8_t REG[];
extern uint8_t togglePedestrianGreen;
extern uint8_t togglePedestrianBlue;

extern uint8_t buttonNorthFlag;
extern uint8_t buttonWestFlag;

extern uint8_t statusVehicle_N;
extern uint8_t statusVehicle_S;
extern uint8_t statusVehicle_E;
extern uint8_t statusVehicle_W;

extern uint8_t statusTraffic_NS;
extern uint8_t statusTraffic_EW;
extern uint8_t statusPedestrian_N;
extern uint8_t statusPedestrian_W;
extern uint8_t lastActive_NS;

extern long startTime;
extern long endTime;
extern long elapsedTime;

extern const TickType_t sysDelay;
extern const TickType_t toggleFreq;
extern const TickType_t pedestrianDelay;
extern const TickType_t walkingDelay;
extern const TickType_t safetyDelay;
extern const TickType_t greenDelay;
extern const TickType_t orangeDelay;
extern const TickType_t redDelayMax;

/* NOT IN USE - PRIMARILY USED FOR TESTING
// Stage the shift register with new bits
void stageReg(){
	HAL_SPI_Transmit(&hspi3, &REG[2], 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &REG[1], 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &REG[0], 1, HAL_MAX_DELAY);
}

// Latch and store new bits in shift register
void latchReg(){
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
}
 */

/* STAGING AND LATCHING LED BITS TO SHIFT REGISTERS
 * -------------------------------------------------------------------------------- *
 * The setReg function transmits set LED bits over the SPI-interface to	the three 	*
 * shift registers on the traffic board, known as staging. When the bits have been	*
 * staged for each shift register, the latch bit for the shift registers is set to 	*
 * high, and then reset to avoid immediate transfer from stage to storage register.	*
 * 																					*
 * As the registers are connected in series, the very first transmission will stage	*
 * The bits to the first register, as the second transmission is initiated the bits	*
 * stored in the first register proceeds by being shifted to the second register.	*
 * Likewise, when the third transmission is initiated, the bits stored in the first	*
 * register are shifted to the second register, while the bits stored in the second	*
 * register are shifted to the third register. 										*
 * 																					*
 * At last, all registers will have received their dedicated bits.					*
 * -------------------------------------------------------------------------------- *
 */
void setReg(){
	HAL_SPI_Transmit(&hspi3, &REG[2], 1, HAL_MAX_DELAY);	// Stage bits meant for third register
	HAL_SPI_Transmit(&hspi3, &REG[1], 1, HAL_MAX_DELAY);	// Stage bits meant for second register
	HAL_SPI_Transmit(&hspi3, &REG[0], 1, HAL_MAX_DELAY);	// Stage bits meant for first register
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);		// Latch shift register
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);	// Reset latching bit
}

// Function for changing the TRAFFIC LIGHT LED status color of a particular "cardinal" direction
// Accepts the requested traffic light color as status and the desired direction of a street as arguments
void trafficLight(enum LED status, enum Street t_dir){
	// Set light status of NORTH and SOUTH street
	if(t_dir == T_NORTHSOUTH) {
		// Start by masking and storing the state of all traffic lights that should be unaffected
		// Then proceed by clearing all the traffic lights that should change status color.
		REG[2] = REG[2] & 0b000111; // Mask current traffic lights at EAST street, clear NORTH traffic lights
		REG[1] = REG[1] & 0b111000;	// Mask current pedestrian lights at NORTH, clear SOUTH traffic lights
		// Set lights per input argument status color
		switch(status) {
		case GREEN:	// Set the lights GREEN
			// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
			REG[2] = REG[2] | 0b100000; // Set NORTH traffic light bit to GREEN
			REG[1] = REG[1] | 0b000100; // Set SOUTH traffic light bit to GREEN
			statusTraffic_NS = 1;		// Indicate that traffic is enabled for the NORTH and SOUTH street direction
			lastActive_NS = 1;			// Indicate that the most recently enabled street direction was NORTH and SOUTH
			break;
		case ORANGE: // Set the lights YELLOW
			// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
			REG[2] = REG[2] | 0b010000; // Set NORTH traffic light bit to YELLOW
			REG[1] = REG[1] | 0b000010; // Set SOUTH traffic light bit to YELLOW
			break;
		case RED: // Set the lights RED
			// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
			REG[2] = REG[2] | 0b001000; // Set NORTH traffic light bit to RED
			REG[1] = REG[1] | 0b000001; // Set SOUTH traffic light bit to RED
			statusTraffic_NS = 0;		// Indicate that the traffic is disabled for the NORTH and SOUTH street direction
			break;
		}
		// Set light status of EAST and WEST street
	} else if (t_dir == T_EASTWEST) {
		// Start by masking and storing the state of all traffic lights that should be unaffected
		// Then proceed by clearing all the traffic lights that should change status color.
		REG[2] = REG[2] & 0b111000; // Mask current traffic lights at NORTH, clear EAST traffic lights
		REG[0] = REG[0] & 0b111000; // Mask current pedestrian lights at WEST, clear WEST traffic lights
		// Set lights per input argument status color
		switch(status) {
		case GREEN:	// Set the lights GREEN
			// Bitwise-OR merge masked current states with new states for EAST and WEST traffic lights
			REG[2] = REG[2] | 0b000100; // Set EAST traffic light bit to GREEN
			REG[0] = REG[0] | 0b000100; // Set WEST traffic light bit to GREEN
			statusTraffic_EW = 1;		// Indicate that traffic is enabled for the EAST and WEST street direction
			lastActive_NS = 0;			// Indicate that the most recently enabled street direction was EAST and WEST
			break;
		case ORANGE: // Set the lights YELLOW
			// Bitwise-OR merge masked current states with new states for EAST and WEST traffic lights
			REG[2] = REG[2] | 0b000010; // Set EAST traffic light bit to YELLOW
			REG[0] = REG[0] | 0b000010; // Set WEST traffic light bit to YELLOW
			break;
		case RED: // Set the lights RED
			// Bitwise-OR merge masked current states with new states for EAST and WEST traffic lights
			REG[2] = REG[2] | 0b000001; // Set NORTH traffic light bit to RED
			REG[0] = REG[0] | 0b000001; // Set SOUTH traffic light bit to RED
			statusTraffic_EW = 0;		// Indicate that the traffic is disabled for the NORTH and SOUTH street direction
			break;
		}
	}			// Once all bits are configured in the simulated shift register array, proceed with
	setReg();	// staging and latching data via SPI to shift registers, causing LEDs to change status
}

// Function for changing the PEDESTRIAN LIGHT LED status color of a particular "cardinal" direction
// Accepts the requested traffic light color as status and the desired direction of a street as arguments
void pedestrianLight(enum LED status, enum Street p_dir){
	// Unlike the trafficLight function, the enumerators of the pedestrian crossing directions are assigned integer
	// values to represent the shift register array index relevant for the particular pedestrian crossing.
	// This can be done as both pedestrian crossing directions hold the same bit structure for their LED colors.
	REG[p_dir] = REG[p_dir] & 0b100111;	// Start by masking and storing the state of all other lights
	// Set lights per input status
	switch(status) {
	case GREEN:	// Set the lights GREEN
		// Bitwise-OR merge masked current states with new states for pedestrian lights
		REG[p_dir] = REG[p_dir] | 0b010000; // Set the desired pedestrian light bit to GREEN
		if (p_dir == P_NORTH) {				// If the passed pedestrian crossing is the NORTH crossing
			statusPedestrian_N = 1;			// Indicate that the NORTH pedestrian crossing is enabled
			buttonNorthFlag = 0;			// Hard reset flag for the NORTH pedestrian buttons as the crossing is activated
		} else {							// Else the passed pedestrian crossing is the WEST crossing
			statusPedestrian_W = 1;			// Hence, indicate that the WEST pedestrian crossing is enabled
			buttonWestFlag = 0;				// Hard reset flag for the WEST pedestrian buttons as the crossing is activated
		}
		break;
	case RED: // Set the lights RED
		// Bitwise-OR merge masked current states with new states for pedestrian lights
		REG[p_dir] = REG[p_dir] | 0b001000; // Set the desired pedestrian light bit to RED
		if (p_dir == P_NORTH) {
			statusPedestrian_N = 0;			// Indicate that the NORTH pedestrian crossing is disabled
		} else {
			statusPedestrian_W = 0;			// Indicate that the WEST pedestrian crossing is disabled
		}
		break;
	}			// Once all bits are configured in the simulated shift register array, proceed with
	setReg();	// staging and latching data via SPI to shift registers, causing LEDs to change status
}

// Function for enabling the TOGGLING PEDESTRIAN BLUE LED status color of a particular "cardinal" direction
// Used to indicate that a pedestrian has pushed a pedestrian crossing button and wish to cross over
void pedestrianPending(enum Street p_dir){
	// Start by masking and storing the state of all other lights
	togglePedestrianBlue = REG[p_dir] & 0b100000;	// Set toggle variable to current state of the BLUE LED for the input direction
	if (togglePedestrianBlue) {						// If the toggle variable indicates that the BLUE LED is on at the input direction
		REG[p_dir] = REG[p_dir] & 0b011111;			// Then set the bit of the BLUE LED to zero, disabling the BLUE LED at input direction
	} else {
		REG[p_dir] = REG[p_dir] | 0b100000;			// Else set the bit of the BLUE LED to one, enabling the BLUE LED at input direction
	}
	setReg();	// Stage and latching data via SPI to shift registers, causing LEDs to change status
}

// Function for enabling a TOGGLING PEDESTRIAN GREEN LED status color of a particular "cardinal" direction
// Used to indicate that a particular direction crossing is about to turn red
void pedestrianWarning(enum Street p_dir){
	// Start by masking and storing the state of all other lights
	togglePedestrianGreen = REG[p_dir] & 0b010000; 	// Set toggle variable to current state of the GREEN LED for the input direction
	if(togglePedestrianGreen) {						// If the toggle variable indicates that the GREEN LED is on at the input direction
		REG[p_dir] = REG[p_dir] & 0b101111;			// Then set the bit of the GREEN LED to zero, disabling the GREEN LED at input direction
	} else {
		REG[p_dir] = REG[p_dir] | 0b010000;			// Else set the bit of the GREEN LED to one, enabling the GREEN LED at input direction
	}
	setReg();	// Stage and latching data via SPI to shift registers, causing LEDs to change status
}


// Function for hard reseting the PEDESTRIAN BLUE LIGHT, disabling the BLUE LED of a particular "cardinal" direction
void pedestrianReset(enum Street p_dir){
	REG[p_dir] = REG[p_dir] & 0b011111;	// Mask all other bits, but cancel the BLUE LED bit, causing it to stay 0, disabling the BLUE LED
	setReg();	// Stage and latching data via SPI to shift registers, causing LEDs to change status
}
