/*
 * functions_traffic.c
 *
 *  Created on: Dec 8, 2023
 *      Author: Love Mitteregger
 */
#include "pro1_funct.h"
#include "spi.h"
#include "gpio.h"
#include "FreeRTOS.h"

// Instantiate the shift register array
extern uint8_t REG[];
extern uint8_t togglePedestrianGreen;
extern uint8_t togglePedestrianBlue;

extern uint8_t statusVehicle_N;
extern uint8_t statusVehicle_S;
extern uint8_t statusVehicle_E;
extern uint8_t statusVehicle_W;

extern uint8_t statusTraffic_NS;
extern uint8_t statusTraffic_EW;
extern uint8_t statusPedestrian_N;
extern uint8_t statusPedestrian_W;


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

void setReg(){
	HAL_SPI_Transmit(&hspi3, &REG[2], 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &REG[1], 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &REG[0], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
}

// Change NORTH & SOUTH TRAFFIC lights
void trafficLight(enum LED status, enum Street t_dir){
	if(t_dir == T_NORTHSOUTH) {
		// Start by masking and storing the state of all other traffic lights
		REG[2] = REG[2] & 0b000111; // Mask current traffic lights at EAST, clear NORTH traffic lights
		REG[1] = REG[1] & 0b111000;	// Mask current pedestrian lights at NORTH, clear SOUTH traffic lights
		// Set lights per input status
		switch(status) {
		case GREEN:	// Set the lights GREEN
			// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
			REG[2] = REG[2] | 0b100000; // Set NORTH traffic light bit to GREEN
			REG[1] = REG[1] | 0b000100; // Set SOUTH traffic light bit to GREEN
			statusTraffic_NS = 1;
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
			statusTraffic_NS = 0;
			break;
		}
	} else if (t_dir == T_EASTWEST) {
		// Start by masking and storing the state of all other traffic lights
		REG[2] = REG[2] & 0b111000; // Mask current traffic lights at NORTH, clear EAST traffic lights
		REG[0] = REG[0] & 0b111000; // Mask current pedestrian lights at WEST, clear WEST traffic lights
		// Set lights per input status
		switch(status) {
			case GREEN:	// Set the lights GREEN
				// Bitwise-OR merge masked current states with new states for EAST and WEST traffic lights
				REG[2] = REG[2] | 0b000100; // Set EAST traffic light bit to GREEN
				REG[0] = REG[0] | 0b000100; // Set WEST traffic light bit to GREEN
				statusTraffic_EW = 1;
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
				statusTraffic_EW = 0;
				break;
		}
	}
	setReg_Test();
}

// Change PEDESTRIAN lights
void pedestrianLight(enum LED status, enum Street p_dir){
	// Start by masking and storing the state of all other lights
	REG[p_dir] = REG[p_dir] & 0b100111;	// Mask current traffic lights at SOUTH, clear RED and GREEN pedestrian lights at NORTH
	// Set lights per input status
	switch(status) {
	case GREEN:	// Set the lights GREEN
		// Bitwise-OR merge masked current states with new states for NORTH pedestrian and SOUTH traffic lights
		REG[p_dir] = REG[p_dir] | 0b010000; // Set NORTH pedestrian light bit to GREEN
		if (p_dir == P_NORTH) {
			statusPedestrian_N = 1;
		} else {
			statusPedestrian_W = 1;
		}
		break;
	case RED: // Set the lights RED
		// Bitwise-OR merge masked current states with new states for NORTH pedestrian and SOUTH traffic lights
		REG[p_dir] = REG[p_dir] | 0b001000; // Set NORTH pedestrian light bit to RED
		if (p_dir == P_NORTH) {
			statusPedestrian_N = 0;
		} else {
			statusPedestrian_W = 0;
		}
		break;
	}
	setReg_Test();
}

// Activate PEDESTRIAN BLUE lights
void pedestrianPending(enum Street p_dir){
	// Start by masking and storing the state of all other lights
	togglePedestrianBlue = REG[p_dir] & 0b100000;
	if (togglePedestrianBlue) {
		REG[p_dir] = REG[p_dir] & 0b011111;
	} else {
		REG[p_dir] = REG[p_dir] | 0b100000;
	}
	setReg_Test();
}

// Activate PEDESTRIAN GREEN WARNING lights
void pedestrianWarning(enum Street p_dir){
	togglePedestrianGreen = REG[p_dir] & 0b010000; // Mask the pedestrian bit
	if(togglePedestrianGreen) {
		REG[p_dir] = REG[p_dir] & 0b101111; // Set NORTH pedestrian light bit to GREEN
	} else {
		REG[p_dir] = REG[p_dir] | 0b010000; // Set NORTH pedestrian light bit to GREEN
	}
	setReg_Test();
}


