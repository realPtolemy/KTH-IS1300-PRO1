/*
 * functions_traffic.c
 *
 *  Created on: Dec 8, 2023
 *      Author: Love Mitteregger
 */
#include "pro1_funct.h"
#include "spi.h"
#include "gpio.h"

// Instantiate the shift register array
extern uint8_t REG[];

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

/* Change NORTH & SOUTH TRAFFIC lights */
void traffic_NS(uint8_t status){
	// Start by masking and storing the state of all other traffic lights
	REG[2] = REG[2] & 0b000111; // Mask current traffic lights at EAST, clear NORTH traffic lights
	REG[1] = REG[1] & 0b111000;	// Mask current pedestrian lights at NORTH, clear SOUTH traffic lights
	// Set lights per input status
	switch(status) {
		case 1:	// Set the lights GREEN
			// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
			REG[2] = REG[2] | 0b100000; // Set NORTH traffic light bit to GREEN
			REG[1] = REG[1] | 0b000100; // Set SOUTH traffic light bit to GREEN
			break;
		case 2: // Set the lights YELLOW
			// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
			REG[2] = REG[2] | 0b010000; // Set NORTH traffic light bit to YELLOW
			REG[1] = REG[1] | 0b000010; // Set SOUTH traffic light bit to YELLOW
			break;
		case 3: // Set the lights RED
			// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
			REG[2] = REG[2] | 0b001000; // Set NORTH traffic light bit to RED
			REG[1] = REG[1] | 0b000001; // Set SOUTH traffic light bit to RED
			break;
	}
	setReg();
}

/* Change EAST & WEST TRAFFIC lights */
void traffic_EW(uint8_t status){
	// Start by masking and storing the state of all other traffic lights
	REG[2] = REG[2] & 0b111000; // Mask current traffic lights at NORTH, clear EAST traffic lights
	REG[0] = REG[0] & 0b111000; // Mask current pedestrian lights at WEST, clear WEST traffic lights
	// Set lights per input status
	switch(status) {
		case 1:	// Set the lights GREEN
			// Bitwise-OR merge masked current states with new states for EAST and WEST traffic lights
			REG[2] = REG[2] | 0b000100; // Set EAST traffic light bit to GREEN
			REG[0] = REG[0] | 0b000100; // Set WEST traffic light bit to GREEN
			break;
		case 2: // Set the lights YELLOW
			// Bitwise-OR merge masked current states with new states for EAST and WEST traffic lights
			REG[2] = REG[2] | 0b000010; // Set EAST traffic light bit to YELLOW
			REG[0] = REG[0] | 0b000010; // Set WEST traffic light bit to YELLOW
			break;
		case 3: // Set the lights RED
			// Bitwise-OR merge masked current states with new states for EAST and WEST traffic lights
			REG[2] = REG[2] | 0b000001; // Set NORTH traffic light bit to RED
			REG[0] = REG[0] | 0b000001; // Set SOUTH traffic light bit to RED
			break;
	}
	setReg();
}

/* Change NORTH PEDESTRIAN lights */
void pedestrian_N(uint8_t status){
	// Start by masking and storing the state of all other lights
	REG[1] = REG[1] & 0b100111;	// Mask current traffic lights at SOUTH, clear RED and GREEN pedestrian lights at NORTH
	// Set lights per input status
	switch(status) {
		case 1:	// Set the lights GREEN
			// Bitwise-OR merge masked current states with new states for NORTH pedestrian and SOUTH traffic lights
			REG[1] = REG[1] | 0b010000; // Set NORTH pedestrian light bit to GREEN
			break;
		case 2: // Set the lights RED
			// Bitwise-OR merge masked current states with new states for NORTH pedestrian and SOUTH traffic lights
			REG[1] = REG[1] | 0b001000; // Set NORTH pedestrian light bit to RED
			break;
	}
	setReg();
}

/* Change NORTH PEDESTRIAN lights */
void pedestrian_W(uint8_t status){
	// Start by masking and storing the state of all other lights
	REG[0] = REG[0] & 0b100111;	// Mask current WEST traffic lights and BLUE indicator light, clear WEST pedestrian lights
	// Set lights per input status
	switch(status) {
		case 1:	// Set the lights GREEN
			// Bitwise-OR merge masked current states with new states for WEST pedestrian and WEST traffic lights
			REG[0] = REG[0] | 0b010000; // Set WEST pedestrian light bit to GREEN
			break;
		case 2: // Set the lights RED
			// Bitwise-OR merge masked current states with new states for WEST pedestrian and WEST traffic lights
			REG[0] = REG[0] | 0b001000; // Set WEST pedestrian light bit to RED
			break;
	}
	setReg();
}

/* Activate NORTH PEDESTRIAN BLUE lights */
void pedestrianPending_N(){
	// Start by masking and storing the state of all other lights
	if (REG[1] >= 0b100000) {
		REG[1] = REG[1] & 0b011111;
	} else {
		REG[1] = REG[1] | 0b100000;
	}
	setReg();
}
/* Activate WEST PEDESTRIAN BLUE lights */
void pedestrianPending_W(){
	// Start by masking and storing the state of all other lights
	if (REG[0] >= 0b100000) {
		REG[0] = REG[0] & 0b011111;
	} else {
		REG[0] = REG[0] | 0b100000;
	}
	setReg();
}


