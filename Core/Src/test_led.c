/*
 * test_led.c
 *
 *  Created on: Dec 4, 2023
 *      Author: Love Mitteregger
 */
/* Includes ------------------------------------------------------------------*/
#include "pro1_funct.h"
#include "pro1_test.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

uint8_t togglePedestrianGreen;
uint8_t togglePedestrianBlue;

extern uint8_t statusVehicle_N;
extern uint8_t statusVehicle_S;
extern uint8_t statusVehicle_E;
extern uint8_t statusVehicle_W;

extern uint8_t statusTraffic_NS;
extern uint8_t statusTraffic_EW;
extern uint8_t statusPedestrian_N;
extern uint8_t statusPedestrian_W;


// Test function for north and south traffic lights
void trafficLED_Test_NS() {
	// Set bits high for LEDs that should light up
	uint8_t T_ADDRESS_NS = 0x38;
	// Transmit the address data to the first shift register via SPI
	HAL_SPI_Transmit(&hspi3, &T_ADDRESS_NS, 1, HAL_MAX_DELAY);

	// Update bits and transmit data to the first shift register via SPI,
	// ... pushing previous transmission to the second shift register
	T_ADDRESS_NS = 0x07;
	HAL_SPI_Transmit(&hspi3, &T_ADDRESS_NS, 1, HAL_MAX_DELAY);

	// Reset address bits to low and transmit it as data to the first shift register via SPI,
	// ... pushing the initial address data to the third shift register,
	// ... and the the previous data to the second register
	T_ADDRESS_NS = 0x00;
	HAL_SPI_Transmit(&hspi3, &T_ADDRESS_NS, 1, HAL_MAX_DELAY);

	// Update latch of shift register, pushing the shift register data to storage for output
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
}

void trafficLED_Test_EW() {
	// Set bits high for LEDs that should light up
	uint8_t T_ADDRESS_EW = 0x07;

	// Transmit the address data to the first shift register via SPI
	HAL_SPI_Transmit(&hspi3, &T_ADDRESS_EW, 1, HAL_MAX_DELAY);

	// Reset address bits to low and transmit it as data to the first shift register via SPI,
	// ... pushing previous transmission to the second shift register
	T_ADDRESS_EW = 0x00;
	HAL_SPI_Transmit(&hspi3, &T_ADDRESS_EW, 1, HAL_MAX_DELAY);

	// Update bits and transmit data to the first shift register via SPI,
	// ... pushing the initial address data to the third shift register,
	// ... and the the previous data to the second register
	T_ADDRESS_EW = 0x07;
	HAL_SPI_Transmit(&hspi3, &T_ADDRESS_EW, 1, HAL_MAX_DELAY);

	// Update latch of shift register, pushing the shift register data to storage for output
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
}

void pedestrianLED_Test_N() {
	// Set bits high for LEDs that should light up
	uint8_t P_ADDRESS_N = 0x00;
	// Transmit the address data to the first shift register via SPI
	HAL_SPI_Transmit(&hspi3, &P_ADDRESS_N, 1, HAL_MAX_DELAY);

	// Reset address bits to low and transmit it as data to the first shift register via SPI,
	// ... pushing previous transmission to the second shift register
	P_ADDRESS_N = 0x38;
	HAL_SPI_Transmit(&hspi3, &P_ADDRESS_N, 1, HAL_MAX_DELAY);

	// Update bits and transmit data to the first shift register via SPI,
	// ... pushing the initial address data to the third shift register,
	// ... and the the previous data to the second register
	P_ADDRESS_N = 0x00;
	HAL_SPI_Transmit(&hspi3, &P_ADDRESS_N, 1, HAL_MAX_DELAY);

	// Update latch of shift register, pushing the shift register data to storage for output
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
}

void pedestrianLED_Test_W() {
	// Set bits high for LEDs that should light up
	uint8_t P_ADDRESS_W = 0x00;
	// Transmit the address data to the first shift register via SPI
	HAL_SPI_Transmit(&hspi3, &P_ADDRESS_W, 1, HAL_MAX_DELAY);

	// Reset address bits to low and transmit it as data to the first shift register via SPI,
	// ... pushing previous transmission to the second shift register
	P_ADDRESS_W = 0x00;
	HAL_SPI_Transmit(&hspi3, &P_ADDRESS_W, 1, HAL_MAX_DELAY);

	// Update bits and transmit data to the first shift register via SPI,
	// ... pushing the initial address data to the third shift register,
	// ... and the the previous data to the second register
	P_ADDRESS_W = 0x38;
	HAL_SPI_Transmit(&hspi3, &P_ADDRESS_W, 1, HAL_MAX_DELAY);

	// Update latch of shift register, pushing the shift register data to storage for output
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
}

// Test of enhanced staging and latching of bits to register
extern uint8_t REG[];
// Stage bits via the shift register buffer array
void stageReg_Test(){
	HAL_SPI_Transmit(&hspi3, &REG[2], 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &REG[1], 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &REG[0], 1, HAL_MAX_DELAY);
}

// Latch bits stored in shift register to storage register, for output
void latchReg_Test(){
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
}

void setReg_Test(){
	HAL_SPI_Transmit(&hspi3, &REG[2], 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &REG[1], 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &REG[0], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IC595_STCP_GPIO_Port, IC595_STCP_Pin, GPIO_PIN_RESET);
}

// Simply test activating LEDs using shift register buffer array
void activateLED_Test(){
	REG[2] = 0b00000000;	// Third shift register, only the first 6 bits controls LEDs
	REG[1] = 0b00000000;	// Second shift register
	REG[0] = 0b00000000;	// First shift register
	stageReg_Test();
	latchReg_Test();
}

// Activate LEDs on current set of LEDs, by masking
void activateMultiLED_Test(){
	REG[2] = REG[2] & 0b00111111;	// Try applying masking to shift registers
	REG[1] = REG[1] & 0b00111111;
	REG[0] = REG[0] & 0b00111111;
}

void trafficRed_NS_Test(){
	// Start by masking and storing the state of all other traffic lights
	REG[2] = REG[2] & 0b000111; // Mask current traffic lights at EAST, clear NORTH traffic lights
	REG[1] = REG[1] & 0b111000;	// Mask current pedestrian lights at NORTH, clear SOUTH traffic lights
	REG[0] = REG[0] & 0b111111; // Mask current traffic and pedestrian lights at WEST
	// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
	REG[2] = REG[2] | 0b001000; // Set NORTH traffic light bit to red
	REG[1] = REG[1] | 0b000001; // Set SOUTH traffic light bit to red
}

/* Change NORTH & SOUTH TRAFFIC lights */
void traffic_NS_Test(uint8_t status){
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
	setReg_Test();
}

/* Change EAST & WEST TRAFFIC lights */
void traffic_EW_Test(uint8_t status){
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
	setReg_Test();
}

/* Change NORTH PEDESTRIAN lights */
void pedestrian_N_Test(uint8_t status){
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
	setReg_Test();
}

/* Change NORTH PEDESTRIAN lights */
void pedestrian_W_Test(uint8_t status){
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
	setReg_Test();
}

void pedestrianWarning_N_Test(){
	togglePedestrianGreen = REG[1] & 0b010000; // Mask the pedestrian bit
	if(togglePedestrianGreen) {
		REG[1] = REG[1] & 0b101111; // Set NORTH pedestrian light bit to GREEN
	} else {
		REG[1] = REG[1] | 0b010000; // Set NORTH pedestrian light bit to GREEN
	}
	setReg_Test();
}

void pedestrianWarning_W_Test(){
	togglePedestrianGreen = REG[0] & 0b010000; // Mask the pedestrian bit
	if(togglePedestrianGreen) {
		REG[0] = REG[0] & 0b101111; // Set NORTH pedestrian light bit to GREEN
	} else {
		REG[0] = REG[0] | 0b010000; // Set NORTH pedestrian light bit to GREEN
	}
	setReg_Test();
}

/* Activate NORTH PEDESTRIAN BLUE lights */
void pedestrianPending_N_Test(){
	// Start by masking and storing the state of all other lights
	togglePedestrianBlue = REG[1] & 0b100000;
	if (togglePedestrianBlue) {
		REG[1] = REG[1] & 0b011111;
	} else {
		REG[1] = REG[1] | 0b100000;
	}
	setReg_Test();
}
/* Activate WEST PEDESTRIAN BLUE lights */
void pedestrianPending_W_Test(){
	// Start by masking and storing the state of all other lights
	togglePedestrianBlue = REG[0] & 0b100000;
	if (togglePedestrianBlue) {
		REG[0] = REG[0] & 0b011111;
	} else {
		REG[0] = REG[0] | 0b100000;
	}
	setReg_Test();
}

/* Change NORTH & SOUTH TRAFFIC lights */
void trafficLight_Test(enum LED status, enum Street t_dir){
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

/* Change PEDESTRIAN lights */
void pedestrianLight_Test(enum LED status, enum Street p_dir){
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

/* Activate PEDESTRIAN BLUE lights */
void pedestrianPending_Test(enum Street p_dir){
	// Start by masking and storing the state of all other lights
	togglePedestrianBlue = REG[p_dir] & 0b100000;
	if (togglePedestrianBlue) {
		REG[p_dir] = REG[p_dir] & 0b011111;
	} else {
		REG[p_dir] = REG[p_dir] | 0b100000;
	}
	setReg_Test();
}

/* Activate PEDESTRIAN GREEN WARNING lights */
void pedestrianWarning_Test(enum Street p_dir){
	togglePedestrianGreen = REG[p_dir] & 0b010000; // Mask the pedestrian bit
	if(togglePedestrianGreen) {
		REG[p_dir] = REG[p_dir] & 0b101111; // Set NORTH pedestrian light bit to GREEN
	} else {
		REG[p_dir] = REG[p_dir] | 0b010000; // Set NORTH pedestrian light bit to GREEN
	}
	setReg_Test();
}

void pedestrianReset_Test(enum Street p_dir){
	REG[p_dir] = REG[p_dir] & 0b011111;
}
