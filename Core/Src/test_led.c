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

/* Change NORTH & SOUTH TRAFFIC lights */
void traffic_NS_Test(int status){
	switch(status) {
		case 1:	// Set the lights GREEN
			// Start by masking and storing the state of all other traffic lights
			REG[2] = REG[2] & 0b000111; // Mask current traffic lights at EAST, clear NORTH traffic lights
			REG[1] = REG[1] & 0b111000;	// Mask current pedestrian lights at NORTH, clear SOUTH traffic lights
			REG[0] = REG[0] & 0b111111; // Mask current traffic and pedestrian lights at WEST
			// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
			REG[2] = REG[2] | 0b100000; // Set NORTH traffic light bit to red
			REG[1] = REG[1] | 0b000100; // Set SOUTH traffic light bit to red
			break;
		case 2: // Set the lights YELLOW
			// Start by masking and storing the state of all other traffic lights
			REG[2] = REG[2] & 0b000111; // Mask current traffic lights at EAST, clear NORTH traffic lights
			REG[1] = REG[1] & 0b111000;	// Mask current pedestrian lights at NORTH, clear SOUTH traffic lights
			REG[0] = REG[0] & 0b111111; // Mask current traffic and pedestrian lights at WEST
			// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
			REG[2] = REG[2] | 0b010000; // Set NORTH traffic light bit to red
			REG[1] = REG[1] | 0b000010; // Set SOUTH traffic light bit to red
			break;
		case 3: // Set the lights RED
			// Start by masking and storing the state of all other traffic lights
			REG[2] = REG[2] & 0b000111; // Mask current traffic lights at EAST, clear NORTH traffic lights
			REG[1] = REG[1] & 0b111000;	// Mask current pedestrian lights at NORTH, clear SOUTH traffic lights
			REG[0] = REG[0] & 0b111111; // Mask current traffic and pedestrian lights at WEST
			// Bitwise-OR merge masked current states with new states for NORTH and SOUTH traffic lights
			REG[2] = REG[2] | 0b001000; // Set NORTH traffic light bit to red
			REG[1] = REG[1] | 0b000001; // Set SOUTH traffic light bit to red
			break;
	}
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
void trafficYellow_NS_Test(){}
void trafficGreen_NS_Test(){}

/* Set bits for EAST & WEST TRAFFIC lights */
void trafficRed_EW_Test(){}
void trafficYellow_EW_Test(){}
void trafficGreen_EW_Test(){}

/* Set bits for NORTH PEDESTRIAN lights */
void pedestrianRed_N_Test(){}
void pedestrianYellow_N_Test(){}
void pedestrianGreen_N_Test(){}

/* Set bits for West Pedestrian Lights */
void pedestrianRed_W_Test(){}
void pedestrianYellow_W_Test(){}
void pedestrianGreen_W_Test(){}


