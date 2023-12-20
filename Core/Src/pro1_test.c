/*
 * pro1_test.c
 *
 *  Created on: Dec 20, 2023
 *      Author: ptolemy
 */
#include "pro1_funct.h"
#include "pro1_test.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "pro1_funct.h"
#include "pro1_test.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "pro1_funct.h"
#include "pro1_test.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

uint8_t togglePedestrianGreen;
uint8_t togglePedestrianBlue;

extern volatile uint8_t buttonNorthFlag;
extern volatile uint8_t buttonWestFlag;

extern const TickType_t testingDelay;
extern volatile uint8_t statusVehicle_N;
extern volatile uint8_t statusVehicle_S;
extern volatile uint8_t statusVehicle_E;
extern volatile uint8_t statusVehicle_W;

extern uint8_t statusTraffic_NS;
extern uint8_t statusTraffic_EW;
extern uint8_t statusPedestrian_N;
extern uint8_t statusPedestrian_W;

extern long startTime;
extern long endTime;
extern long elapsedTime;

extern const TickType_t sysDelay;
extern const TickType_t toggleFreq;
extern const TickType_t walkingDelay;
extern const TickType_t safetyDelay;
extern const TickType_t greenDelay;
extern const TickType_t orangeDelay;
extern const TickType_t redDelayMax;
extern const TickType_t pedestrianDelay;
extern const TickType_t safetyDelay;
extern const TickType_t testingDelay2;

extern uint8_t REG[];

//TickType_t = xLastWakeTime;
//const TickType_t xFrequency = pdMS_TO_TICKS(2500);


//////////////////////////////////////// START OF LED TEST FUNCTIONS


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


// OLD FUNCTIONS ABOVE THIS COMMENT


// Change NORTH & SOUTH TRAFFIC lights
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

// Change PEDESTRIAN lights
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
			buttonNorthFlag = 0;
		} else {
			statusPedestrian_W = 1;
			buttonWestFlag = 0;
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

// Activate PEDESTRIAN GREEN WARNING lights
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
	setReg_Test();
}

//////////////////////////////////////// END OF LED TEST FUNCTIONS












//////////////////////////////////////// START OF SWITCH TEST FUNCTIONS


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


uint8_t checkTraffic_Test() {
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
	vTaskDelay(testingDelay);
};

//////////////////////////////////////// END OF SWITCH TEST FUNCTIONS














//////////////////////////////////////// START OF TRAFFIC LOGIC TEST FUNCTIONS



void init_Test(){
	// Initialize the enable pin and reset pin of the shift register
	HAL_GPIO_WritePin(IC595_Enable_GPIO_Port, IC595_Enable_Pin, 0);
	HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 1);

	// Set shift register bits high for the LEDs that should light up at initialization
	REG[2] = 0b100001;  // Bits to control TL4 and TL3 - NORTH TL Green & EAST TL Red
	REG[1] = 0b001100;  // Bits to control TL2 and PL2 - SOUTH TL Green & NORTH Red
	REG[0] = 0b010001;  // Bits to control TL1 and PL1 - WEST TL Red & WEST PL Green

	//stageReg();  // Stage the newly set bits to the shift register
	//latchReg();  // Latch the staged bits to shift register storage for output

	HAL_Delay(5000); // Delay for us humans to realize that things have happened
}

void activateTraffic_NS_Test() {
	trafficLight_Test(RED, T_NORTHSOUTH);
	trafficLight_Test(ORANGE, T_NORTHSOUTH);
	vTaskDelay(orangeDelay);
	pedestrianLight_Test(GREEN, P_WEST);
	trafficLight_Test(GREEN, T_NORTHSOUTH);
	statusTraffic_NS = 1;
}

void disableTraffic_NS_Test() {
	traffic_NS(1);
	startTime = xTaskGetTickCount();
	while (elapsedTime < orangeDelay ) {
		pedestrianWarning_W();
		endTime = xTaskGetTickCount();
		elapsedTime = endTime -  startTime;
		vTaskDelay( toggleFreq );
	}
	elapsedTime = 0;
	pedestrian_W(2);
	vTaskDelay( safetyDelay );
	traffic_NS(2);
	vTaskDelay( orangeDelay );
	traffic_NS(3);
	statusTraffic_NS = 0;
	vTaskDelay(safetyDelay);
}

void activateTraffic_EW_Test() {
	traffic_EW(3);
	traffic_EW(2);
	vTaskDelay( orangeDelay );
	pedestrian_N(1);
	traffic_EW(1);
	statusTraffic_EW = 1;
}

void disableTraffic_EW_Test() {
	traffic_EW(1);
	startTime = xTaskGetTickCount();
	while (elapsedTime < orangeDelay ) {
		pedestrianWarning_N();
		endTime = xTaskGetTickCount();
		elapsedTime = endTime -  startTime;
		vTaskDelay( toggleFreq );
	}
	elapsedTime = 0;
	pedestrian_N(2);
	vTaskDelay(safetyDelay);
	traffic_EW(2);
	vTaskDelay( orangeDelay );
	traffic_EW(3);
	statusTraffic_EW = 0;
	vTaskDelay(safetyDelay);
}

void staticTraffic_NS_Test(){
	while(statusVehicle_N || statusVehicle_S) {
		if(statusVehicle_E || statusVehicle_W) {
			vTaskDelay( redDelayMax );
			break;
		}
		traffic_NS(1);
		pedestrian_W(1);
		vTaskDelay(testingDelay2);
		checkTraffic();
	}
}

void staticTraffic_EW_Test(){
	while(statusVehicle_E || statusVehicle_W) {
		if(statusVehicle_N || statusVehicle_S) {
			vTaskDelay( redDelayMax );
			break;
		}
		traffic_EW(1);
		pedestrian_N(1);
		vTaskDelay(testingDelay2);
		checkTraffic();
	}
}

void activatePedestrian_N_Test(){
	statusPedestrian_W = 1;
}

void disablePedestrian_N_Test(){
	statusPedestrian_N = 0;
}

void activatePedestrian_W_Test(){
	statusPedestrian_W = 1;
}

void disablePedestrian_W_Test(){
	statusPedestrian_W = 0;
}


// OLDER FUNCTIONS ABOVE THIS COMMENT, IGNORE

void activateTraffic_Test(enum Street t_dir, enum Street p_dir) {
	trafficLight_Test(RED, t_dir);
	trafficLight_Test(ORANGE, t_dir);
	vTaskDelay(orangeDelay);
	pedestrianLight_Test(GREEN, p_dir);
	trafficLight_Test(GREEN, t_dir);
}

void disableTraffic_Test(enum Street t_dir, enum Street p_dir) {
	trafficLight_Test(GREEN, t_dir);
//	for(uint8_t i = 0; i < 10; i++) {
//		pedestrianWarning(p_dir);
//		vTaskDelay(toggleFreq);
//	}
	pedestrianLight_Test(RED, p_dir);
	vTaskDelay(safetyDelay);
	trafficLight_Test(ORANGE, t_dir);
	vTaskDelay(orangeDelay);
	trafficLight_Test(RED, t_dir);
	vTaskDelay(safetyDelay);
}

void staticTraffic_Test(){
	while(statusVehicle_E || statusVehicle_W) {
		if(buttonWestFlag) {
			vTaskDelay(pedestrianDelay);
			break;
		} else if(statusVehicle_N || statusVehicle_S) {
			vTaskDelay(redDelayMax);
			break;
		}
		trafficLight_Test(GREEN, T_EASTWEST);
		pedestrianLight_Test(GREEN, P_NORTH);
		vTaskDelay(sysDelay);
		checkTraffic_Test();
	}
	while(statusVehicle_N || statusVehicle_S) {
		if(buttonNorthFlag) {
			vTaskDelay(pedestrianDelay);
			break;
		} else if(statusVehicle_E || statusVehicle_W) {
			vTaskDelay(redDelayMax);
			break;
		}
		trafficLight_Test(GREEN, T_NORTHSOUTH);
		pedestrianLight_Test(GREEN, P_WEST);
		vTaskDelay(sysDelay);
		checkTraffic_Test();
	}
}


//////////////////////////////////////// END OF TRAFFIC LOGIC TEST FUNCTIONS
