/*
 * test_system.c
 *
 *  Created on: Dec 8, 2023
 *      Author: Love Mitteregger
 */
#include "pro1_funct.h"
#include "pro1_test.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"

extern uint8_t REG[];

void init_Test(){
	// Initialize the enable pin and reset pin of the shift register
	HAL_GPIO_WritePin(IC595_Enable_GPIO_Port, IC595_Enable_Pin, 0);
	HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 1);

	// Set shift register bits high for the LEDs that should light up at initialization
	//REG[2] = 0b100001;  // Bits to control TL4 and TL3 - NORTH TL Green & EAST TL Red
	//REG[1] = 0b001100;  // Bits to control TL2 and PL2 - SOUTH TL Green & NORTH Red
	//REG[0] = 0b010001;  // Bits to control TL1 and PL1 - WEST TL Red & WEST PL Green

	activateLED_Test();
	activateMultiLED_Test();
	traffic_NS_Test(2);
	stageReg_Test();  // Stage the newly set bits to the shift register
	latchReg_Test();  // Latch the staged bits to shift register storage for output
}
