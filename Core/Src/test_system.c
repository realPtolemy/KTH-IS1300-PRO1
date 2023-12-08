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

	// Set shift register bits high for the LEDs that should light up
	REG[2] = 0b00100001;
	REG[1] = 0b00001100;
	REG[0] = 0b00010001;

	stageReg_Test();  // Stage the newly set bits to the shift register
	latchReg_Test();  // Latch the staged bits to shift register storage for output
}
