/*
 * system_config.c
 *
 *  Created on: Dec 8, 2023
 *      Author: Love Mitteregger
 *
 */
#include "pro1_funct.h"
#include "spi.h"
#include "gpio.h"

/* Instantiate global variables */

/*
 * Shift register bits array, each element represents the
 * bits stored in each respective shift register.
 *
 * Used to conveniently store and push bits to the serially
 * connected shift registers
 *
 * REG[3] = { U1, U2, U3 }
 *
 */
uint8_t REG[3] = {0x00, 0x00, 0x00};

// Required program delays, in ms.
static uint16_t greenDelay = 100;
static uint16_t orangeDelay = 100;
static uint16_t redDelayMax = 100;
static uint16_t pedestrianDelay = 100;
static uint16_t toggleFreq = 100;

// Set all LEDs to their initial stage
void system_init(){
	// Initialize the enable pin and reset pin of the shift register
	HAL_GPIO_WritePin(IC595_Enable_GPIO_Port, IC595_Enable_Pin, 0);
	HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 1);

	// Set shift register bits high for the LEDs that should light up at initialization
	REG[2] = 0b100001;  // Bits to control TL4 and TL3 - NORTH TL Green & EAST TL Red
	REG[1] = 0b001100;  // Bits to control TL2 and PL2 - SOUTH TL Green & NORTH Red
	REG[0] = 0b010001;  // Bits to control TL1 and PL1 - WEST TL Red & WEST PL Green

	stageReg();  // Stage the newly set bits to the shift register
	latchReg();  // Latch the staged bits to shift register storage for output
}

