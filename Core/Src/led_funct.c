/*
 * led_funct.c
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


