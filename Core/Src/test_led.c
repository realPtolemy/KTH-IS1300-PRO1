/*
 * test_led.c
 *
 *  Created on: Dec 4, 2023
 *      Author: Love Mitteregger
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "test_led.h"


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

