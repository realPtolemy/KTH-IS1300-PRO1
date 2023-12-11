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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern uint8_t REG[];
extern const TickType_t toggleFreq;
extern const TickType_t greenDelay;
extern const TickType_t orangeDelay;
extern const TickType_t redDelayMax;
extern const TickType_t pedestrianDelay;

extern TickType_t startTime;
extern TickType_t endTime;
extern TickType_t elapsedTime;

extern uint8_t statusTraffic_NS;
extern uint8_t statusTraffic_EW;
extern uint8_t statusPedestrian_N;
extern uint8_t statusPedestrian_W;



void init_Test(){

	const TickType_t toggleFreq = 1000 / portTICK_PERIOD_MS;


	// Initialize the enable pin and reset pin of the shift register
	HAL_GPIO_WritePin(IC595_Enable_GPIO_Port, IC595_Enable_Pin, 0);
	HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 1);

	// Set shift register bits high for the LEDs that should light up at initialization
	//REG[2] = 0b100001;  // Bits to control TL4 and TL3 - NORTH TL Green & EAST TL Red
	//REG[1] = 0b001100;  // Bits to control TL2 and PL2 - SOUTH TL Green & NORTH Red
	//REG[0] = 0b010001;  // Bits to control TL1 and PL1 - WEST TL Red & WEST PL Green

	activateLED_Test();
	activateMultiLED_Test();
	traffic_EW_Test(1);
	traffic_NS_Test(1);
	pedestrian_N_Test(2);
	pedestrian_W_Test(2);
	stageReg_Test();  // Stage the newly set bits to the shift register
	latchReg_Test();  // Latch the staged bits to shift register storage for output
}

void activateTraffic_NS_Test() {
	traffic_NS_Test(3);
	traffic_NS_Test(2);
	vTaskDelay( orangeDelay );
	pedestrian_W_Test(1);
	traffic_NS_Test(1);
	statusTraffic_NS = 1;
}

void disableTraffic_NS_Test() {
	traffic_NS_Test(1);
	startTime = xTaskGetTickCount();
	traffic_NS_Test(2);
	while (elapsedTime < orangeDelay ) {
		pedestrianWarning_W_Test();
		endTime = xTaskGetTickCount();
		elapsedTime = endTime -  startTime;
	}
	elapsedTime = 0;
	pedestrian_W_Test(2);
	traffic_NS_Test(3);
	statusTraffic_NS = 0;
}

void activateTraffic_EW_Test() {
	traffic_EW_Test(3);
	traffic_EW_Test(2);
	vTaskDelay( orangeDelay );
	pedestrian_N_Test(1);
	traffic_EW_Test(1);
	statusTraffic_EW = 1;
}

void disableTraffic_EW_Test() {
	traffic_EW_Test(1);
	traffic_EW_Test(2);
	vTaskDelay( orangeDelay );
	pedestrian_N_Test(2);
	traffic_EW_Test(3);
	statusTraffic_EW = 0;
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
