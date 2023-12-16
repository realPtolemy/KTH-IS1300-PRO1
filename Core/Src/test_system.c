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
extern const TickType_t sysDelay;
extern const TickType_t toggleFreq;
extern const TickType_t greenDelay;
extern const TickType_t orangeDelay;
extern const TickType_t redDelayMax;
extern const TickType_t pedestrianDelay;
extern const TickType_t safetyDelay;
extern const TickType_t testingDelay2;

extern TickType_t startTime;
extern TickType_t endTime;
extern TickType_t elapsedTime;

extern uint8_t statusTraffic_NS;
extern uint8_t statusTraffic_EW;
extern uint8_t statusPedestrian_N;
extern uint8_t statusPedestrian_W;

extern uint8_t statusVehicle_N;
extern uint8_t statusVehicle_S;
extern uint8_t statusVehicle_E;
extern uint8_t statusVehicle_W;



void init_Test(){
	// Initialize the enable pin and reset pin of the shift register
	HAL_GPIO_WritePin(IC595_Enable_GPIO_Port, IC595_Enable_Pin, 0);
	HAL_GPIO_WritePin(IC595_Reset_GPIO_Port, IC595_Reset_Pin, 1);

	// Set shift register bits high for the LEDs that should light up at initialization
	REG[2] = 0b100001;  // Bits to control TL4 and TL3 - NORTH TL Green & EAST TL Red
	REG[1] = 0b001100;  // Bits to control TL2 and PL2 - SOUTH TL Green & NORTH Red
	REG[0] = 0b010001;  // Bits to control TL1 and PL1 - WEST TL Red & WEST PL Green

	stageReg();  // Stage the newly set bits to the shift register
	latchReg();  // Latch the staged bits to shift register storage for output

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


void activateTraffic_Test(enum Street t_dir, enum Street p_dir) {
	trafficLight_Test(RED, t_dir);
	trafficLight_Test(ORANGE, t_dir);
	vTaskDelay(orangeDelay);
	pedestrianLight_Test(GREEN, p_dir);
	trafficLight_Test(GREEN, t_dir);
}

void disableTraffic_Test(enum Street t_dir, enum Street p_dir) {
	trafficLight_Test(GREEN, t_dir);
	startTime = xTaskGetTickCount();
	while (elapsedTime < orangeDelay ) {
		pedestrianWarning_Test(p_dir);
		endTime = xTaskGetTickCount();
		elapsedTime = endTime -  startTime;
		vTaskDelay( toggleFreq );
	}
	elapsedTime = 0;
	pedestrianLight_Test(RED, p_dir);
	vTaskDelay(safetyDelay);
	trafficLight_Test(ORANGE, t_dir);
	vTaskDelay(orangeDelay);
	trafficLight_Test(RED, t_dir);
	statusTraffic_NS = 0;
	vTaskDelay(safetyDelay);
}

void staticTraffic_Test(){
	while(statusVehicle_E || statusVehicle_W) {
		if(statusVehicle_N || statusVehicle_S) {
			vTaskDelay(redDelayMax);
			break;
		}
		trafficLight_Test(GREEN, T_EASTWEST);
		pedestrianLight_Test(GREEN, P_NORTH);
		vTaskDelay(sysDelay);
		checkTraffic_Test();
	}
	while(statusVehicle_N || statusVehicle_S) {
		if(statusVehicle_E || statusVehicle_W) {
			vTaskDelay(redDelayMax);
			break;
		}
		trafficLight_Test(GREEN, T_NORTHSOUTH);
		pedestrianLight_Test(GREEN, P_WEST);
		vTaskDelay(sysDelay);
		checkTraffic_Test();
	}
}
