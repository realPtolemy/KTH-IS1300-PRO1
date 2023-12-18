/*
 * functions_traffic.c
 *
 *  Created on: Dec 8, 2023
 *      Author: Love Mitteregger
 */

#include "pro1_funct.h"
#include "spi.h"
#include "gpio.h"
#include "FreeRTOS.h"

extern const TickType_t sysDelay;
extern const TickType_t toggleFreq;
extern const TickType_t pedestrianDelay;
extern const TickType_t walkingDelay;
extern const TickType_t safetyDelay;
extern const TickType_t greenDelay;
extern const TickType_t orangeDelay;
extern const TickType_t redDelayMax;

extern long startTime;
extern long endTime;
extern long elapsedTime;

extern volatile uint8_t buttonNorthFlag;
extern volatile uint8_t buttonWestFlag;

extern uint8_t statusTraffic_NS;
extern uint8_t statusTraffic_EW;
extern uint8_t statusPedestrian_N;
extern uint8_t statusPedestrian_W;

extern uint8_t statusVehicle_N;
extern uint8_t statusVehicle_S;
extern uint8_t statusVehicle_E;
extern uint8_t statusVehicle_W;

void activateTraffic(enum Street t_dir, enum Street p_dir) {
	trafficLight(RED, t_dir);
	trafficLight(ORANGE, t_dir);
	vTaskDelay(orangeDelay);
	pedestrianLight(GREEN, p_dir);
	trafficLight(GREEN, t_dir);
}

void disableTraffic(enum Street t_dir, enum Street p_dir) {
	trafficLight(GREEN, t_dir);
//	for(uint8_t i = 0; i < 10; i++) {
//		pedestrianWarning(p_dir);
//		vTaskDelay(toggleFreq);
//	}
	pedestrianLight(RED, p_dir);
	vTaskDelay(safetyDelay);
	trafficLight(ORANGE, t_dir);
	vTaskDelay(orangeDelay);
	trafficLight(RED, t_dir);
	vTaskDelay(safetyDelay);
}

void staticTraffic(){
	while(statusVehicle_E || statusVehicle_W) {
		if(buttonWestFlag) {
			vTaskDelay(pedestrianDelay);
			break;
		} else if(statusVehicle_N || statusVehicle_S) {
			vTaskDelay(redDelayMax);
			break;
		}
		trafficLight(GREEN, T_EASTWEST);
		pedestrianLight(GREEN, P_NORTH);
		vTaskDelay(sysDelay);
		checkTraffic();
	}
	while(statusVehicle_N || statusVehicle_S) {
		if(buttonNorthFlag) {
			vTaskDelay(pedestrianDelay);
			break;
		} else if(statusVehicle_E || statusVehicle_W) {
			vTaskDelay(redDelayMax);
			break;
		}
		trafficLight(GREEN, T_NORTHSOUTH);
		pedestrianLight(GREEN, P_WEST);
		vTaskDelay(sysDelay);
		checkTraffic();
	}
}
