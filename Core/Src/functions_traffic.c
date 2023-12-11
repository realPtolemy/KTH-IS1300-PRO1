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

void activateTraffic_NS() {
	traffic_NS(3);
	traffic_NS(2);
	vTaskDelay( orangeDelay );
	pedestrian_W(1);
	traffic_NS(1);
	statusTraffic_NS = 1;
}

void disableTraffic_NS() {
	traffic_NS(1);
	startTime = xTaskGetTickCount();
	traffic_NS(2);
	while (elapsedTime < orangeDelay ) {
		pedestrianWarning_W();
		endTime = xTaskGetTickCount();
		elapsedTime = endTime -  startTime;
		vTaskDelay( toggleFreq );
	}
	elapsedTime = 0;
	pedestrian_W(2);
	traffic_NS(3);
	statusTraffic_NS = 0;
}

void activateTraffic_EW() {
	traffic_EW(3);
	traffic_EW(2);
	vTaskDelay( orangeDelay );
	pedestrian_N(1);
	traffic_EW(1);
	statusTraffic_EW = 1;
}

void disableTraffic_EW() {
	traffic_EW(1);
	startTime = xTaskGetTickCount();
	traffic_EW(2);
	while (elapsedTime < orangeDelay ) {
		pedestrianWarning_N();
		endTime = xTaskGetTickCount();
		elapsedTime = endTime -  startTime;
		vTaskDelay( toggleFreq );
	}
	elapsedTime = 0;
	pedestrian_N(2);
	traffic_EW(3);
	statusTraffic_EW = 0;
}

