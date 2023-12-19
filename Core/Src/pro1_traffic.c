/************************************************************************************
 *	pro1_traffic.c																	*
 ************************************************************************************
 *  Created on: Dec 8, 2023															*
 *      Author: Love Mitteregger													*
 ************************************************************************************
 *	MIT License																		*
 *																					*
 *	Copyright (c) 2023 Love Mitteregger												*
 *																					*
 *	Permission is hereby granted, free of charge, to any person obtaining a copy	*
 *	of this software and associated documentation files (the "Software"), to deal	*
 *	in the Software without restriction, including without limitation the rights	*
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell		*
 *	copies of the Software, and to permit persons to whom the Software is			*
 *	furnished to do so, subject to the following conditions:						*
 *																					*
 *	The above copyright notice and this permission notice shall be included in all	*
 *	copies or substantial portions of the Software.									*
 *																					*
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR		*
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,		*
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE		*
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER			*
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,	*
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE	*
 *	SOFTWARE.																		*
 *																					*
 ************************************************************************************/

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

// Function for enabling the traffic and pedestrian crossing of the passed "cardinal" directions, see pro1_lights.c for underlying logic
void activateTraffic(enum Street t_dir, enum Street p_dir) {
	trafficLight(RED, t_dir);			// Set traffic LED of the input direction to RED, NOTE: Should already be red, just a safety mechanism, COULD POTENTIALLY BE REMOVED
	trafficLight(ORANGE, t_dir);		// Set traffic LED of the input direction to ORANGE
	vTaskDelay(orangeDelay);			// Immediately after turning the LED orange, initiate a delay that keeps the LED orange for orangeDelay ms
	trafficLight(GREEN, t_dir);			// Once the orangeDelay ms delay has passed, set traffic LED of the input direction to GREEN
	pedestrianLight(GREEN, p_dir);		// Set the pedestrian LED of the input direction to GREEN, input crossing that does not cross the input traffic direction
}

// Function for disabling the traffic and pedestrian crossing of the passed "cardinal" directions, see pro1_lights.c for underlying logic
void disableTraffic(enum Street t_dir, enum Street p_dir) {
	trafficLight(GREEN, t_dir);			// Set traffic LED of the input direction to GREEN, NOTE: Should already be green, just a safety mechanism, COULD POTENTIALLY BE REMOVED
	pedestrianLight(RED, p_dir);		// Set the pedestrian LED of the input direction to RED, input crossing that does not cross the input traffic direction
	vTaskDelay(safetyDelay);			// Activate a safety delay that allows any potential final cars in a lane to turn right before the traffic lights turn red
	trafficLight(ORANGE, t_dir);		// Set traffic LED of the input direction to ORANGE
	vTaskDelay(orangeDelay);			// Immediately after turning the LED orange, initiate a delay that keeps the LED orange for orangeDelay ms
	trafficLight(RED, t_dir);			// Set traffic LED of the input direction to RED,
	vTaskDelay(safetyDelay);			// Activate another safety delay that ensures that all crossings are red for a brief period of time, provides traffic harmony
}

// Function for keeping traffic STATIC in a scenario where there is active vehicle traffic in one direction and one direction only
void staticTraffic(){
	while(statusVehicle_E || statusVehicle_W) {			// While EAST and WEST vehicle switches indicate that there is traffic
		if(buttonWestFlag) {							// If the WEST PEDESTRIAN CROSSING BUTTON is triggered, then wait for pedestrianDelay ms
			vTaskDelay(pedestrianDelay);				// after waiting pedestrianDelay ms, break out of the while loop and proceed with next instruction (disabling the EAST and WEST traffic lights)
			break;
		} else if(statusVehicle_N || statusVehicle_S) {	// If the NORTH OR SOUTH vehicle switches indicate that there is traffic
			vTaskDelay(redDelayMax);					// then wait redDelayMax, before breaking out of the while loop and proceeding with next instruction (disabling EAST and WEST traffic lights)
			break;
		}												// Incase the while loop is not broken out of, proceed to recursively...
		trafficLight(GREEN, T_EASTWEST);				// Set the color of the EAST AND WEST traffic LEDs to GREEN
		pedestrianLight(GREEN, P_NORTH);				// Set the color of the NORTH pedestrian LEDs to GREEN
		vTaskDelay(sysDelay);							// Activate a brief system delay to aovid polling hiccups
		checkTraffic();									// Check current status of the vehicle switches, may be the case that neither EAST or WEST vehicle switches any traffic
	}
	while(statusVehicle_N || statusVehicle_S) {			// While NORTH and SOUTH vehicle switches indicate that there is traffic
		if(buttonNorthFlag) {							// If the NORTH PEDESTRIAN CROSSING BUTTON is triggered, then wait for pedestrianDelay ms
			vTaskDelay(pedestrianDelay);				// after waiting pedestrianDelay ms, break out of the while loop and proceed with next instruction (disabling the NORTH and SOUTH traffic lights)
			break;
		} else if(statusVehicle_E || statusVehicle_W) {	// If the EAST OR WEST vehicle switches indicate that there is traffic
			vTaskDelay(redDelayMax);					// then wait redDelayMax, before breaking out of the while loop and proceeding with next instruction (disabling NORTH and SOUTH traffic lights)
			break;
		}												// Incase the while loop is not broken out of, proceed to recursively...
		trafficLight(GREEN, T_NORTHSOUTH);				// Set the color of the NORTH AND SOUTH traffic LEDs to GREEN
		pedestrianLight(GREEN, P_WEST);					// Set the color of the WEST pedestrian LEDs to GREEN
		vTaskDelay(sysDelay);							// Activate a brief system delay to aovid polling hiccups
		checkTraffic();									// Check current status of the vehicle switches, may be the case that neither NORTH or SOUTH vehicle switches any traffic
	}
}
