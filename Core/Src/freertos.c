/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/*
 ************************************************************************************
 *  Created on: Dec 8, 2023															*
 *      Author: Love Mitteregger													*
 ************************************************************************************
 *																					*
 *	MIT License is applied for the following tasks, and the following tasks only:	*
 *	- StartIdle																		*
 *	- StartPedestrian																*
 *	- StartTraffic																	*
 *	- StartToggleW																	*
 *	- StartToggleN																	*
 *	- StartPedestrianB																*
 *	Defined in the code below within this 'freertos.c' file.						*
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
#include "pro1_funct.h"
#include "pro1_test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define RUN_TEST_PROGRAM
//#define RUN_TEST_IDLE
//#define RUN_TEST_PEDESTRIAN
//#define RUN_TEST_TRAFFIC
//#define RUN_TEST_TOGGLE
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

#ifdef RUN_TEST_PROGRAM
// Required program delays, in ms.
const TickType_t sysDelay = pdMS_TO_TICKS(800);			// used to avoid mutex hickup
const TickType_t toggleFreq = pdMS_TO_TICKS(250);		// arguments is ms, then coverted to ticks
const TickType_t walkingDelay = pdMS_TO_TICKS(12000);	// Real life ?? ms 3 for testing 17 for simulating
const TickType_t pedestrianDelay = pdMS_TO_TICKS(3000);
const TickType_t safetyDelay = pdMS_TO_TICKS(3000);		// Real life ~6000 ms
const TickType_t greenDelay = pdMS_TO_TICKS(8000); 		// Real life ~47000 ms
const TickType_t orangeDelay = pdMS_TO_TICKS(2500); 	// Real life ~5000 ms
const TickType_t redDelayMax = pdMS_TO_TICKS(500);		// Real life ?? ms
const TickType_t testingDelay = pdMS_TO_TICKS(500); // ms to ticks
const TickType_t testingDelay2 = pdMS_TO_TICKS(80000); // ms to ticks

long startTime;
long endTime;
long elapsedTime;

extern volatile uint8_t buttonNorthFlag;
extern volatile uint8_t buttonWestFlag;

volatile uint8_t statusPedestrian_N = 0;
volatile uint8_t statusPedestrian_W = 1;
volatile uint8_t statusTraffic_NS = 1;
volatile uint8_t statusTraffic_EW = 0;

volatile uint8_t statusVehicle_N = 0;
volatile uint8_t statusVehicle_S = 0;
volatile uint8_t statusVehicle_E = 0;
volatile uint8_t statusVehicle_W = 0;
uint8_t pendingTraffic = 0;

uint8_t testVar = 1;

#else
const TickType_t sysDelay = pdMS_TO_TICKS(800);			// used to avoid system hickup
const TickType_t mutexDelay = pdMS_TO_TICKS(500);		// used to avoid mutex hickup
const TickType_t toggleFreq = pdMS_TO_TICKS(250);		// arguments is ms, then coverted to ticks
const TickType_t walkingDelay = pdMS_TO_TICKS(3000);	// Real life ?? ms 3 for testing 17 for simulating
const TickType_t pedestrianDelay = pdMS_TO_TICKS(3000);
const TickType_t safetyDelay = pdMS_TO_TICKS(3000);		// Real life ~6000 ms
const TickType_t greenDelay = pdMS_TO_TICKS(8000); 		// Real life ~47000 ms
const TickType_t orangeDelay = pdMS_TO_TICKS(2500); 	// Real life ~5000 ms
const TickType_t redDelayMax = pdMS_TO_TICKS(500);		// Real life ?? ms

long startTime;
long endTime;
long elapsedTime;

extern volatile uint8_t buttonNorthFlag;
extern volatile uint8_t buttonWestFlag;

volatile uint8_t statusPedestrian_N = 0;
volatile uint8_t statusPedestrian_W = 1;
volatile uint8_t statusTraffic_NS = 1;
volatile uint8_t statusTraffic_EW = 0;
volatile uint8_t lastActive_NS = 1;

volatile uint8_t statusVehicle_N = 0;
volatile uint8_t statusVehicle_S = 0;
volatile uint8_t statusVehicle_E = 0;
volatile uint8_t statusVehicle_W = 0;
uint8_t pendingTraffic = 0;

#endif
/* USER CODE END Variables */
/* Definitions for idleTask */
osThreadId_t idleTaskHandle;
const osThreadAttr_t idleTask_attributes = {
		.name = "idleTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for pedestrianTask */
osThreadId_t pedestrianTaskHandle;
const osThreadAttr_t pedestrianTask_attributes = {
		.name = "pedestrianTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for trafficTask */
osThreadId_t trafficTaskHandle;
const osThreadAttr_t trafficTask_attributes = {
		.name = "trafficTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for toggleW */
osThreadId_t toggleWHandle;
const osThreadAttr_t toggleW_attributes = {
		.name = "toggleW",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for toggleN */
osThreadId_t toggleNHandle;
const osThreadAttr_t toggleN_attributes = {
		.name = "toggleN",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for pedestrianTaskB */
osThreadId_t pedestrianTaskBHandle;
const osThreadAttr_t pedestrianTaskB_attributes = {
		.name = "pedestrianTaskB",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for mutex */
osMutexId_t mutexHandle;
const osMutexAttr_t mutex_attributes = {
		.name = "mutex"
};
/* Definitions for buttonMutex */
osMutexId_t buttonMutexHandle;
const osMutexAttr_t buttonMutex_attributes = {
		.name = "buttonMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartIdle(void *argument);
void StartPedestrian(void *argument);
void StartTraffic(void *argument);
void StartToggleW(void *argument);
void StartToggleN(void *argument);
void StartPedestrianB(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */
	/* USER CODE END Init */
	/* Create the mutex(es) */
	/* creation of mutex */
	mutexHandle = osMutexNew(&mutex_attributes);

	/* creation of buttonMutex */
	buttonMutexHandle = osMutexNew(&buttonMutex_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of idleTask */
	idleTaskHandle = osThreadNew(StartIdle, NULL, &idleTask_attributes);

	/* creation of pedestrianTask */
	pedestrianTaskHandle = osThreadNew(StartPedestrian, NULL, &pedestrianTask_attributes);

	/* creation of trafficTask */
	trafficTaskHandle = osThreadNew(StartTraffic, NULL, &trafficTask_attributes);

	/* creation of toggleW */
	toggleWHandle = osThreadNew(StartToggleW, NULL, &toggleW_attributes);

	/* creation of toggleN */
	toggleNHandle = osThreadNew(StartToggleN, NULL, &toggleN_attributes);

	/* creation of pedestrianTaskB */
	pedestrianTaskBHandle = osThreadNew(StartPedestrianB, NULL, &pedestrianTaskB_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartIdle */
/**
 * @brief  Function implementing the idleTask thread.
 * @param  argument: Not used
 * @retval None
 *
 * The StartIdle task is assigned a priority level of 9 out of 48.
 *
 * The purpose of the task is to simulate a state where there are no
 * active cars in any direction, and no pedestrians who wish to cross
 * at a given point of time.
 *
 * Despite no external activity, the idle task circulates between
 * the enable and disable traffic protocols, pending the active
 * traffic and pedestrian lights of each street direction
 * synchroniously, without keeping traffic enabled in both
 * directions simultaneously.
 *
 * The conditions for the StartIdle task to run is that no other
 * higher priority task currently is executing a larger set of
 * instructions (occupying resources).
 *
 * Furthermore, the functions within the task are only executed if
 * the StartIdle task has been given the mutex semaphore or if
 * there is no pending traffic in any direction and if neither the
 * pedestrian crossing buttons have been activated. Note that
 * the StartIdle task can of course run while there is pending
 * traffic, or either of the pedestrian corssing buttons have been
 * activated, if it has already taken the mutex semaphore.
 *
 * The StartIdle task ensures that an enable or disable sequence
 * completes for a given direction, before it can be interrupted by
 * a higher priority task. This is done by giving the StartIdle
 * task the mutexHandle semaphore, preventing the other tasks to
 * utilize the resources of the microprocessor unit.
 *
 * Once a sequence is completed and the traffic lights of a
 * direction is set, the mutex is returned and available
 * for another task to take.
 *
 * Another task may then take the mutex as it is returned and,
 * for example, renable the traffic lights of the direction that just
 * were disabled, before returning the mutex and allowing for the
 * StartIdle task to retake the mutex and proceeding with its next
 * instruction.
 *
 * If this occurs, there may be a risk that the StartIdle tasks next
 * in line instruction is set to enable the traffic lights of the
 * opposite direction of the renabled direction, as the StartIdle task
 * previously had deactiavated the renabled direction. Causing all
 * traffic lights to go green, resulting in havoc.
 *
 * To avoid that the green lights of a traffic direction is enabled while
 * the opposite traffic direction already is enabled, due to for example
 * the routine of another task that for a while took the mutex, a check
 * is implemented to confirm that the traffic lights of the opposite
 * direction are indeed already turned red, before proceeding with
 * sequence for enabling the traffic lights for the specific direction.
 *
 */
/* USER CODE END Header_StartIdle */
void StartIdle(void *argument)
{
	/* USER CODE BEGIN StartIdle */
	/* Infinite loop */
	while(1)
	{

#ifdef RUN_TEST_IDLE
		if( !pendingTraffic && (!buttonWestFlag || !buttonNorthFlag)) {
			if (statusTraffic_NS && !buttonNorthFlag) {
				xSemaphoreTake(mutexHandle, 0);
				disableTraffic_Test(T_NORTHSOUTH, P_WEST);
				xSemaphoreGive(mutexHandle);
				vTaskDelay(testingDelay);
				xSemaphoreTake(mutexHandle, portMAX_DELAY);
				if (!statusTraffic_NS) {
					activateTraffic_Test(T_EASTWEST, P_NORTH);
				}
				xSemaphoreGive(mutexHandle);
			} else if (statusTraffic_EW && !buttonWestFlag){
				xSemaphoreTake(mutexHandle, 0);
				disableTraffic_Test(T_EASTWEST, P_NORTH);
				xSemaphoreGive(mutexHandle);
				vTaskDelay( testingDelay );
				xSemaphoreTake(mutexHandle, portMAX_DELAY);
				if (!statusTraffic_EW) {
					activateTraffic_Test(T_NORTHSOUTH, P_WEST);
				}
				xSemaphoreGive(mutexHandle);
			}
			vTaskDelay(greenDelay);
		}
#else
		// Recursive while loop that first checks if there is NOT any pending traffic NOR any activated pedestrian crossing buttons, before proceeding
		if( !pendingTraffic && (!buttonWestFlag || !buttonNorthFlag) ) {
			// If there currently is traffic in the NORTH and SOUTH direction, and the NORTH pedestrian crossing buttons have not been activated
			if (statusTraffic_NS && !buttonNorthFlag) {
				xSemaphoreTake(mutexHandle, 0);				// Take the mutexHandle semaphore IF it is available AT THIS VERY INSTANT
				disableTraffic(T_NORTHSOUTH, P_WEST);		// If mutex could be taken, proceed with disable sequence for the NORTH and SOUTH direction and WEST pedestrian crossing
				xSemaphoreGive(mutexHandle);				// Give mutex back for any other task to take
				vTaskDelay(mutexDelay);						// Small delay to ensure that the MPU properly gives back the mutex semaphore
				xSemaphoreTake(mutexHandle, portMAX_DELAY);	// If mutex is available, take it again, else WAIT INDEFINITELY until the mutex semaphore becomes available
				if (!statusTraffic_NS) {					// When mutex finally is taken, confirm that there currently is not enabled traffic in the NORTH and SOUTH direction
					activateTraffic(T_EASTWEST, P_NORTH);	// If traffic in NORTH and SOUTH direction is disabled, then enable traffic in EAST and WEST direction and NORTH pedestrian crossing
				}
				xSemaphoreGive(mutexHandle);				// Return the semaphore for other tasks to take
			// Else, if there currently is traffic in the EAST and WEST direction, and the WEST pedestrian crossing buttons have not been activated
			} else if (statusTraffic_EW && !buttonWestFlag){
				xSemaphoreTake(mutexHandle, 0);				// Take the mutexHandle semaphore IF it is available AT THIS VERY INSTANT
				disableTraffic(T_EASTWEST, P_NORTH);		// If mutex could be taken, proceed with disable sequence for the EAST and WEST direction and NORTH pedestrian crossing
				xSemaphoreGive(mutexHandle);				// Give mutex back for any other task to take
				vTaskDelay(mutexDelay);						// Small delay to ensure that the MPU properly gives back the mutex semaphore
				xSemaphoreTake(mutexHandle, portMAX_DELAY); // If mutex is available, take it again, else WAIT INDEFINITELY until the mutex semaphore becomes available
				if (!statusTraffic_EW) {					// When mutex finally is taken, confirm that there currently is not enabled traffic in the EAST and WEST direction
					activateTraffic(T_NORTHSOUTH, P_WEST);	// If traffic in EAST and WEST direction is disabled, then enable traffic in NORTH and SOUTH direction and WEST pedestrian crossing
				}
				xSemaphoreGive(mutexHandle);				// Return the semaphore for other tasks to take
			}
			vTaskDelay(greenDelay);							// Either the NORT and SOUTH or EAST and WEST directions have been enabled, the traffic lights should remain GREEN for greenDelay ms
		}													// before the StartIdle task recursively runs for another loop and cycles the traffic lights statuses
#endif
		osDelay(1);
	}
	/* USER CODE END StartIdle */
}

/* USER CODE BEGIN Header_StartPedestrian */
/**
 * @brief Function implementing the pedestrianTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPedestrian */
void StartPedestrian(void *argument)
{
	/* USER CODE BEGIN StartPedestrian */
	/* Infinite loop */
	for(;;)
	{
#ifdef RUN_TEST_PEDESTRIAN
		if (buttonWestFlag || buttonNorthFlag) {
			xSemaphoreTake(mutexHandle, portMAX_DELAY);
			if (buttonWestFlag && !buttonNorthFlag) {
				if (statusPedestrian_W) {
					buttonWestFlag = 0;
				} else if (statusTraffic_EW) {
					disableTraffic_Test(T_EASTWEST, P_NORTH);
					activateTraffic_Test(T_NORTHSOUTH, P_WEST);
					vTaskDelay(walkingDelay);
				} else if (!statusTraffic_NS && !statusTraffic_EW) {
					activateTraffic_Test(T_NORTHSOUTH, P_WEST);
					vTaskDelay(walkingDelay);
				}
			} else if (!buttonWestFlag && buttonNorthFlag) {
				if (statusPedestrian_N) {
					buttonWestFlag = 0;
				} else if (statusTraffic_NS) {
					disableTraffic_Test(T_NORTHSOUTH, P_WEST);
					activateTraffic_Test(T_EASTWEST, P_NORTH);
					vTaskDelay(walkingDelay);
				} else if (!statusTraffic_NS && !statusTraffic_EW) {
					activateTraffic_Test(T_EASTWEST, P_NORTH);
					vTaskDelay(walkingDelay);
				}

			} else if (buttonWestFlag && buttonNorthFlag) {
				if (statusPedestrian_W) {
					buttonWestFlag = 0;
				} else if (statusPedestrian_N) {
					buttonWestFlag = 0;
				}
			}
			xSemaphoreGive(mutexHandle);
			vTaskDelay(sysDelay);
		}
#else
		if (buttonWestFlag || buttonNorthFlag) {
			//if (buttonWestFlag) {
			xSemaphoreTake(mutexHandle, portMAX_DELAY);
			if (buttonWestFlag && buttonNorthFlag) {
				if (statusPedestrian_W) {
					buttonWestFlag = 0;
				} else if (statusPedestrian_N) {
					buttonNorthFlag = 0;
				} else if (lastActive_NS && !statusTraffic_EW && !statusTraffic_NS) {
					activateTraffic(T_EASTWEST, P_NORTH);
					vTaskDelay(walkingDelay);
				} else if (lastActive_NS && statusPedestrian_N) {
					buttonNorthFlag = 0;
				} else if (!lastActive_NS && !statusTraffic_EW && !statusTraffic_NS) {
					activateTraffic(T_NORTHSOUTH, P_WEST);
					vTaskDelay(walkingDelay);
				}  else if (!lastActive_NS && statusPedestrian_W) {
					buttonWestFlag = 0;
				}
			} else if (buttonWestFlag && !buttonNorthFlag) {
				if (statusPedestrian_W) {
					buttonWestFlag = 0;
				} else if (statusTraffic_EW) {
					disableTraffic(T_EASTWEST, P_NORTH);
					activateTraffic(T_NORTHSOUTH, P_WEST);
					vTaskDelay(walkingDelay);
				} else if (!statusTraffic_NS && !statusTraffic_EW) {
					activateTraffic(T_NORTHSOUTH, P_WEST);
					vTaskDelay(walkingDelay);
				}
			} else if (!buttonWestFlag && buttonNorthFlag) {
				if (statusPedestrian_N) {
					buttonWestFlag = 0;
				} else if (statusTraffic_NS) {
					disableTraffic(T_NORTHSOUTH, P_WEST);
					activateTraffic(T_EASTWEST, P_NORTH);
					vTaskDelay(walkingDelay);
				} else if (!statusTraffic_NS && !statusTraffic_EW) {
					activateTraffic(T_EASTWEST, P_NORTH);
					vTaskDelay(walkingDelay);
				}
			 /*else if (buttonWestFlag && buttonNorthFlag) {
				if (statusPedestrian_W) {
					buttonWestFlag = 0;
				} else if (statusPedestrian_N) {
					buttonNorthFlag = 0;
				} else if (lastActive_NS && !statusTraffic_EW && !statusTraffic_NS) {
					activateTraffic(T_EASTWEST, P_NORTH);
					vTaskDelay(walkingDelay);
				} else if (!lastActive_NS && !statusTraffic_EW && !statusTraffic_NS) {
					activateTraffic(T_NORTHSOUTH, P_WEST);
					vTaskDelay(walkingDelay);
				}
			 */
			//vTaskDelay(sysDelay);
		}
		vTaskDelay(sysDelay);
		xSemaphoreGive(mutexHandle);
	}
#endif
	osDelay(1);
	}
/* USER CODE END StartPedestrian */
}

/* USER CODE BEGIN Header_StartTraffic */
/**
 * @brief Function implementing the trafficTask thread.
 * @param argument: Not used
 * @retval None
 *
 * The StartTraffic task is assigned a priority level 17 out of 48
 *
 * The purpose of the task is to simulate a state where there are
 * active vehicles in any direction or multiple directions
 * simultaneously, and no pedestrians who wish to cross at a
 * given point of time.
 *
 * The conditions for the StartIdle task to run is that no other
 * higher priority task currently is executing a larger set of
 * instructions (occupying resources).
 *
 * Furthermore, the functions within the task are only executed if
 * the polling conducted by the checkTraffic function returns that
 * there is indeed active traffic at a given moment of time. Again,
 * the mutex semaphore must be taken to continue executing tasks.
 * If the mutex semaphore is not available, the system will wait
 * INDEFINETIELY until it is.
 *
 * The mutex is kept until every possible traffic scenario has been
 * checked, and executed if valid.
 *
 * The possible traffic scenarios are as follows:
 * 1.	Traffic lights in all street directions are RED, and a vehicle
 * 		appears from ONE direction.
 *
 * 2.	Traffic lights remain GREEN if there are active cars in either
 * 		allowed direction and NO active cars are waiting on red traffic lights.
 * 		-	Note that a subscenario to consider in scenario 2:
 * 			The sub scenario is that a car arrives at a red light
 * 			while there are active cars in either allowed direction.
 *
 * 3.	A car arrives at a RED light and there are NO a active cars in
 * 		either allowed direction, causing the RED traffic lights to
 * 		immediately transition to GREEN.
 *
 * 4.	Congestion, which requires cycling between currently enabled direction
 * 		similar to that of the idle task.
 */
/* USER CODE END Header_StartTraffic */
void StartTraffic(void *argument)
{
	/* USER CODE BEGIN StartTraffic */
	/* Infinite loop */
	while(1)
	{

#ifdef RUN_TEST_TRAFFIC
		pendingTraffic = checkTraffic_Test();
		if ( pendingTraffic ) {
			xSemaphoreTake(mutexHandle, portMAX_DELAY);
			// If all roads are red, and a vehicle appears from ONE direction
			if ( (!statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_N || statusVehicle_S) && (!statusVehicle_E && !statusVehicle_W)) {
				activateTraffic_Test(T_NORTHSOUTH, P_WEST);
				vTaskDelay(greenDelay);
			} else if ( (!statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_E || statusVehicle_W ) && (!statusVehicle_N && !statusVehicle_S)) {
				activateTraffic_Test(T_EASTWEST, P_NORTH);
				vTaskDelay(greenDelay);
				// R2.5 A traffic light remains green if there are active cars in either allowed direction and no active cars are waiting on red traffic lights
			} else if ( (statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_N || statusVehicle_S) && (!statusVehicle_E && !statusVehicle_W)) {
				if (statusTraffic_NS && !buttonNorthFlag) {
					staticTraffic_Test(); // R2.6 is considered within the staticTraffic_Test function
				} else if (buttonNorthFlag) {
					disableTraffic_Test(T_NORTHSOUTH, P_WEST);
					activateTraffic_Test(T_EASTWEST, P_NORTH);
					pedestrianReset_Test(P_NORTH);
				} else {
					activateTraffic_Test(T_NORTHSOUTH, P_WEST);
					staticTraffic_Test();
				}
			} else if ( (!statusTraffic_NS && statusTraffic_EW) && (statusVehicle_E || statusVehicle_W ) && (!statusVehicle_N && !statusVehicle_S)) {
				if (statusTraffic_EW && !buttonWestFlag) {
					staticTraffic_Test(); // R2.6 is considered within the staticTraffic_Test function
				} else if (buttonWestFlag) {
					disableTraffic_Test(T_EASTWEST, P_NORTH);
					activateTraffic_Test(T_NORTHSOUTH, P_WEST);
					pedestrianReset_Test(P_WEST);
				} else {
					activateTraffic_Test(T_EASTWEST, P_NORTH);
					staticTraffic_Test();
				}
				// R2.7 If a car arrives at a red light and there are no active cars in either allowed direction, the signal transitions immediately to green
			} else if ( (!statusTraffic_NS && statusTraffic_EW) && (statusVehicle_N || statusVehicle_S) && (!statusVehicle_E && !statusVehicle_W)) {
				disableTraffic_Test(T_EASTWEST, P_NORTH);
				activateTraffic_Test(T_NORTHSOUTH, P_WEST);
			} else if ( (statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_E || statusVehicle_W) && (!statusVehicle_N && !statusVehicle_S)) {
				disableTraffic_Test(T_NORTHSOUTH, P_WEST);
				activateTraffic_Test(T_EASTWEST, P_NORTH);
				// Congestion - Conduct almost same logic as forever loop
			} else if ( (statusVehicle_E || statusVehicle_W ) && (statusVehicle_N || statusVehicle_S)) {
				if (statusTraffic_NS) {
					disableTraffic_Test(T_NORTHSOUTH, P_WEST);
					activateTraffic_Test(T_EASTWEST, P_NORTH);
				} else {
					disableTraffic_Test(T_EASTWEST, P_NORTH);
					activateTraffic_Test(T_NORTHSOUTH, P_WEST);
				}
				vTaskDelay(greenDelay);	// Must be placed last, if not then redDelayMax logic will not work properly.
			}
			xSemaphoreGive(mutexHandle);
		}
#else
		pendingTraffic = checkTraffic();									// Start by checking the current status of all vehicles
		if ( pendingTraffic ) {												// If there are any vehicles enabled
			xSemaphoreTake(mutexHandle, portMAX_DELAY);						// Wait INDEFINITELY to take the mutex and proceed with any of the scenarios below once taken

			// SCENARIO 1: Traffic lights in all street directions are RED, and a vehicle appears from ONE direction
			// If NEITHER DIRECTION is ENABLED and there is traffic from the NORTH OR SOUTH
			if ( (!statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_N || statusVehicle_S) && (!statusVehicle_E && !statusVehicle_W) ) {
				activateTraffic(T_NORTHSOUTH, P_WEST);						// Enable traffic in NORTH and SOUTH direction, and the WEST pedestrian crossing
				vTaskDelay(greenDelay);										// Hold the lights green for greenDelay ms.
			// If NEITHER DIRECTION is ENABLED and there is traffic from the EAST OR WEST
			} else if ( (!statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_E || statusVehicle_W ) && (!statusVehicle_N && !statusVehicle_S) ) {
				activateTraffic(T_EASTWEST, P_NORTH);						// Enable traffic in EAST and WEST direction, and the NORTH pedestrian crossing
				vTaskDelay(greenDelay);										// Hold the lights green for greenDelay ms.


			// SCENARIO 2 (R2.5): Traffic lights remain green if there are active cars in either allowed direction and no active cars are waiting on red traffic lights
			// If there is only traffic in the NORTH and SOUTH direction, and it is caused by either the NORTH or SOUTH vehicle only
			} else if ( (statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_N || statusVehicle_S) && (!statusVehicle_E && !statusVehicle_W) ) {
				// Then confirm that there is no pedestrian that wants to cross the NORTH pedestrian crossing AND that the traffic of the NORTH and SOUTH direction is enabled
				if (statusTraffic_NS && !buttonNorthFlag) {					// If there is no pedestrian that wish to cross, keep the traffic static until a vehicle appears in ooposite direction
					staticTraffic(); 										// Subscenario 2 (R2.6), where a vehicle appears in opposite direction, is handled within the staticTraffic function
				// If a pedestrian suddenly wish to cross the NORTH pedestrian crossing
				} else if (buttonNorthFlag) {
					disableTraffic(T_NORTHSOUTH, P_WEST);					// Proceed by disabling traffic in NORTH and SOUTH direction, and for the WEST pedestrian crossing
					activateTraffic(T_EASTWEST, P_NORTH);					// Enable traffic in EAST and WEST direction, and the NORTH pedestrian crossing
					pedestrianReset(P_NORTH);								// Disable the BLUE NORTH crossing pending pedestrian indicator light by resetting it
				// Else the traffic of the NORTH and SOUTH direction is not yet enabled and as such must be anbled before being kept at a static state
				} else {
					activateTraffic(T_NORTHSOUTH, P_WEST);					// Enable traffic in NORTH and SOUTH direction, and the WEST pedestrian crossing
					staticTraffic();										// Keep the traffic static until either a pedestrian or vehicle wants to cross the current traffic direction
				}
			// If there is only traffic in the EAST and WEST direction, and it is caused by either the EAST or WEST vehicle only
			} else if ( (!statusTraffic_NS && statusTraffic_EW) && (statusVehicle_E || statusVehicle_W ) && (!statusVehicle_N && !statusVehicle_S) ) {
				// Then confirm that there is no pedestrian that wants to cross the WEST pedestrian crossing AND that the traffic of the EAST and WEST direction is enabled
				if (statusTraffic_EW && !buttonWestFlag) {
					staticTraffic(); 										// Subscenario 2 (R2.6), where a vehicle appears in opposite direction, is handled within the staticTraffic function
					// If a pedestrian suddenly wish to cross the WEST pedestrian crossing
				} else if (buttonWestFlag) {
					disableTraffic(T_EASTWEST, P_NORTH);					// Proceed by disabling traffic in EAST and WEST direction, and for the NORTH pedestrian crossing
					activateTraffic(T_NORTHSOUTH, P_WEST);					// Enable traffic in NORTH and SOUTH direction, and the WEST pedestrian crossing
					pedestrianReset(P_WEST);								// Disable the BLUE WEST crossing pending pedestrian indicator light by resetting it
				// Else the traffic of the EAST and WEST direction is not yet enabled and as such must be anbled before being kept at a static state
				} else {
					activateTraffic(T_EASTWEST, P_NORTH);					// Enable traffic in EAST and WEST direction, and the NORTH pedestrian crossing
					staticTraffic();										// Keep the traffic static until either a pedestrian or vehicle wants to cross the current traffic direction
				}


			// SCENARIO 3 (R2.7): A car arrives at a RED light and there are NO a active cars in either allowed direction, causing the RED traffic lights to immediately transition to GREEN
			// If traffic is enabled for the EAST and WEST direction, and vehicles appear from NORTH and SOUTH when there are no vehicles approaching from EAST or WEST
			} else if ( (!statusTraffic_NS && statusTraffic_EW) && (statusVehicle_N || statusVehicle_S) && (!statusVehicle_E && !statusVehicle_W)) {
				disableTraffic(T_EASTWEST, P_NORTH);						// Proceed by disabling traffic in EAST and WEST direction, and for the NORTH pedestrian crossing
				activateTraffic(T_NORTHSOUTH, P_WEST);						// Enable traffic in NORTH and SOUTH direction, and the WEST pedestrian crossing
			// If traffic is enabled for the NORTH and SOUTH direction, and vehicles appear from EAST and WEST when there are no vehicles approaching from NORTH or SOUTH
			} else if ( (statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_E || statusVehicle_W) && (!statusVehicle_N && !statusVehicle_S)) {
				disableTraffic(T_NORTHSOUTH, P_WEST);						// Proceed by disabling traffic in NORTH and SOUTH direction, and for the WEST pedestrian crossing
				activateTraffic(T_EASTWEST, P_NORTH);						// Enable traffic in EAST and WEST direction, and the NORTH pedestrian crossing


			// SCENARIO 4: Congestion, which requires cycling between currently enabled direction, similar to the logic of the idle task
			// If there are vehicles appearing from every direction
			} else if ( (statusVehicle_E || statusVehicle_W ) && (statusVehicle_N || statusVehicle_S)) {
				// Confirm if the currently enabled traffic direction is NORTH and SOUTH
				if (statusTraffic_NS) {
					disableTraffic(T_NORTHSOUTH, P_WEST);					// Proceed by disabling traffic in NORTH and SOUTH direction, and for the WEST pedestrian crossing
					activateTraffic(T_EASTWEST, P_NORTH);					// Enable traffic in EAST and WEST direction, and the NORTH pedestrian crossing
				// If currently enabled traffic direction is NOT NORTH and SOUTH, it must be EAST and WEST
				} else {
					disableTraffic(T_EASTWEST, P_NORTH);					// Proceed by disabling traffic in EAST and WEST direction, and for the NORTH pedestrian crossing
					activateTraffic(T_NORTHSOUTH, P_WEST);					// Enable traffic in NORTH and SOUTH direction, and the WEST pedestrian crossing
				}
				vTaskDelay(greenDelay); 									// Must be placed last, if not then redDelayMax logic will not work properly.

			// END OF ALL SCENARIOS
			}
			xSemaphoreGive(mutexHandle);									// Return mutex for other tasks to take
		}
#endif
		osDelay(1);
	}
	/* USER CODE END StartTraffic */
}

/* USER CODE BEGIN Header_StartToggleW */
/**
 * @brief Function implementing the toggleW thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartToggleW */
void StartToggleW(void *argument)
{
	/* USER CODE BEGIN StartToggleW */
	/* Infinite loop */
	for(;;)
	{
		if(statusPedestrian_W || !buttonWestFlag) {
			pedestrianReset(P_WEST);
		} else if (buttonWestFlag) {
			pedestrianPending(P_WEST);
			vTaskDelay(toggleFreq);
		}
		osDelay(1);
	}
	/* USER CODE END StartToggleW */
}

/* USER CODE BEGIN Header_StartToggleN */
/**
 * @brief Function implementing the toggleN thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartToggleN */
void StartToggleN(void *argument)
{
	/* USER CODE BEGIN StartToggleN */
	/* Infinite loop */
	for(;;)
	{
		if(statusPedestrian_N || !buttonNorthFlag) {
			pedestrianReset(P_NORTH);
		} else if (buttonNorthFlag) {
			pedestrianPending(P_NORTH);
			vTaskDelay(toggleFreq);
		}
		osDelay(1);
	}
	/* USER CODE END StartToggleN */
}

/* USER CODE BEGIN Header_StartPedestrianB */
/**
 * @brief Function implementing the pedestrianTaskB thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPedestrianB */
void StartPedestrianB(void *argument)
{
	/* USER CODE BEGIN StartPedestrianB */
	/* Infinite loop */
	for(;;)
	{
		/*
		if (buttonNorthFlag) {
			xSemaphoreTake(mutexHandle, portMAX_DELAY);
			if (!buttonWestFlag && buttonNorthFlag) {
				if (statusPedestrian_N) {
					buttonNorthFlag = 0;
				} else if (statusTraffic_NS) {
					disableTraffic(T_NORTHSOUTH, P_WEST);
					activateTraffic(T_EASTWEST, P_NORTH);
					vTaskDelay(walkingDelay);
				} else if (!statusTraffic_NS && !statusTraffic_EW) {
					activateTraffic(T_EASTWEST, P_WEST);
					vTaskDelay(walkingDelay);
				}
			}
			xSemaphoreGive(mutexHandle);
			vTaskDelay(sysDelay);
		}
		 */
		osDelay(1);
	}
	/* USER CODE END StartPedestrianB */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

