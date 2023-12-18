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
#define RUN_TEST_PROGRAM
#define RUN_TEST_IDLE
#define RUN_TEST_PEDESTRIAN
#define RUN_TEST_TRAFFIC
#define RUN_TEST_TOGGLE
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
const TickType_t walkingDelay = pdMS_TO_TICKS(3000);	// Real life ?? ms 3 for testing 17 for simulating
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

volatile uint8_t statusTraffic_NS = 1;
volatile uint8_t statusTraffic_EW = 0;
volatile uint8_t statusPedestrian_N = 0;
volatile uint8_t statusPedestrian_W = 1;

volatile uint8_t statusVehicle_N = 0;
volatile uint8_t statusVehicle_S = 0;
volatile uint8_t statusVehicle_E = 0;
volatile uint8_t statusVehicle_W = 0;
uint8_t pendingTraffic = 0;

uint8_t testVar = 1;

#else
const TickType_t sysDelay = pdMS_TO_TICKS(500);			// used to avoid mutex hickup
const TickType_t toggleFreq = pdMS_TO_TICKS(250);		// arguments is ms, then coverted to ticks
const TickType_t walkingDelay = pdMS_TO_TICKS(17000);	// Real life ?? ms
const TickType_t pedestrianDelay = pdMS_TO_TICKS(3000);
const TickType_t safetyDelay = pdMS_TO_TICKS(3000);		// Real life ~6000 ms
const TickType_t greenDelay = pdMS_TO_TICKS(8000); 		// Real life ~47000 ms
const TickType_t orangeDelay = pdMS_TO_TICKS(2500); 	// Real life ~5000 ms
const TickType_t redDelayMax = pdMS_TO_TICKS(500);		// Real life ?? ms

TickType_t startTime;
TickType_t endTime;
TickType_t elapsedTime;

volatile uint8_t statusTraffic_NS = 1;
volatile uint8_t statusTraffic_EW = 0;
volatile uint8_t statusPedestrian_N = 0;
volatile uint8_t statusPedestrian_W = 1;

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
/* Definitions for pedestrianTaskB */
osThreadId_t pedestrianTaskBHandle;
const osThreadAttr_t pedestrianTaskB_attributes = {
		.name = "pedestrianTaskB",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityHigh,
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
void StartPedestrianB(void *argument);
void StartToggleW(void *argument);
void StartToggleN(void *argument);

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

	/* creation of pedestrianTaskB */
	pedestrianTaskBHandle = osThreadNew(StartPedestrianB, NULL, &pedestrianTaskB_attributes);

	/* creation of toggleW */
	toggleWHandle = osThreadNew(StartToggleW, NULL, &toggleW_attributes);

	/* creation of toggleN */
	toggleNHandle = osThreadNew(StartToggleN, NULL, &toggleN_attributes);

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
		if(!pendingTraffic) {
			if (statusTraffic_NS) {
				xSemaphoreTake(mutexHandle, 0);
				disableTraffic(T_NORTHSOUTH, P_WEST);
				xSemaphoreGive(mutexHandle);
				vTaskDelay(sysDelay);
				xSemaphoreTake(mutexHandle, portMAX_DELAY);
				if (!statusTraffic_NS) {
					activateTraffic(T_EASTWEST, P_NORTH);
				}
				xSemaphoreGive(mutexHandle);
			} else {
				xSemaphoreTake(mutexHandle, 0);
				disableTraffic(T_EASTWEST, P_NORTH);
				xSemaphoreGive(mutexHandle);
				vTaskDelay(sysDelay);
				xSemaphoreTake(mutexHandle, portMAX_DELAY);
				if (!statusTraffic_EW) {
					activateTraffic(T_NORTHSOUTH, P_WEST);
				}
				xSemaphoreGive(mutexHandle);
			}
			vTaskDelay(greenDelay);
		}
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

		//		vTaskDelay(sysDelay);
		if (buttonWestFlag || buttonNorthFlag) {
			xSemaphoreTake(mutexHandle, portMAX_DELAY);
			if (buttonWestFlag && !buttonNorthFlag) {
				if (statusPedestrian_W) {
					buttonWestFlag = 0;
				} else if (statusTraffic_EW) {
				//	vTaskResume(toggleWHandle);
					disableTraffic_Test(T_EASTWEST, P_NORTH);
					activateTraffic_Test(T_NORTHSOUTH, P_WEST);
					vTaskDelay(walkingDelay);
				//	pedestrianReset_Test(P_WEST);
				} else if (!statusTraffic_NS && !statusTraffic_EW) { // Works like a charm when singly pressed,
				//	vTaskResume(toggleWHandle);
					activateTraffic_Test(T_NORTHSOUTH, P_WEST);
					vTaskDelay(walkingDelay);
				//	pedestrianReset_Test(P_WEST);
				}
			} else if (!buttonWestFlag && buttonNorthFlag) {
				if (statusPedestrian_N) {
					buttonWestFlag = 0;
				} else if (statusTraffic_NS) {
				//	vTaskResume(toggleWHandle);
					disableTraffic_Test(T_NORTHSOUTH, P_WEST);
					activateTraffic_Test(T_EASTWEST, P_NORTH);
					vTaskDelay(walkingDelay);
			//		pedestrianReset_Test(P_NORTH);
				} else if (!statusTraffic_NS && !statusTraffic_EW) {
				//	vTaskResume(toggleWHandle);
					activateTraffic_Test(T_EASTWEST, P_NORTH);
					vTaskDelay(walkingDelay);
			//		pedestrianReset_Test(P_NORTH);
				}

			} else if (buttonWestFlag && buttonNorthFlag) {
				if (statusPedestrian_W) {
					buttonWestFlag = 0;
				} else if (statusPedestrian_N) {
					buttonWestFlag = 0;
				}
				//while (buttonWestFlag && buttonNorthFlag) {
				//	trafficLight_Test(ORANGE, P_NORTH);
				//	trafficLight_Test(ORANGE, P_WEST);
				//	vTaskDelay(toggleFreq);
				//	trafficLight_Test(GREEN, P_NORTH);
				//	trafficLight_Test(GREEN, P_WEST);
				//	vTaskDelay(toggleFreq);
				//}
				//vTaskResume(toggleWHandle);
				//vTaskResume(toggleNHandle);
				//if (statusTraffic_NS) {
				//	buttonWestFlag = 0;
				//} else if (statusTraffic_EW) {
				//	buttonNorthFlag = 0;
				//} else {
				//
				//}

			}
			xSemaphoreGive(mutexHandle);
			vTaskDelay(sysDelay);
		}
#else
		if ( buttonNorthFlag ) {
			while (!statusPedestrian_N) {
				xSemaphoreTake(buttonMutexHandle, 0);
				pedestrianPending_Test(P_NORTH);
				vTaskDelay( toggleFreq );
				xSemaphoreGive(buttonMutexHandle);
			}
			pedestrianReset_Test(P_NORTH);
			startTime = xTaskGetTickCount();
			pedestrianLight(GREEN, P_NORTH);
			while (elapsedTime < walkingDelay ) {
				endTime = xTaskGetTickCount();
				elapsedTime = endTime - startTime;
			}
			vTaskDelay(testingDelay);
			elapsedTime = 0;
			buttonNorthFlag = 0;
		} else if ( buttonWestFlag ) {
			while (!statusPedestrian_W) {
				xSemaphoreTake(buttonMutexHandle, 0);
				pedestrianPending_Test(P_WEST);
				vTaskDelay( toggleFreq );
				xSemaphoreGive(buttonMutexHandle);
			}
			pedestrianReset_Test(P_WEST);
			startTime = xTaskGetTickCount();
			pedestrianLight(GREEN, P_WEST);
			while (elapsedTime < walkingDelay ) {
				endTime = xTaskGetTickCount();
				elapsedTime = endTime -  startTime;
			}
			vTaskDelay(testingDelay);
			elapsedTime = 0;
			buttonWestFlag = 0;
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
			//vTaskDelay(testingDelay);
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
				vTaskDelay(greenDelay); // Must be placed last, if not then redDelayMax logic will not work properly.
			}
			xSemaphoreGive(mutexHandle);
		}
#else
		pendingTraffic = checkTraffic();
		if ( pendingTraffic ) {
			//vTaskDelay(testingDelay);
			xSemaphoreTake(mutexHandle, portMAX_DELAY);
			// If all roads are red, and a vehicle appears from ONE direction
			if ( (!statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_N || statusVehicle_S) && (!statusVehicle_E && !statusVehicle_W)) {
				activateTraffic(T_NORTHSOUTH, P_WEST);
				vTaskDelay(greenDelay);
			} else if ( (!statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_E || statusVehicle_W ) && (!statusVehicle_N && !statusVehicle_S)) {
				activateTraffic(T_EASTWEST, P_NORTH);
				vTaskDelay(greenDelay);
				// R2.5 A traffic light remains green if there are active cars in either allowed direction and no active cars are waiting on red traffic lights
			} else if ( (statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_N || statusVehicle_S) && (!statusVehicle_E && !statusVehicle_W)) {
				if (statusTraffic_NS) {
					staticTraffic(); // R2.6 is considered within the staticTraffic_Test function
				} else {
					activateTraffic(T_NORTHSOUTH, P_WEST);
					staticTraffic();
				}
			} else if ( (!statusTraffic_NS && statusTraffic_EW) && (statusVehicle_E || statusVehicle_W ) && (!statusVehicle_N && !statusVehicle_S)) {
				if (statusTraffic_EW) {
					staticTraffic(); // R2.6 is considered within the staticTraffic_Test function
				} else {
					activateTraffic(T_EASTWEST, P_NORTH);
					staticTraffic();
				}
				// R2.7 If a car arrives at a red light and there are no active cars in either allowed direction, the signal transitions immediately to green
			} else if ( (!statusTraffic_NS && statusTraffic_EW) && (statusVehicle_N || statusVehicle_S) && (!statusVehicle_E && !statusVehicle_W)) {
				disableTraffic(T_EASTWEST, P_NORTH);
				activateTraffic(T_NORTHSOUTH, P_WEST);
			} else if ( (statusTraffic_NS && !statusTraffic_EW) && (statusVehicle_E || statusVehicle_W) && (!statusVehicle_N && !statusVehicle_S)) {
				disableTraffic(T_NORTHSOUTH, P_WEST);
				activateTraffic(T_EASTWEST, P_NORTH);
				// Congestion - Conduct almost same logic as forever loop
			} else if ( (statusVehicle_E || statusVehicle_W ) && (statusVehicle_N || statusVehicle_S)) {
				if (statusTraffic_NS) {
					disableTraffic(T_NORTHSOUTH, P_WEST);
					activateTraffic(T_EASTWEST, P_NORTH);
				} else {
					disableTraffic(T_EASTWEST, P_NORTH);
					activateTraffic(T_NORTHSOUTH, P_WEST);
				}
				vTaskDelay(greenDelay); // Must be placed last, if not then redDelayMax logic will not work properly.
			}
			xSemaphoreGive(mutexHandle);
		}
#endif
		osDelay(1);
	}
	/* USER CODE END StartTraffic */
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
#ifdef RUN_TEST_PEDESTRIAN
		/*
		vTaskDelay(testingDelay);
		if (buttonWestFlag && statusPedestrian_N && !statusPedestrian_W) {
			// Button is pressed,
			while (!statusPedestrian_W && statusPedestrian_N) {
				pedestrianPending_Test(P_WEST);
				vTaskDelay( toggleFreq );
			}
			while (!statusPedestrian_N && !statusPedestrian_W) {
			//	xSemaphoreTake(buttonMutexHandle, 0);
				pedestrianPending_Test(P_WEST);
				if (buttonNorthFlag && !statusPedestrian_N) {
					pedestrianPending_Test(P_NORTH);
				}
			//	xSemaphoreGive(buttonMutexHandle);
				vTaskDelay( toggleFreq );
			}
			pedestrianReset_Test(P_WEST);
			pedestrianLight(HOLD, P_WEST);
			disableTraffic_Test(T_NORTHSOUTH, P_WEST);
		} else if (buttonWestFlag && !statusPedestrian_N && !statusPedestrian_W) {
			while(buttonWestFlag && (!statusPedestrian_N || !statusPedestrian_W)) {
				if (statusPedestrian_W) {
					break;
				}
				pedestrianPending_Test(P_WEST);
				vTaskDelay( toggleFreq );
			}
			pedestrianReset_Test(P_WEST);
			pedestrianLight(HOLD, P_WEST);
			disableTraffic_Test(T_NORTHSOUTH, P_WEST);
		}
		 */
#else
#endif
		osDelay(1);
	}
	/* USER CODE END StartPedestrianB */
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
			pedestrianReset_Test(P_WEST);
		} else if (buttonWestFlag) {
			pedestrianPending_Test(P_WEST);
			vTaskDelay( toggleFreq );
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
			pedestrianReset_Test(P_NORTH);
		} else if (buttonNorthFlag) {
			pedestrianPending_Test(P_NORTH);
			vTaskDelay( toggleFreq );
		}
		osDelay(1);
	}
	/* USER CODE END StartToggleN */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

