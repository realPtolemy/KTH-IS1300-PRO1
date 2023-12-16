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
//#define RUN_TEST_PROGRAM
//#define RUN_TEST_IDLE
//#define RUN_TEST_PEDESTRIAN
//#define RUN_TEST_TRAFFIC
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
const TickType_t sysDelay = pdMS_TO_TICKS(500);			// used to avoid mutex hickup
const TickType_t toggleFreq = pdMS_TO_TICKS(250);		// arguments is ms, then coverted to ticks
const TickType_t pedestrianDelay = pdMS_TO_TICKS(100);	// Real life ?? ms
const TickType_t safetyDelay = pdMS_TO_TICKS(3000);		// Real life ~6000 ms
const TickType_t greenDelay = pdMS_TO_TICKS(8000); 		// Real life ~47000 ms
const TickType_t orangeDelay = pdMS_TO_TICKS(2500); 	// Real life ~5000 ms
const TickType_t redDelayMax = pdMS_TO_TICKS(500);		// Real life ?? ms
const TickType_t testingDelay = pdMS_TO_TICKS(500); // ms to ticks
const TickType_t testingDelay2 = pdMS_TO_TICKS(800); // ms to ticks

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

uint8_t testVar = 2;

#else
const TickType_t sysDelay = pdMS_TO_TICKS(500);			// used to avoid mutex hickup
const TickType_t toggleFreq = pdMS_TO_TICKS(250);		// arguments is ms, then coverted to ticks
const TickType_t pedestrianDelay = pdMS_TO_TICKS(100);	// Real life ?? ms
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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pedestrianTask */
osThreadId_t pedestrianTaskHandle;
const osThreadAttr_t pedestrianTask_attributes = {
  .name = "pedestrianTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for trafficTask */
osThreadId_t trafficTaskHandle;
const osThreadAttr_t trafficTask_attributes = {
  .name = "trafficTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for mutex */
osMutexId_t mutexHandle;
const osMutexAttr_t mutex_attributes = {
  .name = "mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartIdle(void *argument);
void StartPedestrian(void *argument);
void StartTraffic(void *argument);

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
		if( !pendingTraffic ) {
			if (statusTraffic_NS) {
				xSemaphoreTake(mutexHandle, 0);
				disableTraffic_Test(T_NORTHSOUTH, P_WEST);
				xSemaphoreGive(mutexHandle);
				vTaskDelay(testingDelay);
				xSemaphoreTake(mutexHandle, portMAX_DELAY);
				if (!statusTraffic_NS) {
					activateTraffic_Test(T_EASTWEST, P_NORTH);
				}
				xSemaphoreGive(mutexHandle);
			} else {
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
		if( !pendingTraffic ) {
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
		if ( testVar == 1 ) {
			xSemaphoreTake(pedestrianMutexHandle, 0);
			traffic_NS_Test(2);
			traffic_EW_Test(2);
			pedestrianPending_N_Test();
			pedestrianPending_W_Test();
			vTaskDelay( testingDelay );
			xSemaphoreGive( pedestrianMutexHandle );
		}
#else
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
				if (statusTraffic_NS) {
					staticTraffic_Test(); // R2.6 is considered within the staticTraffic_Test function
				} else {
					activateTraffic_Test(T_NORTHSOUTH, P_WEST);
					staticTraffic_Test();
				}
			} else if ( (!statusTraffic_NS && statusTraffic_EW) && (statusVehicle_E || statusVehicle_W ) && (!statusVehicle_N && !statusVehicle_S)) {
				if (statusTraffic_EW) {
					staticTraffic_Test(); // R2.6 is considered within the staticTraffic_Test function
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

