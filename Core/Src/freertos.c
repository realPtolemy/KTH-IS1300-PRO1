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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// Required program delays, in ms.
const TickType_t toggleFreq = pdMS_TO_TICKS(1000); // ms to ticks
const TickType_t greenDelay = pdMS_TO_TICKS(8000);
const TickType_t orangeDelay = pdMS_TO_TICKS(5000);
const TickType_t redDelayMax = pdMS_TO_TICKS(100);
const TickType_t pedestrianDelay = pdMS_TO_TICKS(100);

extern uint8_t statusTraffic_NS;
extern uint8_t statusTraffic_EW;
extern uint8_t statusPedestrian_N;
extern uint8_t statusPedestrian_W;

/* USER CODE END Variables */
/* Definitions for idleTask */
osThreadId_t idleTaskHandle;
const osThreadAttr_t idleTask_attributes = {
  .name = "idleTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
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
  .priority = (osPriority_t) osPriorityAboveNormal,
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
  for(;;)
  {
	  #ifdef RUN_TEST_PROGRAM
	  if (statusTraffic_NS == 1) {
		  disableTraffic_NS_Test();
		  activateTraffic_EW_Test();
		  vTaskDelay( greenDelay );
	  } else {
		  disableTraffic_EW_Test();
		  activateTraffic_NS_Test();
		  vTaskDelay( greenDelay );
	  };
	  #else
	  // Do real stuff
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTraffic */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

