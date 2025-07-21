/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

//****************Thread_Func******************//
osThreadId_t Key_TaskHandle;
const osThreadAttr_t Key_Task_attributes = {
  .name = "Key_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/**
 * @brief Key thread function.
 * 
 * Steps:
 *  1. 
 *  
 * @param[in] void *argument        : Pointer to the target of handler.
 * 
 * @return void.
 * 
 * */
void StartKeyTask(void *argument);

//****************Thread_Func******************//

//****************Queue_Handler******************//

QueueHandle_t Key_queue;
//****************Queue_Handler******************//

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
	// Create a queue capable of containing 10 uint32_t values.
	Key_queue = xQueueCreate( 10, sizeof( uint32_t ) );
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	Key_TaskHandle = osThreadNew(StartKeyTask, NULL, &Key_Task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		printf("Default_task_running\r\n");
		uint32_t received_value;
		if( Key_queue != 0 )
			{
				// Receive a message on the created queue.  Block the task if a
				// message is not immediately available.
				if( xQueueReceive( Key_queue, &( received_value ), portMAX_DELAY ) )
				{
					printf("received value: [%d]\n",received_value);
				}
			}
		osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	
	KEY_status_t key_ret 					 = KEY_OK;
	KEY_PRESSE_STATUS_t key_status = KEY_RELEASED;
	uint32_t counter_tick = 0;
	
	if( Key_queue == 0 )
	{
		// Queue was not created and must not be used.
	}
	
  /* Infinite loop */
  for(;;)
  {
		uint32_t tick = osKernelGetTickCount();
		counter_tick ++;
		
		key_ret = key_scan(&key_status);
		
		if(KEY_OK == key_ret)
		{
			if(KEY_PRESSED == key_status)
			{
				printf("[%d]Key pressed\n", tick);
				
				//check if queue send successful
				if(pdPASS == xQueueSendToFront(Key_queue, &counter_tick, 0))
				{
					printf("send successfully\r\n");
				}
			}
		}
		else if(KEY_OK != key_ret )
		{
			printf("key not pressed\r\n");
		}
		osDelay(100);
  }
}
/* USER CODE END Application */

