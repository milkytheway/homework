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

//****************Thread_Handler******************//
osThreadId_t key_TaskHandle;
const osThreadAttr_t key_Task_attributes = {
  .name = "key_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};


osThreadId_t led_TaskHandle;
const osThreadAttr_t led_Task_attributes = {
  .name = "led_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

//****************Thread_Handler******************//


//****************Queue_Handler******************//

QueueHandle_t Key_queue;

//****************Queue_Handler******************//

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
/**
 * @brief Key thread function.
 * 
 *  
 * @param[in] void *argument        : Pointer to the target of handler.
 * 
 * @return void.
 * 
 * */
void StartKeyTask(void *argument);

/**
 * @brief led thread function.
 * 
 *  
 * @param[in] void *argument        : Pointer to the target of handler.
 * 
 * @return void.
 * 
 * */
void StartLedTask(void *argument);
//****************Thread_Func******************//

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
	Key_queue = xQueueCreate( 10, sizeof( LED_operation_t ) );
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	//key_TaskHandle = osThreadNew(StartKeyTask, NULL, &key_Task_attributes);
	led_TaskHandle = osThreadNew(StartLedTask, NULL, &led_Task_attributes);
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
	KEY_status_t 						key_func_ret 			=				KEY_OK;
	KEY_PRESSE_STATUS_t			key_status   			=				KEY_RELEASED;
	LED_operation_t 				operation 				= 			LED_TOGGLE;
  for(;;)
  {
		printf("default task thread running\r\n");
		
		key_func_ret = key_scan_time(&key_status, 1000);
		
		if(KEY_OK == key_func_ret)
		{
			//判断短按
			if(key_status == KEY_SHORT_PRESSED)
			{
				printf("short pressed at [%d] tick \r\n", HAL_GetTick());
				operation = LED_TOGGLE;
				
				if(pdPASS == xQueueSendToFront(Key_queue, &operation, 0))
				{
					printf("send successfully\r\n");
				}
			}
			//判断长按
			
			if(KEY_LONG_PRESSED == key_status)
			{
				printf("key long pressed at [%d] tick \r\n", HAL_GetTick());
				operation = LED_BLINK;
				
				if(pdPASS == xQueueSendToFront(Key_queue, &operation, 0))
				{
					printf("send successfully\r\n");
				}
			}
		}
		
		osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief Detect key press event, send data to queue if key pressed
 * 
 * Steps:
 *  1, detect key pressed or not, within a shrot period
 *  2, if pressed, 
 * 
 * @param[in]  void *argument        : Pointer to the target of handler.
 * @param[out] void
 * 
 * @return 		 void.
 * 
 * */
void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	
	KEY_status_t key_ret 					 		= KEY_OK;
	KEY_PRESSE_STATUS_t key_status 		= KEY_RELEASED;
	LED_operation_t operation 				= LED_TOGGLE;
	
	if( Key_queue == 0 )
	{
		// Queue was not created and must not be used.
	}
	
  /* Infinite loop */
  for(;;)
  {
		
		key_ret = key_scan(&key_status);
		
		if(KEY_OK == key_ret)
		{
			if(KEY_PRESSED == key_status)
			{
				//check if queue send successful
				if(pdPASS == xQueueSendToFront(Key_queue, &operation, 0))
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

/**
 * @brief Implementation of led thread
 * 
 * Steps:
 *  1, detect key pressed or not, within a shrot period
 *  2, if pressed, 
 * 
 * @param[in] uint32_t *key_value : an adress to store the status of the key
 * @param[out] array_vaild_number : The member number of this array.
 * 
 * @return  LED_status_t.
 * 
 * */

void StartLedTask(void *argument)
{
	
	LED_status_t 		led_ret 				= LED_OK;
	LED_operation_t led_operation 	= LED_ON;
	
	for(;;)
	{
		printf("led_thread_running\r\n");
		if( Key_queue != 0 )
			{
				// Receive a message on the created queue.  Block the task if a
				// message is not immediately available.
				if( xQueueReceive( Key_queue, &( led_operation ), portMAX_DELAY ) )
				{
					printf("received\r\n");
					led_ret = led_on_off(led_operation);
					
					if(LED_OK == led_ret)
					{
						printf("led_operation_success\r\n");
					}
				}
			}
		osDelay(100);
		}
}
/* USER CODE END Application */

