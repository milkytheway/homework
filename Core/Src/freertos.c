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
QueueHandle_t Key_queue_irq;

static uint32_t irq_type = FALING_TYPE;
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
	Key_queue_irq = xQueueCreate( 10, sizeof( KEY_PRESSE_EVENT_t *) );
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	key_TaskHandle = osThreadNew(StartKeyTask, NULL, &key_Task_attributes);
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

  for(;;)
  {
		//short press--toggle
		
		//long press--blink 3 times
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

	
	//KEY_status_t 				key_ret 						= KEY_OK;
	//KEY_PRESSE_STATUS_t key_status 					= KEY_RELEASED;
	LED_operation_t 		operation 					= LED_TOGGLE;
	KEY_PRESSE_EVENT_t	key_event = {key_event.edge_t = FALING, key_event.trigger_tick = 0};
  KEY_PRESSE_EVENT_t *p_key_event = NULL;
	uint32_t						event_index					= 0;
	uint32_t						first_trigger_tick	= 0;
	
	if(NULL ==  Key_queue || NULL == Key_queue_irq)
	{
		// Queue was not created and must not be used.
		return;
	}
	
  /* Infinite loop */
  for(;;)
  {

		//check key queue
		if(pdTRUE == xQueueReceive(Key_queue_irq, &(p_key_event), (TickType_t) 0))
		{
      key_event.trigger_tick = p_key_event->trigger_tick;
      key_event.edge_t       = p_key_event->edge_t;

			if(first_trigger_tick == key_event.trigger_tick)
			{
				printf("equal tick errror \r\n");
				continue;
			}
			//if new data --> uodate status machine
			if(RASING == key_event.edge_t && 0 == event_index)
			{}
				
			if(FALING == key_event.edge_t && 0 == event_index)
			{
				//index an event
				event_index += 1;
				
				//
				first_trigger_tick = key_event.trigger_tick;
			}
			
			if(RASING == key_event.edge_t && 1 == event_index)
			{
				//duration < 10ms --> press invalid\
				
				if(key_event.trigger_tick - first_trigger_tick < 10)
				{
					first_trigger_tick = key_event.trigger_tick;
					event_index = 0;
					continue;
				}
				//duration > 10ms --> valid
			
				if(key_event.trigger_tick - first_trigger_tick < SHORT_PRESS_THRESHOLD)
				{
				// < threshold --> short press
				//send short message
					operation = LED_TOGGLE;
					if(pdTRUE == xQueueSendToFront(Key_queue, &operation, 0))
					{
						printf("send short successfully\r\n");
						printf("key_event.trigger_tick - first_trigger_tick = [%d]\r\n", key_event.trigger_tick - first_trigger_tick);
						first_trigger_tick = key_event.trigger_tick;
						event_index = 0;
						
						irq_type = FALING_TYPE;
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = Key_Pin;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(Key_GPIO_Port, &GPIO_InitStruct);
					}
				}
				// > threshold --> long press
				//send long message
				
				if(key_event.trigger_tick - first_trigger_tick > SHORT_PRESS_THRESHOLD)
				{
				// < threshold --> short press
				//send short message
					operation = LED_BLINK_10_TIMES;
					if(pdTRUE == xQueueSendToFront(Key_queue, &operation, 0))
					{
						printf("send long press successfully\r\n");
						printf("key_event.trigger_tick - first_trigger_tick = [%d]\r\n", key_event.trigger_tick - first_trigger_tick);
						first_trigger_tick = key_event.trigger_tick;
            event_index = 0;

            irq_type = FALING_TYPE;
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = Key_Pin;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(Key_GPIO_Port, &GPIO_InitStruct);
					}
				}
			}
			
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
					led_ret = led_on_off_timer_irq(led_operation);
					
					if(LED_OK == led_ret)
					{
						printf("led_operation_success\r\n");
					}
				}
			}
		osDelay(100);
		}
}

/**
 * @brief key interuption callback.
 * 
 * Steps:
 *  1. 1st trigger with falling
 *  1.1 if the key is pressed, check if it is short pressed.
 *  1.2 if the key is long pressed
 *  
 * @param[in] key_value         : Pointer to the target of handler.
 * @param[in] threshold         : threshold to determine short or long, short < thresh
 * 
 * @return KEY_status_t 			  : Status of the function.
 * 
 * */
KEY_CALLBACK
{
   /*
    1.if trigger first time with falling type,\
      send the event to the inter_key_queue \
      changing the interruption type to Raising
    */
	
		//*********************************P1*********************************************//
    HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_RESET);
    //*******************************************************************************//
	
    //.data
    static KEY_PRESSE_EVENT_t key_press_event_1 =       
    {
        .edge_t    = FALING,
        .trigger_tick = 0
    };

    static KEY_PRESSE_EVENT_t key_press_event_2 = 
    {
        .edge_t    = RASING,
        .trigger_tick = 0
    };

    KEY_PRESSE_EVENT_t *p_key_press_event1 = &key_press_event_1;
    KEY_PRESSE_EVENT_t *p_key_press_event2 = &key_press_event_2;

    BaseType_t xHigherPrioritTaskWoken;
		
		//**********************************P2*********************************************//
		HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_RESET);
		//*******************************************************************************//
    
    if ( FALING_TYPE == irq_type )
    {
        if ( NULL == Key_queue_irq )
        {
            printf( "inter_key_queue not created"
                    " at [%d] tick \r\n", 
                                   HAL_GetTick());
        }        

        key_press_event_1.trigger_tick = HAL_GetTick();

        if ( pdTRUE == xQueueSendToFrontFromISR(            Key_queue_irq, 
                                                         &p_key_press_event1, 
                                                    &xHigherPrioritTaskWoken ))
        {
            printf( "send FALING_event successfully"
                    " at [%d] tick \r\n", 
                                   HAL_GetTick());
					
					//**********************************P3*********************************************//
						HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_RESET);
					//**********************************P3*********************************************//
        }
				
				
    /*
    1.1 changing the irq type
    */
        irq_type = RASING_TYPE;
    /*
    1.2 changing the GPIO irq trigger type
    */ 
        GPIO_InitTypeDef GPIO_InitStruct = {0};
            
        GPIO_InitStruct.Pin = Key_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(Key_GPIO_Port, &GPIO_InitStruct);
				
				//***********************************P4********************************************//
				HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_RESET);
				//*******************************************************************************//
				
    }
    else if ( RASING_TYPE == irq_type )
    {
            
    /*  
    2.if trigger first time with Raising type,\
      send the event to the inter_key_queue \
      changing the interruption type back to falling
    */

        if ( NULL == Key_queue_irq )
        {
            printf( "inter_key_queue not created"
                    " at [%d] tick \r\n", 
                                   HAL_GetTick());
        }

        key_press_event_2.trigger_tick = HAL_GetTick();

        if ( pdTRUE == xQueueSendToFrontFromISR(              Key_queue_irq, 
                                                           &p_key_press_event2, 
                                                     &xHigherPrioritTaskWoken ))
        {
            printf( "send RASING_event successfully"
                    " at [%d] tick \r\n", 
                                   HAL_GetTick());
					
					//***********************************P3********************************************//
						HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_RESET);
					//***********************************P3********************************************//
        }
        
				
    /*
    1.1 changing the irq type
    */
        irq_type = FALING_TYPE;
    /*
    1.2 changing the GPIO irq trigger type
    */ 
        GPIO_InitTypeDef GPIO_InitStruct = {0};
            
        GPIO_InitStruct.Pin = Key_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(Key_GPIO_Port, &GPIO_InitStruct);
    }
		
		//***********************************P5********************************************//
    HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IRQ_TRACE_GPIO_Port, IRQ_TRACE_Pin, GPIO_PIN_RESET);
		//***********************************P5********************************************//
}


/* USER CODE END Application */

