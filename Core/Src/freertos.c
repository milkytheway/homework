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
#include "AHT21_adapter.h"
#include "MPU6050_adapter.h"
#include "ec_bsp_temp_humi_handler.h"
#include "elog.h"
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
extern temp_humi_handler_all_input_arg_t input_args;
/* USER CODE END Variables */
/* Definitions for HandlerTask */
osThreadId_t HandlerTaskHandle;
const osThreadAttr_t HandlerTask_attributes = {
  .name = "HandlerTask",
  .stack_size = 128 * 6,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for userTask */
osThreadId_t userTaskHandle;
const osThreadAttr_t userTask_attributes = {
  .name = "userTask",
  .stack_size = 128 * 6,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void temp_humi_handler_thread(void *argument);
void userTaskFunction(void *argument);

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
  /* creation of HandlerTask */
  HandlerTaskHandle = osThreadNew(temp_humi_handler_thread, (void*)&input_args, &HandlerTask_attributes);

  /* creation of userTask */
  userTaskHandle = osThreadNew(userTaskFunction, NULL, &userTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_temp_humi_handler_thread */
/**
  * @brief  Function implementing the HandlerTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_temp_humi_handler_thread */
__weak void temp_humi_handler_thread(void *argument)
{
  /* USER CODE BEGIN temp_humi_handler_thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END temp_humi_handler_thread */
}

/* USER CODE BEGIN Header_userTaskFunction */
/**
* @brief Function implementing the userTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_userTaskFunction */
void userTaskFunction(void *argument)
{
  /* USER CODE BEGIN userTaskFunction */
	log_d("userTaskFunction start");
  temp_humi_event_t user_event = 
  {
    .humidity = 0,
    .temperature = 0,
    .type = TEMP_HUMI_EVENT_TEMP_HUMI,
    .pf_callback = temp_humi_callback,
    .timestamp = 0,
    .lifetime = 100
  };
  /* Infinite loop */
  for(;;)
  {
    bsp_temp_humi_xxx_read(&user_event);
    osDelay(500);
  }
  /* USER CODE END userTaskFunction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

