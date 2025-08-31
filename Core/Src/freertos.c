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
#include <stdlib.h>
#include <string.h>
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BUFFER_SIZE 1
#define DMA_ADC_COMPLETE_IT 0xff
#define BUFFER1_READY 0x01
#define BUFFER2_READY 0x02
#define TAG "FreeRTOS"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t *buffer1 = NULL;
uint32_t *buffer2 = NULL;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

QueueHandle_t queue;
QueueHandle_t adc_queue;
SemaphoreHandle_t xMutex;

uint32_t buffer_idx = 1;


osThreadId_t adc_output_task_handle;
const osThreadAttr_t adc_output_task_attributes = {
  .name = "adc_output_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
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
void StartAdcOutputTask(void *argument);
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
  xMutex = xSemaphoreCreateMutex();
  if(NULL == xMutex) {
    elog_error(TAG, "Mutex creation failed");
    return;
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  queue = xQueueCreate(1, sizeof(uint32_t));

  if(NULL == queue) {
    printf("Queue creation failed \r\n");
    return;
  }

  adc_queue = xQueueCreate(1, sizeof(uint32_t));

  if(NULL == adc_queue) {
    printf("ADC Queue creation failed \r\n");
    return;
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  adc_output_task_handle = osThreadNew(StartAdcOutputTask, NULL, &adc_output_task_attributes);
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
  buffer1 = (uint32_t *)malloc(sizeof(uint32_t) * BUFFER_SIZE);
  buffer2 = (uint32_t *)malloc(sizeof(uint32_t) * BUFFER_SIZE);

  if(NULL == buffer1 || NULL == buffer2) 
  {
    elog_error(TAG, "Memory allocation failed");
    osDelay(1000);
    return;
  }

  elog_info(TAG, "Memory allocation successful");

  memset(buffer1, 0xff, sizeof(uint32_t) * BUFFER_SIZE);
  memset(buffer2, 0xff, sizeof(uint32_t) * BUFFER_SIZE);


  HAL_StatusTypeDef ret = HAL_OK;
  ret = HAL_ADC_Start_DMA(&hadc1, buffer1, BUFFER_SIZE);

  if(HAL_OK != ret)
  {
    elog_error(TAG, "Failed to start ADC DMA");
    osDelay(1000);
    return;
  }

  uint32_t queue_receive_value = 0xff;
  uint32_t buffer_ready_signal = 0xff;
  /* Infinite loop */
  for(;;)
  {
    elog_info(TAG, "default task running");
    if(pdPASS == xQueueReceive(queue, &queue_receive_value, portMAX_DELAY))
    {
      if(uxQueueMessagesWaiting(adc_queue) == 0)
      {
        if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        //buffer 1 gets new data
        if(1 == buffer_idx)
        {
          elog_info(TAG, "Buffer1 data aaa = [%d]", buffer1[0]);

          // Start ADC DMA for Buffer2
          ret = HAL_ADC_Start_DMA(&hadc1, buffer2, BUFFER_SIZE);

          if(HAL_OK != ret)
          {
            elog_error(TAG, "Failed to start ADC DMA");
            osDelay(1000);
            return;
          }

          buffer_idx = 2;

          // Send buffer1 ready signal
          buffer_ready_signal = BUFFER1_READY;
          if(pdPASS == xQueueSend(adc_queue, &buffer_ready_signal, 0))
          {
            elog_info(TAG, "adc_queue send successful");
          }
        }
        //buffer 2 gets new data
        else if(2 == buffer_idx)
        {
          elog_info(TAG, "Buffer2 data bbb = [%d]", buffer2[0]);

          // Start ADC DMA for Buffer1
          ret = HAL_ADC_Start_DMA(&hadc1, buffer1, BUFFER_SIZE);

          if(HAL_OK != ret)
          {
            elog_error(TAG, "Failed to start ADC DMA");
            osDelay(1000);
            return;
          }

          buffer_idx = 1;

          // Send buffer2 ready signal
          buffer_ready_signal = BUFFER2_READY;
          if(pdPASS == xQueueSend(adc_queue, &buffer_ready_signal, 0))
          {
            elog_info(TAG, "adc_queue send successful");
          }
        }
        xSemaphoreGive(xMutex);
      }
      }
      else
      {
        elog_info(TAG, "No buffer ready signal in adc_queue");
      }
    }

    //osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartAdcOutputTask(void *argument)
{
  uint32_t received_buffer_ready_signal = 0xff;
  /* Infinite loop */
  for(;;)
  {
    elog_info(TAG, "adc_output_task running");
    if(pdPASS == xQueueReceive(adc_queue, &received_buffer_ready_signal, portMAX_DELAY))
    {
      if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        if(BUFFER1_READY == received_buffer_ready_signal)
        {
          elog_info(TAG, "Buffer1 is ready");
        }
        else if(BUFFER2_READY == received_buffer_ready_signal)
        {
          elog_info(TAG, "Buffer2 is ready");
        }
        xSemaphoreGive(xMutex);
      }
    }
  }
}

/**
 * @brief  ADC conversion complete callback (DMA mode).
 *         This function is automatically called by the HAL library when
 *         the ADC DMA transfer is finished (i.e., when the requested number
 *         of ADC samples have been transferred to memory through DMA).
 *
 * @param  hadc: Pointer to the ADC handle (not used here).
 * 
 * @note   This callback runs in interrupt context. Keep the code short and non-blocking.
 *         Here, it notifies a FreeRTOS task by sending a signal to a queue.
 *         The receiving task can then process the new ADC data.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  UNUSED(hadc);

  BaseType_t xHighPriorityTaskWoken = pdFALSE;
  uint32_t adc_dma_complete_signal = DMA_ADC_COMPLETE_IT;

	if(pdPASS == xQueueSendFromISR(queue, &adc_dma_complete_signal, &xHighPriorityTaskWoken))
  {
    elog_info(TAG, "Queue send successful");
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  UNUSED(hadc);
  elog_error(TAG, "ADC error occurred");
}

/* USER CODE END Application */

