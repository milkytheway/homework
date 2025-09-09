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

/* Includes ------------------------------------------------------------------*/
#include "uart_parse_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "queue.h"
#include "cmsis_os.h"
#include "elog.h"
/* Includes ------------------------------------------------------------------*/

#define TAG "uart_parse_task"

QueueHandle_t queue_irq_rec_A;
uint8_t receivedata = 0;

void uart_rec_A_func(void *argument)
{
  /* USER CODE BEGIN uart_rec_A_func */
  elog_info(TAG, "uart_rec_A_func is running...");
  queue_irq_rec_A = NULL;
  queue_irq_rec_A = xQueueCreate(1, 4);
  if (NULL == queue_irq_rec_A)
  {
    elog_error(TAG, "queue_irq_rec_A creation failed");
    return;
  }
  else
  {
    elog_info(TAG, "queue_irq_rec_A creation success");
  }
  /* Infinite loop */
  for(;;)
  {
    if (pdTRUE == xQueueReceive(queue_irq_rec_A, &receivedata, portMAX_DELAY))
    {
      elog_info(TAG, "Data received from queue: %d", receivedata);
    }
    elog_info(TAG, "uart_rec_A_func is running...");
    osDelay(1000);
  }
  /* USER CODE END uart_rec_A_func */
}


