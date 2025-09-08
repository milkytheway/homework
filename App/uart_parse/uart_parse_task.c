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


void uart_rec_A_func(void *argument)
{
  /* USER CODE BEGIN uart_rec_A_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END uart_rec_A_func */
}


