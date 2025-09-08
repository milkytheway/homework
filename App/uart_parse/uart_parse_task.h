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
#ifndef UART_PARSE_TASK_H
#define UART_PARSE_TASK_H

#include <stdint.h>

uint8_t buffer1[1] = {0};
uint8_t buffer2[1] = {0};

void uart_rec_A_func(void *argument);






#endif
