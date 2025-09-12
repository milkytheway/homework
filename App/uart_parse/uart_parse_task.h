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
#ifndef UART_PARSE_TASK_H__
#define UART_PARSE_TASK_H__

#include <stdint.h>

#define FRAME_NOT_DETECTED 0x01
#define FRAME_HEAD 0x02
#define FRAME_TAIL 0x03

#define FRAME_HEAD_FLAG 0xB1
#define FRAME_TAIL_FLAG 0xB2

// uint8_t buffer1[1] = {0};
// uint8_t buffer2[1] = {0};

void uart_rec_A_func(void *argument);






#endif
