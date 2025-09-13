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
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "queue.h"
#include "cmsis_os.h"
#include "elog.h"
#include "circular_buffer.h"
#include "bsp_uart_driver.h"
/* Includes ------------------------------------------------------------------*/

#define TAG "uart_parse_task"

QueueHandle_t queue_data_proc = NULL;
uint8_t receivedata = 0;
static CircularBuffer_t *g_pbuf_thread = NULL;

void uart_rec_A_func(void *argument)
{
  /* USER CODE BEGIN uart_rec_A_func */
  elog_info(TAG, "uart_rec_A_func is running...");
  queue_data_proc = NULL;
  queue_data_proc = xQueueCreate(1, 4);
  if (NULL == queue_data_proc)
  {
    elog_error(TAG, "queue_data_proc creation failed");
    return;
  }
  else
  {
    elog_info(TAG, "queue_data_proc creation success");
  }

  uint8_t status = get_circular_buffer_handle((void **)&g_pbuf_thread);
  if(0 == status)
  {
      elog_error(TAG, "Get circular buffer handle failed");
      return;
  }
  else
  {
      elog_info(TAG, "Get circular buffer handle success");
  }
  /* Infinite loop */
  for(;;)
  {
    if (pdTRUE == xQueueReceive(queue_data_proc, &receivedata, portMAX_DELAY))
    {
      elog_info(TAG, "Data received from queue: %x", receivedata);
      if(NULL == g_pbuf_thread)
      {
          elog_error(TAG, "Global circular buffer pointer is NULL");
          return;
      }

      while(!is_buffer_empty(g_pbuf_thread))
      {
          data_type_t data_from_cbuf;
          if(0x01 == read_data(g_pbuf_thread, &data_from_cbuf))
          {
              //elog_debug(TAG, "Read data: %x from circular buffer", data_from_cbuf);
              osDelay(5); // Simulate data processing delay
          }

          static uint8_t buffer_frame_check[20] = {0};
          static uint8_t data_counter = 0;

          static uint8_t frame_status = FRAME_NOT_DETECTED;
          switch(frame_status)
          {
              case FRAME_NOT_DETECTED:
                  if(FRAME_HEAD_FLAG == data_from_cbuf)
                  {
                      frame_status = FRAME_HEAD;
                      elog_info(TAG, "Frame head detected");
                  }
                  break;
              case FRAME_HEAD:
                  if(FRAME_TAIL_FLAG == data_from_cbuf)
                  {
                      frame_status = FRAME_TAIL;
                      elog_info(TAG, "Frame tail detected");
                      //complete frame received, process it
                      uint32_t data_sum_frame = (uint32_t)buffer_frame_check[data_counter-1];
                      elog_info(TAG,"data_sum_frame: %d",data_sum_frame);
                      uint32_t data_sum = 0;
                      for(uint8_t i = 0; i < data_counter-1; i++)
                      {
                        data_sum += (uint32_t)buffer_frame_check[i];
                        elog_info(TAG, "Frame data[%d]: %x", i, buffer_frame_check[i]);
                      }
                      elog_info(TAG,"data_sum: %d",data_sum);
                      if(data_sum_frame == data_sum)
                      {
                          elog_info(TAG, "Frame data checksum valid");
                      }
                      else
                      {
                          elog_error(TAG, "Frame data checksum invalid");
                      }

                      data_counter = 0;
                      memset(buffer_frame_check, 0, sizeof(buffer_frame_check));
                  }
                  else if(FRAME_HEAD_FLAG == data_from_cbuf)
                  {
                      // Stay in FRAME_HEAD state
                      elog_info(TAG, "Another frame head detected, staying in FRAME_HEAD state");
                  }
                  else
                  {
                      // Process frame data
                      buffer_frame_check[data_counter] = data_from_cbuf;
                      data_counter++;
                      elog_info(TAG, "data_counter: %d", data_counter);
                      if(data_counter >= sizeof(buffer_frame_check))
                      {
                          data_counter = sizeof(buffer_frame_check) - 1; // Prevent overflow
                      }
                      elog_info(TAG, "Processing frame data: %x", data_from_cbuf);
                  }
                  break;
              case FRAME_TAIL:
                  // Frame complete, reset status
                  frame_status = FRAME_NOT_DETECTED;
                  elog_info(TAG, "Frame complete");
                  break;
              default:
                  frame_status = FRAME_NOT_DETECTED;
                  break;
          }
      }
    } else
    {
      elog_error(TAG, "Failed to receive data from queue");
    }
    elog_info(TAG, "uart_rec_A_func is running...");
    //osDelay(1000);
  }
  /* USER CODE END uart_rec_A_func */
}


