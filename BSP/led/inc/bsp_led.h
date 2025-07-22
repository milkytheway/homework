/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file bsp_led.h
 * 
 * @par dependencies 
 * - stdio.h
 * - stdint.h
 * 
 * @author ll | XXXXX Dept. | EternalChip XXXXX
 * 
 * @brief Provide the HAL APIs of AHT21 and corresponding opetions.
 * 
 * Processing flow:
 * 
 * call directly.
 * 
 * @version V1.0 2025-7-19
 *
 * @note 1 tab == 4 spaces!
 * 
 *****************************************************************************/


#ifndef __BSP_LED_H__
#define __BSP_LED_H__

//******************************** Includes *********************************//

#include <stdint.h>               
#include <stdio.h>

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"


//******************************** Includes *********************************//

//******************************** Defines **********************************//
/* function return status                  */
typedef enum
{
  LED_OK                = 0,         /* Operation completed successfully.  */
  LED_ERROR             = 1,         /* Run-time error without case matched*/
  LED_ERRORTIMEOUT      = 2,         /* Operation failed with timeout      */
  LED_ERRORRESOURCE     = 3,         /* Resource not available.            */
  LED_ERRORPARAMETER    = 4,         /* Parameter error.                   */
  LED_ERRORNOMEMORY     = 5,         /* Out of memory.                     */
  LED_ERRORISR          = 6,         /* Not allowed in ISR context         */
  LED_RESERVED          = 0x7FFFFFFF /* Reserved                           */
} LED_status_t;

typedef enum
{
  LED_ON                = 0,         /*      				LED on 							  */
  LED_OFF               = 1,         /*      				LED off							  */
	LED_TOGGLE						= 2          /*      				LED toggle		  		  */
} LED_operation_t;

//******************************** Defines **********************************//


//******************************** Declaring ********************************//

/**
 * @brief Instantiates the bsp_led_handler_t target.
 * 
 * Steps:
 *  1. Adds Core interfaces into bsp_led_driver instance target.
 *  2. Adds OS interfaces into bsp_led_driver instance target.
 *  3. Adds timebase interfaces into bsp_led_driver instance target.
 *  
 * @param[in] self        : Pointer to the target of handler.
 * @param[in] os_delay    : Pointer to the os_delay_interface.
 * @param[in] os_queue    : Pointer to the os_queue_interface.
 * @param[in] os_critical : Pointer to the os_critical_interface.
 * @param[in] os_thread   : Pointer to the os_thread_interface.
 * @param[in] time_base   : Pointer to the time_base_interface.
 * 
 * @return led_handler_status_t : Status of the function.
 * 
 * */
LED_status_t led_on_off(LED_operation_t led_operation);

//******************************** Declaring ********************************//


#endif // End of __BSP_LED_H__

