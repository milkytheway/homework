/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file bsp_key.h
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


#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__

//******************************** Includes *********************************//

#include <stdint.h>               
#include <stdio.h>

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"


//******************************** Includes *********************************//

//******************************** Defines **********************************//

typedef enum
{
  KEY_OK                = 0,         /* Operation completed successfully.  */
  KEY_ERROR             = 1,         /* Run-time error without case matched*/
  KEY_ERRORTIMEOUT      = 2,         /* Operation failed with timeout      */
  KEY_ERRORRESOURCE     = 3,         /* Resource not available.            */
  KEY_ERRORPARAMETER    = 4,         /* Parameter error.                   */
  KEY_ERRORNOMEMORY     = 5,         /* Out of memory.                     */
  KEY_ERRORISR          = 6,         /* Not allowed in ISR context         */
  KEY_RESERVED          = 0x7FFFFFFF /* Reserved                           */
} KEY_status_t;


typedef enum
{
	KEY_PRESSED						= 0,
	KEY_RELEASED					= 1
} KEY_PRESSE_STATUS_t;

//******************************** Defines **********************************//


//******************************** Declaring ********************************//
KEY_status_t key_scan(KEY_PRESSE_STATUS_t *key_value);

//******************************** Declaring ********************************//


#endif // End of __BSP_KEY_H__

