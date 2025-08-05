/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file key.c
 * 
 * @par dependencies 
 * - bsp_key.h
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
#include "FreeRTOS.h"
#include "bsp_key_irq.h"



/**
 * @brief Detect key press event during the life time of key_scan
 * 
 * Steps:
 *  1, detect key pressed or not, within a shrot period
 *  2, if pressed, 
 * 
 * @param[in] uint32_t *key_value : an adress to store the status of the key
 * @param[out] array_vaild_number : The member number of this array.
 * 
 * @return  KEY_status_t.
 * 
 * */
 
KEY_status_t key_scan(KEY_PRESSE_STATUS_t *key_value)
{
	
	uint32_t count = 0;
	KEY_PRESSE_STATUS_t key_status_value = KEY_RELEASED;
	
	while(count<1000)
	{
		// key pressed
		if(HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin) == GPIO_PIN_RESET)
		{
			key_status_value = KEY_PRESSED;
			*key_value = key_status_value;
			return KEY_OK;
		}
		count ++;
	}
	
	*key_value = key_status_value;
	return KEY_ERRORTIMEOUT;
}



/**
 * @brief Instantiates the bsp_key_handler_t target.
 * 
 * Steps:
 *  1.  check if the key is pressed
 *  1.1 if the key is pressed, check if it is short pressed.
 *  1.2 if the key is long pressed
 *  
 * @param[in] key_value         : Pointer to the target of handler.
 * @param[in] threshold         : threshold to determine short or long, short < thresh
 * 
 * @return KEY_status_t 			  : Status of the function.
 * 
 * */


KEY_status_t key_scan_time(KEY_PRESSE_STATUS_t *key_value, 
													 uint32_t 					  threshold)
{
	KEY_status_t 				key_func_ret 				= 	KEY_OK;
	KEY_PRESSE_STATUS_t key_value_temp 			= 	KEY_RELEASED;
	uint32_t 		 				counter_tick 				= 	0;
	
	key_func_ret = key_scan(&key_value_temp);
	
	//task enter critical
	if(KEY_OK == key_func_ret)
	{
		if(KEY_PRESSED == key_value_temp)
		{
			counter_tick = HAL_GetTick();
			
			while(HAL_GetTick() < counter_tick + threshold)
			;
			
			//get key status agian to know whether the key is still pressed
			key_func_ret = key_scan(&key_value_temp);
			
			if(KEY_RELEASED == key_value_temp)
			{
				//this is short press
				*key_value = KEY_SHORT_PRESSED;
				return KEY_OK;
			}
			else
			{
				*key_value = KEY_LONG_PRESSED;
				
				//wait until key released
				KEY_status_t ret;
				do {
						ret = key_scan(&key_value_temp);
						if (ret != KEY_OK) break;
				} while (key_value_temp == KEY_PRESSED);
				
				return KEY_OK;
			}
		}
	}
	return key_func_ret;
}

