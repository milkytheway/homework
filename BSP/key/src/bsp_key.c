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

#include "bsp_key.h"



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
 
KEY_status_t key_scan(KEY_PRESSE_STATUS_t *key_value){
	
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

