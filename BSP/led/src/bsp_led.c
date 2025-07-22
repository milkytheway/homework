/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file bsp_led.c
 * 
 * @par dependencies 
 * - bsp_led.h
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

#include "bsp_led.h"



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
 * @return  LED_status_t.
 * 
 * */
 LED_status_t led_on_off(LED_operation_t led_operation)
 {
	 LED_status_t func_status = LED_ERROR;
	 
	 if(LED_ON == led_operation)
	 {
		 // turn on the LED
		 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		 // A check mechanism should be added here to determine GPIO operation
		 func_status = LED_OK;
	 }
	 
	 if(LED_OFF == led_operation)
	 {
		 // turn off the LED
		 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		 func_status = LED_OK;
	 }
	 
	 if(LED_TOGGLE == led_operation)
	 {
		 // toggle the LED
		 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		 func_status = LED_OK;
	 }
	 
	 return func_status;
 }

