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

//******************************** global variables **************************//

static uint32_t g_blink_times = 0;
static uint32_t g_blink_order = 0;

//******************************** global variables **************************//


//******************************** Functions *********************************//
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
	 
	 if(LED_BLINK == led_operation)
	 {
		 //blink LED 3 times
		 for(int i=0; i<6; i++)
		 {
			 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			 
			 HAL_Delay(500);
		 }
	 }
	 
	 return func_status;
 }

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
 LED_status_t led_on_off_timer_irq(LED_operation_t led_operation)
{
	if(LED_TOGGLE == led_operation)
	{
		g_blink_times = 1;
		g_blink_order = 0;
	}

	if(LED_BLINK_10_TIMES == led_operation)
	{
		g_blink_times = 10;
		g_blink_order = 0;
	}
	return LED_OK;
}

/**
 * @brief callback function for TIM2 interrupt to handle LED operations.
 * 
 * Steps:
 *  1. Instantiates the bsp_led_handler_t target.
 *  2. Adds Core interfaces into bsp_led_driver instance target.
 *  3. Adds OS interfaces into bsp_led_driver instance target.
 *  4. Adds timebase interfaces into bsp_led_driver instance target.
 *  
 * @param[in] void : The operation to be performed on the LED.
 * @param[out] void
 * @return led_handler_status_t : Status of the function.
 * 
 * */
LED_status_t led_callback_for_TIM2(void)
{
	//1.if the g_blink_times is not zero, then start blinking
	if(g_blink_times > 0)
	{
		if(g_blink_order % 2 == 0)
		{
			led_on_off(LED_ON);
		}
		else
		{
			led_on_off(LED_OFF);
			g_blink_times--;
		}
		g_blink_order++;
	}
	else
	{
		g_blink_order = 0;
	}

	return LED_OK;
}
