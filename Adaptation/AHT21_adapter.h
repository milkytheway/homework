/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file system_adaptation.h
 * 
 * @brief System adaptation layer for AHT21 sensor and FreeRTOS integration
 * 
 * @par Dependencies:
 * - FreeRTOS.h
 * - cmsis_os2.h
 * - ec_bsp_aht21_driver.h
 * - ec_bsp_temp_humi_handler.h
 * 
 * @author Jack | R&D Dept. | EternalChip
 * 
 * @version V1.0 2025.8.20
 *
 * @note 1 tab == 4 spaces
 *****************************************************************************/

#ifndef SYSTEM_ADAPTATION_H
#define SYSTEM_ADAPTATION_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************** Includes *********************************//
#include <stdint.h>
#include "ec_bsp_aht21_driver.h"
#include "ec_bsp_temp_humi_handler.h"
//******************************** Includes *********************************//

//***************************** Global Variables ****************************//


//***************************** Global Variables ****************************//

//***************************** Function Prototypes *************************//

/* DWT (Data Watchpoint and Trace) Delay Functions */

/**
 * @brief Initialize DWT for precise timing delays
 * 
 * @details
 * Enables the DWT module and cycle counter for microsecond-precision delays
 * using the ARM Cortex-M4's debug features.
 * 
 * @note Must be called before using delay_us() or delay_ms()
 */
// void dwt_delay_init(void);

/**
 * @brief Microsecond precision delay function
 * 
 * @param us Number of microseconds to delay
 * 
 * @details
 * Uses DWT cycle counter for precise timing. Based on SystemCoreClock frequency.
 * 
 * @warning Maximum delay is limited by 32-bit counter rollover
 */
// void delay_us(uint32_t us);

/**
 * @brief Millisecond precision delay function
 * 
 * @param ms Number of milliseconds to delay
 * 
 * @details
 * Implemented as a wrapper around delay_us() for millisecond delays.
 */
// void delay_ms(uint32_t ms);

/* I2C Driver Wrapper Functions for AHT21 Interface */

/**
 * @brief Initialize I2C bus for AHT21 communication
 * 
 * @param p_bus Pointer to I2C bus configuration structure
 * @return AHT21_OK on success, AHT21_ERROR on failure
 */
AHT21_status_t IICInit_wrapper(void *p_bus);

/**
 * @brief Generate I2C start condition
 * 
 * @param p_bus Pointer to I2C bus configuration structure
 * @return AHT21_OK on success, AHT21_ERROR on failure
 */
AHT21_status_t IICStart_wrapper(void *p_bus);

/**
 * @brief Generate I2C stop condition
 * 
 * @param p_bus Pointer to I2C bus configuration structure
 * @return AHT21_OK on success, AHT21_ERROR on failure
 */
AHT21_status_t IICStop_wrapper(void *p_bus);

/**
 * @brief Send single byte over I2C
 * 
 * @param p_bus Pointer to I2C bus configuration structure
 * @param data Byte to transmit
 * @return AHT21_OK on success, AHT21_ERROR on failure
 */
AHT21_status_t IICSendByte_wrapper(void *p_bus, uint8_t data);

/**
 * @brief Receive single byte over I2C
 * 
 * @param p_bus Pointer to I2C bus configuration structure
 * @param p_data Pointer to store received byte
 * @return AHT21_OK on success, AHT21_ERROR on failure
 */
AHT21_status_t IICReceiveByte_wrapper(void *p_bus, uint8_t *p_data);

/**
 * @brief Wait for I2C acknowledge from slave
 * 
 * @param p_bus Pointer to I2C bus configuration structure
 * @return AHT21_OK if ACK received, AHT21_ERROR if NACK received
 */
AHT21_status_t IICWaitAck_wrapper(void *p_bus);

/**
 * @brief Send I2C acknowledge to slave
 * 
 * @param p_bus Pointer to I2C bus configuration structure
 * @return AHT21_OK on success
 */
AHT21_status_t IICSendAck_wrapper(void *p_bus);

/**
 * @brief Send I2C not-acknowledge to slave
 * 
 * @param p_bus Pointer to I2C bus configuration structure
 * @return AHT21_OK on success
 */
AHT21_status_t IICSendNotAck_wrapper(void *p_bus);

/* FreeRTOS Queue Operations for Temperature/Humidity Handler */

/**
 * @brief Create a FreeRTOS message queue
 * 
 * @param item_num Maximum number of items in queue
 * @param item_size Size of each item in bytes
 * @param p_queue_handle Pointer to store created queue handle
 * @return HANDLER_OK on success, HANDLER_ERRORRESOURCE on failure
 */
HANDLER_status_t os_queue_create(uint32_t item_num, 
                                 uint32_t item_size, 
                                 void **p_queue_handle);

/**
 * @brief Put an item into FreeRTOS message queue
 * 
 * @param p_queue_handle Handle to the message queue
 * @param item Pointer to item to be queued
 * @param timeout Timeout in milliseconds (osWaitForever for infinite)
 * @return HANDLER_OK on success, HANDLER_ERROR on failure
 */
HANDLER_status_t os_queue_put(void *p_queue_handle, 
                              void *item, 
                              uint32_t timeout);

/**
 * @brief Get an item from FreeRTOS message queue
 * 
 * @param p_queue_handle Handle to the message queue
 * @param msg Pointer to store received message
 * @param timeout Timeout in milliseconds (osWaitForever for infinite)
 * @return HANDLER_OK on success, HANDLER_ERROR on failure
 */
HANDLER_status_t os_queue_get(void *p_queue_handle, 
                              void *msg, 
                              uint32_t timeout);

/* Temperature and Humidity Callback Function */

/**
 * @brief Callback function for processed temperature and humidity data
 * 
 * @param temperature Pointer to temperature value in Celsius
 * @param humidity Pointer to humidity value in percentage
 * 
 * @details
 * This function is called by the temperature/humidity handler when new
 * sensor data is available. It applies a 1.5x calibration factor to the
 * temperature reading and logs the values.
 * 
 * @note The calibrated temperature is stored in g_temperature global variable
 */
void temp_humi_callback(float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_ADAPTATION_H */
