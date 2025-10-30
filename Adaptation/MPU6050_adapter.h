/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file MPU6050_adapter.h
 * 
 * @brief Adapter layer for MPU6050 sensor and FreeRTOS integration
 * 
 * @par Dependencies:
 * - FreeRTOS.h
 * - task.h
 * - bsp_mpuxxxx_handler.h
 * - bsp_mpuxxxx_driver.h
 * 
 * @author Jack | R&D Dept. | EternalChip
 * 
 * @version V1.0 2025.8.20
 *
 * @note 1 tab == 4 spaces
 *****************************************************************************/

#ifndef MPU6050_ADAPTER_H
#define MPU6050_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************** Includes *********************************//
#include <stdint.h>
#include "bsp_mpuxxxx_handler.h"
#include "bsp_mpuxxxx_driver.h"
//******************************** Includes *********************************//

//***************************** Function Prototypes *************************//

/**
 * @brief Initialize MPU6050 handler system
 * 
 * @return MPU_HANDLER_status_t
 * @retval MPU_HANDLER_OK Success
 * @retval MPU_HANDLER_ERRORRESOURCE Resource allocation failed
 * @retval MPU_HANDLER_ERRORPARAMETER Parameter error
 * 
 * @details
 * This function performs complete initialization of the MPU6050 handler:
 * 1. Allocates handler and driver instances (static variables)
 * 2. Configures all OS interface implementations
 * 3. Links handler to driver instance
 * 4. Calls handler initialization which creates thread internally
 * 
 * @note Adapter layer responsibility:
 *       - Resource allocation (handler/driver instances)
 *       - OS interface implementations
 *       - Proper initialization sequence
 *       
 *       Handler layer responsibility:
 *       - Logic encapsulation
 *       - Thread creation through abstracted interface
 *       
 *       Architecture: Following LED handler pattern
 */
MPU_HANDLER_status_t MPU6050_adapter_init(void);

/**
 * @brief Get pointer to MPU6050 handler instance for ISR access
 * 
 * @return pointer to handler instance, or NULL if not initialized
 * 
 * @note This function provides the handler instance pointer for use
 *       in HAL_GPIO_EXTI_Callback interrupt handler.
 *       The handler instance is allocated in adapter layer as static variable.
 */
bsp_mpuxxxx_handler_t* MPU6050_adapter_get_handler_instance(void);

/**
 * @brief MPU6050 data ready callback function
 * 
 * @param p_data Pointer to MPU6050 sensor data
 * 
 * @details
 * This function is called by the handler when new sensor data is available.
 * Application can register this callback or provide its own.
 */
void MPU6050_data_callback(mpu6050_data_t *p_data);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_ADAPTER_H */

