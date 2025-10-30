/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file bsp_mpuxxxx_handler.h
 * 
 * @brief APIs for MPU6050 sensor handler layer.
 * 
 * @par Dependencies:
 * - bsp_mpuxxxx_driver.h
 * - stdbool.h
 * - stdio.h
 * - stdint.h
 * 
 * @author Jack | R&D Dept. | EternalChip
 * 
 * @version V1.0 2025.8.20
 *
 * @note 1 tab == 4 spaces
 *****************************************************************************/

#ifndef __BSP_MPUXXXX_HANDLER_H__
#define __BSP_MPUXXXX_HANDLER_H__

//******************************** Includes *********************************//
#include "bsp_mpuxxxx_driver.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
//******************************** Includes *********************************//

//********************** private macro definitions **************************//
#define HANDLER_DEBUG
#define OS_SUPPORTING
//********************** private macro definitions **************************//

//******************************** variables ********************************//
typedef enum
{
  MPU_HANDLER_OK                = 0,         /* Operation completed successfully.  */
  MPU_HANDLER_ERROR             = 1,         /* Run-time error without case matched*/
  MPU_HANDLER_ERRORTIMEOUT      = 2,         /* Operation failed with timeout      */
  MPU_HANDLER_ERRORRESOURCE     = 3,         /* Resource not available.            */
  MPU_HANDLER_ERRORPARAMETER    = 4,         /* Parameter error.                   */
  MPU_HANDLER_ERRORNOMEMORY     = 5,         /* Out of memory.                     */
  MPU_HANDLER_ERRORISR          = 6,         /* Not allowed in ISR context         */
  MPU_HANDLER_ERRORNOTRESP      = 7,         /* No response from device            */
  MPU_HANDLER_RESERVED          = 0x7FFFFFFF /* Reserved                           */
} MPU_HANDLER_status_t;

//Callback function type for MPU6050 data ready notification
typedef void (*mpu_data_callback_t)(mpu6050_data_t *p_data);

//******************************** variables ********************************//

//************************** Interface structures ***************************//
//Interface from OS layer
typedef struct
{
    void *task_handle;
    void (*os_delay_ms)(uint32_t const ms);

    MPU_HANDLER_status_t (*os_queue_create)(
                            uint32_t const item_num,
                            uint32_t const item_size,
                            void **  p_queue_handle);

    MPU_HANDLER_status_t (*os_queue_put)(
                            void * const p_queue_handle,
                            void * const item,
                            uint32_t timeout);

    MPU_HANDLER_status_t (*os_queue_get)(
                            void * const p_queue_handle,
                            void * const msg,
                            uint32_t timeout);

    MPU_HANDLER_status_t (*os_TaskNotifyGiveFromISR)(void *task_handle, void *higher_priority_task_woken);
    MPU_HANDLER_status_t (*os_TaskNotifyTake)(void *task_handle, uint32_t timeout, uint32_t *p_notify_value);
    
    /* Thread creation interface - handler creates its own thread during init
     * Adapter layer provides OS-specific implementation
     */
    MPU_HANDLER_status_t (*os_thread_create)(
                            void * const task_code,
                            const char * const task_name,
                            const uint32_t stack_depth,
                            void * const parameters,
                            uint32_t priority,
                            void ** const task_handler);
} mpu_handler_os_interface_t;


typedef struct
{
    iic_driver_interface_t *iic_driver_interface;
    timebase_interface_t  *timebase_interface;
    mpu_handler_os_interface_t *os_interface;
    void *bus_instance;
    interuption_interface_t *interruption_interface;
} mpu_handler_all_input_arg_t;
//************************** Interface structures ***************************//

//***************************** class definition ****************************//
/**
 * @brief Private data for handler instance (internal state)
 * @note This structure is embedded in each handler instance to ensure
 *       multi-instance support without shared state conflicts.
 */
typedef struct mpu_handler_private_data
{
    uint8_t inited;        /* Initialization state flag */
    uint32_t instance_id;  /* Instance identifier for debugging/logging */
} mpu_handler_private_data_t;

/**
 * @brief MPU6050 Handler instance structure
 * @note Multi-instance safe design:
 *       - Each instance has independent private_data (embedded, not pointer)
 *       - Each instance has independent data_buffer
 *       - Each instance binds to its own driver, bus, and OS resources
 *       - Multiple instances can coexist without conflicts
 *       
 *       Typical multi-device usage:
 *       @code
 *       // Device 1
 *       bsp_mpuxxxx_handler_t mpu_handler_1;
 *       bsp_mpuxxxx_handler_inst(&mpu_handler_1, &args_1);
 *       
 *       // Device 2
 *       bsp_mpuxxxx_handler_t mpu_handler_2;
 *       bsp_mpuxxxx_handler_inst(&mpu_handler_2, &args_2);
 *       @endcode
 */
typedef struct bsp_mpuxxxx_handler
{
    /* Interface passed to driver layer */
    iic_driver_interface_t          *iic_driver_interface;
    timebase_interface_t            *timebase_interface;
    void                            *bus_instance;
    interuption_interface_t         *interruption_interface;
    
    /* Interface from OS layer */
    mpu_handler_os_interface_t      *os_interface;

    /* Driver instance */
    bsp_mpuxxxx_driver              *p_mpu_driver_instance;

    /* Private data - embedded for multi-instance support */
    mpu_handler_private_data_t      private_data;

    /* User callback for data ready notification */
    mpu_data_callback_t             pf_data_callback;

    /* Data buffer to store sensor readings */
    mpu6050_data_t                  data_buffer;
} bsp_mpuxxxx_handler_t;
//***************************** class definition ****************************//

//********************************** APIs ***********************************//
/**
 * @brief MPU6050 handler thread function - internal use only
 * @param argument[in] pointer to handler instance (bsp_mpuxxxx_handler_t*)
 * @return void
 * @note This function is called internally by handler during initialization.
 *       External code should NOT call this function directly.
 *       
 *       Architecture (following LED handler pattern):
 *       - Handler instance is allocated in adapter layer
 *       - Handler init (bsp_mpuxxxx_handler_inst) creates thread internally
 *       - Thread receives pre-initialized handler instance pointer
 *       - TaskHandle is stored BEFORE thread starts (timing fix)
 */
void bsp_mpuxxxx_handler_thread(void *argument);

/**
 * @brief Construct and initialize the MPU6050 handler instance
 * @param p_handler[in] pointer to the handler instance (allocated by adapter layer)
 * @param p_input_args[in] pointer to all input arguments including interfaces
 * @return MPU_HANDLER_status_t
 * @note This function (following LED handler pattern):
 *       1. Initializes handler structure with provided interfaces
 *       2. Creates driver instance and initializes hardware
 *       3. Creates handler thread using os_interface->os_thread_create
 *       4. Stores task_handle in os_interface->task_handle BEFORE thread starts
 *       5. Exports handler instance pointer to adapter layer for ISR access
 *       
 *       Architecture responsibilities:
 *       - Handler instance allocation: Adapter layer (static variable)
 *       - Thread creation abstraction: Handler layer (calls os_thread_create)
 *       - OS-specific implementation: Adapter layer (provides os_thread_create)
 *       
 *       Key timing fix:
 *       TaskHandle is available immediately after this function returns.
 *       No race condition between thread creation and ISR callback registration.
 */
MPU_HANDLER_status_t bsp_mpuxxxx_handler_inst(
        bsp_mpuxxxx_handler_t * const p_handler,
        mpu_handler_all_input_arg_t * const p_input_args
                                            );

/**
 * @brief Interrupt callback function for adapter layer to call
 * @param p_handler[in] pointer to the handler instance
 * @param higher_priority_task_woken[out] pointer to yield flag
 * @return MPU_HANDLER_status_t
 * @note This function should be called from HAL_GPIO_EXTI_Callback in adapter layer.
 *       It encapsulates the interrupt notification logic (TaskNotifyFromISR).
 *       
 *       Design principle:
 *       - Handler layer abstracts the interrupt notification logic
 *       - Adapter layer provides OS resources (task_handle) and implementation
 *       - Driver layer does NOT participate in interrupt handling
 *       
 *       Example usage in adapter layer:
 *       @code
 *       void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 *       {
 *           if (GPIO_Pin == MPU_INT_Pin) {
 *               BaseType_t yield = pdFALSE;
 *               mpuxxxx_int_callback(&mpu_handler, &yield);
 *               portYIELD_FROM_ISR(yield);
 *           }
 *       }
 *       @endcode
 */
MPU_HANDLER_status_t mpuxxxx_int_callback(
        bsp_mpuxxxx_handler_t * const p_handler, 
        void *higher_priority_task_woken
                                            );

/**
 * @brief Register user callback for data ready notification
 * @param p_handler[in] pointer to the handler instance
 * @param pf_callback[in] callback function pointer
 * @return MPU_HANDLER_status_t
 */
MPU_HANDLER_status_t bsp_mpuxxxx_handler_register_callback(
        bsp_mpuxxxx_handler_t * const p_handler,
        mpu_data_callback_t pf_callback
                                                          );

/**
 * @brief Get pointer to handler's data buffer
 * @param p_handler[in] pointer to the handler instance
 * @return pointer to data buffer, or NULL if invalid
 * @note This allows external access to the latest sensor data stored in handler.
 */
mpu6050_data_t* bsp_mpuxxxx_handler_get_data_buffer(
        bsp_mpuxxxx_handler_t * const p_handler
                                                    );
//********************************** APIs ***********************************//

#endif /* __BSP_MPUXXXX_HANDLER_H__ */
