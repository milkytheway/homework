/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file bsp_mpuxxxx_handler.c
 * 
 * @brief APIs for MPU6050 sensor handler layer.
 * 
 * @par Dependencies:
 * - bsp_mpuxxxx_driver.h
 * - stdio.h
 * - stdint.h
 * 
 * @author Jack | R&D Dept. | EternalChip
 * 
 * @version V1.0 2025.8.20
 *
 * @note 1 tab == 4 spaces
 *****************************************************************************/

//******************************** Includes *********************************//
#include "bsp_mpuxxxx_handler.h"
#include "elog.h"
#include <string.h>
//******************************** Includes *********************************//

//********************** private macro definitions **************************//
#define MPU_HANDLER_NOT_INITED    0
#define MPU_HANDLER_INITED        1

#define MAX_DELAY                 0xFFFFFFFF
//********************** private macro definitions **************************//

//******************************** variables ********************************//
/* Instance counter for auto-generating instance IDs */
static uint32_t g_mpu_handler_instance_counter = 0;
//******************************** variables ********************************//

//******************************** functions ********************************//

/******************************************************************************
  * @name    mpuxxxx_int_callback
  * @brief   Interrupt callback function for adapter layer to call
  * @param   p_handler[in] pointer to the handler instance
  * @param   higher_priority_task_woken[out] pointer to yield flag
  * @return  MPU_HANDLER_status_t
  * @note    This function encapsulates the interrupt notification logic.
  *          It should be called from HAL_GPIO_EXTI_Callback in adapter layer.
  *          
  *          Execution flow:
  *          Hardware Interrupt → HAL_GPIO_EXTI_Callback (Adapter layer) →
  *          mpuxxxx_int_callback() → os_TaskNotifyGiveFromISR() →
  *          Handler Thread wakes up → Reads sensor data
  *          
  *          This function only abstracts the notification logic - TaskNotifyFromISR.
  *          Actual OS resources (task_handle) and implementation 
  *          (os_TaskNotifyGiveFromISR → vTaskNotifyGiveFromISR) 
  *          are provided by adapter layer.
 *****************************************************************************/
MPU_HANDLER_status_t mpuxxxx_int_callback(
        bsp_mpuxxxx_handler_t * const p_handler, 
        void *higher_priority_task_woken
                                            )
{
    if (NULL == p_handler || 
        NULL == p_handler->os_interface ||
        NULL == p_handler->os_interface->os_TaskNotifyGiveFromISR ||
        NULL == p_handler->os_interface->task_handle)
    {
        return MPU_HANDLER_ERRORPARAMETER;
    }

    /* Notify handler thread that data is ready
     * This is the abstracted logic that handler layer provides
     * The os_interface->os_TaskNotifyGiveFromISR points to actual 
     * FreeRTOS API (vTaskNotifyGiveFromISR) implemented in adapter layer
     */
    p_handler->os_interface->os_TaskNotifyGiveFromISR(
        p_handler->os_interface->task_handle,
        higher_priority_task_woken
    );

    return MPU_HANDLER_OK;
}

/******************************************************************************
  * @name    bsp_mpu_handler_init
  * @brief   Initialize the MPU6050 handler instance
  * @param   handler_instance[in] pointer to the handler instance
  * @return  MPU_HANDLER_status_t
 *****************************************************************************/
static MPU_HANDLER_status_t bsp_mpu_handler_init(
        bsp_mpuxxxx_handler_t * const handler_instance
)
{
    MPUXXXX_status_t driver_ret = MPU_OK;

    if (NULL == handler_instance)
    {
        return MPU_HANDLER_ERRORPARAMETER;
    }

    /* Initialize driver instance */
    driver_ret = mpuxxxx_inst(
            handler_instance->p_mpu_driver_instance,
            handler_instance->iic_driver_interface,
            handler_instance->bus_instance,
            handler_instance->timebase_interface,
#ifdef OS_SUPPORTING
            NULL,  // OS interface not used in driver layer directly
#endif
            handler_instance->interruption_interface
            );

#ifdef HANDLER_DEBUG
    log_d("mpuxxxx_inst ret = %d", driver_ret);
#endif

    if (MPU_OK != driver_ret)
    {
        return MPU_HANDLER_ERRORRESOURCE;
    }

    /* Initialize driver hardware */
    driver_ret = handler_instance->p_mpu_driver_instance->pf_init(
            handler_instance->p_mpu_driver_instance);

#ifdef HANDLER_DEBUG
    log_d("mpu driver init ret = %d", driver_ret);
#endif

    if (MPU_OK != driver_ret)
    {
        return MPU_HANDLER_ERRORRESOURCE;
    }

    /* Note: Interrupt handling is done by adapter layer
     * Adapter layer will call mpuxxxx_int_callback() from HAL_GPIO_EXTI_Callback
     * No need to register callback to driver layer
     */

#ifdef HANDLER_DEBUG
    log_d("handler init end");
#endif

    return MPU_HANDLER_OK;
}

/******************************************************************************
  * @name    bsp_mpuxxxx_handler_inst
  * @brief   Construct the MPU6050 handler instance
  * @param   p_handler[in] pointer to the handler instance
  * @param   p_input_args[in] pointer to all input arguments
  * @return  MPU_HANDLER_status_t
  * @note    Multi-instance safe: Each handler instance has independent state.
  *          This function can be called multiple times for different instances.
  *          
  *          Thread-safe considerations:
  *          - Instance counter uses atomic increment (or protect with mutex if needed)
  *          - Each instance has isolated private_data (no shared state)
 *****************************************************************************/
MPU_HANDLER_status_t bsp_mpuxxxx_handler_inst(
        bsp_mpuxxxx_handler_t * const p_handler,
        mpu_handler_all_input_arg_t * const p_input_args
                                                  )
{
    MPU_HANDLER_status_t ret = MPU_HANDLER_OK;

    /* Check input parameters */
#ifdef HANDLER_DEBUG
    log_d("bsp_mpuxxxx_handler_inst start");
#endif

    if (NULL == p_handler || NULL == p_input_args)
    {
#ifdef HANDLER_DEBUG
        log_d("handler_instance or p_input_args is NULL");
#endif
        return MPU_HANDLER_ERRORPARAMETER;
    }

    if (NULL == p_input_args->iic_driver_interface)
    {
#ifdef HANDLER_DEBUG
        log_d("iic_driver_interface is NULL");
#endif
        return MPU_HANDLER_ERRORPARAMETER;
    }

    if (NULL == p_input_args->timebase_interface)
    {
#ifdef HANDLER_DEBUG
        log_d("timebase_interface is NULL");
#endif
        return MPU_HANDLER_ERRORPARAMETER;
    }

    if (NULL == p_input_args->os_interface)
    {
#ifdef HANDLER_DEBUG
        log_d("os_interface is NULL");
#endif
        return MPU_HANDLER_ERRORPARAMETER;
    }

    if (NULL == p_input_args->interruption_interface)
    {
#ifdef HANDLER_DEBUG
        log_d("interruption_interface is NULL");
#endif
        return MPU_HANDLER_ERRORPARAMETER;
    }

    if (NULL == p_input_args->os_interface->os_thread_create)
    {
#ifdef HANDLER_DEBUG
        log_d("os_thread_create is NULL");
#endif
        return MPU_HANDLER_ERRORPARAMETER;
    }

    /* Initialize handler structure members */
    p_handler->iic_driver_interface = p_input_args->iic_driver_interface;
    p_handler->timebase_interface = p_input_args->timebase_interface;
    p_handler->os_interface = p_input_args->os_interface;
    p_handler->bus_instance = p_input_args->bus_instance;
    p_handler->interruption_interface = p_input_args->interruption_interface;
    p_handler->pf_data_callback = NULL;

    /* Initialize data buffer to zero */
    memset(&p_handler->data_buffer, 0, sizeof(mpu6050_data_t));

    /* Initialize private data - embedded structure, no pointer needed
     * Each instance has its own private_data for multi-instance support
     */
    p_handler->private_data.inited = MPU_HANDLER_NOT_INITED;
    p_handler->private_data.instance_id = g_mpu_handler_instance_counter++;

#ifdef HANDLER_DEBUG
    log_d("Handler instance %u created", p_handler->private_data.instance_id);
#endif

    /* Initialize handler (creates driver instance, registers callbacks) */
    ret = bsp_mpu_handler_init(p_handler);
    if (MPU_HANDLER_OK != ret)
    {
#ifdef HANDLER_DEBUG
        log_e("Handler instance %u: bsp_mpu_handler_init failed, ret = %d",
              p_handler->private_data.instance_id, ret);
#endif
        return ret;
    }

    /* Create handler thread - Following LED handler pattern
     * Handler creates its own thread during initialization
     * Thread receives handler instance pointer as parameter
     */
#ifdef HANDLER_DEBUG
    log_d("Creating handler thread for instance %u...", p_handler->private_data.instance_id);
#endif
    
    ret = p_handler->os_interface->os_thread_create(
                (void *)bsp_mpuxxxx_handler_thread,     /* Thread function */
                "MPU_Handler",                           /* Thread name */
                128 * 4,                                 /* Stack size in words */
                (void *)p_handler,                       /* Parameter: handler instance pointer */
                3,                                       /* Priority */
                &(p_handler->os_interface->task_handle) /* Output: task handle */
                );
    
    if (MPU_HANDLER_OK != ret)
    {
#ifdef HANDLER_DEBUG
        log_e("Failed to create handler thread for instance %u, ret = %d", 
              p_handler->private_data.instance_id, ret);
#endif
        return MPU_HANDLER_ERRORRESOURCE;
    }

#ifdef HANDLER_DEBUG
    log_i("Handler thread created successfully for instance %u, task_handle = %p", 
          p_handler->private_data.instance_id, 
          p_handler->os_interface->task_handle);
#endif


    p_handler->private_data.inited = MPU_HANDLER_INITED;

#ifdef HANDLER_DEBUG
    log_i("Handler instance %u initialized successfully", 
          p_handler->private_data.instance_id);
#endif

    return MPU_HANDLER_OK;
}

/******************************************************************************
  * @name    bsp_mpuxxxx_handler_register_callback
  * @brief   Register user callback for data ready notification
  * @param   p_handler[in] pointer to the handler instance
  * @param   pf_callback[in] callback function pointer
  * @return  MPU_HANDLER_status_t
 *****************************************************************************/
MPU_HANDLER_status_t bsp_mpuxxxx_handler_register_callback(
        bsp_mpuxxxx_handler_t * const p_handler,
        mpu_data_callback_t pf_callback
                                                          )
{
    if (NULL == p_handler)
    {
        return MPU_HANDLER_ERRORPARAMETER;
    }

    p_handler->pf_data_callback = pf_callback;

    return MPU_HANDLER_OK;
}

/******************************************************************************
  * @name    bsp_mpuxxxx_handler_get_data_buffer
  * @brief   Get pointer to handler's data buffer
  * @param   p_handler[in] pointer to the handler instance
  * @return  pointer to data buffer, or NULL if invalid
 *****************************************************************************/
mpu6050_data_t* bsp_mpuxxxx_handler_get_data_buffer(
        bsp_mpuxxxx_handler_t * const p_handler
                                                    )
{
    if (NULL == p_handler)
    {
        return NULL;
    }

    return &p_handler->data_buffer;
}

/******************************************************************************
  * @name    bsp_mpuxxxx_handler_thread
  * @brief   MPU6050 handler thread function - Following LED handler pattern
  * @param   argument[in] pointer to handler instance (bsp_mpuxxxx_handler_t*)
  * @return  void
  * @note    Multi-instance safe: Each thread operates on its own handler instance.
  *          
  *          Architecture pattern (following LED handler):
  *          - Handler instance is allocated in adapter layer
  *          - Handler initialization (bsp_mpuxxxx_handler_inst) creates this thread
  *          - Thread receives pre-initialized handler instance pointer
  *          - Thread does NOT create handler instance on stack
  *          
  *          Execution flow:
  *          1. Receive handler instance pointer from parameter
  *          2. Enter loop waiting for TaskNotify from interrupt
  *          3. When notified:
  *             - Read sensor data from driver to handler's data_buffer
  *             - Call user callback with buffer pointer
  *          
  *          Interrupt handling flow:
  *          Hardware INT → HAL_GPIO_EXTI_Callback (Adapter) →
  *          mpuxxxx_int_callback(&handler_instance) → os_TaskNotifyGiveFromISR() →
  *          This thread wakes up → Reads data
  *          
  *          Key timing fix:
  *          - TaskHandle is stored in os_interface->task_handle BEFORE thread starts
  *          - ISR callbacks can immediately access valid task_handle
  *          - No race condition between thread creation and task_handle availability
 *****************************************************************************/
void bsp_mpuxxxx_handler_thread(void *argument)
{
    uint32_t notify_value = 0;
    MPU_HANDLER_status_t ret = MPU_HANDLER_OK;
    MPUXXXX_status_t driver_ret = MPU_OK;
    bsp_mpuxxxx_handler_t *p_handler = NULL;

    /* Validate argument - should be handler instance pointer */
    if (NULL == argument)
    {
#ifdef HANDLER_DEBUG
        log_e("Handler thread: argument is NULL, thread exiting");
#endif
        return;
    }

    /* Get handler instance pointer - handler is already fully initialized */
    p_handler = (bsp_mpuxxxx_handler_t *)argument;

#ifdef HANDLER_DEBUG
    log_i("Handler instance %u: thread started and ready, task_handle = %p", 
          p_handler->private_data.instance_id,
          p_handler->os_interface->task_handle);
#endif

    /* Main loop: wait for data ready notification from interrupt */
    for (;;)
    {
        /* Wait for TaskNotify from interrupt callback
         * This blocks until mpuxxxx_int_callback() sends notification
         * task_handle is already available before this loop starts
         */
        ret = p_handler->os_interface->os_TaskNotifyTake(
                p_handler->os_interface->task_handle,
                MAX_DELAY,
                &notify_value
                );

        if (MPU_HANDLER_OK == ret && notify_value > 0)
        {
#ifdef HANDLER_DEBUG
            // log_d("Instance %u: data ready notification received (notify_value=%u)", 
            //       p_handler->private_data.instance_id, notify_value);
#endif

            /* Read sensor data from driver to handler's buffer */
            driver_ret = p_handler->p_mpu_driver_instance->pf_read_all(
                    p_handler->p_mpu_driver_instance,
                    &p_handler->data_buffer
                    );

            if (MPU_OK == driver_ret)
            {
                /* Call user callback if registered, passing buffer pointer */
                if (NULL != p_handler->pf_data_callback)
                {
                    p_handler->pf_data_callback(&p_handler->data_buffer);
                }
            }
            else
            {
#ifdef HANDLER_DEBUG
                log_e("Instance %u: read data failed, ret = %d", 
                      p_handler->private_data.instance_id, driver_ret);
#endif
            }
        }
    }
}
//******************************** functions ********************************//
