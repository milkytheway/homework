/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file driver_resource.h
 * 
 * @brief Common BSP driver resource definitions shared across all sensor drivers
 * 
 * @details This file contains common interface structures used by multiple
 *          BSP driver layers (AHT21, MPU6050, etc.) to avoid redefinition errors.
 *          
 *          All sensor drivers should include this header instead of defining
 *          these interfaces independently.
 * 
 * @author Jack | R&D Dept. | EternalChip
 * 
 * @version V1.0 2025.10.30
 *
 * @note 1 tab == 4 spaces
 *****************************************************************************/

#ifndef __DRIVER_RESOURCE_H__
#define __DRIVER_RESOURCE_H__

//******************************** Includes *********************************//
#include <stdint.h>
#define OS_SUPPORTING
//******************************** Includes *********************************//

//************************** IIC Driver Interface ***************************//
/**
 * @brief IIC driver interface structure
 * @note This interface is used by all sensor drivers that communicate via IIC.
 *       Function pointers return driver-specific status types (cast as needed).
 *       
 *       Different drivers may cast the return type to their own status enums:
 *       - AHT21: (AHT21_status_t)
 *       - MPU6050: (MPUXXXX_status_t)
 */
#ifndef HARDWARE_IIC
typedef struct
{
    int (*pf_iic_init)                     (void *);
    int (*pf_iic_deinit)                   (void *);
    int (*pf_iic_start)                    (void *);
    int (*pf_iic_stop)                     (void *);
    int (*pf_iic_send_byte)                (void *, uint8_t);
    int (*pf_iic_receive_byte)             (void *, uint8_t *);
    int (*pf_iic_wait_ack)                 (void *);
    int (*pf_iic_send_ack)                 (void *);
    int (*pf_iic_send_nack)                (void *);
    int (*pf_iic_write_reg)                (void *, uint8_t daddr, uint8_t reg, uint8_t data);
    int (*pf_iic_read_reg)                 (void *, uint8_t daddr, uint8_t reg);
    int (*pf_iic_read_multi_byte)          (void *, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[]);

    void (*pf_delay_ms)                    (uint32_t);

#ifdef OS_SUPPORTING
    void (*pf_critical_enter)              (void);
    void (*pf_critical_exit)               (void);
#endif /* OS_SUPPORTING */
} iic_driver_interface_t;
#endif /* HARDWARE_IIC */

#ifdef HARDWARE_IIC
typedef struct
{
    int (*pf_iic_init)                     (void *);
    int (*pf_iic_deinit)                   (void *);
    int (*pf_iic_start)                    (void *);
    int (*pf_iic_stop)                     (void *);
    int (*pf_iic_send_byte)                (uint8_t);
    int (*pf_iic_receive_byte)             (uint8_t *, uint8_t);
    int (*pf_iic_wait_ack)                 (void *);
    int (*pf_iic_send_ack)                 (void *);
    int (*pf_iic_send_nack)                (void *);
    int (*pf_iic_read_dma)                 (void *, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t *buff);

    int (*pf_delay_ms)                     (uint32_t);

    int (*pf_critical_enter)               (void);
    int (*pf_critical_exit)                (void);
} iic_driver_interface_t;
#endif /* HARDWARE_IIC */
//************************** IIC Driver Interface ***************************//

//************************** Timebase Interface *****************************//
/**
 * @brief Timebase interface structure
 * @note Provides system tick count for timeout and delay calculations.
 *       Typically implemented using HAL_GetTick() or equivalent.
 */
typedef struct
{
    uint32_t (*pf_get_tick_count) (void);
} timebase_interface_t;
//************************** Timebase Interface *****************************//

//************************** OS Yield Interface *****************************//
#ifdef OS_SUPPORTING
/**
 * @brief OS yield interface structure
 * @note Provides RTOS delay/yield functionality.
 *       Typically implemented using vTaskDelay() or equivalent.
 */
typedef struct
{
    void (*pf_rtos_yield) (const uint32_t);
} yield_interface_t;
#endif /* OS_SUPPORTING */
//************************** OS Yield Interface *****************************//

#ifdef OS_SUPPORTING
typedef struct 
{
    void *task_handle;
    void *mutex_handle;
    
    int (*pf_os_delay) (uint32_t);

    int (*os_queue_create)(
                            uint32_t const item_num,
                            uint32_t const item_size,
                            void **  p_queue_handle);

    int (*os_queue_put)(
                            void * const p_queue_handle,
                            void * const item,
                            uint32_t timeout);

    int (*os_queue_get)(
                            void * const p_queue_handle,
                            void * const msg,
                            uint32_t timeout);
    
    int (*os_TaskNotifyGiveFromISR)(void *task_handle, void *higher_priority_task_woken);
    int (*os_TaskNotifyTake)(void *task_handle, void *higher_priority_task_woken);

    int (*pf_os_semaphore_create_mutex) (void **);
    int (*pf_os_semaphore_create_binary) (void **);
    int (*pf_os_semaphore_take) (void *semaphore_handle, uint32_t ticksToWait);
    int (*pf_os_semaphore_give) (void *);
} os_interface_t;

#endif

#endif /* __DRIVER_RESOURCE_H__ */

