/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file ec_bsp_aht21_driver.h
 * 
 * @brief HAL APIs for AHT21 sensor and related operations.
 * 
 * @par Dependencies:
 * - ec_bsp_aht21_reg.h
 * - stdio.h
 * - stdint.h
 * 
 * @author Jack | R&D Dept. | EternalChip
 * 
 * @version V1.0 2025.8.20
 *
 * @note 1 tab == 4 spaces
 *****************************************************************************/
#ifndef __BSP_AHT21_DRIVER_H__
#define __BSP_AHT21_DRIVER_H__
//******************************** Includes *********************************//
#include <stdio.h>
#include <stdint.h>
//******************************** Includes *********************************//

//********************** private macro definitions **************************//
#define OS_SUPPORTING // Define this macro if the project supports an RTOS
// #define HARDWARE_IIC
//********************** private macro definitions **************************//


//******************************** variables ********************************//
/*      function return values      */
typedef enum
{
  AHT21_OK                = 0,         /* Operation completed successfully.  */
  AHT21_ERROR             = 1,         /* Run-time error without case matched*/
  AHT21_ERRORTIMEOUT      = 2,         /* Operation failed with timeout      */
  AHT21_ERRORRESOURCE     = 3,         /* Resource not available.            */
  AHT21_ERRORPARAMETER    = 4,         /* Parameter error.                   */
  AHT21_ERRORNOMEMORY     = 5,         /* Out of memory.                     */
  AHT21_ERRORISR          = 6,         /* Not allowed in ISR context         */
  AHT21_ERRORNOTRESP      = 7,         /* No response from device            */
  AHT21_RESERVED          = 0x7FFFFFFF /* Reserved                           */
} AHT21_status_t;
//******************************** variables ********************************//

//********************************** APIs ***********************************//
#ifndef HARDWARE_IIC
typedef struct
{
    AHT21_status_t (*pf_iic_init)                     (void *);
    AHT21_status_t (*pf_iic_deinit)                   (void *);
    AHT21_status_t (*pf_iic_start)                    (void *);
    AHT21_status_t (*pf_iic_stop)                     (void *);
    AHT21_status_t (*pf_iic_send_byte)                (void *, uint8_t);
    AHT21_status_t (*pf_iic_receive_byte)             (void *, uint8_t *);
    AHT21_status_t (*pf_iic_wait_ack)                 (void *);
    AHT21_status_t (*pf_iic_send_ack)                 (void *);
    AHT21_status_t (*pf_iic_send_nack)                (void *);

    void (*pf_delay_ms)                               (uint32_t);

#ifdef OS_SUPPORTING
    void (*pf_critical_enter)                         (void);
    void (*pf_critical_exit)                          (void);
#endif /* OS_SUPPORTING */
} iic_driver_interface_t;
#endif /* HARDWARE_IIC */

#ifdef HARDWARE_IIC
typedef struct
{
    AHT21_status_t (*pf_iic_init)           (void *);
    AHT21_status_t (*pf_iic_deinit)         (void *);
    AHT21_status_t (*pf_iic_start)          (void *);
    AHT21_status_t (*pf_iic_stop)           (void *);
    AHT21_status_t (*pf_iic_send_byte)      (uint8_t);
    AHT21_status_t (*pf_iic_receive_byte)   (uint8_t *, uint8_t);
    AHT21_status_t (*pf_iic_wait_ack)       (void *);
    AHT21_status_t (*pf_iic_send_ack)       (void *);
    AHT21_status_t (*pf_iic_send_nack)      (void *);

    AHT21_status_t (*pf_delay_ms)           (uint32_t);

    AHT21_status_t (*pf_critical_enter)     (void);
    AHT21_status_t (*pf_critical_exit)      (void);
} iic_driver_interface_t;
#endif /* HARDWARE_IIC */

/*      time base      */
typedef struct
{
    uint32_t (*pf_get_tick_count) (void);
} timebase_interface_t;

#ifdef OS_SUPPORTING
typedef struct
{
    void (*pf_rtos_yield) (const uint32_t);
} yield_interface_t;
#endif
//********************************** APIs ***********************************//

//***************************** class definition ***************************//
typedef struct
{
    iic_driver_interface_t     *p_iic_driver_interface;
    void                       *p_bus_instance;
    timebase_interface_t       *p_timebase_interface;
#ifdef OS_SUPPORTING
    yield_interface_t          *p_yield_interface;
    
    /* IIC bus lock function pointers for multi-device arbitration
     * These function pointers allow the driver to acquire/release a bus lock
     * without depending on specific OS APIs. The actual lock implementation
     * is provided by the system resource layer.
     * 
     * pf_bus_lock: Acquire the IIC bus lock (should block until acquired)
     *              Returns AHT21_OK on success, AHT21_ERROR on failure
     * pf_bus_unlock: Release the IIC bus lock
     * 
     * If set to NULL, no locking is performed (for single-device systems)
     */
    AHT21_status_t (*pf_bus_lock)   (void * const p_context, uint32_t timeout);
    void           (*pf_bus_unlock) (void * const p_context);
    void           *p_bus_lock_context;  // Context passed to lock functions
#endif

    uint8_t (*pfinst)(
                                      void * const p_aht21_driver,
            iic_driver_interface_t * const p_iic_driver_interface,
#ifdef OS_SUPPORTING
            yield_interface_t *           const p_yield_interface,
#endif
            timebase_interface_t *     const p_timebase_interface,
            void *                           const p_bus_instance
                    );

    AHT21_status_t (*pf_init)          (void * const);
    AHT21_status_t (*pf_deinit)        (void * const);
    AHT21_status_t (*pf_read_id)       (void * const);
    AHT21_status_t (*pf_read_temp_humi)(void * const, float * const temp, float * const humi);
    AHT21_status_t (*pf_read_temp)     (void * const, float * const temp);
    AHT21_status_t (*pf_read_humidity) (void * const, float * const humi);
    AHT21_status_t (*pf_sleep)         (void * const);
    AHT21_status_t (*pf_wakeup)        (void * const);
} bsp_aht21_driver_t;

AHT21_status_t aht21_inst(
        bsp_aht21_driver_t *             const p_aht21_driver,
        iic_driver_interface_t * const p_iic_driver_interface,
#ifdef OS_SUPPORTING
        yield_interface_t *           const p_yield_interface,
#endif
        timebase_interface_t *     const p_timebase_interface,
        void *                           const p_bus_instance
                         );
//***************************** class definition ***************************//
#endif /* __BSP_AHT21_DRIVER_H__ */
