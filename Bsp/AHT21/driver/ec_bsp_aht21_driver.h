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
#include "driver_resource.h"
//******************************** Includes *********************************//

//********************** private macro definitions **************************//
//#define OS_SUPPORTING // Define this macro if the project supports an RTOS
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
/* Common interface definitions (iic_driver_interface_t, timebase_interface_t, 
 * yield_interface_t) are now provided by driver_resource.h to avoid 
 * redefinition errors when multiple sensor drivers are used together.
 */
//********************************** APIs ***********************************//

//***************************** class definition ***************************//
typedef struct
{
    iic_driver_interface_t     *p_iic_driver_interface;
    void                       *p_bus_instance;
    timebase_interface_t       *p_timebase_interface;
#ifdef OS_SUPPORTING
    yield_interface_t          *p_yield_interface;
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
