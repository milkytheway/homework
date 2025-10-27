/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file ec_bsp_temp_humi_handler.h
 * 
 * @brief APIs for temperature and humidity handler using AHT21 sensor.
 * 
 * @par Dependencies:
 * - ec_bsp_aht21_driver.h
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

 #ifndef __EC_BSP_TEMP_HUMI_HANDLER_H__
 #define __EC_BSP_TEMP_HUMI_HANDLER_H__
//******************************** Includes *********************************//
#include "ec_bsp_aht21_driver.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
//******************************** Includes *********************************//.

//********************** private macro definitions **************************//
#define HANDLER_DEBUG
#define OS_SUPPORTING
//********************** private macro definitions **************************//

//******************************** variables ********************************//
typedef enum
{
  HANDLER_OK                = 0,         /* Operation completed successfully.  */
  HANDLER_ERROR             = 1,         /* Run-time error without case matched*/
  HANDLER_ERRORTIMEOUT      = 2,         /* Operation failed with timeout      */
  HANDLER_ERRORRESOURCE     = 3,         /* Resource not available.            */
  HANDLER_ERRORPARAMETER    = 4,         /* Parameter error.                   */
  HANDLER_ERRORNOMEMORY     = 5,         /* Out of memory.                     */
  HANDLER_ERRORISR          = 6,         /* Not allowed in ISR context         */
  HANDLER_ERRORNOTRESP      = 7,         /* No response from device            */
  HANDLER_RESERVED          = 0x7FFFFFFF /* Reserved                           */
} HANDLER_status_t;

typedef enum
{
    TEMP_HUMI_EVENT_TEMP = 0,
    TEMP_HUMI_EVENT_HUMI,
    TEMP_HUMI_EVENT_TEMP_HUMI
} temp_humi_event_data_t;

//this is the class definition for temp humi event
typedef struct
{
    float *temperature;
    float *humidity;
    uint32_t lifetime;
    uint32_t timestamp;
    temp_humi_event_data_t type;
    void (*pf_callback)(float *, float *);
} temp_humi_event_t;
//******************************** variables ********************************//

//************************** Interface structures ***************************//
//Interface from OS layer
typedef struct
{
    void (*os_delay_ms)(uint32_t const ms);

    HANDLER_status_t (*os_queue_create)(
                            uint32_t const item_num,
                            uint32_t const item_size,
                            void **  p_queue_handle);

    HANDLER_status_t (*os_queue_put)(
                            void * const p_queue_handle,
                            void * const item,
                            uint32_t timeout);

    HANDLER_status_t (*os_queue_get)(
                            void * const p_queue_handle,
                            void * const msg,
                            uint32_t timeout);
} temp_humi_handler_os_interface_t;

typedef struct
{
    iic_driver_interface_t *iic_driver_interface;
    timebase_interface_t  *timebase_interface;
    temp_humi_handler_os_interface_t *os_interface;
    yield_interface_t *yield_interface;
    void *bus_isntance;
} temp_humi_handler_all_input_arg_t;
//************************** Interface structures ***************************//

//***************************** class definition ****************************//
typedef struct temp_humi_handler_private_data temp_humi_handler_private_data_t;

typedef struct bsp_temp_humi_xxx_handler
{
    /* Interface passed to driver layer */
    iic_driver_interface_t  *iic_driver_interface;
    timebase_interface_t      *timebase_interface;
    yield_interface_t            *yield_interface;
    void                            *bus_instance;
    /* Interface from OS layer */
    temp_humi_handler_os_interface_t *os_interface;

    /* Driver instance */
    bsp_aht21_driver_t *p_aht21_instance;

    /* Handler of event queue */
    void *event_queue_handle;

    /* Private data */
    temp_humi_handler_private_data_t *p_private_data;

    /* Timestamp of last temperature reading */
    uint32_t last_temp_tick;
    /* Timestamp of last humidity reading */
    uint32_t last_humi_tick;
} bsp_temp_humi_xxx_handler_t;

//***************************** class definition ****************************//

//********************************** APIs ***********************************//
void temp_humi_handler_thread(void *argument);

HANDLER_status_t bsp_temp_humi_xxx_handler_inst(
        bsp_temp_humi_xxx_handler_t * const p_handler,
        temp_humi_handler_all_input_arg_t * const p_input_args
                                            );

HANDLER_status_t bsp_temp_humi_xxx_read(temp_humi_event_t * const p_event);
//********************************** APIs ***********************************//

 #endif /* __EC_BSP_TEMP_HUMI_HANDLER_H__ */
