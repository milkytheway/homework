/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file ec_bsp_temp_humi_handler.c
 * 
 * @brief APIs for temperature and humidity handler using AHT21 sensor.
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

//******************************** Includes *********************************//
#include "ec_bsp_temp_humi_handler.h"
#include "elog.h"
//******************************** Includes *********************************//

//********************** private macro definitions **************************//
#define TEMP_HUMI_NOT_INITED    0
#define TEMP_HUMI_INITED        1

#define OS_QUEUE_CREATE handler_instance->os_interface->os_queue_create

#define MAX_DELAY 0xFFFFFFFF
//********************** private macro definitions **************************//

//******************************** variables ********************************//
static bsp_temp_humi_xxx_handler_t *gp_temp_humi_instance = NULL;

typedef struct temp_humi_handler_private_data
{
    uint8_t inited;
} temp_humi_handler_private_data_t;
//******************************** variables ********************************//

//******************************** functions ********************************//

/******************************************************************************
  * @name    __mount_handler
  * @brief   Mount the temperature and humidity handler instance
  * @param   instance[in] pointer to the handler instance
  * @return  void
 *****************************************************************************/
void __mount_handler(bsp_temp_humi_xxx_handler_t *instance)
{
    gp_temp_humi_instance = instance;
}

/******************************************************************************
  * @name    bsp_temp_xxx_handler_init
  * @brief   initialize the temperature and humidity handler instance
  * @param   handler_instance[in] pointer to the handler instance
  * @return  HANDLER_status_t
 *****************************************************************************/
static HANDLER_status_t bsp_temp_xxx_handler_init(
        bsp_temp_humi_xxx_handler_t * const handler_instance
)
{
    HANDLER_status_t ret = HANDLER_OK;
    AHT21_status_t driver_ret = AHT21_OK;
    if(NULL == handler_instance)
    {
        return HANDLER_ERRORPARAMETER;
    }

    if(NULL == OS_QUEUE_CREATE)
    {
        return HANDLER_ERRORRESOURCE;
    }

    ret = OS_QUEUE_CREATE(
            10,
            sizeof(temp_humi_event_t),
            &handler_instance->event_queue_handle
                        );

    if(HANDLER_OK != ret)
    {
#ifdef HANDLER_DEBUG
    log_d("create queue failed");
#endif
        return HANDLER_ERRORRESOURCE;
    }

    driver_ret = aht21_inst(
            handler_instance->p_aht21_instance,
            handler_instance->iic_driver_interface,
#ifdef OS_SUPPORTING
            handler_instance->yield_interface,
#endif
            handler_instance->timebase_interface,
            handler_instance->bus_instance
                    );

#ifdef HANDLER_DEBUG
    log_d("aht21_inst ret = %d", driver_ret);
#endif

    if(AHT21_OK != driver_ret)
    {
        return HANDLER_ERRORRESOURCE;
    }

#ifdef HANDLER_DEBUG
    log_d("handler init end");
#endif

    return HANDLER_OK;
}

/******************************************************************************
  * @name    bsp_temp_xxx_handler_deinit
  * @brief   deinitialize the temperature and humidity handler instance
  * @param   handler_instance[in] pointer to the handler instance
  * @return  HANDLER_status_t
 *****************************************************************************/
static HANDLER_status_t bsp_temp_xxx_handler_deinit(void)
{
    gp_temp_humi_instance->p_private_data->inited = TEMP_HUMI_NOT_INITED;
    return HANDLER_OK;
}

/******************************************************************************
  * @name    bsp_temp_humi_xxx_handler_inst
  * @brief   construct the temperature and humidity handler instance
  * @param   handler_instance[in] pointer to the handler instance
  * @param   p_input_args[in] pointer to all input arguments
  * @return  HANDLER_status_t
 *****************************************************************************/
HANDLER_status_t bsp_temp_humi_xxx_handler_inst(
        bsp_temp_humi_xxx_handler_t * const handler_instance,
        temp_humi_handler_all_input_arg_t * const p_input_args
                                            )
{
    HANDLER_status_t ret = HANDLER_OK;

/*      Check input parameters       */
#ifdef HANDLER_DEBUG
    log_d("bsp_temp_humi_xxx_handler_inst start");
#endif

    if(NULL == handler_instance ||
       NULL == p_input_args)
    {
#ifdef HANDLER_DEBUG
        log_d("handler_instance or p_input_args is NULL");
#endif
        return HANDLER_ERRORPARAMETER;
    }

    if(NULL == p_input_args->iic_driver_interface)
    {
#ifdef HANDLER_DEBUG
        log_d("iic_driver_interface is NULL");
#endif
        return HANDLER_ERRORPARAMETER;
    }

    if(NULL == p_input_args->timebase_interface)
    {
#ifdef HANDLER_DEBUG
        log_d("timebase_interface is NULL");
#endif
        return HANDLER_ERRORPARAMETER;
    }

    if(NULL == p_input_args->os_interface)
    {
#ifdef HANDLER_DEBUG
        log_d("os_interface is NULL");
#endif
        return HANDLER_ERRORPARAMETER;
    }

    /*      Initialize private data       */
    handler_instance->iic_driver_interface = p_input_args->iic_driver_interface;
    handler_instance->timebase_interface = p_input_args->timebase_interface;
    handler_instance->os_interface = p_input_args->os_interface;
    handler_instance->yield_interface = p_input_args->yield_interface;
    handler_instance->bus_instance = p_input_args->bus_isntance;

    ret = bsp_temp_xxx_handler_init(handler_instance);
    if(HANDLER_OK != ret)
    {
#ifdef HANDLER_DEBUG
        log_d("bsp_temp_xxx_handler_init ret = %d",ret);
#endif
        return ret;
    }

    handler_instance->p_private_data->inited = TEMP_HUMI_INITED;
#ifdef HANDLER_DEBUG
    log_d("bsp_temp_humi_xxx_handler_inst end");
#endif
    return HANDLER_OK;
}

/******************************************************************************
  * @name    get_temp_humi
  * @brief   Get temperature and humidity data from the AHT21 sensor
  * @param   p_event[in] pointer to the event
  * @param   handler_instance[in] pointer to the handler instance
  * @param   temperature[out] pointer to store temperature data
  * @param   humidity[out] pointer to store humidity data
  * @return  HANDLER_status_t
 *****************************************************************************/
HANDLER_status_t get_temp_humi(
        bsp_temp_humi_xxx_handler_t * const handler_instance,
        temp_humi_event_t * const p_event,
        float *temperature,
        float *humidity)
{
    if(NULL == handler_instance ||
       NULL == p_event ||
       NULL == temperature ||
       NULL == humidity)
    {
        return HANDLER_ERRORPARAMETER;
    }

    uint32_t tim = handler_instance->timebase_interface->pf_get_tick_count();

    switch (p_event->type)
    {
        case TEMP_HUMI_EVENT_TEMP:
            if((tim - handler_instance->last_temp_tick) >= p_event->lifetime)
            {
								AHT21_status_t driver_ret = AHT21_OK;
                driver_ret = handler_instance->p_aht21_instance->pf_read_temp_humi(
                        handler_instance->p_aht21_instance,
                        temperature,
                        humidity);

                if(AHT21_OK != driver_ret)
                {
                    return HANDLER_ERRORNOTRESP;
                }
                handler_instance->last_temp_tick = tim;
            }
            break;

        case TEMP_HUMI_EVENT_HUMI:
            if((tim - handler_instance->last_humi_tick) >= p_event->lifetime)
            {
								AHT21_status_t driver_ret = AHT21_OK;
                driver_ret = handler_instance->p_aht21_instance->pf_read_humidity(
                        handler_instance->p_aht21_instance,
                        humidity);

                if(AHT21_OK != driver_ret)
                {
                    return HANDLER_ERRORNOTRESP;
                }
                handler_instance->last_humi_tick = tim;
            }
            break;
        
        case TEMP_HUMI_EVENT_TEMP_HUMI:
            if((tim - handler_instance->last_temp_tick) >= p_event->lifetime)
            {
								AHT21_status_t driver_ret = AHT21_OK;
                driver_ret = handler_instance->p_aht21_instance->pf_read_temp_humi(
                        handler_instance->p_aht21_instance,
                        temperature,
                        humidity);

                if(AHT21_OK != driver_ret)
                {
                    return HANDLER_ERRORNOTRESP;
                }
                handler_instance->last_temp_tick = tim;
            }
            break;

        default:
            *temperature = 0;
            *humidity = 0;
            return HANDLER_ERRORPARAMETER;

    }
		return HANDLER_OK;
}

/******************************************************************************
  * @name    temp_humi_handler_thread
  * @brief   Temperature and humidity handler thread function
  * @param   argument[in] pointer to the input argument
  * @return  HANDLER_status_t
 *****************************************************************************/
void temp_humi_handler_thread(void *argument)
{
// #ifdef HANDLER_DEBUG
//     log_d("temp_humi_handler_thread start");
// #endif

    float temperature = 0.0f;
    float humidity = 0.0f;
    temp_humi_event_t event;
    temp_humi_handler_all_input_arg_t *input_arg = NULL;
    HANDLER_status_t ret = HANDLER_OK;

    //AHT21 instance
    bsp_aht21_driver_t bsp_aht21_driver;
    //handler instance
    bsp_temp_humi_xxx_handler_t handler_instance = {0};
    temp_humi_handler_private_data_t private_data = {0};

    if(NULL == argument)
    {
#ifdef HANDLER_DEBUG
        log_d("argument is NULL");
#endif
        return;
    }

    input_arg = (temp_humi_handler_all_input_arg_t *)argument;
    handler_instance.p_aht21_instance = &bsp_aht21_driver;
    handler_instance.p_private_data = &private_data;

    ret = bsp_temp_humi_xxx_handler_inst(
            &handler_instance,
            input_arg
                                        );

    if(HANDLER_OK == ret)
    {
        __mount_handler(&handler_instance);
#ifdef HANDLER_DEBUG
        log_d("mount gp_temp_humi_instance");
#endif
    }

    for(;;)
    {
        ret = handler_instance.os_interface->os_queue_get(
                handler_instance.event_queue_handle,
                &event,
                MAX_DELAY);

        if(HANDLER_OK == get_temp_humi(
                &handler_instance,
                &event,
                &temperature,
                &humidity))
        {
            if(NULL != event.pf_callback)
            {
// #ifdef HANDLER_DEBUG
// 								log_d("event.pf_callback called");
// #endif
                event.pf_callback(&temperature, &humidity);
            }
        }
    }
}

/******************************************************************************
  * @name    bsp_temp_humi_xxx_read
  * @brief   Read temperature and humidity data through the handler
  * @param   p_event[in] pointer to the event
  * @return  HANDLER_status_t
 *****************************************************************************/
HANDLER_status_t bsp_temp_humi_xxx_read(temp_humi_event_t * const p_event)
{
// #ifdef HANDLER_DEBUG
//     log_d("bsp_temp_humi_xxx_read start");
// #endif

    if(NULL == gp_temp_humi_instance ||
       NULL == p_event)
    {
#ifdef HANDLER_DEBUG
				log_d("gp_temp_humi_instance or input arg invalid");
#endif
        return HANDLER_ERRORPARAMETER;
    }

    HANDLER_status_t ret = HANDLER_OK;

    //check if the handler is initialized
    if(TEMP_HUMI_NOT_INITED == gp_temp_humi_instance->p_private_data->inited)
    {
        return HANDLER_ERRORRESOURCE;
    }

    //queue the event for processing
    ret = gp_temp_humi_instance->os_interface->os_queue_put(
            gp_temp_humi_instance->event_queue_handle,
            p_event,
            MAX_DELAY);

// #ifdef HANDLER_DEBUG
//     log_d("os_queue_put ret = %d",ret);
// #endif

    if(HANDLER_OK != ret)
    {
        return HANDLER_ERRORRESOURCE;
    }
// #ifdef HANDLER_DEBUG
//     log_d("bsp_temp_humi_xxx_read end");
// #endif
    return HANDLER_OK;
}
//******************************** functions ********************************//
