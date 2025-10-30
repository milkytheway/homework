/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file system_adaptation.c
 * 
 * @brief HAL APIs for AHT21 sensor and related operations.
 * 
 * @par Dependencies:
 * - AHT21_adapter.h
 * 
 * @author Jack | R&D Dept. | EternalChip
 * 
 * @version V1.0 2025.8.20
 *
 * @note 1 tab == 4 spaces
 *****************************************************************************/

//******************************** Includes *********************************//
#include "ec_bsp_aht21_driver.h"
#include "AHT21_adapter.h"
#include "ec_bsp_temp_humi_handler.h"
#include "iic_hal.h"
#include "elog.h"

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"
#include "core_cm4.h"
#include "FreeRTOS.h"
#include "task.h"
//******************************** Includes *********************************//

//***************************** Driver variables ****************************//
extern void dwt_delay_init(void);
extern void delay_ms(uint32_t ms);
extern void delay_us(uint32_t us);

iic_bus_t AHT_bus = 
{
        .IIC_SDA_PORT = GPIOB,
        .IIC_SCL_PORT = GPIOB,
        .IIC_SDA_PIN  = GPIO_PIN_13,
        .IIC_SCL_PIN  = GPIO_PIN_14,
};
//***************************** Driver variables ****************************//

//********************************** Driver *********************************//

/* wrapper functions for iic_driver_interface_t */
AHT21_status_t IICInit_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICInit(bus);
    return AHT21_OK;
}

AHT21_status_t IICStart_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICStart(bus);
    return AHT21_OK;
}

AHT21_status_t IICStop_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICStop(bus);
    return AHT21_OK;
}

AHT21_status_t IICSendByte_wrapper(void *p_bus, uint8_t data)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICSendByte(bus, data);
    return AHT21_OK;
}

AHT21_status_t IICReceiveByte_wrapper(void *p_bus, uint8_t *p_data)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    unsigned char received = IICReceiveByte(bus);
    *p_data = received;
	//log_e("IIC receive = %d",received);
    return AHT21_OK;
}

AHT21_status_t IICWaitAck_wrapper(void *p_bus)
{
	//0:success 
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    unsigned char result = IICWaitAck(bus);
	//log_e("ACK val = %x",result);
    return (result == 0) ? AHT21_OK : AHT21_ERROR;
}

AHT21_status_t IICSendAck_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICSendAck(bus);
    return AHT21_OK;
}

AHT21_status_t IICSendNotAck_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICSendNotAck(bus);
    return AHT21_OK;
}

iic_driver_interface_t iic_driver_interface = 
{
    .pf_iic_init         = IICInit_wrapper,
    .pf_iic_deinit       = NULL,
    .pf_iic_start        = IICStart_wrapper,
    .pf_iic_stop         = IICStop_wrapper,
    .pf_iic_send_byte    = IICSendByte_wrapper,
    .pf_iic_receive_byte = IICReceiveByte_wrapper,
    .pf_iic_wait_ack     = IICWaitAck_wrapper,
    .pf_iic_send_ack     = IICSendAck_wrapper,
    .pf_iic_send_nack    = IICSendNotAck_wrapper,
    .pf_delay_ms         = delay_ms,
    .pf_critical_enter   = vPortEnterCritical,
    .pf_critical_exit    = vPortExitCritical
};

timebase_interface_t timebase_interface = 
{
    .pf_get_tick_count = HAL_GetTick
};

yield_interface_t yield_interface = 
{
    .pf_rtos_yield = vTaskDelay
};

//********************************** Driver *********************************//

//***************************** Handler variables ***************************//
static float g_temperature = 0.0f;
//***************************** Handler variables ***************************//

//********************************** Handler ********************************//
HANDLER_status_t os_queue_create(
                        uint32_t const item_num,
                        uint32_t const item_size,
                        void **  p_queue_handle)
{
    *p_queue_handle = osMessageQueueNew(item_num, item_size, NULL);
    if( NULL == *p_queue_handle )
    {
      return HANDLER_ERRORRESOURCE;
    }
    else
    {
      return HANDLER_OK;
    }
}

HANDLER_status_t os_queue_put(
                        void * const p_queue_handle,
                        void * const item,
                        uint32_t timeout)
{
    if(osOK == osMessageQueuePut(p_queue_handle, item, NULL, timeout))
    {
        return HANDLER_OK;
    }
    else
    {
        return HANDLER_ERROR;
    }
}

HANDLER_status_t os_queue_get(
                        void * const p_queue_handle,
                        void * const msg,
                        uint32_t timeout)
{
    if(osOK == osMessageQueueGet(p_queue_handle, msg, NULL, timeout))
    {
        return HANDLER_OK;
    }
    else
    {
        return HANDLER_ERROR;
    }
}

void temp_humi_callback(float *temperature, float *humidity)
{
    log_d("Temperature: %.2f C, Humidity: %.2f %%", *temperature, *humidity);
    g_temperature = (*temperature)*(float)1.5;
}

temp_humi_handler_os_interface_t os_interface =
{
    .os_delay_ms = vTaskDelay,
    .os_queue_create = os_queue_create,
    .os_queue_put = os_queue_put,
    .os_queue_get = os_queue_get
};

temp_humi_handler_all_input_arg_t input_args =
{
    .iic_driver_interface = &iic_driver_interface,
    .timebase_interface = &timebase_interface,
    .os_interface = &os_interface,
    .yield_interface = &yield_interface,
    .bus_isntance = &AHT_bus
};
//********************************** Handler ********************************//
