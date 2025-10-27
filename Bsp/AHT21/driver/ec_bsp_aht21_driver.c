/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file ec_bsp_aht21_driver.c
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

//******************************** Includes *********************************//
#include "ec_bsp_aht21_driver.h"
#include "ec_bsp_aht21_reg.h"
#include "elog.h"
//******************************** Includes *********************************//

//********************** private macro definitions **************************//
#define DEBUG

#define AHT21_MEASURE_WAITING_TIME  80U  /*ms*/
#define AHT21_NOT_INITED            0
#define AHT21_INITED                1
#define AHT21_ID                    0x18

#define CRC8_POLYNOMIAL             0x31
#define CRC8_INIT                   0xFF

#define IS_INITED                   (AHT21_INITED == g_inited)

#define AHT21_IIC_INSTANCE          p_aht21_instance->p_iic_driver_interface
#define AHT21_TIMEBASE_INSTANCE     p_aht21_instance->p_timebase_interface
#define BUS                         p_aht21_instance->p_bus_instance
//********************** private macro definitions **************************//

//********************** private function prototypes ************************//
static uint8_t CheckCrc8(const uint8_t *p_data, const uint8_t length);
static AHT21_status_t __read_id(bsp_aht21_driver_t * const p_aht21_instance);
static AHT21_status_t aht21_read_id(bsp_aht21_driver_t * const p_aht21_instance);
static AHT21_status_t aht21_init(bsp_aht21_driver_t * const p_aht21_instance);
static AHT21_status_t aht21_deinit(bsp_aht21_driver_t * const p_aht21_instance);
static uint8_t aht21_read_status(bsp_aht21_driver_t * const p_aht21_instance);
static AHT21_status_t aht21_read_temp_humi(\
                                bsp_aht21_driver_t * const p_aht21_instance,
                                float * const temperature,
                                float * const humidity);
//********************** private function prototypes ************************//

//********************** private variables **********************************//
static int8_t g_inited      = AHT21_NOT_INITED;
static uint8_t g_device_id  = 0;
//********************** private variables **********************************//

/******************************************************************************
  * @name    CheckCrc8
  * @brief   Check the CRC8 of the AHT21 data
  * @param   p_data[in] pointer to the data buffer
  * @param   length[in] length of the data buffer
  * @return  calculated CRC8 value
 *****************************************************************************/
 static uint8_t CheckCrc8(const uint8_t *p_data, const uint8_t length)
 {
     uint8_t crc = CRC8_INIT;
     for (uint8_t i = 0; i < length; i++) {
         crc ^= p_data[i];
         for (uint8_t j = 0; j < 8; j++) {
             if (crc & 0x80) {
                 crc = (crc << 1) ^ CRC8_POLYNOMIAL;
             } else {
                 crc <<= 1;
             }
         }
     }
     return crc;
 }

 /******************************************************************************
  * @name    __read_id
  * @brief   Read the ID of the AHT21 sensor
  * @param   p_aht21_instance[in] pointer to the AHT21 instance
  * @return  AHT21 status
 *****************************************************************************/
static AHT21_status_t __read_id(bsp_aht21_driver_t * const p_aht21_instance)
{
    uint8_t data = 0;
#ifdef OS_SUPPORTING
    AHT21_IIC_INSTANCE->pf_critical_enter();
#endif //OS_SUPPORTING

    //send the IIC start signal
    AHT21_IIC_INSTANCE->pf_iic_start(BUS);

    //send command to read ID
    AHT21_IIC_INSTANCE->pf_iic_send_byte(BUS, AHT21_REG_READ_ADDR);

    //wait for ACK
    if(AHT21_OK == AHT21_IIC_INSTANCE->pf_iic_wait_ack(BUS))
    {
        AHT21_IIC_INSTANCE->pf_iic_receive_byte(BUS, &data);
    }

    //send stop signal
    AHT21_IIC_INSTANCE->pf_iic_stop(BUS);

#ifdef OS_SUPPORTING
    AHT21_IIC_INSTANCE->pf_critical_exit();
#endif //OS_SUPPORTING

#ifdef DEBUG
	log_a("aht21 read id data = %x",data);
#endif

    if(AHT21_ID == (data & AHT21_ID))
    {
        g_device_id = AHT21_ID;
#ifdef DEBUG
		log_a("g_device_id = %x",g_device_id);
#endif
        return AHT21_OK;
    }
    else
    {
#ifdef DEBUG
		log_e("read failed");
#endif
        return AHT21_ERRORRESOURCE;
    }
}

/******************************************************************************
  * @name    aht21_read_id
  * @brief   Read the ID of the AHT21 sensor
  * @param   p_aht21_instance[in] pointer to the AHT21 instance
  * @return  g_device_id
 *****************************************************************************/
static AHT21_status_t aht21_read_id(bsp_aht21_driver_t * const p_aht21_instance)
{
    if(!IS_INITED)
    {
        return AHT21_ERRORRESOURCE;
    }
    return AHT21_OK;
}

 /******************************************************************************
  * @name    aht21_init
  * @brief   Initialize the AHT21 sensor
  * @param   p_aht21_instance[in] pointer to the AHT21 instance
  * @return  AHT21 status
 *****************************************************************************/
static AHT21_status_t aht21_init(bsp_aht21_driver_t * const p_aht21_instance)
{
#ifdef DEBUG
		log_d("aht21_init start");
#endif
	
    AHT21_status_t ret = AHT21_OK;

#ifdef OS_SUPPORTING
   p_aht21_instance->p_yield_interface->pf_rtos_yield(300);
#endif //OS_SUPPORTING

    if(NULL == p_aht21_instance->p_iic_driver_interface ||
       NULL == p_aht21_instance->p_iic_driver_interface->pf_iic_init)
    {
        return AHT21_ERRORPARAMETER;
    }

#ifdef OS_SUPPORTING
    AHT21_IIC_INSTANCE->pf_critical_enter();
#endif //OS_SUPPORTING

    //init the iic interface
    AHT21_IIC_INSTANCE->pf_iic_init(BUS);

    //read the device id
    ret = __read_id(p_aht21_instance);
    if(AHT21_OK != ret)
    {
#ifdef DEBUG
				log_e("aht21 read_id failed");
#endif
        return AHT21_ERRORRESOURCE;
    }

#ifdef OS_SUPPORTING
    AHT21_IIC_INSTANCE->pf_critical_exit();
#endif //OS_SUPPORTING

    g_inited = AHT21_INITED;

    return AHT21_OK;
}


 /******************************************************************************
  * @name    aht21_deinit
  * @brief   Deinitialize the AHT21 sensor
  * @param   p_aht21_instance[in] pointer to the AHT21 instance
  * @return  AHT21 status
 *****************************************************************************/
static AHT21_status_t aht21_deinit(bsp_aht21_driver_t * const p_aht21_instance)
{
    g_inited = AHT21_NOT_INITED;
    return AHT21_OK;
}

 /******************************************************************************
  * @name    aht21_read_status
  * @brief   Read the status of the AHT21 sensor
  * @param   p_aht21_instance[in] pointer to the AHT21 instance
  * @return  uint8_t rx_data status byte
 *****************************************************************************/
static uint8_t aht21_read_status(bsp_aht21_driver_t * const p_aht21_instance)
{
    if(!IS_INITED)
    {
        return AHT21_ERRORRESOURCE;
    }
    
    uint8_t rx_data = 0;

#ifdef OS_SUPPORTING
    AHT21_IIC_INSTANCE->pf_critical_enter();
#endif //OS_SUPPORTING

    //send the IIC start signal
    AHT21_IIC_INSTANCE->pf_iic_start(BUS);

    //send the adress of IIC slave device
    AHT21_IIC_INSTANCE->pf_iic_send_byte(BUS,AHT21_REG_READ_ADDR);

    //wait for ACK
    if(AHT21_OK == AHT21_IIC_INSTANCE->pf_iic_wait_ack(BUS))
    {
        AHT21_IIC_INSTANCE->pf_iic_receive_byte(BUS, &rx_data);
    }

    //send nack
    AHT21_IIC_INSTANCE->pf_iic_send_nack(BUS);

    //send stop signal
    AHT21_IIC_INSTANCE->pf_iic_stop(BUS);

#ifdef OS_SUPPORTING
    AHT21_IIC_INSTANCE->pf_critical_exit();
#endif //OS_SUPPORTING

    return rx_data;
}

/******************************************************************************
  * @name    aht21_read_temp_humi
  * @brief   Read the temperature and humidity from the AHT21 sensor
  * @param   p_aht21_instance[in] pointer to the AHT21 instance
  * @param   temperature[out] pointer to store temperature value
  * @param   humidity[out] pointer to store humidity value
  * @return  AHT21_status_t
 *****************************************************************************/
static AHT21_status_t aht21_read_temp_humi(\
                                bsp_aht21_driver_t * const p_aht21_instance,
                                float * const temperature,
                                float * const humidity)
{
    if(!IS_INITED)
    {
        return AHT21_ERRORRESOURCE;
    }
    uint8_t        cnt = 5;
    uint8_t     byte_1 = 0;
    uint8_t     byte_2 = 0;
    uint8_t     byte_3 = 0;
    uint8_t     byte_4 = 0;
    uint8_t     byte_5 = 0;
    uint8_t     byte_6 = 0;
    uint32_t  ret_data = 0;

#ifdef OS_SUPPORTING
    AHT21_IIC_INSTANCE->pf_critical_enter();
#endif //OS_SUPPORTING

    //send the IIC start signal
    AHT21_IIC_INSTANCE->pf_iic_start(BUS);

    //send the adress of IIC slave device
    AHT21_IIC_INSTANCE->pf_iic_send_byte(BUS,AHT21_REG_WRITE_ADDR);

    //wait for ACK
    if(AHT21_OK == AHT21_IIC_INSTANCE->pf_iic_wait_ack(BUS))
    {
        //send measure command
        AHT21_IIC_INSTANCE->pf_iic_send_byte(BUS,AHT21_REG_MEASURE_CMD);
    }

    //wait for ACK
    if(AHT21_OK == AHT21_IIC_INSTANCE->pf_iic_wait_ack(BUS))
    {
        //send command to configure the measurement
        AHT21_IIC_INSTANCE->pf_iic_send_byte(BUS,AHT21_REG_MEASURE_CMD_ARGS1);
    }

    //wait for ACK
    if(AHT21_OK == AHT21_IIC_INSTANCE->pf_iic_wait_ack(BUS))
    {
        //send command to configure the measurement
        AHT21_IIC_INSTANCE->pf_iic_send_byte(BUS,AHT21_REG_MEASURE_CMD_ARGS2);
    }

    //wait for ACK
    AHT21_IIC_INSTANCE->pf_iic_wait_ack(BUS);

    //send stop signal
    AHT21_IIC_INSTANCE->pf_iic_stop(BUS);

#ifdef OS_SUPPORTING
    AHT21_IIC_INSTANCE->pf_critical_exit();
#endif //OS_SUPPORTING

    //Delay AHT21_MEASURE_WAITING_TIME ms for measurement
    int32_t start_time = AHT21_TIMEBASE_INSTANCE->pf_get_tick_count();
    while((AHT21_TIMEBASE_INSTANCE->pf_get_tick_count() - start_time) < AHT21_MEASURE_WAITING_TIME)
    {
#ifdef OS_SUPPORTING
    p_aht21_instance->p_yield_interface->pf_rtos_yield(AHT21_MEASURE_WAITING_TIME);
#endif //OS_SUPPORTING
    }

    //check the device is ready
    while((0x80 == (aht21_read_status(p_aht21_instance) & 0x80)) && cnt)
    {
#ifdef OS_SUPPORTING
        p_aht21_instance->p_yield_interface->pf_rtos_yield(5);
#endif //OS_SUPPORTING
        //Device is busy
        cnt--;
        if(0 == cnt)
        {
            return AHT21_ERRORTIMEOUT;
        }
    }

#ifdef OS_SUPPORTING
    AHT21_IIC_INSTANCE->pf_critical_enter();
#endif //OS_SUPPORTING

    //send the IIC start signal
    AHT21_IIC_INSTANCE->pf_iic_start(BUS);

    //send the adress of IIC slave device
    AHT21_IIC_INSTANCE->pf_iic_send_byte(BUS,AHT21_REG_READ_ADDR);

    //wait for ACK
    if(AHT21_OK == AHT21_IIC_INSTANCE->pf_iic_wait_ack(BUS))
    {
        AHT21_IIC_INSTANCE->pf_iic_receive_byte(BUS, &byte_1);
        AHT21_IIC_INSTANCE->pf_iic_send_ack(BUS);

        AHT21_IIC_INSTANCE->pf_iic_receive_byte(BUS, &byte_2);
        AHT21_IIC_INSTANCE->pf_iic_send_ack(BUS);

        AHT21_IIC_INSTANCE->pf_iic_receive_byte(BUS, &byte_3);
        AHT21_IIC_INSTANCE->pf_iic_send_ack(BUS);

        AHT21_IIC_INSTANCE->pf_iic_receive_byte(BUS, &byte_4);
        AHT21_IIC_INSTANCE->pf_iic_send_ack(BUS);

        AHT21_IIC_INSTANCE->pf_iic_receive_byte(BUS, &byte_5);
        AHT21_IIC_INSTANCE->pf_iic_send_ack(BUS);

        AHT21_IIC_INSTANCE->pf_iic_receive_byte(BUS, &byte_6);
        AHT21_IIC_INSTANCE->pf_iic_send_nack(BUS);
    }

    //send stop signal
    AHT21_IIC_INSTANCE->pf_iic_stop(BUS);

#ifdef OS_SUPPORTING
    AHT21_IIC_INSTANCE->pf_critical_exit();
#endif //OS_SUPPORTING

    ret_data = (ret_data | byte_2) << 8;
    ret_data = (ret_data | byte_3) << 8;
    ret_data = (ret_data | byte_4);
    ret_data = ret_data >> 4;
    *humidity = (ret_data * 1000) >> 20;
    *humidity /= 10;

    ret_data = 0;
    ret_data = (ret_data | (byte_4 & 0x0F)) << 8;
    ret_data = (ret_data | byte_5) << 8;
    ret_data = (ret_data | byte_6);
    ret_data = ret_data& 0xFFFFF;
    *temperature = ((ret_data * 2000) >> 20) - 500;
    *temperature /= 10;

    return AHT21_OK;
}

/******************************************************************************
  * @name    aht21_read_temp
  * @brief   Read the temperature from the AHT21 sensor
  * @param   p_aht21_instance[in] pointer to the AHT21 instance
  * @param   temperature[out] pointer to store temperature value
  * @return  AHT21_status_t
 *****************************************************************************/
static AHT21_status_t aht21_read_temp(\
                                bsp_aht21_driver_t * const p_aht21_instance,
                                float * const temperature)
{
    float humidity = 0.0f;
    return aht21_read_temp_humi(p_aht21_instance, temperature, &humidity);
}

/******************************************************************************
  * @name    aht21_read_humidity
  * @brief   Read the humidity from the AHT21 sensor
  * @param   p_aht21_instance[in] pointer to the AHT21 instance
  * @param   humidity[out] pointer to store humidity value
  * @return  AHT21_status_t
 *****************************************************************************/
static AHT21_status_t aht21_read_humidity(\
                                bsp_aht21_driver_t * const p_aht21_instance,
                                float * const humidity)
{
    float temperature = 0.0f;
    return aht21_read_temp_humi(p_aht21_instance, &temperature, humidity);
}

/******************************************************************************
  * @name    aht21_sleep
  * @brief   Put the AHT21 sensor to sleep
  * @param   p_aht21_instance[in] pointer to the AHT21 instance
  * @return  AHT21_status_t
 *****************************************************************************/
static AHT21_status_t aht21_sleep(bsp_aht21_driver_t * const p_aht21_instance)
{
    if(!IS_INITED)
    {
        return AHT21_ERRORRESOURCE;
    }
    return AHT21_OK;
}

/******************************************************************************
  * @name    aht21_wakeup
  * @brief   Put the AHT21 sensor to wakeup
  * @param   p_aht21_instance[in] pointer to the AHT21 instance
  * @return  AHT21_status_t
 *****************************************************************************/
static AHT21_status_t aht21_wakeup(bsp_aht21_driver_t * const p_aht21_instance)
{
    if(!IS_INITED)
    {
        return AHT21_ERRORRESOURCE;
    }
    return AHT21_OK;
}

/******************************************************************************
  * @name    aht21_inst
  * @brief   Instantiate the AHT21 sensor driver
  * @param   p_aht21_instance[in] pointer to the AHT21 driver instance
  * @param   p_iic_driver_interface[in] pointer to the IIC driver interface
  * @param   p_timebase_interface[in] pointer to the timebase interface
  * @param   p_yield_interface[in] pointer to the yield interface
  * @return  AHT21_status_t
 *****************************************************************************/
AHT21_status_t aht21_inst(
        bsp_aht21_driver_t *           const p_aht21_instance,
        iic_driver_interface_t * const p_iic_driver_interface,
#ifdef OS_SUPPORTING
        yield_interface_t *           const p_yield_interface,
#endif
        timebase_interface_t *     const p_timebase_interface,
        void *                           const p_bus_instance
                         )
{
    if(IS_INITED)
    {
        return AHT21_ERRORRESOURCE;
    }

    //log aht21 init start
    uint8_t ret = 0;

    if(NULL == p_aht21_instance ||
       NULL == p_iic_driver_interface ||
       NULL == p_timebase_interface ||
       NULL == p_bus_instance)
    {
        return AHT21_ERRORPARAMETER;
    }

    AHT21_IIC_INSTANCE = p_iic_driver_interface;
    AHT21_TIMEBASE_INSTANCE = p_timebase_interface;
#ifdef OS_SUPPORTING
    p_aht21_instance->p_yield_interface = p_yield_interface;
#endif
    p_aht21_instance->p_bus_instance = p_bus_instance;

    p_aht21_instance->pf_init = (AHT21_status_t (*)(void * const))aht21_init;
    p_aht21_instance->pf_deinit = (AHT21_status_t (*)(void * const))aht21_deinit;
    p_aht21_instance->pf_read_id = (AHT21_status_t (*)(void * const))aht21_read_id;
    p_aht21_instance->pf_read_temp_humi = \
        (AHT21_status_t (*)(void * const, float * const, float * const))\
                                                    aht21_read_temp_humi;
    p_aht21_instance->pf_read_temp = \
        (AHT21_status_t (*)(void * const, float * const))\
                                                    aht21_read_temp;
    p_aht21_instance->pf_read_humidity = \
        (AHT21_status_t (*)(void * const, float * const))\
                                                    aht21_read_humidity;
    p_aht21_instance->pf_sleep = (AHT21_status_t (*)(void * const))\
                                                    aht21_sleep;
    p_aht21_instance->pf_wakeup = (AHT21_status_t (*)(void * const))\
                                                    aht21_wakeup;

    //call the init function
    ret = aht21_init(p_aht21_instance);
    if(AHT21_OK != ret)
    {
        return AHT21_ERRORRESOURCE;
    }
    return AHT21_OK;
}
