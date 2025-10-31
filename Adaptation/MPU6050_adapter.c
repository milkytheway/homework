/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file MPU6050_adapter.c
 * 
 * @brief Adapter layer implementation for MPU6050 sensor and FreeRTOS integration
 * 
 * @par Dependencies:
 * - MPU6050_adapter.h
 * - iic_hal.h
 * - gpio.h
 * - FreeRTOS.h
 * - task.h
 * 
 * @author Jack | R&D Dept. | EternalChip
 * 
 * @version V1.0 2025.8.20
 *
 * @note 1 tab == 4 spaces
 *****************************************************************************/

//******************************** Includes *********************************//
#include "bsp_mpuxxxx_handler.h"
#include "MPU6050_adapter.h"
#include "iic_hal.h"
#include "elog.h"

#include "main.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
//******************************** Includes *********************************//

//***************************** External Variables **************************//
extern void dwt_delay_init(void);
extern void delay_ms(uint32_t ms);
extern void delay_us(uint32_t us);
//***************************** External Variables **************************//

//***************************** IIC Bus Configuration **********************//
/* IIC bus configuration for MPU6050 */
static iic_bus_t MPU_bus = 
{
    .IIC_SDA_PORT = GPIOB,    // Configure according to your hardware
    .IIC_SCL_PORT = GPIOB,    // Configure according to your hardware
    .IIC_SDA_PIN  = GPIO_PIN_13,
    .IIC_SCL_PIN  = GPIO_PIN_14,
};
//***************************** IIC Bus Configuration **********************//

//***************************** Resource Allocation *************************//
/* Handler and driver instances - allocated in adapter layer
 * Following LED handler pattern: adapter layer manages resources
 */
static bsp_mpuxxxx_driver g_mpu_driver_instance;
static bsp_mpuxxxx_handler_t g_mpu_handler_instance;
//***************************** Resource Allocation *************************//

//********************************** IIC Interface **************************//
/* IIC interface wrapper functions for MPU6050 driver */
static int MPU_IICInit_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICInit(bus);
    return (int)MPU_OK;
}

static int MPU_IICStart_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICStart(bus);
    return (int)MPU_OK;
}

static int MPU_IICStop_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICStop(bus);
    return (int)MPU_OK;
}

static int MPU_IICSendByte_wrapper(void *p_bus, uint8_t data)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICSendByte(bus, data);
    return (int)MPU_OK;
}

static int MPU_IICReceiveByte_wrapper(void *p_bus, uint8_t *p_data)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    unsigned char received = IICReceiveByte(bus);
    if (p_data != NULL)
    {
        *p_data = received;
    }
    return (int)MPU_OK;
}

static int MPU_IICWaitAck_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    unsigned char result = IICWaitAck(bus);
    return (result == 0) ? (int)MPU_OK : (int)MPU_ERROR;
}

static int MPU_IICSendAck_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICSendAck(bus);
    return (int)MPU_OK;
}

static int MPU_IICSendNotAck_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICSendNotAck(bus);
    return (int)MPU_OK;
}

static int MPU_IICWriteReg_wrapper(void *p_bus, uint8_t daddr, uint8_t reg, uint8_t data)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IIC_Write_One_Byte(bus, daddr, reg, data);
    return (int)MPU_OK;
}

static int MPU_IICReadReg_wrapper(void *p_bus, uint8_t daddr, uint8_t reg)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IIC_Read_One_Byte(bus, daddr, reg);
    return (int)MPU_OK;
}

static int MPU_IICReadMultiByte_wrapper(void *p_bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t *p_data)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IIC_Read_Multi_Byte(bus, daddr, reg, length, p_data);
    return (int)MPU_OK;
}

/* IIC driver interface structure */
static iic_driver_interface_t mpu_iic_driver_interface = 
{
    .pf_iic_init         = MPU_IICInit_wrapper,
    .pf_iic_deinit       = NULL,
    .pf_iic_start        = MPU_IICStart_wrapper,
    .pf_iic_stop         = MPU_IICStop_wrapper,
    .pf_iic_send_byte    = MPU_IICSendByte_wrapper,
    .pf_iic_receive_byte = MPU_IICReceiveByte_wrapper,
    .pf_iic_wait_ack     = MPU_IICWaitAck_wrapper,
    .pf_iic_send_ack     = MPU_IICSendAck_wrapper,
    .pf_iic_send_nack    = MPU_IICSendNotAck_wrapper,
    .pf_iic_write_reg    = MPU_IICWriteReg_wrapper,
    .pf_iic_read_reg     = MPU_IICReadReg_wrapper,
    .pf_iic_read_multi_byte = MPU_IICReadMultiByte_wrapper,
    .pf_delay_ms         = delay_ms,
    .pf_critical_enter   = vPortEnterCritical,
    .pf_critical_exit    = vPortExitCritical
};
//********************************** IIC Interface **************************//

//***************************** Timebase Interface **************************//
static timebase_interface_t mpu_timebase_interface = 
{
    .pf_get_tick_count = HAL_GetTick
};
//***************************** Timebase Interface **************************//

//***************************** Yield Interface ****************************//

//***************************** Interruption Interface ********************//
/* Interruption interface wrapper functions */
static MPUXXXX_status_t MPU_IntInit_wrapper(void)
{
    // Configure interrupt GPIO here if needed
    // GPIO initialization is typically done in main.c
    return MPU_OK;
}

static MPUXXXX_status_t MPU_IntDeinit_wrapper(void)
{
    // Disable interrupt if needed
    return MPU_OK;
}

static MPUXXXX_status_t MPU_IntEnable_wrapper(void *instance, uint8_t int_type)
{
    // Enable interrupt based on int_type
    // Implementation depends on GPIO configuration
    return MPU_OK;
}

static MPUXXXX_status_t MPU_IntDisable_wrapper(void *instance, uint8_t int_type)
{
    // Disable interrupt based on int_type
    return MPU_OK;
}

static MPUXXXX_status_t MPU_IntCallback_wrapper(void)
{
    // This will be called from HAL_GPIO_EXTI_Callback
    // The actual interrupt handling is done via mpuxxxx_int_callback()
    return MPU_OK;
}

static interuption_interface_t mpu_interruption_interface = 
{
    .pf_init = MPU_IntInit_wrapper,
    .pf_deinit = MPU_IntDeinit_wrapper,
    .pf_enable_int = MPU_IntEnable_wrapper,
    .pf_disable_int = MPU_IntDisable_wrapper,
    .pf_callback = MPU_IntCallback_wrapper
};
//***************************** Interruption Interface ********************//

//***************************** OS Interface ******************************//
/* OS interface wrapper functions */

/* Thread creation wrapper - adapter layer provides OS-specific implementation */
static MPU_HANDLER_status_t MPU_os_thread_create(
                        void * const task_code,
                        const char * const task_name,
                        const uint32_t stack_depth,
                        void * const parameters,
                        uint32_t priority,
                        void ** const task_handler)
{
    BaseType_t ret = pdPASS;
    
    if(NULL == task_code || NULL == task_handler)
    {
        log_e("MPU_os_thread_create: invalid parameters");
        return MPU_HANDLER_ERRORPARAMETER;
    }
    
    /* Create FreeRTOS task
     * stack_depth is in words (4 bytes each on STM32)
     */
    ret = xTaskCreate(
                (TaskFunction_t)task_code,
                task_name,
                stack_depth,
                parameters,
                (UBaseType_t)priority,
                (TaskHandle_t *)task_handler
                );
    
    if(pdPASS != ret)
    {
        log_e("MPU_os_thread_create: xTaskCreate failed");
        return MPU_HANDLER_ERRORRESOURCE;
    }
    
    log_i("MPU thread '%s' created successfully, handle = %p", task_name, *task_handler);
    return MPU_HANDLER_OK;
}

/* TaskNotify wrapper functions */
static MPU_HANDLER_status_t MPU_TaskNotifyGiveFromISR_wrapper(void *task_handle, void *higher_priority_task_woken)
{
    BaseType_t *pYield = (BaseType_t*)higher_priority_task_woken;
    if (NULL != task_handle)
    {
        vTaskNotifyGiveFromISR((TaskHandle_t)task_handle, pYield);
    }
    return MPU_HANDLER_OK;
}

static MPU_HANDLER_status_t MPU_TaskNotifyTake_wrapper(void *task_handle, uint32_t timeout, uint32_t *p_notify_value)
{
    if (NULL != task_handle && NULL != p_notify_value)
    {
        *p_notify_value = ulTaskNotifyTake(pdTRUE, timeout);
        return MPU_HANDLER_OK;
    }
    return MPU_HANDLER_ERRORPARAMETER;
}

/* OS delay wrapper */
static void MPU_os_delay_ms(uint32_t const ms)
{
    vTaskDelay(ms);
}

/* OS queue wrappers */
static MPU_HANDLER_status_t MPU_os_queue_create(
                        uint32_t const item_num,
                        uint32_t const item_size,
                        void **  p_queue_handle)
{
    QueueHandle_t queue = xQueueCreate(item_num, item_size);
    if(NULL == queue)
    {
        return MPU_HANDLER_ERRORRESOURCE;
    }
    *p_queue_handle = queue;
    return MPU_HANDLER_OK;
}

static MPU_HANDLER_status_t MPU_os_queue_put(
                        void * const p_queue_handle,
                        void * const item,
                        uint32_t timeout)
{
    if (NULL == p_queue_handle || NULL == item)
    {
        return MPU_HANDLER_ERRORPARAMETER;
    }
    
    BaseType_t ret = xQueueSend(p_queue_handle, item, pdMS_TO_TICKS(timeout));
    return (ret == pdTRUE) ? MPU_HANDLER_OK : MPU_HANDLER_ERROR;
}

static MPU_HANDLER_status_t MPU_os_queue_get(
                        void * const p_queue_handle,
                        void * const msg,
                        uint32_t timeout)
{
    if (NULL == p_queue_handle || NULL == msg)
    {
        return MPU_HANDLER_ERRORPARAMETER;
    }
    
    BaseType_t ret = xQueueReceive(p_queue_handle, msg, pdMS_TO_TICKS(timeout));
    return (ret == pdTRUE) ? MPU_HANDLER_OK : MPU_HANDLER_ERROR;
}

/* OS interface structure */
static mpu_handler_os_interface_t mpu_os_interface = 
{
    .task_handle = NULL,  /* Will be set by handler during initialization */
    .os_delay_ms = MPU_os_delay_ms,
    .os_queue_create = MPU_os_queue_create,
    .os_queue_put = MPU_os_queue_put,
    .os_queue_get = MPU_os_queue_get,
    .os_TaskNotifyGiveFromISR = MPU_TaskNotifyGiveFromISR_wrapper,
    .os_TaskNotifyTake = MPU_TaskNotifyTake_wrapper,
    .os_thread_create = MPU_os_thread_create
};
//***************************** OS Interface ******************************//

//***************************** Input Arguments ***************************//
static mpu_handler_all_input_arg_t mpu_input_args = 
{
    .iic_driver_interface = &mpu_iic_driver_interface,
    .timebase_interface = &mpu_timebase_interface,
    .os_interface = &mpu_os_interface,
    .bus_instance = &MPU_bus,
    .interruption_interface = &mpu_interruption_interface
};
//***************************** Input Arguments ***************************//

//***************************** Callback Functions ************************//
void MPU6050_data_callback(mpu6050_data_t *p_data)
{
    if (NULL != p_data)
    {
        log_d("MPU6050 Data: Accel[X=%.2f, Y=%.2f, Z=%.2f], "
              "Gyro[X=%.2f, Y=%.2f, Z=%.2f], Temp=%.2f",
              p_data->ax, p_data->ay, p_data->az,
              p_data->gx, p_data->gy, p_data->gz,
              p_data->tempreture);
    }
}
//***************************** Callback Functions ************************//

//***************************** Public Functions **************************//
/**
 * @brief Initialize MPU6050 handler system
 * 
 * @return MPU_HANDLER_status_t
 */
MPU_HANDLER_status_t MPU6050_adapter_init(void)
{
    MPU_HANDLER_status_t ret = MPU_HANDLER_OK;
    
    log_i("Initializing MPU6050 handler system...");
    
    /* Prepare handler instance - link to driver instance
     * Following LED handler pattern: adapter allocates resources
     */
    g_mpu_handler_instance.p_mpu_driver_instance = &g_mpu_driver_instance;
    
    /* Initialize handler - this will create the handler thread internally
     * TaskHandle is stored in mpu_os_interface.task_handle before thread starts
     */
    ret = bsp_mpuxxxx_handler_inst(
                &g_mpu_handler_instance,
                &mpu_input_args
                );
    
    if(MPU_HANDLER_OK != ret)
    {
        log_e("Failed to initialize MPU6050 handler, ret = %d", ret);
        return ret;
    }
    
    /* Register callback for data ready notification */
    ret = bsp_mpuxxxx_handler_register_callback(&g_mpu_handler_instance, MPU6050_data_callback);
    if(MPU_HANDLER_OK != ret)
    {
        log_w("Failed to register MPU6050 callback, ret = %d", ret);
    }
    
    log_i("MPU6050 handler system initialized successfully");
    log_i("  Handler instance: %p", &g_mpu_handler_instance);
    log_i("  Task handle: %p", mpu_os_interface.task_handle);
    
    return MPU_HANDLER_OK;
}

/**
 * @brief Get pointer to MPU6050 handler instance for ISR access
 * 
 * @return pointer to handler instance, or NULL if not initialized
 */
bsp_mpuxxxx_handler_t* MPU6050_adapter_get_handler_instance(void)
{
    return &g_mpu_handler_instance;
}

/**
 * @brief HAL GPIO EXTI Callback - handles MPU6050 data ready interrupt
 * 
 * @param GPIO_Pin GPIO pin that triggered the interrupt
 * 
 * @note This function overrides the weak HAL_GPIO_EXTI_Callback implementation.
 *       When MPU6050 data ready interrupt occurs, it calls mpuxxxx_int_callback
 *       to notify the handler thread.
 *       
 *       Execution flow:
 *       Hardware INT → EXTI0_IRQHandler → HAL_GPIO_EXTI_IRQHandler →
 *       HAL_GPIO_EXTI_Callback → mpuxxxx_int_callback → TaskNotifyFromISR →
 *       Handler thread wakes up
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Check if this is the MPU6050 interrupt pin */
    if (GPIO_Pin == mpu_int_pin_Pin)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;
        bsp_mpuxxxx_handler_t *p_handler = MPU6050_adapter_get_handler_instance();
        
        /* Call handler interrupt callback to notify handler thread */
        if (NULL != p_handler)
        {
            mpuxxxx_int_callback(p_handler, &higher_priority_task_woken);
            
            /* Yield if higher priority task was woken */
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
}
//***************************** Public Functions **************************//

