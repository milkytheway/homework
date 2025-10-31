/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file bsp_mpuxxxx_driver.h
 * 
 * @brief HAL APIs for MPUxxxx sensor and related operations.
 * 
 * @par Dependencies:
 * - stdio.h
 * - stdint.h
 * 
 * @author Jack | R&D Dept. | EternalChip
 * 
 * @version V1.0 2025.8.20
 *
 * @note 1 tab == 4 spaces
 *****************************************************************************/

 #ifndef __BSP_MPUXXXX_DRIVER_H__
 #define __BSP_MPUXXXX_DRIVER_H__
//******************************** Includes *********************************//
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver_resource.h"
//******************************** Includes *********************************//

//********************** private macro definitions **************************//

/**
 * Hardware I2C + DMA Mode Configuration
 * 
 * Define HARDWARE_IIC to enable DMA mode:
 *   - Uses hardware I2C peripheral with DMA support
 *   - Requires pf_iic_read_dma implementation
 *   - Data automatically transferred to dma_buffer
 *   - Lower CPU usage, faster response
 * 
 * Comment out HARDWARE_IIC to use software I2C mode:
 *   - Uses GPIO bit-banging I2C
 *   - No DMA support needed
 *   - Task actively reads data using pf_read_all()
 *   - More flexible, works without hardware I2C peripheral
 */
//#define HARDWARE_IIC  // Uncomment for Hardware I2C + DMA mode
//********************** private macro definitions **************************//

//******************************** variables ********************************//
typedef enum
{
  MPU_OK                = 0,         /* Operation completed successfully.  */
  MPU_ERROR             = 1,         /* Run-time error without case matched*/
  MPU_ERRORTIMEOUT      = 2,         /* Operation failed with timeout      */
  MPU_ERRORRESOURCE     = 3,         /* Resource not available.            */
  MPU_ERRORPARAMETER    = 4,         /* Parameter error.                   */
  MPU_ERRORNOMEMORY     = 5,         /* Out of memory.                     */
  MPU_ERRORISR          = 6,         /* Not allowed in ISR context         */
  MPU_ERRORNOTRESP      = 7,         /* No response from device            */
  MPU_RESERVED          = 0x7FFFFFFF /* Reserved                           */
} MPUXXXX_status_t;
//******************************** variables ********************************//


//********************************** APIs ***********************************//
/* Common interface definitions (iic_driver_interface_t, timebase_interface_t) 
 * are now provided by driver_resource.h to avoid redefinition errors when 
 * multiple sensor drivers are used together.
 */

typedef struct 
{
    /* data */
    MPUXXXX_status_t (*pf_init) (void);
    MPUXXXX_status_t (*pf_deinit) (void);
    MPUXXXX_status_t (*pf_enable_int) (void *instance, uint8_t int_type);
    MPUXXXX_status_t (*pf_disable_int) (void *instance, uint8_t int_type);
    MPUXXXX_status_t (*pf_callback) (void);
} interuption_interface_t;

typedef struct 
{
    /* data */
    MPUXXXX_status_t (*pf_get_buffer_addr) (uint32_t);
} buffer_interface_t;

#ifdef OS_SUPPORTING
typedef struct 
{
    void *task_handle;
    MPUXXXX_status_t (*pf_os_semaphore_create_mutex) (void **);
    MPUXXXX_status_t (*pf_os_semaphore_create_binary) (void **);
    MPUXXXX_status_t (*pf_os_semaphore_give) (void *);
    MPUXXXX_status_t (*pf_os_delay) (uint32_t);

    MPUXXXX_status_t (*os_queue_create)(
                            uint32_t const item_num,
                            uint32_t const item_size,
                            void **  p_queue_handle);

    MPUXXXX_status_t (*os_queue_put)(
                            void * const p_queue_handle,
                            void * const item,
                            uint32_t timeout);

    MPUXXXX_status_t (*os_queue_get)(
                            void * const p_queue_handle,
                            void * const msg,
                            uint32_t timeout);
    
    MPUXXXX_status_t (*os_TaskNotifyGiveFromISR)(void *task_handle, void *higher_priority_task_woken);
    MPUXXXX_status_t (*os_TaskNotifyTake)(void *task_handle, void *higher_priority_task_woken);
} os_interface_t;

#endif

typedef struct 
{
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    double ax;
    double ay;
    double az;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    double gx;
    double gy;
    double gz;

    float tempreture;

    double kalman_angle_x;
    double kalman_angle_y;
} mpu6050_data_t;

//********************************** APIs ***********************************//

//***************************** class definition ***************************//
typedef struct
{
    iic_driver_interface_t                *iic_interface;
    void                                 *p_bus_instance;
    timebase_interface_t           *p_timebase_interface;
#ifdef OS_SUPPORTING
    os_interface_t                 *p_os_interface;
    buffer_interface_t             *p_buffer_interface;
#endif
    interuption_interface_t       *interuption_interface;

#ifdef OS_SUPPORTING
    MPUXXXX_status_t (*pf_bus_lock)   (void * const p_context, uint32_t timeout);
    void             (*pf_bus_unlock) (void * const p_context);
    void             *p_bus_lock_context;  // Context passed to lock functions
#endif

    /* basic functions */
    MPUXXXX_status_t (*pf_init)           (void * const);
    MPUXXXX_status_t (*pf_deinit)         (void * const);
    MPUXXXX_status_t (*pf_wakeup)         (void * const);
    MPUXXXX_status_t (*pf_sleep)          (void * const);
    MPUXXXX_status_t (*pf_read_id)        (void * const);
    MPUXXXX_status_t (*pf_check_data_ready) (void * const, bool *ready);

    /* configurations */
    MPUXXXX_status_t (*pf_set_gyro_fsr)   (void * const, uint8_t);
    MPUXXXX_status_t (*pf_set_accel_fsr)  (void * const, uint8_t);
    MPUXXXX_status_t (*pf_set_lpf)        (void * const);
    MPUXXXX_status_t (*pf_set_rate)       (void * const, uint16_t);

    /* read data */
    MPUXXXX_status_t (*pf_read_accel)(\
                            void * const, 
                            mpu6050_data_t *);
    MPUXXXX_status_t (*pf_read_gyro)(\
                            void * const, 
                            mpu6050_data_t *);
    MPUXXXX_status_t (*pf_read_temp)(\
                            void * const, 
                            mpu6050_data_t *);
    MPUXXXX_status_t (*pf_read_all)(\
                            void * const,
                            mpu6050_data_t *);

#ifdef OS_SUPPORTING
    void *semaphore_mutex_handle;
    void *semaphore_binary_handle;
    void *queue_handle;
#endif

#ifdef HARDWARE_IIC
    /* DMA transfer state and buffer (Hardware I2C + DMA mode only)
     * Each instance has its own DMA buffer and state for true multi-instance support
     */
    uint8_t dma_buffer[14] __attribute__((aligned(4)));  // 14 bytes: Accel(6) + Temp(2) + Gyro(6)
    volatile bool dma_busy;                               // DMA transfer in progress flag
    
    /* DMA complete callback for hardware I2C + DMA mode
     * Called when DMA completes transferring data to dma_buffer
     */
    void (*pf_dma_complete_callback)(void);
#endif
    
    /* INT interrupt callback
     * Hardware I2C mode: Optional, for statistics/debugging (data already transferred by DMA)
     * Software I2C mode: Required, notifies task to actively read data
     */
    void (*pf_int_interrupt_callback)(void);
} bsp_mpuxxxx_driver;

MPUXXXX_status_t mpuxxxx_inst(
        bsp_mpuxxxx_driver *             const mpuxxxx_driver,
        iic_driver_interface_t *          const iic_interface,
        void *                           const p_bus_instance,
        timebase_interface_t *     const p_timebase_interface,
#ifdef OS_SUPPORTING
        os_interface_t *           const p_os_interface,
#endif
        interuption_interface_t * const interuption_interface
);
//***************************** class definition ***************************//

//***************************** interrupt callbacks ************************//
/**
 * @brief   Hardware INT pin interrupt callback (Data Ready)
 * @param   p_instance Pointer to MPU driver instance that triggered the interrupt
 * @note    Called when MPU6050 INT pin triggers hardware interrupt.
 *          
 *          HARDWARE_IIC mode (DMA):
 *            - Starts DMA transfer to dma_buffer
 *            - Optionally calls pf_int_interrupt_callback (for statistics)
 *            - Data will be ready after mpu_dma_interrupt_callback
 *          
 *          Software I2C mode (Polling):
 *            - Calls pf_int_interrupt_callback to notify task
 *            - Task should use pf_read_all() to fetch data
 *          
 *          User must call this from hardware interrupt handler and pass
 *          the correct instance pointer.
 */
void mpu_int_interrupt_callback(bsp_mpuxxxx_driver *p_instance);

#ifdef HARDWARE_IIC
/**
 * @brief   DMA transfer complete interrupt callback (Hardware I2C mode only)
 * @param   p_instance Pointer to MPU driver instance that completed DMA
 * @note    Called when DMA completes transferring sensor data to memory.
 *          Calls registered pf_dma_complete_callback to notify data ready.
 *          
 *          User must call this from DMA complete interrupt handler and pass
 *          the correct instance pointer.
 *          
 *          Only available when HARDWARE_IIC is defined.
 */
void mpu_dma_interrupt_callback(bsp_mpuxxxx_driver *p_instance);
#endif

/**
 * @brief   Register callback for INT interrupt
 * @param   p_instance Pointer to MPU driver instance
 * @param   callback Function pointer to ISR-safe callback
 * @return  MPUXXXX_status_t operation status
 * @note    HARDWARE_IIC mode: Optional, for statistics/debugging
 *          Software I2C mode: Required, notifies task to read data
 *          
 *          Alternatively, can directly assign to p_instance->pf_int_interrupt_callback.
 */
MPUXXXX_status_t mpu_register_int_callback(
    bsp_mpuxxxx_driver *p_instance,
    void (*callback)(void));

#ifdef HARDWARE_IIC
/**
 * @brief   Register callback for DMA complete interrupt (Hardware I2C mode only)
 * @param   p_instance Pointer to MPU driver instance
 * @param   callback Function pointer to ISR-safe callback
 * @return  MPUXXXX_status_t operation status
 * @note    Required for HARDWARE_IIC mode.
 *          Callback should perform OS-specific operations (TaskNotify, Queue, etc).
 *          
 *          Alternatively, can directly assign to p_instance->pf_dma_complete_callback.
 *          
 *          Only available when HARDWARE_IIC is defined.
 */
MPUXXXX_status_t mpu_register_dma_complete_callback(
    bsp_mpuxxxx_driver *p_instance,
    void (*callback)(void));

/**
 * @brief   Get pointer to DMA buffer of specific instance (Hardware I2C mode only)
 * @param   p_instance Pointer to MPU driver instance
 * @return  Pointer to 14-byte DMA buffer containing sensor data, or NULL if invalid
 * @note    Data processing task uses this to access raw sensor data.
 *          Each instance has its own independent DMA buffer.
 *          
 *          Only available when HARDWARE_IIC is defined.
 */
uint8_t* mpu_get_dma_buffer(bsp_mpuxxxx_driver *p_instance);

/**
 * @brief   Get DMA buffer length (Hardware I2C mode only)
 * @return  Length of DMA buffer (14 bytes)
 * @note    Only available when HARDWARE_IIC is defined.
 */
uint8_t mpu_get_dma_buffer_length(void);

/**
 * @brief   Check if DMA is currently busy for specific instance (Hardware I2C mode only)
 * @param   p_instance Pointer to MPU driver instance
 * @return  true if DMA transfer in progress, false otherwise
 * @note    Only available when HARDWARE_IIC is defined.
 */
bool mpu_is_dma_busy(bsp_mpuxxxx_driver *p_instance);
#endif
//***************************** interrupt callbacks ************************//

#endif /* __EC_BSP_MPUXXXX_H__ */
