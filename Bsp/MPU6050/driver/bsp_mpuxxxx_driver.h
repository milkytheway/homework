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
//******************************** Includes *********************************//

//********************** private macro definitions **************************//
#define OS_SUPPORTING
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
typedef struct
{
    MPUXXXX_status_t (*pf_iic_init)                     (void *);
    MPUXXXX_status_t (*pf_iic_deinit)                   (void *);
    MPUXXXX_status_t (*pf_iic_start)                    (void *);
    MPUXXXX_status_t (*pf_iic_stop)                     (void *);
    MPUXXXX_status_t (*pf_iic_send_byte)                (void *, uint8_t);
    MPUXXXX_status_t (*pf_iic_receive_byte)             (void *, uint8_t *);
    MPUXXXX_status_t (*pf_iic_wait_ack)       (void *);
    MPUXXXX_status_t (*pf_iic_send_ack)       (void *);
    MPUXXXX_status_t (*pf_iic_send_nack)      (void *);
    MPUXXXX_status_t (*pf_iic_write_reg)      (void *, uint8_t daddr,uint8_t reg,uint8_t data);
    MPUXXXX_status_t (*pf_iic_read_reg)       (void *, uint8_t daddr,uint8_t reg);

    void (*pf_delay_ms)           (uint32_t);

#ifdef OS_SUPPORTING
    void (*pf_critical_enter)               (void);
    void (*pf_critical_exit)                (void);
#endif /* OS_SUPPORTING */
} iic_driver_interface_t;

typedef struct
{
    uint32_t (*pf_get_tick_count) (void);
} timebase_interface_t;

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

    /* basic functions */
    MPUXXXX_status_t (*pf_init)           (void * const);
    MPUXXXX_status_t (*pf_deinit)         (void * const);
    MPUXXXX_status_t (*pf_start)          (void * const);
    MPUXXXX_status_t (*pf_stop)           (void * const);
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
    void (*pf_dma_complete_callback) (void);
    void (*pf_int_interrupt_callback) (void);
    void *queue_handle;
#endif
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

 #endif /* __EC_BSP_MPUXXXX_H__ */