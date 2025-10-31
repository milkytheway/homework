//******************************** Includes *********************************//
#include "bsp_mpuxxxx_driver.h"
#include "bsp_mpuxxxx_reg.h"
#include "iic_hal.h"
#include "main.h"

#ifdef OS_SUPPORTING
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#endif

#ifdef MPU_DEBUG
#include "elog.h"
#endif
//******************************** Includes *********************************//

//***************************** Driver variables ****************************//
extern void dwt_delay_init(void);
extern void delay_ms(uint32_t ms);
extern void delay_us(uint32_t us);

iic_bus_t MPU6050_bus = 
{
        .IIC_SDA_PORT = GPIOB,
        .IIC_SCL_PORT = GPIOB,
        .IIC_SDA_PIN  = GPIO_PIN_13,
        .IIC_SCL_PIN  = GPIO_PIN_14,
};
//***************************** Driver variables ****************************//

//********************************** Driver *********************************//
/* wrapper functions for iic_driver_interface_t */
static int IICInit_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICInit(bus);
    return (int)MPU_OK;
}

static int IICStart_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICStart(bus);
    return (int)MPU_OK;
}

static int IICStop_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICStop(bus);
    return (int)MPU_OK;
}

static int IICSendByte_wrapper(void *p_bus, uint8_t data)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICSendByte(bus, data);
    return (int)MPU_OK;
}

static int IICReceiveByte_wrapper(void *p_bus, uint8_t *p_data)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    unsigned char received = IICReceiveByte(bus);
    *p_data = received;
    return (int)MPU_OK;
}

static int IICWaitAck_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    unsigned char result = IICWaitAck(bus);
    // IICWaitAck returns 0 on success (ACK received), 1 on failure (NACK)
    return (result == 0) ? (int)MPU_OK : (int)MPU_ERROR;
}

static int IICSendAck_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICSendAck(bus);
    return (int)MPU_OK;
}

static int IICSendNotAck_wrapper(void *p_bus)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IICSendNotAck(bus);
    return (int)MPU_OK;
}

static int IICWriteReg_wrapper(void *p_bus, uint8_t daddr, uint8_t reg, uint8_t data)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IIC_Write_One_Byte(bus, daddr, reg, data);
    return (int)MPU_OK;
}

static int IICReadReg_wrapper(void *p_bus, uint8_t daddr, uint8_t reg)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IIC_Read_One_Byte(bus, daddr, reg);
    return (int)MPU_OK;
}

static int IICReadMultiByte_wrapper(void *p_bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t *p_data)
{
    iic_bus_t *bus = (iic_bus_t *)p_bus;
    IIC_Read_Multi_Byte(bus, daddr, reg, length, p_data);
    return (int)MPU_OK;
}
//********************************** Driver *********************************//

//***************************** Driver variables ****************************//
iic_driver_interface_t iic_interface = 
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
    .pf_iic_write_reg    = IICWriteReg_wrapper,
    .pf_iic_read_reg     = IICReadReg_wrapper,
    .pf_iic_read_multi_byte = IICReadMultiByte_wrapper,
    .pf_delay_ms         = delay_ms,
#ifdef OS_SUPPORTING
    .pf_critical_enter   = vPortEnterCritical,
    .pf_critical_exit    = vPortExitCritical
#endif
};

static timebase_interface_t timebase_interface = 
{
    .pf_get_tick_count = HAL_GetTick
};
//***************************** Driver variables ****************************//

//***************************** Driver instance *****************************//
bsp_mpuxxxx_driver mpu6050_driver;  // 只保留对象声明
//***************************** Driver instance *****************************//


//***************************** test functions *****************************//
void test_mpu6050_init(void)
{
    
    MPUXXXX_status_t status = mpuxxxx_inst(&mpu6050_driver, 
                            &iic_interface, 
                            &MPU6050_bus, 
                            &timebase_interface, 
#ifdef OS_SUPPORTING
                            NULL,  // No OS interface 
#endif
                            NULL);  // No interrupt interface
    if (status != MPU_OK) {
        log_e("MPU6050 driver instance creation failed: %d", status);
        return;
    }
    else {
        log_i("MPU6050 driver instance created successfully");
    }
    
    // 然后初始化设备
    status = mpu6050_driver.pf_init(&mpu6050_driver);
    if (status != MPU_OK) {
        log_e("MPU6050 initialization failed: %d", status);
    }
    else {
        log_i("MPU6050 initialization successful");
    }
}

void test_mpu6050_read_id(void)
{
    MPUXXXX_status_t status = mpu6050_driver.pf_read_id(&mpu6050_driver);
    if (status != MPU_OK) {
        log_e("MPU6050 read ID failed: %d", status);
    }
    else {
        log_i("MPU6050 read ID successful");
    }
}

void test_mpu6050_read_all(void)
{
    mpu6050_data_t data;
    MPUXXXX_status_t status = mpu6050_driver.pf_read_all(&mpu6050_driver, &data);
    if (status != MPU_OK) {
        log_e("MPU6050 read all failed: %d", status);
    }
}

//***************************** test functions *****************************//
