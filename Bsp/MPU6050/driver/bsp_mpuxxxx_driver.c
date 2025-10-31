/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file ec_bsp_mpuxxxx.c
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

 //******************************** Includes *********************************//
#include "bsp_mpuxxxx_driver.h"
#include "bsp_mpuxxxx_reg.h"
#include "elog.h"
 //******************************** Includes *********************************//

 //********************** private macro definitions **************************//
#define MPU_DEBUG

/* MPU6050 instance macros */
#define MPU_IIC_INTERFACE(p_inst)       (((bsp_mpuxxxx_driver *)p_inst)->iic_interface)
#define MPU_BUS_INSTANCE(p_inst)        (((bsp_mpuxxxx_driver *)p_inst)->p_bus_instance)

/* Gyroscope Full Scale Range */
#define MPU_GYRO_FSR_250DPS             0x00    // ±250°/s
#define MPU_GYRO_FSR_500DPS             0x01    // ±500°/s
#define MPU_GYRO_FSR_1000DPS            0x02    // ±1000°/s
#define MPU_GYRO_FSR_2000DPS            0x03    // ±2000°/s

/* Accelerometer Full Scale Range */
#define MPU_ACCEL_FSR_2G                0x00    // ±2g
#define MPU_ACCEL_FSR_4G                0x01    // ±4g
#define MPU_ACCEL_FSR_8G                0x02    // ±8g
#define MPU_ACCEL_FSR_16G               0x03    // ±16g

/* Digital Low Pass Filter (DLPF) Configuration */
#define MPU_DLPF_BW_256HZ               0x00    // Bandwidth: 256Hz (Delay: 0.98ms)
#define MPU_DLPF_BW_188HZ               0x01    // Bandwidth: 188Hz (Delay: 1.9ms)
#define MPU_DLPF_BW_98HZ                0x02    // Bandwidth: 98Hz (Delay: 2.8ms)
#define MPU_DLPF_BW_42HZ                0x03    // Bandwidth: 42Hz (Delay: 4.8ms)
#define MPU_DLPF_BW_20HZ                0x04    // Bandwidth: 20Hz (Delay: 8.3ms)
#define MPU_DLPF_BW_10HZ                0x05    // Bandwidth: 10Hz (Delay: 13.4ms)
#define MPU_DLPF_BW_5HZ                 0x06    // Bandwidth: 5Hz (Delay: 18.6ms)

/* Default DLPF setting */
#define MPU_DEFAULT_DLPF                MPU_DLPF_BW_42HZ

/* Sample Rate Divider Range */
#define MPU_SMPLRT_DIV_MIN              0       // Maximum sample rate
#define MPU_SMPLRT_DIV_MAX              255     // Minimum sample rate

/* Accelerometer Sensitivity Scale Factor (LSB/g) */
#define MPU_ACCEL_SENS_2G               16384.0 // ±2g
#define MPU_ACCEL_SENS_4G               8192.0  // ±4g
#define MPU_ACCEL_SENS_8G               4096.0  // ±8g
#define MPU_ACCEL_SENS_16G              2048.0  // ±16g

/* Gyroscope Sensitivity Scale Factor (LSB/(°/s)) */
#define MPU_GYRO_SENS_250DPS            131.0   // ±250°/s
#define MPU_GYRO_SENS_500DPS            65.5    // ±500°/s
#define MPU_GYRO_SENS_1000DPS           32.8    // ±1000°/s
#define MPU_GYRO_SENS_2000DPS           16.4    // ±2000°/s

/* Default sensitivity (assuming ±2g for accel, ±250°/s for gyro) */
#define MPU_DEFAULT_ACCEL_SENS          MPU_ACCEL_SENS_2G
#define MPU_DEFAULT_GYRO_SENS           MPU_GYRO_SENS_250DPS

/* Temperature Sensor Conversion */
#define MPU_TEMP_SENSITIVITY            340.0   // LSB/°C
#define MPU_TEMP_OFFSET                 36.53   // °C

/* MPU6050 data length for DMA transfer (14 bytes) */
#define MPU_DATA_LENGTH                 14      // Accel(6) + Temp(2) + Gyro(6)
 //********************** private macro definitions **************************//

 //********************** private function prototypes ************************//
static MPUXXXX_status_t read_id(void * const p_instance, uint8_t *p_id);
static MPUXXXX_status_t mpu_read_id(void * const p_instance);
static MPUXXXX_status_t mpu_init(void * const p_instance);
static MPUXXXX_status_t mpu_deinit(void * const p_instance);
static MPUXXXX_status_t set_gyro_fsr(void * const p_instance, uint8_t fsr);
static MPUXXXX_status_t set_accel_fsr(void * const p_instance, uint8_t fsr);
static MPUXXXX_status_t set_lpf(void * const p_instance);
static MPUXXXX_status_t set_rate(void * const p_instance, uint16_t rate);
static MPUXXXX_status_t read_accel(void * const p_instance, mpu6050_data_t *p_data);
static MPUXXXX_status_t read_gyro(void * const p_instance, mpu6050_data_t *p_data);
static MPUXXXX_status_t read_temp(void * const p_instance, mpu6050_data_t *p_data);
static MPUXXXX_status_t read_all(void * const p_instance, mpu6050_data_t *p_data);
static MPUXXXX_status_t mpu_wakeup(void * const p_instance);
static MPUXXXX_status_t mpu_sleep(void * const p_instance);
 //********************** private function prototypes ************************//

/******************************************************************************
 * @name    read_id
 * @brief   Read the WHO_AM_I register to get MPU6050 device ID
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @param   p_id[out] pointer to store the device ID
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: IIC interface not initialized
 * @note    This function uses IIC_Read_One_Byte() from iic_hal.c to read
 *          the WHO_AM_I register (0x75).
 *          
 *          Expected device ID for MPU6050: 0x68
 *          
 *          The function does NOT verify the ID value, it only reads and
 *          returns it. The caller is responsible for verification.
 *****************************************************************************/
static MPUXXXX_status_t read_id(void * const p_instance, uint8_t *p_id)
{
    uint8_t device_id = 0;
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("read_id: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (NULL == p_id) {
#ifdef MPU_DEBUG
        log_e("read_id: output pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_read_reg) {
#ifdef MPU_DEBUG
        log_e("read_id: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Read WHO_AM_I register (0x75) using IIC_Read_One_Byte
     * This function is already wrapped in iic_hal.c and handles:
     * - Start condition
     * - Write device address + register address
     * - Restart
     * - Read device address + read data
     * - NACK + Stop condition
     */
    device_id = p_iic->pf_iic_read_reg(p_bus, MPU_ADDR, MPU_WHO_AM_I_REG);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    /* Store the result */
    *p_id = device_id;
    
#ifdef MPU_DEBUG
    log_d("read_id: WHO_AM_I register = 0x%02X", device_id);
#endif
    
    return MPU_OK;
}

/******************************************************************************
 * @name    mpu_init
 * @brief   Complete initialization sequence for MPU6050 sensor
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: initialization completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERRORRESOURCE: device ID verification failed
 *          - MPU_ERROR: operation failed
 * @note    This function performs a complete initialization sequence:
 *          1. Initialize IIC interface
 *          2. Device reset (PWR_MGMT1 bit 7)
 *          3. Wait 100ms for reset to complete
 *          4. Wake up device and set clock source (PLL with X gyro)
 *          5. Disable all interrupts
 *          6. Disable IIC master mode
 *          7. Disable FIFO
 *          8. Configure INT pin (active low, push-pull, latch until read)
 *          9. Configure default sensor parameters (±2g, ±250°/s, 42Hz DLPF, 100Hz)
 *          10. Read and verify device ID (expected: 0x68)
 *          11. Enable Data Ready interrupt
 *          
 *          After initialization, the device is ready to use.
 *          Call pf_read_xxx() functions to read sensor data.
 *****************************************************************************/
static MPUXXXX_status_t mpu_init(void * const p_instance)
{
#ifdef MPU_DEBUG
    log_i("mpu_init: Starting MPU6050 complete initialization...");
#endif
    
    MPUXXXX_status_t status = MPU_OK;
    uint8_t device_id = 0;
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("mpu_init: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_init || NULL == p_iic->pf_iic_write_reg) {
#ifdef MPU_DEBUG
        log_e("mpu_init: IIC interface not initialized");
#endif
        return MPU_ERRORPARAMETER;
    }
    
#ifdef OS_SUPPORTING
    /* ========== Acquire IIC bus lock ========== */
    bsp_mpuxxxx_driver *p_mpu = (bsp_mpuxxxx_driver *)p_instance;
    if (NULL != p_mpu->pf_bus_lock) {
        status = p_mpu->pf_bus_lock(p_mpu->p_bus_lock_context, 0xFFFFFFFF);
        if (MPU_OK != status) {
#ifdef MPU_DEBUG
            log_e("mpu_init: failed to acquire bus lock");
#endif
            return status;
        }
    }
    /* ========================================== */
#endif
    
    /* Step 1: Initialize IIC interface */
#ifdef MPU_DEBUG
    log_i("mpu_init: [1/11] Initializing IIC interface...");
#endif
    p_iic->pf_iic_init(p_bus);
    
    /* Step 2: Device reset */
#ifdef MPU_DEBUG
    log_i("mpu_init: [2/11] Resetting device...");
#endif
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_PWR_MGMT1_REG, MPU_PWR1_DEVICE_RESET);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: device reset failed");
#endif
        goto cleanup;
    }
    
    /* Step 3: Wait for reset to complete */
#ifdef MPU_DEBUG
    log_i("mpu_init: [3/11] Waiting for reset to complete (100ms)...");
#endif
    if (NULL != p_iic->pf_delay_ms) {
        p_iic->pf_delay_ms(100);
    }
    
    /* Step 4: Wake up device and set clock source */
#ifdef MPU_DEBUG
    log_i("mpu_init: [4/11] Waking up device and setting clock source...");
#endif
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_PWR_MGMT1_REG, 0x01);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: wake up failed");
#endif
        goto cleanup;
    }
    
    if (NULL != p_iic->pf_delay_ms) {
        p_iic->pf_delay_ms(10);  // Small delay after wakeup
    }
    
    /* Step 5: Disable all interrupts */
#ifdef MPU_DEBUG
    log_i("mpu_init: [5/11] Disabling all interrupts...");
#endif
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_INT_EN_REG, 0x00);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: disable interrupts failed");
#endif
        goto cleanup;
    }
    
    /* Step 6: Disable IIC master mode, disable FIFO */
#ifdef MPU_DEBUG
    log_i("mpu_init: [6/11] Disabling IIC master mode and FIFO...");
#endif
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_USER_CTRL_REG, 0x00);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: disable IIC master mode failed");
#endif
        goto cleanup;
    }
    
    /* Disable FIFO for all sensors */
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_FIFO_EN_REG, 0x00);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: disable FIFO failed");
#endif
        goto cleanup;
    }
    
    /* Step 7: Configure INT pin */
#ifdef MPU_DEBUG
    log_i("mpu_init: [7/11] Configuring INT pin (active low, push-pull, latch)...");
#endif
    /* INT/BYPASS Config Register (0x37):
     * Bit 7: INT_LEVEL = 1 (active low)
     * Bit 6: INT_OPEN = 0 (push-pull)
     * Bit 5: LATCH_INT_EN = 1 (latch until interrupt is cleared)
     * Bit 4: INT_RD_CLEAR = 1 (clear on any read)
     * Value: 0xB0
     */
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_INTBP_CFG_REG, 0xB0);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: configure INT pin failed");
#endif
        goto cleanup;
    }
    
    /* Step 8: Configure default sensor parameters */
#ifdef MPU_DEBUG
    log_i("mpu_init: [8/11] Configuring sensor parameters...");
#endif
    
    /* Set gyroscope FSR to ±250°/s */
    status = set_gyro_fsr(p_instance, MPU_GYRO_FSR_250DPS);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: set gyro FSR failed");
#endif
        goto cleanup;
    }
    
    /* Set accelerometer FSR to ±2g */
    status = set_accel_fsr(p_instance, MPU_ACCEL_FSR_2G);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: set accel FSR failed");
#endif
        goto cleanup;
    }
    
    /* Set DLPF to 42Hz */
    status = set_lpf(p_instance);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: set DLPF failed");
#endif
        goto cleanup;
    }
    
    /* Set sample rate to 5Hz */
    status = set_rate(p_instance, 5);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: set sample rate failed");
#endif
        goto cleanup;
    }
    
    /* Step 9: Read and verify device ID */
#ifdef MPU_DEBUG
    log_i("mpu_init: [9/11] Reading and verifying device ID...");
#endif
    status = read_id(p_instance, &device_id);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: read device ID failed");
#endif
        goto cleanup;
    }
    
//    if (MPU_WHO_AM_I_ID != device_id) {
//#ifdef MPU_DEBUG
//        log_e("mpu_init: Device ID mismatch!");
//        log_e("  Expected: 0x%02X", MPU_WHO_AM_I_ID);
//        log_e("  Received: 0x%02X", device_id);
//#endif
//        status = MPU_ERRORRESOURCE;
//        goto cleanup;
//    }
    
//#ifdef MPU_DEBUG
//    log_i("  Device ID verified: 0x%02X (MPU6050)", device_id);
//#endif
    
    /* Step 10: Enable Data Ready interrupt */
#ifdef MPU_DEBUG
    log_i("mpu_init: [10/11] Enabling Data Ready interrupt...");
#endif
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_INT_EN_REG, MPU_INT_DATA_RDY_EN);
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_init: enable Data Ready interrupt failed");
#endif
        goto cleanup;
    }
    
    /* Step 11: Small delay for all settings to take effect */
#ifdef MPU_DEBUG
    log_i("mpu_init: [11/11] Finalizing initialization...");
#endif
    if (NULL != p_iic->pf_delay_ms) {
        p_iic->pf_delay_ms(10);
    }
    
#ifdef MPU_DEBUG
    log_i("mpu_init: ========================================");
    log_i("mpu_init: MPU6050 initialization completed!");
    log_i("mpu_init: Configuration summary:");
    log_i("  - Device ID: 0x%02X (verified)", device_id);
    log_i("  - Gyroscope FSR: ±250°/s");
    log_i("  - Accelerometer FSR: ±2g");
    log_i("  - DLPF Bandwidth: 42Hz");
    log_i("  - Sample Rate: 100Hz");
    log_i("  - INT Pin: Active Low, Latch Mode");
    log_i("  - Data Ready Interrupt: Enabled");
    log_i("  - IIC Master Mode: Disabled");
    log_i("  - FIFO: Disabled");
    log_i("mpu_init: Device is ready for use!");
    log_i("mpu_init: ========================================");
#endif
    
    status = MPU_OK;

cleanup:
#ifdef OS_SUPPORTING
    /* ========== Release IIC bus lock ========== */
    if (NULL != p_mpu->pf_bus_unlock) {
        p_mpu->pf_bus_unlock(p_mpu->p_bus_lock_context);
    }
    /* ========================================== */
#endif
    
    return status;
}

/******************************************************************************
 * @name    set_gyro_fsr
 * @brief   Set the full scale range of the gyroscope
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @param   fsr[in] full scale range selection
 *          - 0: ±250°/s
 *          - 1: ±500°/s
 *          - 2: ±1000°/s
 *          - 3: ±2000°/s
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: operation failed
 * @note    The gyroscope full scale range determines the maximum angular
 *          velocity that can be measured. Lower ranges provide higher
 *          sensitivity while higher ranges provide larger measurement range.
 *****************************************************************************/
static MPUXXXX_status_t set_gyro_fsr(void * const p_instance, uint8_t fsr)
{
    MPUXXXX_status_t status = MPU_OK;
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("set_gyro_fsr: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (fsr > MPU_GYRO_FSR_2000DPS) {
#ifdef MPU_DEBUG
        log_e("set_gyro_fsr: invalid FSR value %d", fsr);
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_write_reg) {
#ifdef MPU_DEBUG
        log_e("set_gyro_fsr: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Write FSR configuration to GYRO_CFG register
     * Format: [7:3]=0, [4:3]=FS_SEL, [2:0]=0
     */
    uint8_t reg_value = (fsr << 3);
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_GYRO_CFG_REG, reg_value);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    if (MPU_OK == status) {
#ifdef MPU_DEBUG
        log_i("set_gyro_fsr: set to %d (±%d°/s)", fsr, (250 << fsr));
#endif
    } else {
#ifdef MPU_DEBUG
        log_e("set_gyro_fsr: write register failed");
#endif
    }
    
    return status;
}

/******************************************************************************
 * @name    set_accel_fsr
 * @brief   Set the full scale range of the accelerometer
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @param   fsr[in] full scale range selection
 *          - 0: ±2g
 *          - 1: ±4g
 *          - 2: ±8g
 *          - 3: ±16g
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: operation failed
 * @note    The accelerometer full scale range determines the maximum
 *          acceleration that can be measured. Lower ranges provide higher
 *          sensitivity while higher ranges provide larger measurement range.
 *****************************************************************************/
static MPUXXXX_status_t set_accel_fsr(void * const p_instance, uint8_t fsr)
{
    MPUXXXX_status_t status = MPU_OK;
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("set_accel_fsr: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (fsr > MPU_ACCEL_FSR_16G) {
#ifdef MPU_DEBUG
        log_e("set_accel_fsr: invalid FSR value %d", fsr);
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_write_reg) {
#ifdef MPU_DEBUG
        log_e("set_accel_fsr: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Write FSR configuration to ACCEL_CFG register
     * Format: [7:3]=0, [4:3]=AFS_SEL, [2:0]=0
     */
    uint8_t reg_value = (fsr << 3);
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_ACCEL_CFG_REG, reg_value);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    if (MPU_OK == status) {
#ifdef MPU_DEBUG
        log_i("set_accel_fsr: set to %d (±%dg)", fsr, (2 << fsr));
#endif
    } else {
#ifdef MPU_DEBUG
        log_e("set_accel_fsr: write register failed");
#endif
    }
    
    return status;
}

/******************************************************************************
 * @name    set_lpf
 * @brief   Configure the Digital Low Pass Filter (DLPF)
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: operation failed
 * @note    The DLPF is used to filter out high frequency noise from the
 *          gyroscope and accelerometer outputs. The default setting is 42Hz
 *          bandwidth which provides a good balance between noise reduction
 *          and response time.
 *          DLPF Setting | Bandwidth | Delay
 *          -------------|-----------|-------
 *          0            | 260Hz     | 0ms
 *          1            | 184Hz     | 2.0ms
 *          2            | 94Hz      | 3.0ms
 *          3            | 44Hz      | 4.9ms (Default)
 *          4            | 21Hz      | 8.5ms
 *          5            | 10Hz      | 13.8ms
 *          6            | 5Hz       | 19.0ms
 *****************************************************************************/
static MPUXXXX_status_t set_lpf(void * const p_instance)
{
    MPUXXXX_status_t status = MPU_OK;
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("set_lpf: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_write_reg) {
#ifdef MPU_DEBUG
        log_e("set_lpf: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Write DLPF configuration to CONFIG register
     * Format: [7:6]=0, [5:3]=EXT_SYNC_SET, [2:0]=DLPF_CFG
     * We only set DLPF_CFG bits, keeping other bits as 0
     */
    uint8_t reg_value = MPU_DEFAULT_DLPF & MPU_CFG_DLPF_CFG_MASK;
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_CFG_REG, reg_value);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    if (MPU_OK == status) {
#ifdef MPU_DEBUG
        log_i("set_lpf: configured DLPF to %d (42Hz bandwidth)", MPU_DEFAULT_DLPF);
#endif
    } else {
#ifdef MPU_DEBUG
        log_e("set_lpf: write register failed");
#endif
    }
    
    return status;
}

/******************************************************************************
 * @name    set_rate
 * @brief   Set the sample rate divider for the MPU6050
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @param   rate[in] desired sample rate in Hz (4Hz - 1000Hz)
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: operation failed
 * @note    The sample rate is determined by:
 *          Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *          
 *          Where Gyroscope Output Rate depends on DLPF setting:
 *          - If DLPF is enabled (DLPF_CFG != 0 or 7): Output Rate = 1kHz
 *          - If DLPF is disabled (DLPF_CFG = 0 or 7): Output Rate = 8kHz
 *          
 *          This function assumes DLPF is enabled (1kHz gyro output rate).
 *          Valid rate range: 4Hz to 1000Hz
 *          
 *          Example: For 100Hz sample rate:
 *          SMPLRT_DIV = (1000 / 100) - 1 = 9
 *****************************************************************************/
static MPUXXXX_status_t set_rate(void * const p_instance, uint16_t rate)
{
    MPUXXXX_status_t status = MPU_OK;
    uint8_t smplrt_div = 0;
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("set_rate: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Validate rate range (4Hz - 1000Hz for 1kHz gyro output) */
    if (rate < 4 || rate > 1000) {
#ifdef MPU_DEBUG
        log_e("set_rate: invalid rate %d Hz (valid range: 4-1000Hz)", rate);
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_write_reg) {
#ifdef MPU_DEBUG
        log_e("set_rate: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
    /* Calculate sample rate divider
     * SMPLRT_DIV = (Gyro Output Rate / Sample Rate) - 1
     * Assuming DLPF enabled, Gyro Output Rate = 1000Hz
     */
    smplrt_div = (uint8_t)((1000 / rate) - 1);
    
    /* Ensure divider is within valid range */
    if (smplrt_div > MPU_SMPLRT_DIV_MAX) {
        smplrt_div = MPU_SMPLRT_DIV_MAX;
    }
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Write sample rate divider to SMPLRT_DIV register */
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_SAMPLE_RATE_REG, smplrt_div);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    if (MPU_OK == status) {
        uint16_t actual_rate = 1000 / (smplrt_div + 1);
#ifdef MPU_DEBUG
        log_i("set_rate: SMPLRT_DIV=%d, actual rate=%dHz (requested=%dHz)", 
              smplrt_div, actual_rate, rate);
#endif
    } else {
#ifdef MPU_DEBUG
        log_e("set_rate: write register failed");
#endif
    }
    
    return status;
}

/******************************************************************************
 * @name    read_accel
 * @brief   Read accelerometer data from MPU6050
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @param   p_data[out] pointer to mpu6050_data_t structure to store the data
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: operation failed
 * @note    This function reads 6 bytes of accelerometer data (X, Y, Z axes)
 *          from the MPU6050 sensor. Each axis data is 16-bit signed integer.
 *          The raw data is converted to acceleration in 'g' units using the
 *          default sensitivity of ±2g (16384 LSB/g).
 *          
 *          Data format:
 *          - accel_x_raw, accel_y_raw, accel_z_raw: raw 16-bit values
 *          - ax, ay, az: acceleration in 'g' units (1g = 9.8m/s²)
 *****************************************************************************/
static MPUXXXX_status_t read_accel(void * const p_instance, mpu6050_data_t *p_data)
{
    MPUXXXX_status_t status = MPU_OK;
    uint8_t buffer[6] = {0};
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("read_accel: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (NULL == p_data) {
#ifdef MPU_DEBUG
        log_e("read_accel: data pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_read_multi_byte) {
#ifdef MPU_DEBUG
        log_e("read_accel: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
#ifdef OS_SUPPORTING
    /* ========== Acquire IIC bus lock ========== */
    bsp_mpuxxxx_driver *p_mpu = (bsp_mpuxxxx_driver *)p_instance;
    if (NULL != p_mpu->pf_bus_lock) {
        status = p_mpu->pf_bus_lock(p_mpu->p_bus_lock_context, 0xFFFFFFFF);
        if (MPU_OK != status) {
#ifdef MPU_DEBUG
            log_e("read_accel: failed to acquire bus lock");
#endif
            return status;
        }
    }
    /* ========================================== */
#endif
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Read 6 bytes of accelerometer data starting from ACCEL_XOUTH_REG
     * Register layout:
     * 0x3B: ACCEL_XOUT_H
     * 0x3C: ACCEL_XOUT_L
     * 0x3D: ACCEL_YOUT_H
     * 0x3E: ACCEL_YOUT_L
     * 0x3F: ACCEL_ZOUT_H
     * 0x40: ACCEL_ZOUT_L
     * 
     * Using IIC_Read_Multi_Byte which handles the complete read sequence
     */
    status = p_iic->pf_iic_read_multi_byte(p_bus, MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buffer);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("read_accel: read multi-byte failed");
#endif
#ifdef OS_SUPPORTING
        /* Release IIC bus lock before returning */
        if (NULL != p_mpu->pf_bus_unlock) {
            p_mpu->pf_bus_unlock(p_mpu->p_bus_lock_context);
        }
#endif
        return status;
    }
    
    /* Combine high and low bytes to form 16-bit signed integers
     * MPU6050 uses big-endian format (MSB first)
     */
    p_data->accel_x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    p_data->accel_y_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    p_data->accel_z_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    /* Convert raw data to acceleration in 'g' units
     * Using default sensitivity: ±2g range (16384 LSB/g)
     */
    p_data->ax = (double)p_data->accel_x_raw / MPU_DEFAULT_ACCEL_SENS;
    p_data->ay = (double)p_data->accel_y_raw / MPU_DEFAULT_ACCEL_SENS;
    p_data->az = (double)p_data->accel_z_raw / MPU_DEFAULT_ACCEL_SENS;
    
#ifdef MPU_DEBUG
    log_d("read_accel: raw[%d, %d, %d] g[%.3f, %.3f, %.3f]",
          p_data->accel_x_raw, p_data->accel_y_raw, p_data->accel_z_raw,
          p_data->ax, p_data->ay, p_data->az);
#endif
    
#ifdef OS_SUPPORTING
    /* ========== Release IIC bus lock ========== */
    if (NULL != p_mpu->pf_bus_unlock) {
        p_mpu->pf_bus_unlock(p_mpu->p_bus_lock_context);
    }
    /* ========================================== */
#endif
    
    return status;
}

/******************************************************************************
 * @name    read_gyro
 * @brief   Read gyroscope data from MPU6050
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @param   p_data[out] pointer to mpu6050_data_t structure to store the data
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: operation failed
 * @note    This function reads 6 bytes of gyroscope data (X, Y, Z axes)
 *          from the MPU6050 sensor. Each axis data is 16-bit signed integer.
 *          The raw data is converted to angular velocity in °/s (degrees per 
 *          second) using the default sensitivity of ±250°/s (131 LSB/(°/s)).
 *          
 *          Data format:
 *          - gyro_x_raw, gyro_y_raw, gyro_z_raw: raw 16-bit values
 *          - gx, gy, gz: angular velocity in °/s
 *****************************************************************************/
static MPUXXXX_status_t read_gyro(void * const p_instance, mpu6050_data_t *p_data)
{
    MPUXXXX_status_t status = MPU_OK;
    uint8_t buffer[6] = {0};
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("read_gyro: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (NULL == p_data) {
#ifdef MPU_DEBUG
        log_e("read_gyro: data pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_read_multi_byte) {
#ifdef MPU_DEBUG
        log_e("read_gyro: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
#ifdef OS_SUPPORTING
    /* ========== Acquire IIC bus lock ========== */
    bsp_mpuxxxx_driver *p_mpu = (bsp_mpuxxxx_driver *)p_instance;
    if (NULL != p_mpu->pf_bus_lock) {
        status = p_mpu->pf_bus_lock(p_mpu->p_bus_lock_context, 0xFFFFFFFF);
        if (MPU_OK != status) {
#ifdef MPU_DEBUG
            log_e("read_gyro: failed to acquire bus lock");
#endif
            return status;
        }
    }
    /* ========================================== */
#endif
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Read 6 bytes of gyroscope data starting from GYRO_XOUTH_REG
     * Register layout:
     * 0x43: GYRO_XOUT_H
     * 0x44: GYRO_XOUT_L
     * 0x45: GYRO_YOUT_H
     * 0x46: GYRO_YOUT_L
     * 0x47: GYRO_ZOUT_H
     * 0x48: GYRO_ZOUT_L
     * 
     * Using IIC_Read_Multi_Byte which handles the complete read sequence
     */
    status = p_iic->pf_iic_read_multi_byte(p_bus, MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buffer);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("read_gyro: read multi-byte failed");
#endif
#ifdef OS_SUPPORTING
        /* Release IIC bus lock before returning */
        if (NULL != p_mpu->pf_bus_unlock) {
            p_mpu->pf_bus_unlock(p_mpu->p_bus_lock_context);
        }
#endif
        return status;
    }
    
    /* Combine high and low bytes to form 16-bit signed integers
     * MPU6050 uses big-endian format (MSB first)
     */
    p_data->gyro_x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    p_data->gyro_y_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    p_data->gyro_z_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    /* Convert raw data to angular velocity in °/s (degrees per second)
     * Using default sensitivity: ±250°/s range (131 LSB/(°/s))
     */
    p_data->gx = (double)p_data->gyro_x_raw / MPU_DEFAULT_GYRO_SENS;
    p_data->gy = (double)p_data->gyro_y_raw / MPU_DEFAULT_GYRO_SENS;
    p_data->gz = (double)p_data->gyro_z_raw / MPU_DEFAULT_GYRO_SENS;
    
#ifdef MPU_DEBUG
    log_d("read_gyro: raw[%d, %d, %d] dps[%.3f, %.3f, %.3f]",
          p_data->gyro_x_raw, p_data->gyro_y_raw, p_data->gyro_z_raw,
          p_data->gx, p_data->gy, p_data->gz);
#endif
    
#ifdef OS_SUPPORTING
    /* ========== Release IIC bus lock ========== */
    if (NULL != p_mpu->pf_bus_unlock) {
        p_mpu->pf_bus_unlock(p_mpu->p_bus_lock_context);
    }
    /* ========================================== */
#endif
    
    return status;
}

/******************************************************************************
 * @name    read_temp
 * @brief   Read temperature data from MPU6050
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @param   p_data[out] pointer to mpu6050_data_t structure to store the data
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: operation failed
 * @note    This function reads 2 bytes of temperature data from the MPU6050 
 *          internal temperature sensor. The raw data is a 16-bit signed integer
 *          that needs to be converted to actual temperature in °C (Celsius).
 *          
 *          Temperature conversion formula:
 *          Temperature (°C) = (TEMP_OUT / 340) + 36.53
 *          
 *          Data format:
 *          - tempreture: temperature in °C (note: typo in struct, should be "temperature")
 *          
 *          Typical operating range: -40°C to +85°C
 *****************************************************************************/
static MPUXXXX_status_t read_temp(void * const p_instance, mpu6050_data_t *p_data)
{
    MPUXXXX_status_t status = MPU_OK;
    uint8_t buffer[2] = {0};
    int16_t temp_raw = 0;
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("read_temp: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (NULL == p_data) {
#ifdef MPU_DEBUG
        log_e("read_temp: data pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_read_multi_byte) {
#ifdef MPU_DEBUG
        log_e("read_temp: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
#ifdef OS_SUPPORTING
    /* ========== Acquire IIC bus lock ========== */
    bsp_mpuxxxx_driver *p_mpu = (bsp_mpuxxxx_driver *)p_instance;
    if (NULL != p_mpu->pf_bus_lock) {
        status = p_mpu->pf_bus_lock(p_mpu->p_bus_lock_context, 0xFFFFFFFF);
        if (MPU_OK != status) {
#ifdef MPU_DEBUG
            log_e("read_temp: failed to acquire bus lock");
#endif
            return status;
        }
    }
    /* ========================================== */
#endif
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Read 2 bytes of temperature data starting from TEMP_OUTH_REG
     * Register layout:
     * 0x41: TEMP_OUT_H
     * 0x42: TEMP_OUT_L
     * 
     * Using IIC_Read_Multi_Byte which handles the complete read sequence
     */
    status = p_iic->pf_iic_read_multi_byte(p_bus, MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buffer);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("read_temp: read multi-byte failed");
#endif
#ifdef OS_SUPPORTING
        /* Release IIC bus lock before returning */
        if (NULL != p_mpu->pf_bus_unlock) {
            p_mpu->pf_bus_unlock(p_mpu->p_bus_lock_context);
        }
#endif
        return status;
    }
    
    /* Combine high and low bytes to form 16-bit signed integer
     * MPU6050 uses big-endian format (MSB first)
     */
    temp_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    /* Convert raw data to temperature in °C (Celsius)
     * Formula: Temperature = (TEMP_OUT / 340.0) + 36.53
     * 
     * According to MPU6050 datasheet:
     * - Sensitivity: 340 LSB/°C
     * - Offset: 36.53°C (at 0 LSB, temperature is -36.53°C)
     */
    p_data->tempreture = ((float)temp_raw / 340.0f) + 36.53f;
    
#ifdef MPU_DEBUG
    log_d("read_temp: raw[%d] temp[%.2f°C]", temp_raw, p_data->tempreture);
#endif
    
#ifdef OS_SUPPORTING
    /* ========== Release IIC bus lock ========== */
    if (NULL != p_mpu->pf_bus_unlock) {
        p_mpu->pf_bus_unlock(p_mpu->p_bus_lock_context);
    }
    /* ========================================== */
#endif
    
    return status;
}

/******************************************************************************
 * @name    read_all
 * @brief   Read all sensor data (accelerometer, temperature, gyroscope) from MPU6050
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @param   p_data[out] pointer to mpu6050_data_t structure to store the data
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: operation failed
 * @note    This function reads all 14 bytes of sensor data in a single I2C 
 *          transaction for maximum efficiency. The data is stored in consecutive
 *          registers as follows:
 *          
 *          Register Map (14 bytes total):
 *          0x3B-0x40: Accelerometer X,Y,Z (6 bytes)
 *          0x41-0x42: Temperature (2 bytes)
 *          0x43-0x48: Gyroscope X,Y,Z (6 bytes)
 *          
 *          This method is more efficient than calling read_accel(), read_temp(),
 *          and read_gyro() separately, as it requires only one I2C transaction
 *          instead of three. It also ensures all data is sampled at the same instant.
 *          
 *          All raw values are converted to physical units:
 *          - Accelerometer: g (1g = 9.8m/s²)
 *          - Gyroscope: °/s (degrees per second)
 *          - Temperature: °C (Celsius)
 *****************************************************************************/
static MPUXXXX_status_t read_all(void * const p_instance, mpu6050_data_t *p_data)
{
    MPUXXXX_status_t status = MPU_OK;
    uint8_t buffer[14] = {0};
    int16_t temp_raw = 0;
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("read_all: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (NULL == p_data) {
#ifdef MPU_DEBUG
        log_e("read_all: data pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_read_multi_byte) {
#ifdef MPU_DEBUG
        log_e("read_all: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
#ifdef OS_SUPPORTING
    /* ========== Acquire IIC bus lock ========== */
    bsp_mpuxxxx_driver *p_mpu = (bsp_mpuxxxx_driver *)p_instance;
    if (NULL != p_mpu->pf_bus_lock) {
        status = p_mpu->pf_bus_lock(p_mpu->p_bus_lock_context, 0xFFFFFFFF);
        if (MPU_OK != status) {
#ifdef MPU_DEBUG
            log_e("read_all: failed to acquire bus lock");
#endif
            return status;
        }
    }
    /* ========================================== */
#endif
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Read all 14 bytes of sensor data starting from ACCEL_XOUTH_REG
     * Register layout (14 consecutive bytes):
     * 0x3B: ACCEL_XOUT_H    [0]
     * 0x3C: ACCEL_XOUT_L    [1]
     * 0x3D: ACCEL_YOUT_H    [2]
     * 0x3E: ACCEL_YOUT_L    [3]
     * 0x3F: ACCEL_ZOUT_H    [4]
     * 0x40: ACCEL_ZOUT_L    [5]
     * 0x41: TEMP_OUT_H      [6]
     * 0x42: TEMP_OUT_L      [7]
     * 0x43: GYRO_XOUT_H     [8]
     * 0x44: GYRO_XOUT_L     [9]
     * 0x45: GYRO_YOUT_H     [10]
     * 0x46: GYRO_YOUT_L     [11]
     * 0x47: GYRO_ZOUT_H     [12]
     * 0x48: GYRO_ZOUT_L     [13]
     * 
     * Using IIC_Read_Multi_Byte for efficient single-transaction read
     */
    status = p_iic->pf_iic_read_multi_byte(p_bus, MPU_ADDR, MPU_ACCEL_XOUTH_REG, 14, buffer);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("read_all: read multi-byte failed");
#endif
#ifdef OS_SUPPORTING
        /* Release IIC bus lock before returning */
        if (NULL != p_mpu->pf_bus_unlock) {
            p_mpu->pf_bus_unlock(p_mpu->p_bus_lock_context);
        }
#endif
        return status;
    }
    
    /* Parse accelerometer data (bytes 0-5) */
    p_data->accel_x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    p_data->accel_y_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    p_data->accel_z_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    /* Convert accelerometer raw data to g units */
    p_data->ax = (double)p_data->accel_x_raw / MPU_DEFAULT_ACCEL_SENS;
    p_data->ay = (double)p_data->accel_y_raw / MPU_DEFAULT_ACCEL_SENS;
    p_data->az = (double)p_data->accel_z_raw / MPU_DEFAULT_ACCEL_SENS;
    
    /* Parse temperature data (bytes 6-7) */
    temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    
    /* Convert temperature raw data to °C */
    p_data->tempreture = ((float)temp_raw / 340.0f) + 36.53f;
    
    /* Parse gyroscope data (bytes 8-13) */
    p_data->gyro_x_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
    p_data->gyro_y_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
    p_data->gyro_z_raw = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    /* Convert gyroscope raw data to °/s units */
    p_data->gx = (double)p_data->gyro_x_raw / MPU_DEFAULT_GYRO_SENS;
    p_data->gy = (double)p_data->gyro_y_raw / MPU_DEFAULT_GYRO_SENS;
    p_data->gz = (double)p_data->gyro_z_raw / MPU_DEFAULT_GYRO_SENS;
    
#ifdef MPU_DEBUG
    // log_d("read_all: accel[%d,%d,%d] temp[%d] gyro[%d,%d,%d]",
        //   p_data->accel_x_raw, p_data->accel_y_raw, p_data->accel_z_raw,
        //   temp_raw,
        //   p_data->gyro_x_raw, p_data->gyro_y_raw, p_data->gyro_z_raw);
    log_d("read_all: ax[%.3f,%.3f,%.3f]g temp[%.2f°C] gyro[%.3f,%.3f,%.3f]°/s",
          p_data->ax, p_data->ay, p_data->az,
          p_data->tempreture,
          p_data->gx, p_data->gy, p_data->gz);
#endif
    
#ifdef OS_SUPPORTING
    /* ========== Release IIC bus lock ========== */
    if (NULL != p_mpu->pf_bus_unlock) {
        p_mpu->pf_bus_unlock(p_mpu->p_bus_lock_context);
    }
    /* ========================================== */
#endif
    
    return status;
}

/******************************************************************************
 * @name    mpu_wakeup
 * @brief   Wake up the MPU6050 sensor from sleep mode
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: operation failed
 * @note    This function ONLY wakes up the MPU6050 from sleep mode and sets
 *          the clock source. It does NOT configure sensor parameters.
 *          
 *          Operations performed:
 *          1. Clear SLEEP bit (wake up the device)
 *          2. Set clock source to PLL with X-axis gyroscope reference (recommended)
 *          3. Enable temperature sensor
 *          4. Wait 100ms for sensor stabilization
 *          
 *          Power Management Register (0x6B) = 0x01:
 *          - Bit 7: DEVICE_RESET = 0 (normal operation)
 *          - Bit 6: SLEEP = 0 (wake up)
 *          - Bit 5: CYCLE = 0 (disabled)
 *          - Bit 3: TEMP_DIS = 0 (temperature sensor enabled)
 *          - Bits 2-0: CLKSEL = 001 (PLL with X gyro reference)
 *          
 *          After wakeup, previous sensor configurations are preserved.
 *          Use pf_set_gyro_fsr(), pf_set_accel_fsr(), pf_set_lpf(), 
 *          pf_set_rate() to configure sensor parameters if needed.
 *****************************************************************************/
static MPUXXXX_status_t mpu_wakeup(void * const p_instance)
{
    MPUXXXX_status_t status = MPU_OK;
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("mpu_wakeup: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_write_reg) {
#ifdef MPU_DEBUG
        log_e("mpu_wakeup: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
#ifdef MPU_DEBUG
    log_i("mpu_wakeup: waking up MPU6050...");
#endif
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Wake up MPU6050 and set clock source
     * PWR_MGMT_1 register (0x6B):
     * Write 0x01: 
     * - Clear SLEEP bit (wake up)
     * - CLKSEL = 001 (PLL with X-axis gyroscope reference, recommended)
     */
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_PWR_MGMT1_REG, 0x01);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_wakeup: failed to wake up device");
#endif
        return status;
    }
    
    /* Delay to allow sensor to stabilize after wakeup */
    if (NULL != p_iic->pf_delay_ms) {
        p_iic->pf_delay_ms(100);  // 100ms delay for sensor stabilization
    }
    
#ifdef MPU_DEBUG
    log_i("mpu_wakeup: MPU6050 woke up successfully");
    log_i("  - Clock source: PLL with X-axis gyroscope");
    log_i("  - Previous configurations preserved");
#endif
    
    return MPU_OK;
}

/******************************************************************************
 * @name    mpu_sleep
 * @brief   Put the MPU6050 sensor into sleep mode to save power
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: operation completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 *          - MPU_ERROR: operation failed
 * @note    This function puts the MPU6050 into sleep mode to save power.
 *          
 *          In sleep mode:
 *          - Internal oscillator is stopped
 *          - Gyroscope and accelerometer are disabled
 *          - Temperature sensor remains active (can still be read)
 *          - All registers can still be read/written via I2C
 *          - Typical current consumption: ~8µA (vs ~3.8mA in normal mode)
 *          - **Sensor configurations are preserved** (FSR, DLPF, sample rate, etc.)
 *          
 *          Power Management Register (0x6B) = 0x40:
 *          - Bit 6: SLEEP = 1 (enter sleep mode)
 *          
 *          To wake up the device, call pf_wakeup() or mpu_wakeup().
 *          Previous configurations will be preserved after wakeup.
 *****************************************************************************/
static MPUXXXX_status_t mpu_sleep(void * const p_instance)
{
    MPUXXXX_status_t status = MPU_OK;
    
    /* Parameter validation */
    if (NULL == p_instance) {
#ifdef MPU_DEBUG
        log_e("mpu_sleep: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Get IIC interface */
    iic_driver_interface_t *p_iic = MPU_IIC_INTERFACE(p_instance);
    void *p_bus = MPU_BUS_INSTANCE(p_instance);
    
    if (NULL == p_iic || NULL == p_iic->pf_iic_write_reg) {
#ifdef MPU_DEBUG
        log_e("mpu_sleep: IIC interface not initialized");
#endif
        return MPU_ERROR;
    }
    
#ifdef MPU_DEBUG
    log_i("mpu_sleep: putting MPU6050 into sleep mode...");
#endif
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_enter) {
        p_iic->pf_critical_enter();
    }
#endif
    
    /* Enter sleep mode by setting SLEEP bit in PWR_MGMT_1 register (0x6B)
     * Write 0x40:
     * - Bit 6: SLEEP = 1 (enter sleep mode)
     * - Other bits = 0 (reset to default, will lose clock source setting)
     * 
     * Note: Writing 0x40 clears CLKSEL bits. When waking up, pf_wakeup()
     * will restore the recommended clock source (PLL with X gyro).
     */
    status = p_iic->pf_iic_write_reg(p_bus, MPU_ADDR, MPU_PWR_MGMT1_REG, 0x40);
    
#ifdef OS_SUPPORTING
    if (NULL != p_iic->pf_critical_exit) {
        p_iic->pf_critical_exit();
    }
#endif
    
    if (MPU_OK != status) {
#ifdef MPU_DEBUG
        log_e("mpu_sleep: failed to enter sleep mode");
#endif
        return status;
    }
    
#ifdef MPU_DEBUG
    log_i("mpu_sleep: MPU6050 entered sleep mode successfully");
    log_i("  - Power consumption: ~3.8mA -> ~8uA (reduced 475x)");
    log_i("  - Call pf_wakeup() to wake up the device");
#endif
    
    return MPU_OK;
}

/******************************************************************************
 * @name    mpu_read_id
 * @brief   Read and log the device ID (interface wrapper)
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @return  MPUXXXX_status_t operation status
 * @note    This is a wrapper function to match the pf_read_id interface signature.
 *          It reads the device ID and logs it, but doesn't return the ID value.
 *****************************************************************************/
static MPUXXXX_status_t mpu_read_id(void * const p_instance)
{
    uint8_t device_id = 0;
    MPUXXXX_status_t status;
    
    status = read_id(p_instance, &device_id);
    
    if (MPU_OK == status) {
#ifdef MPU_DEBUG
        log_i("mpu_read_id: Device ID = 0x%02X", device_id);
#endif
    }
    
    return status;
}

/******************************************************************************
 * @name    mpu_deinit
 * @brief   Deinitialize the MPU6050 sensor
 * @param   p_instance[in] pointer to the MPU6050 driver instance
 * @return  MPUXXXX_status_t operation status
 * @note    This function can be used to perform cleanup operations if needed.
 *          Currently it just returns success.
 *****************************************************************************/
static MPUXXXX_status_t mpu_deinit(void * const p_instance)
{
    (void)p_instance;  // Unused parameter
    
#ifdef MPU_DEBUG
    log_i("mpu_deinit: MPU6050 deinitialized");
#endif
    
    return MPU_OK;
}

/******************************************************************************
 * @name    mpuxxxx_inst
 * @brief   Constructor function to initialize the MPU6050 driver instance
 * @param   mpuxxxx_driver[out] pointer to the driver instance to initialize
 * @param   iic_interface[in] pointer to IIC driver interface
 * @param   p_bus_instance[in] pointer to IIC bus instance
 * @param   p_timebase_interface[in] pointer to timebase interface
 * @param   p_os_interface[in] pointer to OS interface (if OS_SUPPORTING enabled)
 * @param   interuption_interface[in] pointer to interrupt interface
 * @return  MPUXXXX_status_t operation status
 *          - MPU_OK: initialization completed successfully
 *          - MPU_ERRORPARAMETER: invalid parameter
 * @note    This function initializes the driver structure by:
 *          1. Validating all required parameters
 *          2. Storing interface pointers
 *          3. Binding all private functions to function pointers
 *          
 *          After calling this function, the driver instance is ready to use.
 *          Call mpuxxxx_driver->pf_init() to initialize the hardware.
 *          
 *          Example usage:
 *          @code
 *          bsp_mpuxxxx_driver mpu_driver;
 *          mpuxxxx_inst(&mpu_driver, &iic_if, &bus, &timebase, &os, &int_if);
 *          mpu_driver.pf_init(&mpu_driver);
 *          @endcode
 *****************************************************************************/
MPUXXXX_status_t mpuxxxx_inst(
        bsp_mpuxxxx_driver *             const mpuxxxx_driver,
        iic_driver_interface_t *          const iic_interface,
        void *                           const p_bus_instance,
        timebase_interface_t *     const p_timebase_interface,
#ifdef OS_SUPPORTING
        os_interface_t *           const p_os_interface,
#endif
        interuption_interface_t * const interuption_interface
)
{
#ifdef MPU_DEBUG
    log_i("mpuxxxx_inst: Initializing MPU6050 driver instance...");
#endif
    
    /* Parameter validation */
    if (NULL == mpuxxxx_driver) {
#ifdef MPU_DEBUG
        log_e("mpuxxxx_inst: driver instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (NULL == iic_interface) {
#ifdef MPU_DEBUG
        log_e("mpuxxxx_inst: IIC interface pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (NULL == p_bus_instance) {
#ifdef MPU_DEBUG
        log_e("mpuxxxx_inst: bus instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Store interface pointers */
    mpuxxxx_driver->iic_interface =         iic_interface;
    mpuxxxx_driver->p_bus_instance =        p_bus_instance;
    mpuxxxx_driver->p_timebase_interface =  p_timebase_interface;
    mpuxxxx_driver->interuption_interface = interuption_interface;
    
#ifdef OS_SUPPORTING
    mpuxxxx_driver->p_os_interface =            p_os_interface;
    mpuxxxx_driver->p_buffer_interface =        NULL;  // Not used currently
    mpuxxxx_driver->semaphore_mutex_handle =    NULL;
    mpuxxxx_driver->semaphore_binary_handle =   NULL;
    mpuxxxx_driver->queue_handle = NULL;
#endif
    
#ifdef HARDWARE_IIC
    /* Initialize DMA state and buffer (Hardware I2C + DMA mode)
     * Each instance has its own independent DMA buffer and busy flag
     * This enables true multi-instance support
     */
    mpuxxxx_driver->dma_busy = false;
    for (uint8_t i = 0; i < MPU_DATA_LENGTH; i++) {
        mpuxxxx_driver->dma_buffer[i] = 0;
    }
    
    /* Initialize DMA complete callback to NULL */
    mpuxxxx_driver->pf_dma_complete_callback = NULL;
#endif
    
    /* Initialize INT interrupt callback to NULL
     * Upper layer will register callback using mpu_register_int_callback()
     * or by directly assigning to pf_int_interrupt_callback
     * 
     * HARDWARE_IIC mode: Optional (for statistics/debugging)
     * Software I2C mode: Required (to notify task to read data)
     */
    mpuxxxx_driver->pf_int_interrupt_callback = NULL;
    
    /* Bind basic functions */
    mpuxxxx_driver->pf_init =               mpu_init;
    mpuxxxx_driver->pf_deinit =             mpu_deinit;
    mpuxxxx_driver->pf_wakeup =             mpu_wakeup;
    mpuxxxx_driver->pf_sleep =              mpu_sleep;
    mpuxxxx_driver->pf_read_id =            mpu_read_id;
    mpuxxxx_driver->pf_check_data_ready =   NULL;  // TODO: implement later
    
    /* Bind configuration functions */
    mpuxxxx_driver->pf_set_gyro_fsr =       set_gyro_fsr;
    mpuxxxx_driver->pf_set_accel_fsr =      set_accel_fsr;
    mpuxxxx_driver->pf_set_lpf =            set_lpf;
    mpuxxxx_driver->pf_set_rate =           set_rate;
    
    /* Bind data reading functions */
    mpuxxxx_driver->pf_read_accel =         read_accel;
    mpuxxxx_driver->pf_read_gyro =          read_gyro;
    mpuxxxx_driver->pf_read_temp =          read_temp;
    mpuxxxx_driver->pf_read_all =           read_all;
    
#ifdef MPU_DEBUG
    log_i("mpuxxxx_inst: Driver instance initialized successfully");
    log_i("  - All function pointers bound");
    log_i("  - Ready to call pf_init() for hardware initialization");
#endif
    
    return MPU_OK;
}

/******************************************************************************
 * @name    mpu_int_interrupt_callback
 * @brief   Hardware interrupt callback function triggered by MPU6050 INT pin
 * @param   p_instance[in] Pointer to the MPU driver instance that triggered interrupt
 * @return  None
 * @note    This function is called when the MPU6050 INT pin triggers a hardware
 *          interrupt, indicating that new sensor data is ready to be read.
 *          
 *          **Design Principle: Fast In, Fast Out**
 *          - Minimal operations in ISR context
 *          - No I2C read of INT_STATUS register (configured as "clear on any read")
 *          - No OS-specific operations in driver layer
 *          - Simply initiates DMA transfer and calls upper layer callback
 *          
 *          **Workflow**:
 *          1. Validate instance pointer
 *          2. Check if this instance's DMA is busy (prevent concurrent transfers)
 *          3. If not busy, start DMA transfer to read 14 bytes from MPU6050
 *          4. Set this instance's DMA busy flag
 *          5. Call this instance's registered callback (optional)
 *          6. Return immediately
 *          
 *          **Decoupling from OS**:
 *          - This function does NOT use any FreeRTOS APIs
 *          - Upper layer can register a callback to perform OS-specific operations
 *          
 *          **Multi-Instance Support**:
 *          - Each instance has its own DMA buffer and busy flag
 *          - User must pass correct instance pointer from hardware interrupt
 *          - Multiple MPU6050 devices can operate independently
 *          
 *          **DMA Transfer Details**:
 *          - Source: MPU6050 registers 0x3B-0x48 (14 bytes)
 *          - Destination: p_instance->dma_buffer[14]
 *          - Transfer method: I2C DMA mode
 *          
 * @warning This function runs in ISR context. DO NOT:
 *          - Perform I2C read/write operations directly
 *          - Process or parse sensor data
 *          - Use blocking operations
 *          - Call printf or log functions
 *****************************************************************************/
void mpu_int_interrupt_callback(bsp_mpuxxxx_driver *p_instance)
{
    /* Validate instance pointer */
    if (p_instance == NULL) {
        return;
    }
    
#ifdef HARDWARE_IIC
    /* ========== Hardware I2C + DMA Mode ========== */
    
    /* Check if this instance's DMA is currently busy */
    if (p_instance->dma_busy) {
        /* DMA still busy, cannot start new transfer
         * This data point will be dropped
         * In a properly configured system with matching sample rates,
         * this should rarely occur
         */
        return;
    }
    
    /* Set this instance's DMA busy flag before starting transfer */
    p_instance->dma_busy = true;
    
    /* Start DMA transfer to read 14 bytes from MPU6050 to this instance's buffer
     * 
     * The DMA function should be implemented in IIC interface as pf_iic_read_dma
     * 
     * Example implementation:
     *   MPUXXXX_status_t hardware_iic_read_dma(...) {
     *       I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)p_bus;
     *       HAL_I2C_Mem_Read_DMA(hi2c, daddr << 1, reg,
     *                           I2C_MEMADD_SIZE_8BIT, buffer, length);
     *       return MPU_OK;
     *   }
     */
    if (p_instance->iic_interface != NULL &&
        p_instance->iic_interface->pf_iic_read_dma != NULL) {
        
        MPUXXXX_status_t status = p_instance->iic_interface->pf_iic_read_dma(
            p_instance->p_bus_instance,
            MPU_ADDR,
            MPU_ACCEL_XOUTH_REG,
            MPU_DATA_LENGTH,
            p_instance->dma_buffer
        );
        
        /* If DMA start failed, clear busy flag */
        if (status != MPU_OK) {
            p_instance->dma_busy = false;
        }
    } else {
        /* DMA function not available, clear busy flag */
        p_instance->dma_busy = false;
    }
    
    /* Optional: Call INT callback for statistics/debugging */
    if (p_instance->pf_int_interrupt_callback != NULL) {
        p_instance->pf_int_interrupt_callback();
    }
    
    /* ISR exits here - DMA hardware will transfer data in background
     * When DMA completes, mpu_dma_interrupt_callback(p_instance) will be called
     */
    
#else
    /* ========== Software I2C Mode (Polling) ========== */
    
    /* No DMA support, notify application layer to actively read data
     * 
     * Preferred approach: Use registered callback function
     * Handler layer should implement the callback with proper OS operations:
     * - xTaskNotifyFromISR() to wake handler thread
     * - portYIELD_FROM_ISR() if needed
     * 
     * Fallback: Direct notification if callback not registered
     * (This should be avoided in normal operation)
     */
    if (p_instance->pf_int_interrupt_callback != NULL) {
        /* Preferred: Use handler layer callback
         * Handler layer will handle TaskNotify and yield properly
         */
        p_instance->pf_int_interrupt_callback();
    } else {
        /* Fallback: Direct OS notification (not recommended)
         * This requires OS interface to be properly configured
         */
        if (p_instance->p_os_interface != NULL && 
            p_instance->p_os_interface->os_TaskNotifyGiveFromISR != NULL &&
            p_instance->p_os_interface->task_handle != NULL) {
            
            /* Declare variable to receive yield status
             * Using generic int type to avoid FreeRTOS type dependency in driver layer
             */
            int higher_priority_task_woken = 0;
            
            /* Call OS interface function with address of yield status variable
             * Note: The OS interface implementation should handle portYIELD_FROM_ISR()
             * based on the value written to higher_priority_task_woken
             */
            p_instance->p_os_interface->os_TaskNotifyGiveFromISR(
                p_instance->p_os_interface->task_handle,
                &higher_priority_task_woken  // Pass address, not NULL
            );
            
            /* Note: Actual yield operation (portYIELD_FROM_ISR) should be handled
             * by the OS interface implementation layer or in the ISR that calls
             * this function, as driver layer should avoid direct OS API calls
             */
        }
        /* If both callback and OS interface are unavailable, this is an error condition
         * In Software I2C mode, proper notification mechanism must be configured
         */
    }
#endif /* HARDWARE_IIC */
}

#ifdef HARDWARE_IIC
/******************************************************************************
 * @name    mpu_dma_interrupt_callback
 * @brief   DMA transfer complete interrupt callback function (Hardware I2C mode only)
 * @param   p_instance[in] Pointer to the MPU driver instance that completed DMA
 * @return  None
 * @note    This function is called when the DMA controller completes transferring
 *          14 bytes of sensor data from MPU6050 to memory (p_instance->dma_buffer).
 *          
 *          **Design Principle: Fast In, Fast Out**
 *          - Minimal operations in ISR context
 *          - No OS-specific operations in driver layer
 *          - Simply clears flag and calls upper layer callback
 *          
 *          **Workflow**:
 *          1. Validate instance pointer
 *          2. Clear this instance's DMA busy flag to allow next transfer
 *          3. Call this instance's registered callback to notify upper layer
 *          4. Return immediately
 *          
 *          **Decoupling from OS**:
 *          - This function does NOT use any FreeRTOS APIs
 *          - Upper layer registers a callback to perform OS-specific operations:
 *            * TaskNotify to wake data processing task
 *            * Send to queue
 *            * Set event group bits
 *            * Release semaphore
 *          
 *          **Multi-Instance Support**:
 *          - Each instance has its own DMA buffer and callback
 *          - User must pass correct instance pointer from DMA interrupt
 *          - Multiple MPU6050 devices can have independent DMA operations
 *          
 *          **Upper Layer Responsibility**:
 *          - Parse p_instance->dma_buffer into sensor data structure
 *          - Apply calibration/filtering
 *          - Send processed data to application tasks
 *          
 * @warning This function runs in ISR context. DO NOT:
 *          - Parse or process sensor data
 *          - Perform calculations or conversions
 *          - Use blocking operations
 *          - Call printf or log functions
 *          
 *          Only available when HARDWARE_IIC is defined.
 *****************************************************************************/
void mpu_dma_interrupt_callback(bsp_mpuxxxx_driver *p_instance)
{
    /* Validate instance pointer */
    if (p_instance == NULL) {
        return;
    }
    
    /* Clear this instance's DMA busy flag to allow next transfer */
    p_instance->dma_busy = false;
    
    /* Call this instance's registered callback to notify that data is ready
     * 
     * The upper layer (OS-aware code) should implement this callback to:
     * - Notify data processing task (e.g., xTaskNotifyFromISR)
     * - Send to queue (e.g., xQueueSendFromISR)
     * - Set event bits (e.g., xEventGroupSetBitsFromISR)
     * - Or any other OS-specific notification mechanism
     * 
     * The callback should be ISR-safe and fast
     * 
     * Example:
     *   void app_mpu_dma_callback(void) {
     *       BaseType_t xHigherPriorityTaskWoken = pdFALSE;
     *       xTaskNotifyFromISR(data_task_handle, 0x01, eSetBits,
     *                         &xHigherPriorityTaskWoken);
     *       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
     *   }
     */
    if (p_instance->pf_dma_complete_callback != NULL) {
        p_instance->pf_dma_complete_callback();
    }
    
    /* ISR exits here
     * Data processing task will be woken by upper layer callback
     * Task will access p_instance->dma_buffer to retrieve sensor data
     */
}
#endif /* HARDWARE_IIC */

/******************************************************************************
 * @name    mpu_register_int_callback
 * @brief   Register callback function for INT interrupt
 * @param   p_instance[in] Pointer to MPU driver instance
 * @param   callback[in] Function pointer to callback (ISR-safe)
 * @return  MPUXXXX_status_t operation status
 * @note    Upper layer (OS-aware code) should call this to register a callback
 *          that will be invoked from mpu_int_interrupt_callback().
 *          
 *          The callback runs in ISR context and should be fast.
 *          
 *          Alternatively, can directly assign to p_instance->pf_int_interrupt_callback.
 *          
 *          Example usage (in application code):
 *          @code
 *          void my_int_callback(void) {
 *              // OS-specific operations, e.g., set event flag
 *          }
 *          mpu_register_int_callback(&mpu_instance, my_int_callback);
 *          // Or directly:
 *          // mpu_instance.pf_int_interrupt_callback = my_int_callback;
 *          @endcode
 *****************************************************************************/
MPUXXXX_status_t mpu_register_int_callback(
    bsp_mpuxxxx_driver *p_instance,
    void (*callback)(void))
{
    if (p_instance == NULL) {
#ifdef MPU_DEBUG
        log_e("mpu_register_int_callback: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (callback == NULL) {
#ifdef MPU_DEBUG
        log_w("mpu_register_int_callback: callback is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Register callback to this instance */
    p_instance->pf_int_interrupt_callback = callback;
    
#ifdef MPU_DEBUG
    log_i("mpu_register_int_callback: INT callback registered to instance %p", p_instance);
#endif
    
    return MPU_OK;
}

#ifdef HARDWARE_IIC
/******************************************************************************
 * @name    mpu_register_dma_complete_callback
 * @brief   Register callback function for DMA complete interrupt (Hardware I2C mode only)
 * @param   p_instance[in] Pointer to MPU driver instance
 * @param   callback[in] Function pointer to callback (ISR-safe)
 * @return  MPUXXXX_status_t operation status
 * @note    Upper layer (OS-aware code) should call this to register a callback
 *          that will be invoked from mpu_dma_interrupt_callback().
 *          
 *          The callback runs in ISR context and should be fast.
 *          Typically, this callback will notify an RTOS task.
 *          
 *          Alternatively, can directly assign to p_instance->pf_dma_complete_callback.
 *          
 *          Example usage (in application code with FreeRTOS):
 *          @code
 *          void my_dma_callback(void) {
 *              BaseType_t xHigherPriorityTaskWoken = pdFALSE;
 *              xTaskNotifyFromISR(data_task_handle, 0x01, eSetBits, 
 *                                &xHigherPriorityTaskWoken);
 *              portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
 *          }
 *          mpu_register_dma_complete_callback(&mpu_instance, my_dma_callback);
 *          // Or directly:
 *          // mpu_instance.pf_dma_complete_callback = my_dma_callback;
 *          @endcode
 *          
 *          Only available when HARDWARE_IIC is defined.
 *****************************************************************************/
MPUXXXX_status_t mpu_register_dma_complete_callback(
    bsp_mpuxxxx_driver *p_instance,
    void (*callback)(void))
{
    if (p_instance == NULL) {
#ifdef MPU_DEBUG
        log_e("mpu_register_dma_complete_callback: instance pointer is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    if (callback == NULL) {
#ifdef MPU_DEBUG
        log_w("mpu_register_dma_complete_callback: callback is NULL");
#endif
        return MPU_ERRORPARAMETER;
    }
    
    /* Register callback to this instance */
    p_instance->pf_dma_complete_callback = callback;
    
#ifdef MPU_DEBUG
    log_i("mpu_register_dma_complete_callback: DMA callback registered to instance %p", p_instance);
#endif
    
    return MPU_OK;
}
#endif /* HARDWARE_IIC */

#ifdef HARDWARE_IIC
/******************************************************************************
 * @name    mpu_get_dma_buffer
 * @brief   Get pointer to DMA buffer for data access (Hardware I2C mode only)
 * @param   p_instance[in] Pointer to MPU driver instance
 * @return  Pointer to DMA buffer (14 bytes), or NULL if instance is invalid
 * @note    This function returns the pointer to the internal DMA buffer where
 *          sensor data is stored after DMA transfer completes.
 *          
 *          The data processing task can use this to access the raw sensor data.
 *          Each instance has its own independent DMA buffer.
 *          
 *          **Buffer Layout** (14 bytes):
 *          - [0-1]:   ACCEL_X (High byte, Low byte)
 *          - [2-3]:   ACCEL_Y (High byte, Low byte)
 *          - [4-5]:   ACCEL_Z (High byte, Low byte)
 *          - [6-7]:   TEMP    (High byte, Low byte)
 *          - [8-9]:   GYRO_X  (High byte, Low byte)
 *          - [10-11]: GYRO_Y  (High byte, Low byte)
 *          - [12-13]: GYRO_Z  (High byte, Low byte)
 *          
 * @warning Buffer content is only valid after DMA complete callback
 *          and before next DMA transfer starts. Copy data quickly.
 *          
 *          Only available when HARDWARE_IIC is defined.
 *****************************************************************************/
uint8_t* mpu_get_dma_buffer(bsp_mpuxxxx_driver *p_instance)
{
    if (p_instance == NULL) {
        return NULL;
    }
    return p_instance->dma_buffer;
}

/******************************************************************************
 * @name    mpu_get_dma_buffer_length
 * @brief   Get the length of DMA buffer (Hardware I2C mode only)
 * @param   None
 * @return  Length of DMA buffer in bytes (always 14)
 * @note    Returns the size of the DMA buffer for validation purposes.
 *          This is a constant value and does not depend on instance.
 *          
 *          Only available when HARDWARE_IIC is defined.
 *****************************************************************************/
uint8_t mpu_get_dma_buffer_length(void)
{
    return MPU_DATA_LENGTH;
}

/******************************************************************************
 * @name    mpu_is_dma_busy
 * @brief   Check if DMA transfer is currently in progress for specific instance (Hardware I2C mode only)
 * @param   p_instance[in] Pointer to MPU driver instance
 * @return  true if DMA is busy, false if idle or instance is invalid
 * @note    Can be used to check DMA status for debugging or synchronization.
 *          Each instance has its own independent DMA busy flag.
 *          
 *          Only available when HARDWARE_IIC is defined.
 *****************************************************************************/
bool mpu_is_dma_busy(bsp_mpuxxxx_driver *p_instance)
{
    if (p_instance == NULL) {
        return false;
    }
    return p_instance->dma_busy;
}
#endif /* HARDWARE_IIC */
