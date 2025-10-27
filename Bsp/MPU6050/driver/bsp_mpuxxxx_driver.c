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
 //********************** private macro definitions **************************//

 //********************** private function prototypes ************************//
static MPUXXXX_status_t set_gyro_fsr(void * const p_instance, uint8_t fsr);
static MPUXXXX_status_t set_accel_fsr(void * const p_instance, uint8_t fsr);
static MPUXXXX_status_t set_lpf(void * const p_instance);
static MPUXXXX_status_t set_rate(void * const p_instance, uint16_t rate);
static MPUXXXX_status_t read_accel(void * const p_instance, mpu6050_data_t *p_data);
static MPUXXXX_status_t read_gyro(void * const p_instance, mpu6050_data_t *p_data);
static MPUXXXX_status_t read_temp(void * const p_instance, mpu6050_data_t *p_data);
static MPUXXXX_status_t read_all(void * const p_instance, mpu6050_data_t *p_data);
 //********************** private function prototypes ************************//

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
    p_data->tempreture = ((float)temp_raw / MPU_TEMP_SENSITIVITY) + MPU_TEMP_OFFSET;
    
#ifdef MPU_DEBUG
    log_d("read_temp: raw[%d] temp[%.2f°C]", temp_raw, p_data->tempreture);
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
    p_data->tempreture = ((float)temp_raw / MPU_TEMP_SENSITIVITY) + MPU_TEMP_OFFSET;
    
    /* Parse gyroscope data (bytes 8-13) */
    p_data->gyro_x_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
    p_data->gyro_y_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
    p_data->gyro_z_raw = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    /* Convert gyroscope raw data to °/s units */
    p_data->gx = (double)p_data->gyro_x_raw / MPU_DEFAULT_GYRO_SENS;
    p_data->gy = (double)p_data->gyro_y_raw / MPU_DEFAULT_GYRO_SENS;
    p_data->gz = (double)p_data->gyro_z_raw / MPU_DEFAULT_GYRO_SENS;
    
#ifdef MPU_DEBUG
    log_d("read_all: accel[%d,%d,%d] temp[%d] gyro[%d,%d,%d]",
          p_data->accel_x_raw, p_data->accel_y_raw, p_data->accel_z_raw,
          temp_raw,
          p_data->gyro_x_raw, p_data->gyro_y_raw, p_data->gyro_z_raw);
    log_d("read_all: ax[%.3f,%.3f,%.3f]g temp[%.2f°C] gyro[%.3f,%.3f,%.3f]°/s",
          p_data->ax, p_data->ay, p_data->az,
          p_data->tempreture,
          p_data->gx, p_data->gy, p_data->gz);
#endif
    
    return status;
}