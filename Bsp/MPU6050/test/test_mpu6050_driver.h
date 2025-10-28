/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file test_mpu6050_driver.h
 * 
 * @brief Test functions for MPU6050 sensor driver
 * 
 * @par Dependencies:
 * - bsp_mpuxxxx_driver.h
 * - test_mpu6050_driver.c
 * 
 * @author Jack | R&D Dept. | EternalChip
 * 
 * @version V1.0 2025.10.28
 *
 * @note 1 tab == 4 spaces
 *****************************************************************************/

#ifndef __TEST_MPU6050_DRIVER_H__
#define __TEST_MPU6050_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

//******************************** Includes *********************************//
#include "bsp_mpuxxxx_driver.h"
#include <stdint.h>
#include <stdbool.h>
//******************************** Includes *********************************//

//***************************** Global Variables ****************************//
/**
 * @brief Global MPU6050 driver instance for testing
 * @note This instance is initialized by test_mpu6050_init()
 */
extern bsp_mpuxxxx_driver g_mpu6050_driver;
//***************************** Global Variables ****************************//

//***************************** Function Prototypes *************************//
MPUXXXX_status_t test_mpu6050_init(void);
MPUXXXX_status_t test_mpu6050_read_id(void);
MPUXXXX_status_t test_mpu6050_read_all(void);
// MPUXXXX_status_t test_mpu6050_wakeup(void);
// MPUXXXX_status_t test_mpu6050_sleep(void);

//***************************** Function Prototypes *************************//

#ifdef __cplusplus
}
#endif

#endif /* __TEST_MPU6050_DRIVER_H__ */

