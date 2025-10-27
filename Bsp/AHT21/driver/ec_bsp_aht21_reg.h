/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file ec_bsp_aht21_reg.c
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
#ifndef __BSP_AHT21_REG_H__
#define __BSP_AHT21_REG_H__

//************************** macro definitions *****************************//
#define AHT21_REG_READ_ADDR             0x71 //0x38 <<1 | 1
#define AHT21_REG_WRITE_ADDR            0x70 //0x38 <<1 | 0

#define AHT21_REG_MEASURE_CMD           0xAC
#define AHT21_REG_MEASURE_CMD_ARGS1     0x33
#define AHT21_REG_MEASURE_CMD_ARGS2     0x00
//************************** macro definitions *****************************//

#endif /* __BSP_AHT21_REG_H__ */

