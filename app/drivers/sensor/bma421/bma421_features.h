/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file       bma421.h
 * @date       2020-05-08
 * @version    V2.14.13
 *
 */

/**
 * \ingroup bma4xy
 * \defgroup bma421 BMA421
 * @brief Sensor driver for BMA421 sensor
 */

#ifndef BMA421_H
#define BMA421_H

#ifdef __cplusplus
extern "C" {
#endif
#include "bma4.h"

/**\name Chip ID of BMA421 sensor */
#define BMA421_CHIP_ID                 UINT8_C(0x11)

/**\ Configuration ID start position of BMA421 sensor */
#define BMA421_CONFIG_ID_START_ADDR    UINT8_C(66)

/**\name Read/Write Lengths */
#define BMA421_RD_WR_MIN_LEN           UINT8_C(2)

/**************************************************************/
/**\name    User macros */
/**************************************************************/
/**\name Any-motion/No-motion axis enable macros */
#define BMA421_X_AXIS_EN               UINT8_C(0x01)
#define BMA421_Y_AXIS_EN               UINT8_C(0x02)
#define BMA421_Z_AXIS_EN               UINT8_C(0x04)
#define BMA421_EN_ALL_AXIS             UINT8_C(0x07)
#define BMA421_DIS_ALL_AXIS            UINT8_C(0x00)

/**\name Feature enable macros for the sensor */
#define BMA421_STEP_CNTR               UINT8_C(0x01)
#define BMA421_STEP_ACT                UINT8_C(0x02)
#define BMA421_WRIST_WEAR              UINT8_C(0x04)
#define BMA421_SINGLE_TAP              UINT8_C(0x08)
#define BMA421_DOUBLE_TAP              UINT8_C(0x10)

/**\name Interrupt status macros */
#define BMA421_SINGLE_TAP_INT          UINT8_C(0x01)
#define BMA421_STEP_CNTR_INT           UINT8_C(0x02)
#define BMA421_ACTIVITY_INT            UINT8_C(0x04)
#define BMA421_WRIST_WEAR_INT          UINT8_C(0x08)
#define BMA421_DOUBLE_TAP_INT          UINT8_C(0x10)
#define BMA421_ANY_MOT_INT             UINT8_C(0x20)
#define BMA421_NO_MOT_INT              UINT8_C(0x40)
#define BMA421_ERROR_INT               UINT8_C(0x80)

/**\name Activity recognition macros */
#define BMA421_USER_STATIONARY         UINT8_C(0x00)
#define BMA421_USER_WALKING            UINT8_C(0x01)
#define BMA421_USER_RUNNING            UINT8_C(0x02)
#define BMA421_STATE_INVALID           UINT8_C(0x03)


/***************************************************************************/

/*!     BMA421 User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bma421
 * \defgroup bma421ApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma421ApiInit
 * \page bma421_api_bma421_init bma421_init
 * \code
 * int8_t bma421_init(struct bma4_dev *dev);
 * \endcode
 * @details This API is the entry point.
 * Call this API before using all other APIs.
 * This API reads the chip-id of the sensor and sets the resolution.
 *
 * @param[in,out] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma421_init(struct bma4_dev *dev);

/**
 * \ingroup bma421
 * \defgroup bma421ApiConfig ConfigFile
 * @brief Write binary configuration in the sensor
 */

/**
 * \ingroup bma421
 * \defgroup bma421ApiConfigId ConfigId
 * @brief Get Configuration ID of the sensor
 */

/*!
 * \ingroup bma421ApiConfig
 * \page bma421_api_bma421_get_config_id bma421_get_config_id
 * \code
 * int8_t bma421_get_config_id(uint16_t *config_id, struct bma4_dev *dev);
 * \endcode
 * @details This API is used to get the configuration id of the sensor.
 *
 * @param[out] config_id : Pointer variable used to store the configuration id.
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma421_get_config_id(uint16_t *config_id, struct bma4_dev *dev);

/**
 * \ingroup bma421
 * \defgroup bma421ApiMapInt Map / Unmap Interrupt
 * @brief Map / Unmap user provided interrupt to interrupt pin1 or pin2 of the sensor
 */

/*!
 * \ingroup bma421ApiMapInt
 * \page bma421_api_bma421_map_interrupt bma421_map_interrupt
 * \code
 * int8_t bma421_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);
 * \endcode
 * @details This API sets/unsets the user provided interrupt to either
 * interrupt pin1 or pin2 in the sensor.
 *
 * @param[in] int_line: Variable to select either interrupt pin1 or pin2.
 *
 *@verbatim
 *  int_line    |   Macros
 *  ------------|-------------------
 *  0x00        |  BMA4_INTR1_MAP
 *  0x01      |  BMA4_INTR2_MAP
 *@endverbatim
 *
 * @param[in] int_map : Variable to specify the interrupts.
 * @param[in] enable : Variable to specify mapping or unmapping of interrupts.
 *
 *@verbatim
 *  enable  |   Macros
 *  --------|-------------------
 *  0x00    |  BMA4_DISABLE
 *  0x01    |  BMA4_ENABLE
 *@endverbatim
 *
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @note Below macros specify the interrupts.
 *
 * Feature Interrupts
 *  - BMA421_STEP_CNTR_INT
 *  - BMA421_ACTIVITY_INT
 *  - BMA421_WRIST_WEAR_INT
 *  - BMA421_WAKEUP_INT
 *  - BMA421_ANY_MOT_INT
 *  - BMA421_NO_MOT_INT
 *  - BMA421_ERROR_INT
 *
 * Hardware Interrupts
 *  - BMA4_FIFO_FULL_INT
 *  - BMA4_FIFO_WM_INT
 *  - BMA4_DATA_RDY_INT
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma421_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma421
 * \defgroup bma421ApiIntS Interrupt Status
 * @brief Read interrupt status of the sensor
 */

/*!
 * \ingroup bma421ApiIntS
 * \page bma421_api_bma421_read_int_status bma421_read_int_status
 * \code
 * int8_t bma421_read_int_status(uint16_t *int_status, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the bma421 interrupt status from the sensor.
 *
 * @param[out] int_status : Variable to store the interrupt status read from
 * the sensor.
 * @param[in] dev : Structure instance of bma4_dev.
 *
 *  @note Below macros are used to check the interrupt status.
 *
 * Feature Interrupts
 *  - BMA421_STEP_CNTR_INT
 *  - BMA421_ACTIVITY_INT
 *  - BMA421_WRIST_WEAR_INT
 *  - BMA421_WAKEUP_INT
 *  - BMA421_ANY_MOT_INT
 *  - BMA421_NO_MOT_INT
 *  - BMA421_ERROR_INT
 *
 * Hardware Interrupts
 *  - BMA4_FIFO_FULL_INT
 *  - BMA4_FIFO_WM_INT
 *  - BMA4_DATA_RDY_INT
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma421_read_int_status(uint16_t *int_status, struct bma4_dev *dev);

#ifdef __cplusplus
}
#endif /*End of CPP guard */

#endif /*End of header guard macro */
