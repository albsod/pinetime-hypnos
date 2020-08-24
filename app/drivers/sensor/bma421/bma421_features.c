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
 * @file       bma421.c
 * @date       2020-05-08
 * @version    V2.14.13
 *
 */

/*! \file bma421.c
 * \brief Sensor Driver for BMA421 sensor
 */

#include "bma421_features.h"

/***************************************************************************/

/**\name        Function definitions
 ****************************************************************************/

/*!
 * @brief This API is the entry point.
 * Call this API before using all other APIs.
 * This API reads the chip-id of the sensor and sets the resolution.
 */
int8_t bma421_init(struct bma4_dev *dev)
{
    int8_t rslt;

    rslt = bma4_init(dev);
    if (rslt == BMA4_OK)
    {
        if (dev->chip_id == BMA421_CHIP_ID)
        {
            /* Resolution of BMA421 sensor is 12 bit */
            dev->resolution = 12;
        }
        else
        {
            rslt = BMA4_E_INVALID_SENSOR;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets/un-sets the user provided interrupt to either interrupt
 * pin1 or pin2 in the sensor.
 */
int8_t bma421_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev)
{
    int8_t rslt = BMA4_OK;

    if (dev != NULL)
    {
        if (dev->chip_id == BMA421_CHIP_ID)
        {
            if (int_line <= 1)
            {
                /* Map/Unmap the interrupt */
                rslt = bma4_map_interrupt(int_line, int_map, enable, dev);
            }
            else
            {
                rslt = BMA4_E_INT_LINE_INVALID;
            }
        }
        else
        {
            rslt = BMA4_E_INVALID_SENSOR;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the bma421 interrupt status from the sensor.
 */
int8_t bma421_read_int_status(uint16_t *int_status, struct bma4_dev *dev)
{
    int8_t rslt = BMA4_OK;

    if (dev != NULL)
    {
        if (dev->chip_id == BMA421_CHIP_ID)
        {
            /* Read the interrupt status */
            rslt = bma4_read_int_status(int_status, dev);
        }
        else
        {
            rslt = BMA4_E_INVALID_SENSOR;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*! @endcond */
