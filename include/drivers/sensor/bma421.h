/*
 * Copyright (c) 2020 Stephane Dorre
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_BMA421_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_BMA421_H_

#include <drivers/sensor.h>

enum bma421_channel {
	BMA421_CHAN_STEP_COUNTER = SENSOR_CHAN_PRIV_START,
	BMA421_CHAN_STEP_ENABLE,
};

/* If necessary at some point */
enum bma421_attribute {
	BMA421_ATTR_CPI = SENSOR_ATTR_PRIV_START,
	BMA421_ATTR_REST_ENABLE,
	BMA421_ATTR_RUN_DOWNSHIFT_TIME,
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_BMA421_H_ */
