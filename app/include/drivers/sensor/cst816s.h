/*
 * Copyright (c) 2020 Stephane Dorre
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_CTS816A_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_CTS816A_H_

#include <drivers/sensor.h>

enum cst816s_gesture {
    NONE,
    SLIDE_UP,
    SLIDE_DOWN,
    SLIDE_LEFT,
    SLIDE_RIGHT,
    CLICK,
    DOUBLE_CLICK,
    LONG_PRESS,
};

enum cst816s_action {
    DOWN = 0,
    UP = 1,
    CONTACT = 2,
};

enum cst816s_channel {
	CST816S_CHAN_GESTURE = SENSOR_CHAN_PRIV_START,
	CST816S_CHAN_TOUCH_POINT_1,
    CST816S_CHAN_TOUCH_POINT_2,
};

/* If necessary at some point */
/*
enum cst816s_attribute {
 	CST816S_ATTR_XXX = SENSOR_ATTR_PRIV_START,
 	CST816S_ATTR_YYY,
};

enum cts816s_trigger {
 	CST816S_TRIG_XXX = SENSOR_TRIG_PRIV_START,
 	CST816S_TRIG_YYY,
};
*/

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_CTS816A_H_ */