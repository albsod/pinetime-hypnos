/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/display.h>
#include <display.h>
#include "log.h"

/* ********** ********** FUNCTIONS ********** ********** */
void touch_sleep(void)
{
	(void)device_set_power_state(device_get_binding(CONFIG_CST816S_NAME), DEVICE_PM_SUSPEND_STATE, NULL,
				     NULL);
#ifdef CONFIG_LOG
	int power_state = 0;
	(void)device_get_power_state(device_get_binding(CONFIG_CST816S_NAME), &power_state);
	LOG_INF("Touch controller power state: %u", power_state);
#endif
}

void touch_wake_up(void)
{
	(void)device_set_power_state(device_get_binding(CONFIG_CST816S_NAME), DEVICE_PM_ACTIVE_STATE, NULL,
				     NULL);
#ifdef CONFIG_LOG
	int power_state = 0;
	(void)device_get_power_state(device_get_binding(CONFIG_CST816S_NAME), &power_state);
	LOG_INF("Touch controller power state: %u", power_state);
#endif
}


/* ********** ********** ********** ********** ********** */
