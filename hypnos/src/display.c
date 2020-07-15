/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/display.h>
#include <display.h>
#include "display.h"
#include "log.h"

/* ********** ********** VARIABLES AND STRUCTS ********** ********** */
static struct device *display_dev;
/* ********** ********** ********** ********** ********** ********** */

/* ********** ********** FUNCTIONS ********** ********** */
void display_init(void)
{
	display_dev = device_get_binding(CONFIG_LVGL_DISPLAY_DEV_NAME);
	display_blanking_off(display_dev);
	LOG_DBG("Display init: Done");
}

void display_sleep(void)
{
	(void)device_set_power_state(display_dev, DEVICE_PM_LOW_POWER_STATE, NULL,
								NULL);
}

void display_wake_up(void)
{
	(void)device_set_power_state(display_dev, DEVICE_PM_ACTIVE_STATE, NULL,
								NULL);
}


/* ********** ********** ********** ********** ********** */
