/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/display.h>
#include <display.h>
#include "display.h"
#include "log.h"

/* ********** ********** VARIABLES AND STRUCTS ********** ********** */
static const struct device *display_dev;
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
	(void)pm_device_state_set(display_dev, PM_DEVICE_STATE_LOW_POWER, NULL,
				     NULL);
}

void display_wake_up(void)
{
	(void)pm_device_state_set(display_dev, PM_DEVICE_STATE_LOW_POWER, NULL,
				     NULL);
}


/* ********** ********** ********** ********** ********** */
