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
	LOG_DBG("Display init: Done");
}

void display_disable_blanking(void)
{
	display_blanking_off(display_dev);
}
/* ********** ********** ********** ********** ********** */
