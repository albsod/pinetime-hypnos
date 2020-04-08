/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <drivers/display.h>
#include <display.h>
#include "display.h"

/* ********** ********** VARIABLES AND STRUCTS ********** ********** */
static struct device *display_dev;
/* ********** ********** ********** ********** ********** ********** */

/* ********** ********** FUNCTIONS ********** ********** */
void display_init(void)
{
	display_dev = device_get_binding(CONFIG_LVGL_DISPLAY_DEV_NAME);
	printk("Display initialized.\n");
}

void display_disable_blanking(void)
{
	display_blanking_off(display_dev);
}
/* ********** ********** ********** ********** ********** */
