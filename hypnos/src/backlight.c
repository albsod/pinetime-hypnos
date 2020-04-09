/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <display.h>
#include "backlight.h"

/* ********** ********** DEFINES ********** ********** ********** */
#define BACKLIGHT_PORT  DT_ALIAS_LED1_GPIOS_CONTROLLER
#define BACKLIGHT       DT_ALIAS_LED1_GPIOS_PIN
/* ********** **********  ********** ********** ********** */

/* ********** ********** VARIABLES AND STRUCTS ********** ********** */
static struct device* backlight_dev;
static bool backlight_enabled = false;
/* ********** ********** ********** ********** ********** ********** */

/* ********** ********** FUNCTIONS ********** ********** */
void backlight_init()
{
	backlight_dev = device_get_binding(BACKLIGHT_PORT);
	gpio_pin_configure(backlight_dev, BACKLIGHT, GPIO_DIR_OUT);
	backlight_enable(true);
	printk("Backlight initialized.\n");
}

void backlight_enable(bool enable) {
	gpio_pin_write(backlight_dev, BACKLIGHT, enable ? 0 : 1);
	backlight_enabled = enable;
}

bool backlight_is_enabled()
{
	return backlight_enabled;
}
/* ********** ********** ********** ********** ********** */
