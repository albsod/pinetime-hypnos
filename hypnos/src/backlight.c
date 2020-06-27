/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdbool.h>
#include <drivers/gpio.h>
#include <display.h>
#include "backlight.h"
#include "log.h"

/* ********** ********** DEFINES ********** ********** ********** */
#define BACKLIGHT_PORT  DT_GPIO_LABEL(DT_ALIAS(led1), gpios)
#define BACKLIGHT       DT_GPIO_PIN(DT_ALIAS(led1), gpios)
/* ********** **********  ********** ********** ********** */

/* ********** ********** VARIABLES AND STRUCTS ********** ********** */
static struct device* backlight_dev;
static bool backlight_enabled = false;
/* ********** ********** ********** ********** ********** ********** */

/* ********** ********** FUNCTIONS ********** ********** */
void backlight_init()
{
	backlight_dev = device_get_binding(BACKLIGHT_PORT);
	gpio_pin_configure(backlight_dev, BACKLIGHT, GPIO_OUTPUT);
	backlight_enable(true);
	LOG_DBG("Backlight init: Done");
}

void backlight_enable(bool enable) {
	gpio_pin_set_raw(backlight_dev, BACKLIGHT, enable ? 0 : 1);
	backlight_enabled = enable;
}

bool backlight_is_enabled()
{
	return backlight_enabled;
}
/* ********** ********** ********** ********** ********** */
