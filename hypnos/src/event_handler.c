/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include "backlight.h"
#include "battery.h"
#include "clock.h"
#include "event_handler.h"

/* ********** defines ********** */
#define BAT_PERCENTAGE_READ_INTERVAL K_MINUTES(5)
#define BAT_CHA_PIN 12
/* ********** defines ********** */


/* ********** variables ********** */
static struct k_timer battery_percentage_timer;
static struct k_timer clock_tick_timer;
static struct device *charging_dev;
static struct gpio_callback charging_cb;
/* ********** variables ********** */

/* ********** init function ********** */
void init_event_handler()
{
        // TODO: Check return values for error handling.

	/* Initialize GPIOs */
        charging_dev = device_get_binding("GPIO_0");
        gpio_pin_configure(charging_dev, BAT_CHA_PIN, GPIO_DIR_IN | GPIO_INT
                | GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE);
        gpio_init_callback(&charging_cb, battery_callback_charging,
                BIT(BAT_CHA_PIN));

	/* Enable GPIOs */
	gpio_add_callback(charging_dev, &charging_cb);
        gpio_pin_enable_callback(charging_dev, BAT_CHA_PIN);

	/* Initialize timers */
        k_timer_init(&battery_percentage_timer,
		     battery_percentage_interrupt_handler, NULL);
	k_timer_init(&clock_tick_timer, clock_callback_tick, NULL);

	/* Start timers */
        k_timer_start(&battery_percentage_timer, BAT_PERCENTAGE_READ_INTERVAL,
		      BAT_PERCENTAGE_READ_INTERVAL);
	k_timer_start(&clock_tick_timer, K_SECONDS(1), K_SECONDS(1));

	/* Special cases */
        /* Get battery charging status */
	k_sleep(10);
        u32_t res = 0U;
        gpio_pin_read(charging_dev, 12, &res);
        battery_update_charging_status(res != 1U);
}
/* ********** init function ********** */


/* ********** handler functions ********** */
void battery_percentage_interrupt_handler(struct k_timer *bat)
{
        battery_update_percentage();
}

void battery_callback_charging(struct device *gpiob, struct gpio_callback *cb, u32_t pins)
{
        u32_t res = 0U;
        gpio_pin_read(charging_dev, 12, &res);
        battery_update_charging_status(res != 1U);
	backlight_enable(!res);
}

void clock_callback_tick(struct k_timer *tick)
{
	clock_increment_local_time();
	clock_print_time();
	battery_print_status();
}

/* ********** handler functions ********** */
