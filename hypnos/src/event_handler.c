/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include "battery.h"
#include "event_handler.h"

/* ********** defines ********** */
#define BAT_PERCENTAGE_READ_INTERVAL K_MINUTES(5)
#define BAT_CHA_PIN 12
/* ********** defines ********** */


/* ********** variables ********** */
static struct k_timer battery_percentage_timer;

static struct device *charging_dev;

static struct gpio_callback charging_cb;
/* ********** variables ********** */

/* ********** init function ********** */
void init_event_handler()
{
        // TODO: Check return values for error handling.

        k_timer_init(&battery_percentage_timer,
                battery_percentage_interrupt_handler, NULL);
        k_timer_start(&battery_percentage_timer, 1000, 1000);
                //BAT_PERCENTAGE_READ_INTERVAL);

        charging_dev = device_get_binding("GPIO_0");
        gpio_pin_configure(charging_dev, BAT_CHA_PIN, GPIO_DIR_IN | GPIO_INT
                | GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE);
        gpio_init_callback(&charging_cb, battery_callback_charging,
                BIT(BAT_CHA_PIN));
        gpio_add_callback(charging_dev, &charging_cb);
        gpio_pin_enable_callback(charging_dev, BAT_CHA_PIN);
}
/* ********** init function ********** */


/* ********** handler functions ********** */
void battery_percentage_interrupt_handler(struct k_timer *timer_id)
{
        battery_update_percentage();
}

void battery_callback_charging(struct device *gpiob, struct gpio_callback *cb, u32_t pins)
{
        ARG_UNUSED(gpiob);
        ARG_UNUSED(cb);
        ARG_UNUSED(pins);
        u32_t res = 0U;
        gpio_pin_read(charging_dev, 12, &res);
        battery_update_charging_status(res != 1U);
}

/* ********** handler functions ********** */
