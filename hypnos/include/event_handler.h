/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#ifndef EVENT_HANDLER__H
#define EVENT_HANDLER__H

#include <drivers/gpio.h>
#include <drivers/sensor.h>

void event_handler_init(void);
void backlight_off_interrupt_handler(struct k_timer *);
void battery_percentage_interrupt_handler(struct k_timer *);
void battery_callback_charging(struct device*, struct gpio_callback *, u32_t);
void clock_callback_tick(struct k_timer *);
void button_callback(struct device *, struct gpio_callback *, u32_t);
void touch_callback_tap(struct device *, struct sensor_trigger *);

#endif /* EVENT_HANDLER */
