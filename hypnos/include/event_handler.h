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
void backlight_off_isr(struct k_timer *);
void battery_percentage_isr(struct k_timer *);
void battery_charging_isr(struct device*, struct gpio_callback *, u32_t);
void clock_tick_isr(struct k_timer *);
void button_pressed_isr(struct device *, struct gpio_callback *, u32_t);
void touch_tap_isr(struct device *, struct sensor_trigger *);

#endif /* EVENT_HANDLER */
