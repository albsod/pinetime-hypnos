/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#ifndef EVENT_HANDLER__H
#define EVENT_HANDLER__H

#include <drivers/gpio.h>

void init_event_handler();
void battery_percentage_interrupt_handler(struct k_timer*);
void battery_callback_charging(struct device*, struct gpio_callback*, u32_t);
void clock_callback_tick(struct k_timer *);

#endif /* EVENT_HANDLER */
