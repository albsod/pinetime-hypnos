/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/gpio.h>

void init_event_handler();
void battery_percentage_interrupt_handler(struct k_timer*);
void battery_callback_charging(struct device*, struct gpio_callback*, u32_t);
