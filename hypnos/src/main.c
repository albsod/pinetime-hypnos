/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdbool.h>
#include "battery.h"
#include "event_handler.h"
#include "clock.h"

void main(void)
{
	printk("Welcome to Hypnos!\n");
	battery_status_init();
	clock_init();
	init_event_handler();

	while (true) {
		k_sleep(1);
		k_cpu_idle();
	}
}
