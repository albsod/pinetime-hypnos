/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdbool.h>
#include "battery.h"
#include "event_handler.h"

void main(void)
{
	printk("Welcome to Hypnos\n");
	battery_status_init();
	init_event_handler();

	while (true) {
		printk("proc: %u %% \n", battery_get_percentage());
		printk("charging: %d %% \n", battery_get_charging_status());
		k_sleep(1000);
	}
}
