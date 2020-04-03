/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdbool.h>
#include "battery.h"

void main(void)
{
	printk("Welcome to Hypnos\n");
	battery_status_init();

	while (true) {
		printk("Battery status: ");
		printk("proc: %u %% ", battery_get_percentage());
		printk("charging: %d\n", battery_get_charging_status());
		k_sleep(500);
	}
}
