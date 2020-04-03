/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdbool.h>

void main(void)
{
	while (true) {
		printk("Welcome to Hypnos\n");
		k_sleep(500);
	}
}
