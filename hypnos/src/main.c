/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include <stdbool.h>
#include "backlight.h"
#include "battery.h"
#include "bt.h"
#include "clock.h"
#include "cts_sync.h"
#include "display.h"
#include "event_handler.h"
#include "gfx.h"
#include "log.h"

void main(void)
{
	LOG_INF("Welcome to Hypnos!");

	gfx_init();
	clock_init();
	battery_status_init();
	display_init();
	backlight_init();
	bt_init();
	event_handler_init();

	display_disable_blanking();

	while (true) {
		if (bt_mode()) {
			clock_sync_time();
			k_sleep(1);
			lv_task_handler();
			k_sleep(1);
		} else {
			k_sleep(1);
			k_cpu_idle();
			lv_task_handler();
		}
	}
}
