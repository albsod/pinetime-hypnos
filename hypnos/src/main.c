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

	lv_task_handler();
	display_disable_blanking();
	bt_on();
	
	while (true) {
		if (bt_mode()) {
			bt_thread();
		} else {
			main_thread();
		}
	}
}
