/*
 * PineTime Hypnos: Smartwatch firmware for the PineTime dev kit
 * Copyright (c) 2020 Endian Technologies AB
 *
 * This is free software with ABSOLUTELY NO WARRANTY.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include "accelerometer.h"
#include "backlight.h"
#include "battery.h"
#include "bt.h"
#include "clock.h"
#include "cts_sync.h"
#include "display.h"
#include "event_handler.h"
#include "gfx.h"
#include "log.h"

/* ******** Thread prototypes, constants and macros ******** */
void main_thread(void);
void bt_thread(void);

#define STACKSIZE 1024
#define PRIORITY 7

K_THREAD_DEFINE(bt_id, STACKSIZE, bt_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);
K_THREAD_DEFINE(main_id, STACKSIZE, main_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);

/* ******** Functions ******** */
void main(void)
{
	LOG_INF("Welcome to PineTime Hypnos!");
	LOG_INF("This is free software with ABSOLUTELY NO WARRANTY.");

	gfx_init();
	clock_init();
	battery_init();
	accelerometer_init();
	display_init();
	event_handler_init();
	gfx_update();
	backlight_init();
}

void main_thread(void)
{
	while (true) {
		bt_await_off();
		LOG_INF("Disabling BLE advertising...");
		bt_adv_stop();
		cts_sync_enable(false);
		k_cpu_idle();
	}
}

void bt_thread(void)
{
	while (true) {
		bt_await_on();
		LOG_INF("Enabling BLE advertising...");
		if (bt_is_initialized()) {
			bt_adv_start();
		} else {
			bt_init();
			cts_sync_init();
		}
		cts_sync_enable(true);
		k_cpu_idle();
	}
}
