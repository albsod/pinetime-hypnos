/*
 * PineTime Hypnos: Smartwatch firmware for the PineTime dev kit
 * Copyright (c) 2020 Endian Technologies AB
 *
 * This is free software with ABSOLUTELY NO WARRANTY.
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include "backlight.h"
#include "battery.h"
#include "bt.h"
#include "clock.h"
#include "cts_sync.h"
#include "display.h"
#include "event_handler.h"
#include "gfx.h"
#include "log.h"

/* ******** thread function declarations and defines ******** */
void main_thread(void);
void bt_thread(void);

#define STACKSIZE 1024
#define PRIORITY 7

K_THREAD_DEFINE(bt_id, STACKSIZE, bt_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);
K_THREAD_DEFINE(main_id, STACKSIZE, main_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);

#define SLEEP_TIME K_MSEC(10)
/* ******** thread function declarations and defines ******** */

void main(void)
{
	LOG_INF("Welcome to PineTime Hypnos!");
	LOG_INF("This is free software with ABSOLUTELY NO WARRANTY.");

	gfx_init();
	clock_init();
	battery_init();
	display_init();
	event_handler_init();
	gfx_update();
	backlight_init();

	while (true) {
		if (bt_mode()) {
			bt_thread();
		} else {
			main_thread();
		}
	}
}

/* ************* thread functions ***************/
void main_thread(void)
{
	while (true) {
	await_disable_bt:
		bt_await_off();
		bt_adv_stop();
		while (true) {
			k_sleep(SLEEP_TIME);
			if (bt_mode()) {
				gfx_bt_set_label(1);
				gfx_update();
				goto await_disable_bt;
			}
			k_cpu_idle();
		}
	}
}

void bt_thread(void)
{
	while (true) {
	await_enable_bt:
		bt_await_on();
		LOG_INF("Entering bluetooth mode...");
		if (bt_is_initialized()) {
			bt_adv_start();
		} else {
			bt_init();
			k_sleep(SLEEP_TIME);
		}
		cts_sync_loop();
		while (true) {
			k_sleep(SLEEP_TIME);
			if (!bt_mode()) {
				LOG_INF("Exiting bluetooth mode...");
				gfx_bt_set_label(0);
				gfx_update();
				goto await_enable_bt;
			}
			k_cpu_idle();
		}
	}
}
/* ***************************************/
