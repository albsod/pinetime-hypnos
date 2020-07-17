/*
 * PineTime Hypnos: Smartwatch firmware for the PineTime dev kit
 * Copyright (c) 2020 Endian Technologies AB
 *
 * This is free software with ABSOLUTELY NO WARRANTY.
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* ******** Thread prototypes, constants and macros ******** */
void main_thread(void);
void bt_thread(void);

#define STACKSIZE 1024
#define PRIORITY 7

K_THREAD_DEFINE(bt_id, STACKSIZE, bt_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);
K_THREAD_DEFINE(main_id, STACKSIZE, main_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);

static struct device *accel_dev;

struct sensor_value accel_data[5];

/* ******** Functions ******** */
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

	accel_dev = device_get_binding("bma421");
	if (!accel_dev) {
		LOG_ERR("Could not get BMA421 binding");
	}

	if (sensor_sample_fetch(accel_dev)) {
		LOG_DBG("sensor_sample_fetch failed\n");
	} else {
		LOG_DBG("sensor_sample_fetch ok");
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &accel_data[0]);
		LOG_DBG("sensor_channel_get accel.X %d", accel_data[0].val1);

		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &accel_data[1]);
		LOG_DBG("sensor_channel_get accel.Y %d", accel_data[1].val1);

		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &accel_data[2]);
		LOG_DBG("sensor_channel_get accel.Z %d", accel_data[2].val1);

		sensor_channel_get(accel_dev, SENSOR_CHAN_DIE_TEMP, &accel_data[3]);
		LOG_DBG("sensor_channel_get temperature %d", accel_data[3].val1);	
	}

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
		// if (bt_is_initialized()) {
		// 	bt_adv_start();
		// } else {
		// 	bt_init();
		// 	cts_sync_init();
		// }
		// cts_sync_enable(true);

		if (sensor_sample_fetch(accel_dev)) {
			LOG_DBG("sensor_sample_fetch failed\n");
		} else {
			LOG_DBG("sensor_sample_fetch ok");
			sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &accel_data[0]);
			LOG_DBG("sensor_channel_get accel.X %d", accel_data[0].val1);

			sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &accel_data[1]);
			LOG_DBG("sensor_channel_get accel.Y %d", accel_data[1].val1);

			sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &accel_data[2]);
			LOG_DBG("sensor_channel_get accel.Z %d", accel_data[2].val1);

			sensor_channel_get(accel_dev, SENSOR_CHAN_DIE_TEMP, &accel_data[3]);
			LOG_DBG("sensor_channel_get temperature %d", accel_data[3].val1);

			sensor_channel_get(accel_dev, SENSOR_ATTR_PRIV_START, &accel_data[3]);
			LOG_DBG("sensor_channel_get temperature %d", accel_data[3].val1);
		}

		k_cpu_idle();
	}
}
