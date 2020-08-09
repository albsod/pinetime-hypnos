/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <drivers/sensor.h>
#include "accelerometer.h"
#include "log.h"
#include "gfx.h"

/* ********** ********** DEFINES ********** ********** ********** */

/* ********** **********  ********** ********** ********** */

/* ********** ********** VARIABLES AND STRUCTS ********** ********** */
static struct device *accel_dev;

struct sensor_value accel_data[3];
struct sensor_value temperature;

struct sensor_trigger trig;
/* ********** ********** ********** ********** ********** ********** */

/* ********** ********** FUNCTIONS ********** ********** */

static void trigger_handler(struct device *accel_dev, struct sensor_trigger *trigger)
{
	LOG_DBG("Accel:trigger_handler type %d", trigger->type);

	if (trigger->type != SENSOR_TRIG_DATA_READY) {
		LOG_ERR("trigger handler: unknown trigger type (%d).", trigger->type);
		return;
	}

	if (sensor_sample_fetch(accel_dev) < 0) {
		LOG_ERR("Accel sample update error.");
	}

	sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel_data);
}

void accelerometer_init()
{
	accel_dev = device_get_binding("bma421");
	if (!accel_dev) {
		LOG_ERR("Could not get BMA421 binding");
	}

	trig.chan = SENSOR_CHAN_ACCEL_XYZ; /* does not matter */
	trig.type = SENSOR_TRIG_DATA_READY;
	if (sensor_trigger_set(accel_dev, &trig, trigger_handler)) {
		LOG_ERR("Trigger set Error");
	}

	LOG_DBG("Accelerometer init: Done");
}

void accelerometer_log_data()
{
	if (sensor_sample_fetch(accel_dev)) {
		LOG_ERR("sensor_sample_fetch failed\n");
	} else {
		// sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel_data);
		LOG_INF("sensor_channel_get accel.X %d.%d", accel_data[0].val1, accel_data[0].val2);
		LOG_INF("sensor_channel_get accel.Y %d.%d", accel_data[1].val1, accel_data[1].val2);
		LOG_INF("sensor_channel_get accel.Z %d.%d", accel_data[2].val1, accel_data[2].val2);

		sensor_channel_get(accel_dev, SENSOR_CHAN_DIE_TEMP, &temperature);
		LOG_INF("sensor_channel_get temperature %d", temperature.val1);
	}
}
/* ********** ********** ********** ********** ********** */
