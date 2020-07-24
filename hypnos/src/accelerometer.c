/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <drivers/sensor.h>
#include <drivers/sensor/bma421.h>
#include "accelerometer.h"
#include "log.h"
#include "gfx.h"

/* ********** ********** DEFINES ********** ********** ********** */

/* ********** **********  ********** ********** ********** */

/* ********** ********** VARIABLES AND STRUCTS ********** ********** */
static struct device *accel_dev;

struct sensor_value accel_data[3];
struct sensor_value step_count;
struct sensor_value temperature;

static char step_label_str[32];
static char temperature_label_str[32];
/* ********** ********** ********** ********** ********** ********** */

/* ********** ********** FUNCTIONS ********** ********** */
void accelerometer_init()
{
	accel_dev = device_get_binding("bma421");
	if (!accel_dev) {
		LOG_ERR("Could not get BMA421 binding");
	}
	LOG_DBG("Accelerometer init: Done");
}

void accelerometer_show_health_data()
{
	if (sensor_sample_fetch(accel_dev)) {
		LOG_ERR("sensor_sample_fetch failed\n");
	} else {
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel_data);
		LOG_DBG("sensor_channel_get accel.X %d.%d", accel_data[0].val1, accel_data[0].val2);
		LOG_DBG("sensor_channel_get accel.Y %d.%d", accel_data[1].val1, accel_data[1].val2);
		LOG_DBG("sensor_channel_get accel.Z %d.%d", accel_data[2].val1, accel_data[2].val2);

		sensor_channel_get(accel_dev, BMA421_CHAN_STEP_COUNTER, &step_count);
		LOG_DBG("sensor_channel_get step count %d", step_count.val1);

		sensor_channel_get(accel_dev, SENSOR_CHAN_DIE_TEMP, &temperature);
		LOG_DBG("sensor_channel_get temperature %d", temperature.val1);
	}

	snprintf(step_label_str, 32, "%d steps", step_count.val1);
	snprintf(temperature_label_str, 32, "%d'C", temperature.val1);

	gfx_step_counter_set_label(step_label_str);
	gfx_temperature_set_label(temperature_label_str);
}
/* ********** ********** ********** ********** ********** */
