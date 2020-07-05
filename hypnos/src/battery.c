/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * battery_mv_to_ppt, battery_level_point, battery_raw_to_mv:
 * Copyright (c) 2020 Nordic Semiconductor
 */

#include <zephyr.h>
#include <stdint.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <syscalls/adc.h>
#include <stdbool.h>
#include "battery.h"
#include "gfx.h"
#include "log.h"

#define CHANNEL_ID 7
#define RESOLUTION 12
#define DIVIDER 2
#define COUNT_DOWN 5000

/* ********** ********** VARIABLES ********** ********** */
static struct device* percentage_dev;
static int16_t data[1];
static uint32_t percentage;
static bool charging;
/* ********** ********** ********** ********** ********** */

/* ********** ********** STRUCTS ********** **********  */
struct battery_level_point {
	/** Remaining life at #lvl_mV. */
	uint16_t lvl_pptt;

	/** Battery voltage at #lvl_pptt remaining life. */
	uint16_t lvl_mV;
};

static const struct battery_level_point lipo[] = {
	{ 10000, 4200 },
	{ 5000, 3660},
	{ 2100, 3600 },
	{ 1000, 3560},
	{ 0, 3100 },
};

static const struct adc_sequence sequence = {
	.channels    = BIT(CHANNEL_ID),
	.buffer      = data,
	.buffer_size = sizeof(data),
	.resolution  = RESOLUTION,
};

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN_1_4,
	.reference        = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 20),
	.channel_id       = CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = 8,
#endif
};

/* ********** ********** ********** ********** ********** */

/* ********** ********** FUNCTIONS ********** ********** */
void battery_init()
{
        percentage_dev = device_get_binding("ADC_0");
        if (percentage_dev == NULL) {
                LOG_ERR("Failed to get binding for ADC_0 device!");
        }

        if (adc_channel_setup(percentage_dev, &m_1st_channel_cfg) < 0) {
                LOG_ERR("Failed to setup channel for adc!");
        }

        LOG_DBG("Battery status init: Done");
}

void battery_update_percentage()
{
	adc_read(percentage_dev, &sequence);
	uint32_t mv = battery_raw_to_mv(data[0]);
	percentage = battery_mv_to_ppt(mv)/100;
}

void battery_update_charging_status(bool value)
{
	charging = value;
}

bool battery_get_charging_status()
{
        return charging;
}

uint32_t battery_raw_to_mv(int16_t raw)
{
	return (DIVIDER*600*(((uint32_t)raw*4*1000) >> RESOLUTION))/1000;
}

uint32_t battery_mv_to_ppt(uint32_t mv)
{
	const struct battery_level_point *pb = lipo;

	if (mv >= pb->lvl_mV) {
		return pb->lvl_pptt;
	}

	while ((pb->lvl_pptt > 0)
	       && (mv < pb->lvl_mV)) {
		++pb;
	}
	if (mv < pb->lvl_mV) {

		return pb->lvl_pptt;
	}

	const struct battery_level_point *pa = pb - 1;

	return pb->lvl_pptt
	       + ((pa->lvl_pptt - pb->lvl_pptt)
		  * (mv - pb->lvl_mV)
		  / (pa->lvl_mV - pb->lvl_mV));
}

void battery_show_status()
{
	battery_update_percentage();
	if (charging) {
		LOG_INF("Battery status: %u%% (charging)",
			percentage);
		gfx_battery_set_label(5);
	} else {
		LOG_INF("Battery status: %u%% (discharging)",
			percentage);

		if (percentage > 89) {
			gfx_battery_set_label(4);
		} else if (percentage > 74) {
			gfx_battery_set_label(3);
		} else if (percentage > 49) {
			gfx_battery_set_label(2);
		} else if (percentage > 24) {
			gfx_battery_set_label(1);
		} else {
			gfx_battery_set_label(0);
		}
	}
}

/* ********** ********** ********** ********** ********** */
