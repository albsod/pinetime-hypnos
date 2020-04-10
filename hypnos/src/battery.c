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
#include <adc.h>
#include <stdbool.h>
#include <lvgl.h>
#include "battery.h"

#define CHANNEL_ID 7
#define RESOLUTION 12
#define DIVIDER 2
#define COUNT_DOWN 5000

/* ********** ********** VARIABLES ********** ********** */
static struct device* percentage_dev;
static s16_t data[1];
static uint32_t battery_percentage;
static bool charging;
static char battery_label_str[32];
static lv_obj_t *battery_label;
/* ********** ********** ********** ********** ********** */

/* ********** ********** STRUCTS ********** **********  */

struct battery_level_point {
	/** Remaining life at #lvl_mV. */
	u16_t lvl_pptt;

	/** Battery voltage at #lvl_pptt remaining life. */
	u16_t lvl_mV;
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
void battery_status_init()
{
        printk("Init battery status: Start\n");
        percentage_dev = device_get_binding("ADC_0");
        if (percentage_dev == NULL) {
                printk("failed to get binding for device\n");
        }

        if (adc_channel_setup(percentage_dev, &m_1st_channel_cfg) < 0) {
                printk("failed to setup channel for adc\n");
        }

	battery_update_percentage();
        printk("Init battery status: Done\n");
}

void battery_update_percentage()
{
	adc_read(percentage_dev, &sequence);
	uint32_t mv = battery_raw_to_mv(data[0]);
	battery_percentage = battery_mv_to_ppt(mv)/100;
}

void battery_update_charging_status(bool value)
{
	charging = value;
}

uint8_t battery_get_percentage()
{
        return battery_percentage;
}

bool battery_get_charging_status()
{
        return charging;
}

uint32_t battery_raw_to_mv(s16_t raw)
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

void battery_print_status()
{
	printk("Battery status: ");
	printk("%u %% ", battery_get_percentage());
	if (charging) {
		printk("(charging)\n");
		sprintf(battery_label_str, "BAT: %u %% (charging)", battery_get_percentage());
	} else {
		printk("(discharging)\n");
		sprintf(battery_label_str, "BAT: %u %% (discharging)", battery_get_percentage());
	}

	lv_label_set_text(battery_label, battery_label_str);
}

void battery_gfx_init()
{
	sprintf(battery_label_str, "%u %%", battery_get_percentage());

	battery_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(battery_label, battery_label_str);
	lv_obj_align(battery_label, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);
}

/* ********** ********** ********** ********** ********** */
