/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/sensor/cst816s.h>
#include "gfx.h"
#include "gui.h"
#include "clock.h"
#include "battery.h"
#include "bt.h"
#include "log.h"

static enum screen sc;

void gui_handle_touch_event(const struct device *touch_dev, enum cst816s_gesture gesture)
{
	switch (gesture) {
	case SLIDE_UP:
		switch (sc) {
		case DIGITAL_WATCH:
			gfx_show_analog_watch();
			sc = ANALOG_WATCH;
			gui_update_clock_and_battery();
			gfx_update();
			break;
		case ANALOG_WATCH:
			gfx_show_info();
			sc = INFO;
			battery_show_status();
			gfx_update();
			break;
		case INFO:
			gfx_show_digital_watch();
			sc = DIGITAL_WATCH;
			gui_update_clock_and_battery();
			gfx_update();
			break;
		}
	default:
		gui_update_clock_and_battery();
	}
}

void gui_update_clock_and_battery(void)
{
	clock_increment_local_time();
	clock_show_time();
	battery_show_status();
}

int gui_handle_button_event(void)
{
	gui_update_clock_and_battery();
	gfx_update();
	return 1;
}

void gui_handle_cts_sync_event(void)
{
	if (sc == INFO) {
		return;
	}
	clock_show_time();

	if (sc == DIGITAL_WATCH) {
	        gfx_show_digital_watch();
	} else {
	        gfx_show_analog_watch();
	}
	gfx_update();
}
