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
	case SLIDE_UP ... SLIDE_RIGHT:
		if (sc == WATCH) {
			gfx_show_info();
			sc = INFO;
			break;
		}
		sc = WATCH;
		/* Fallthough */
	default:
		clock_increment_local_time();
		clock_show_time();
		battery_show_status();
		gfx_show_watch();
	}
	gfx_update();
}

int gui_handle_button_event(void)
{
	clock_increment_local_time();
	clock_show_time();
	battery_show_status();
	gfx_update();

	return 1;
}
