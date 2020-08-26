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

void gui_handle_touch_event(struct device *touch_dev, enum cst816s_gesture gesture)
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
	if (bt_toggle_is_locked()) {
		return 0;
	} else if (bt_mode()) {
		gfx_bt_set_label(BT_ADVERTISING_OFF);
		bt_off();
	} else {
		gfx_bt_set_label(BT_ADVERTISING_ON);
		bt_on();
	}
	gfx_update();

	return 1;
}
