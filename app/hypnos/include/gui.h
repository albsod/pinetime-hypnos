/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GUI__H
#define GUI__H

#include <drivers/sensor.h>

enum screen {
	DIGITAL_WATCH,
	ANALOG_WATCH,
	INFO
};

void gui_handle_touch_event(const struct device *, enum cst816s_gesture);
int gui_handle_button_event(void);
void gui_update_clock_and_battery(void);
void gui_handle_cts_sync_event(void);

#endif /* GUI__H */
