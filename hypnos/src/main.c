/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdbool.h>
#include <lvgl.h>
#include "backlight.h"
#include "battery.h"
#include "clock.h"
#include "display.h"
#include "event_handler.h"

void main(void)
{
	printk("Welcome to Hypnos!\n");

	/* Create welcome screen */
	// TODO: Move this out of main
	lv_obj_t *hypnos_label;
	hypnos_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(hypnos_label, "<<Pinetime Hypnos>>");
	lv_obj_align(hypnos_label, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);
	lv_task_handler();

	clock_init();
	clock_gfx_init();
	battery_status_init();
	battery_gfx_init();
	display_init();
	backlight_init();
	event_handler_init();
	
	display_disable_blanking();

	while (true) {
		k_cpu_idle();
		lv_task_handler();
	}
}
