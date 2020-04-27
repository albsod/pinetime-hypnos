/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <lvgl.h>

/* ********** defines ********** */
#define LV_IMG_DECLARE(var_name) extern const lv_img_dsc_t var_name;
LV_IMG_DECLARE(evelyn);
#define BAT_LABEL_MARGIN 3
/* ********** ******* ********** */

/* ********** variables ********** */
static lv_obj_t *battery_label;
static lv_obj_t *bt_label;
static lv_obj_t *time_label;
static lv_obj_t *date_label;
/* ********** ********* ********** */

void gfx_init(void)
{
	/* Text color */
	lv_style_plain.text.color = LV_COLOR_WHITE;

	/* Background image Night and Sleep (public domain)
	 * See https://en.wikipedia.org/wiki/Night_and_Sleep
	 */
	lv_obj_t * img_bin = lv_img_create(lv_scr_act(), NULL);
	lv_img_set_src(img_bin, &evelyn);

	/* Battery label */
	battery_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_style(battery_label, LV_LABEL_STYLE_MAIN, &lv_style_plain);
	lv_label_set_text(battery_label, "");

	/* Bluetooth label */
	bt_label = lv_label_create(lv_scr_act(), NULL);
	lv_obj_align(bt_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);
	lv_label_set_text(bt_label, "");

	/* Time label */
	time_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_style(time_label, LV_LABEL_STYLE_MAIN, &lv_style_plain);
	lv_label_set_text(time_label, "00:00");
	lv_obj_align(time_label, NULL, LV_ALIGN_CENTER, 0, -10);

	/* Date label */
	date_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_style(date_label, LV_LABEL_STYLE_MAIN, &lv_style_plain);
	lv_label_set_text(date_label, "Mon 10 Jan");
	lv_obj_align(date_label, time_label, LV_ALIGN_CENTER, 0, 26);
}

void gfx_time_set_label(char *str)
{
	lv_label_set_text(time_label, str);
}

void gfx_date_set_label(char *str)
{
	lv_label_set_text(date_label, str);
}

void gfx_bt_set_label(int symbol)
{
	if (symbol) {
		lv_label_set_text(bt_label, LV_SYMBOL_BLUETOOTH);
	} else {
		lv_label_set_text(bt_label, "");
	}
}

void gfx_battery_set_label(int symbol)
{
	switch (symbol) {
	case 5:
		lv_label_set_text(battery_label, LV_SYMBOL_CHARGE);
		lv_obj_align(battery_label, NULL, LV_ALIGN_IN_TOP_RIGHT, -BAT_LABEL_MARGIN, BAT_LABEL_MARGIN);
		break;
	case 4:
		lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_FULL);
		break;
	case 3:
		lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_3);
		break;
	case 2:
		lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_2);
		break;
	case 1:
		lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_1);
		break;
	default:
		lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_EMPTY);
	}
	lv_obj_align(battery_label, NULL, LV_ALIGN_IN_TOP_RIGHT, -BAT_LABEL_MARGIN, 0);
}
