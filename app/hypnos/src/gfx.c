/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <lvgl.h>
#include "gfx.h"
#include "log.h"
#include "version.h"
#include <string.h>

/* ********** Macros and constants ********** */
LV_FONT_DECLARE(rubik_regular_68);
LV_FONT_DECLARE(rubik_regular_34);

#define BAT_LABEL_MARGIN 3
#define TIME_LABEL_VALIGNMENT -25
#define DATE_LABEL_VALIGNMENT 30
/* ********** ******* ********** */

/* ********** Variables ********** */
static lv_obj_t *battery_label;
static lv_obj_t *bt_label;
static lv_obj_t *time_label;
static lv_obj_t *date_label;
static lv_obj_t *info_label;
static lv_style_t style;
static lv_style_t style_time;
static lv_style_t style_date;

/* ********** Functions ********** */
void gfx_init(void)
{
	/* Create styles for time, date and the rest */
	lv_style_init(&style);
	lv_style_init(&style_time);
	lv_style_init(&style_date);

	/* Default style */
	lv_style_set_text_color(&style, LV_STATE_DEFAULT, LV_COLOR_WHITE);
	lv_style_set_text_font(&style, LV_STATE_DEFAULT, &lv_font_montserrat_22);

	/* Battery label */
	battery_label = lv_label_create(lv_scr_act(), NULL);
	lv_obj_add_style(battery_label, LV_LABEL_PART_MAIN, &style);
	lv_label_set_text(battery_label, "");

	/* Bluetooth label */
	bt_label = lv_label_create(lv_scr_act(), NULL);
	lv_obj_align(bt_label, NULL, LV_ALIGN_IN_TOP_LEFT, 6, 4);
	lv_obj_add_style(bt_label, LV_LABEL_PART_MAIN, &style);
	lv_label_set_text(bt_label, LV_SYMBOL_WIFI);

	/* Time label and style */
	lv_style_set_text_font(&style_time, LV_STATE_DEFAULT, &rubik_regular_68);
	lv_style_set_text_color(&style_time, LV_STATE_DEFAULT, LV_COLOR_WHITE);
	time_label = lv_label_create(lv_scr_act(), NULL);
	lv_obj_add_style(time_label, LV_LABEL_PART_MAIN, &style_time);
	lv_label_set_text(time_label, "00:00");
	lv_obj_align(time_label, NULL, LV_ALIGN_CENTER, 0, TIME_LABEL_VALIGNMENT);

	/* Date label and style */
	lv_style_set_text_color(&style_date, LV_STATE_DEFAULT, LV_COLOR_YELLOW);
	lv_style_set_text_font(&style_date, LV_STATE_DEFAULT, &rubik_regular_34);
	date_label = lv_label_create(lv_scr_act(), NULL);
	lv_obj_add_style(date_label, LV_LABEL_PART_MAIN, &style_date);
	lv_label_set_text(date_label, "Mon 10 Jan");
	lv_obj_align(date_label, NULL, LV_ALIGN_CENTER, 0, DATE_LABEL_VALIGNMENT);
	LOG_DBG("Graphics init: Done");

	/* Info label */
	info_label = lv_label_create(lv_scr_act(), NULL);
	lv_obj_add_style(info_label, LV_LABEL_PART_MAIN, &style);
	if (strlen(FW_VERSION) < 10) {
		lv_label_set_text(info_label, "Hypnos " FW_VERSION "\n\n"
				  "This is Free Software" "\n"
				  "without any warranty." "\n\n"
				  "https://github.com/"   "\n"
				  "endian-albin/"         "\n"
				  "pinetime-hypnos");
	} else {
		lv_label_set_text(info_label, "Hypnos"    "\n"
				  FW_VERSION              "\n\n"
				  "This is Free Software" "\n"
				  "without any warranty." "\n"
				  "https://github.com/"   "\n"
				  "endian-albin/"         "\n"
				  "pinetime-hypnos");
	}
	lv_obj_set_hidden(info_label, true);
}

void gfx_update(void)
{
	lv_task_handler();
}

void gfx_time_set_label(char *str)
{
	lv_label_set_text(time_label, str);
	lv_obj_align(time_label, NULL, LV_ALIGN_CENTER, 0, TIME_LABEL_VALIGNMENT);
}

void gfx_date_set_label(char *str)
{
	lv_label_set_text(date_label, str);
	lv_obj_align(date_label, NULL, LV_ALIGN_CENTER, 0, DATE_LABEL_VALIGNMENT);
}

void gfx_bt_set_label(enum bt_symbol s)
{
	switch (s) {
	case BT_ADVERTISING_ON:
		lv_label_set_text(bt_label, LV_SYMBOL_WIFI);
		break;
	case BT_CONNECTED:
		lv_label_set_text(bt_label, LV_SYMBOL_BLUETOOTH);
		break;
	default:
		lv_label_set_text(bt_label, "");
	}
}

void gfx_battery_set_label(enum battery_symbol s)
{
	switch (s) {
	case BAT_CHARGE:
		lv_label_set_text(battery_label, LV_SYMBOL_CHARGE);
		lv_obj_align(battery_label, NULL, LV_ALIGN_IN_TOP_RIGHT, -BAT_LABEL_MARGIN, BAT_LABEL_MARGIN);
		break;
	case BAT_FULL:
		lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_FULL);
		break;
	case BAT_3:
		lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_3);
		break;
	case BAT_2:
		lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_2);
		break;
	case BAT_1:
		lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_1);
		break;
	default:
		lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_EMPTY);
	}
	lv_obj_align(battery_label, NULL, LV_ALIGN_IN_TOP_RIGHT, -BAT_LABEL_MARGIN, 0);
}

void gfx_show_info(void)
{
	lv_obj_set_hidden(time_label, true);
	lv_obj_set_hidden(date_label, true);
	lv_obj_set_hidden(bt_label, true);
	lv_obj_set_hidden(info_label, false);
}

void gfx_show_watch(void)
{
	lv_obj_set_hidden(time_label, false);
	lv_obj_set_hidden(date_label, false);
	lv_obj_set_hidden(bt_label, false);
	lv_obj_set_hidden(info_label, true);
}
