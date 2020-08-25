/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <lvgl.h>
#include "gfx.h"
#include "log.h"

/* ********** Macros and constants ********** */
LV_FONT_DECLARE(rubik_regular_68);
LV_FONT_DECLARE(rubik_regular_34);

#define BAT_LABEL_MARGIN 3
/* ********** ******* ********** */

/* ********** Variables ********** */
static lv_obj_t *battery_label;
static lv_obj_t *bt_label;
static lv_obj_t *time_label;
static lv_obj_t *date_label;
static lv_style_t style;
static lv_style_t style_time;
static lv_style_t style_date;

/* ********** Functions ********** */
void gfx_init(void)
{
	/* Create styles for time, date and the rest */
	lv_style_copy(&style, &lv_style_plain);
	lv_style_copy(&style_time, &lv_style_plain);
	lv_style_copy(&style_date, &lv_style_plain);

	/* Default style */
	style.body.main_color = LV_COLOR_BLACK;
	style.body.grad_color = LV_COLOR_BLACK;
	style.text.color = LV_COLOR_WHITE;
	style.text.font = &lv_font_roboto_22;
	lv_obj_set_style(lv_scr_act(), &style);

	/* Battery label */
	battery_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_style(battery_label, LV_LABEL_STYLE_MAIN, &style);
	lv_label_set_text(battery_label, "");

	/* Bluetooth label */
	bt_label = lv_label_create(lv_scr_act(), NULL);
	lv_obj_align(bt_label, NULL, LV_ALIGN_IN_TOP_LEFT, 6, 4);
	lv_label_set_style(bt_label, LV_LABEL_STYLE_MAIN, &style);
	lv_label_set_text(bt_label, "");

	/* Time label and style */
	style_time.body.main_color = LV_COLOR_BLACK;
	style_time.body.grad_color = LV_COLOR_BLACK;
	style_time.text.font = &rubik_regular_68;

	style_time.text.color = LV_COLOR_WHITE;
	style_time.text.color = LV_COLOR_WHITE;
	lv_obj_set_style(lv_scr_act(), &style_time);
	time_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_style(time_label, LV_LABEL_STYLE_MAIN, &style_time);
	lv_label_set_text(time_label, "00:00");
	lv_obj_align(time_label, NULL, LV_ALIGN_CENTER, 0, -25);

	/* Date label and style */
	style_date.body.main_color = LV_COLOR_BLACK;
	style_date.body.grad_color = LV_COLOR_BLACK;
	style_date.text.font = &rubik_regular_34;
	style_date.text.color = LV_COLOR_YELLOW;
	lv_obj_set_style(lv_scr_act(), &style_date);
	date_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_style(date_label, LV_LABEL_STYLE_MAIN, &style_date);
	lv_label_set_text(date_label, "Mon 10 Jan");
	lv_obj_align(date_label, NULL, LV_ALIGN_CENTER, 0, 30);
	LOG_DBG("Graphics init: Done");
}

void gfx_update(void)
{
	lv_task_handler();
}

void gfx_time_set_label(char *str)
{
	lv_label_set_text(time_label, str);
	lv_obj_align(time_label, NULL, LV_ALIGN_CENTER, 0, -25);
}

void gfx_date_set_label(char *str)
{
	lv_label_set_text(date_label, str);
	lv_obj_align(date_label, NULL, LV_ALIGN_CENTER, 0, 30);
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

void gfx_gui_set_screen(struct gui *g, enum screen s)
{
	g->sc = s;
}

void gfx_show_info(void)
{
	lv_label_set_text(date_label, "Hypnos");
	style_date.text.color = LV_COLOR_RED;
	lv_obj_align(date_label, NULL, LV_ALIGN_CENTER, 0, 30);
}

void gfx_show_date(void)
{
	style_date.text.color = LV_COLOR_YELLOW;
}
