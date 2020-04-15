/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <lvgl.h>
#include "clock.h"
#include "battery.h"
#include "log.h"

/* ********** ********** VARIABLES AND STRUCTS ********** ********** */
static time_t local_time;
static char clock_label_str[32];
static char date_label_str[32];
static lv_obj_t *clock_label;
static lv_obj_t *date_label;

static struct tm ti = {
	.tm_sec = 0,
	.tm_min = 0,
	.tm_hour = 0,
	.tm_mday = 0,
	.tm_mon = 0,
	.tm_year = 0,
	.tm_wday = 0,
	.tm_isdst = 0,
};
/* ********** ********** ********** ********** ********** ********** */

/* ********** ********** FUNCTIONS ********** ********** */
void clock_str_to_local_time(const char *str, struct tm *ti)
{
        /* Date and time format: 2020-04-04T20:48:11+02:00 */
	if (sscanf(str, "%d-%d-%dT%d:%d:%d+%d", &ti->tm_year, &ti->tm_mon,
		   &ti->tm_mday, &ti->tm_hour, &ti->tm_min, &ti->tm_sec,
		   &ti->tm_isdst) != 7) {
		LOG_ERR("Failed to parse time of build!");
	}
	ti->tm_year-=1900;
	ti->tm_mon-=1;
	local_time = mktime(ti);
}

void clock_init()
{
	/* Set time to time of build */
	clock_str_to_local_time(TIME_OF_BUILD, &ti);
	LOG_DBG("Time set to time of build");
	LOG_DBG("Clock init: Done");
}

struct tm *clock_get_time()
{
	return &ti;
}

char *clock_get_local_time()
{
	return ctime(&local_time);
}

/* Called by event handler */
void clock_increment_local_time()
{
	local_time++;
}

void clock_print_time()
{
	char wday[4];
	char mon[4];

	if (sscanf(clock_get_local_time(), "%s %s", wday, mon) != 2) {
		LOG_ERR("Failed to print time!");
	}
	LOG_INF("%s %d %s", log_strdup(wday), localtime(&local_time)->tm_mday,
		log_strdup(mon));
	LOG_INF("%02d:%02d", localtime(&local_time)->tm_hour,
		localtime(&local_time)->tm_min);
	sprintf(clock_label_str, "%02d:%02d", localtime(&local_time)->tm_hour,
		localtime(&local_time)->tm_min);
	sprintf(date_label_str, "%s %d %s", wday,
		localtime(&local_time)->tm_mday, mon);

	/* Make the hours:minutes separator blink to represent seconds */
	if (local_time % 2) {
		clock_label_str[2] = ' ';
	}
	lv_label_set_text(clock_label, clock_label_str);
	lv_label_set_text(date_label, date_label_str);
}

void clock_gfx_init()
{
	char wday[4];
	char mon[4];

	if (sscanf(clock_get_local_time(), "%s %s", wday, mon) != 2) {
		LOG_ERR("Failed to print time!");
	}
	sprintf(clock_label_str, "%02d:%02d", localtime(&local_time)->tm_hour,
		localtime(&local_time)->tm_min);
	sprintf(date_label_str, "%s %d %s", wday,
		localtime(&local_time)->tm_mday, mon);

	clock_label = lv_label_create(lv_scr_act(), NULL);
	date_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(clock_label, clock_label_str);
	lv_label_set_text(date_label, date_label_str);
	lv_obj_align(clock_label, NULL, LV_ALIGN_CENTER, 0, -10);
	lv_obj_align(date_label, clock_label, LV_ALIGN_CENTER, 0, 26);
}
/* ********** ********** ********** ********** ********** */
