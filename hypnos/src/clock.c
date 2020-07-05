/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "clock.h"
#include "battery.h"
#include "log.h"
#include "cts_sync.h"
#include "bt.h"
#include "gfx.h"
#include "event_handler.h"

/* ********** ********** VARIABLES AND STRUCTS ********** ********** */
static time_t local_time;
static char time_label_str[32];
static char date_label_str[32];
static u64_t uptime_ms;
static u64_t last_uptime_ms;
static u64_t elapsed_time_ms;

static struct tm ti = {
	.tm_sec = 0,
	.tm_min = 0,
	.tm_hour = 0,
	.tm_mday = 0,
	.tm_mon = 0,
	.tm_year = 0,
	.tm_wday = 0,
};
/* ********** ********** ********** ********** ********** ********** */

/* ********** ********** FUNCTIONS ********** ********** */
void clock_str_to_local_time(const char *str, struct tm *t)
{
        /* Date and time format: 2020-04-04T20:48:11 */
	if (sscanf(str, "%d-%d-%dT%d:%d:%d", &t->tm_year, &t->tm_mon,
		   &t->tm_mday, &t->tm_hour, &t->tm_min, &t->tm_sec) != 6) {
		LOG_ERR("Failed to parse time of build!");
	}
	t->tm_year-=1900;
	t->tm_mon-=1;
	local_time = mktime(t);
}

void clock_init()
{
	/* Set time to time of build */
	clock_str_to_local_time(TIME_OF_BUILD, &ti);
	LOG_DBG("Time set to time of build");
	LOG_DBG("Clock init: Done");
}

/* Called by cts sync */
void clock_sync_time(cts_datetime_t *cts)
{
	ti.tm_year = cts->year -1900;
	ti.tm_mon = cts->month -1;
	ti.tm_mday = cts->day;
	ti.tm_hour = cts->hours;
	ti.tm_min = cts->minutes;
	ti.tm_sec = cts->seconds;

	local_time = mktime(&ti);

	/* Flush the time incrementer */
	uptime_ms = k_uptime_get();
	elapsed_time_ms = uptime_ms - last_uptime_ms;
	last_uptime_ms = uptime_ms;
}

char *clock_get_local_time()
{
	return ctime(&local_time);
}

/* Called by event handler */
void clock_increment_local_time()
{
	uptime_ms = k_uptime_get();
	elapsed_time_ms = uptime_ms - last_uptime_ms;
	last_uptime_ms = uptime_ms;
	local_time += elapsed_time_ms / 1000;
}

void clock_show_time()
{
	char wday[4];
	char mon[4];

	if (sscanf(clock_get_local_time(), "%s %s", wday, mon) != 2) {
		LOG_ERR("Failed to print time!");
	}
	LOG_INF("%s %d %s | %02d:%02d", log_strdup(wday),
		localtime(&local_time)->tm_mday,
		log_strdup(mon), localtime(&local_time)->tm_hour,
		localtime(&local_time)->tm_min);
	sprintf(time_label_str, "%02d:%02d", localtime(&local_time)->tm_hour,
		localtime(&local_time)->tm_min);
	sprintf(date_label_str, "%s %d %s", wday,
		localtime(&local_time)->tm_mday, mon);

	gfx_time_set_label(time_label_str);
	gfx_date_set_label(date_label_str);
}
