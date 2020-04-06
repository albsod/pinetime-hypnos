/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2020 Endian Technologies AB
 */

#include <zephyr.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/printk.h>
#include "clock.h"
#include "battery.h"

static time_t local_time;
static struct k_timer tick;

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

#define STR_MAX_LEN 48 // A bigger value crashes the system!

static char time_str[STR_MAX_LEN];


/* 2020-04-04T20:48:11+02:00 */
void str_to_time(const char *str, struct tm *ti)
{		
	if (sscanf(str, "%d-%d-%dT%d:%d:%d+%d", &ti->tm_year, &ti->tm_mon,
		   &ti->tm_mday, &ti->tm_hour, &ti->tm_min, &ti->tm_sec,
		   &ti->tm_isdst) != 7) {
		printk("Failed to parse time of build.\n");
	}
	ti->tm_year-=1900;
	ti->tm_mon-=1;
	local_time = mktime(ti);
}

void clock_init()
{
	/* Set time to time of build */
	str_to_time(TIME_OF_BUILD, &ti);
	printk("Time set to time of build\n");

	/* Initialize and start the clock timer */
	k_timer_init(&tick, increment_time, NULL);
	k_timer_start(&tick, K_SECONDS(1), K_SECONDS(1));
}

void increment_time(struct k_timer *tick)
{
	local_time++;
	//strftime(time_str, STR_MAX_LEN, "%Y-%m-%d %H:%M:%S was a %A in %B.", time_get());
	//printk("%s\n", time_str);
	//printk("\n%s\n", time_get_local());
	print_time();
	printk("\nBattery status: ");
	printk("%u %% ", battery_get_percentage());
	if (battery_get_charging_status()) {
		printk("(charging)");
	} else {
		printk("(discharging)");
	}
	printk("\n********************************\n");
}

char *time_get_local()
{
	return ctime(&local_time);
}

struct tm *time_get()
{
	return &ti;
}

void print_time()
{
	char wday[15];
	char mon[15];
	
	if (sscanf(time_get_local(), "%s %s", wday, mon) != 2) {
		printk("Failed to print time.\n");
	}
	printf("\n");
	printf("  %d:%d:%d | ", localtime(&local_time)->tm_hour,
	       localtime(&local_time)->tm_min,
	       localtime(&local_time)->tm_sec);
	printf("%s %d %s\n", wday, localtime(&local_time)->tm_mday, mon);
}
