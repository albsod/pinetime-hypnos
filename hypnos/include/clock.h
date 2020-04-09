/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#ifndef CLOCK__H
#define CLOCK__H

#include <time.h>

/* Stringify build time included by cmake */
#define _xstr(s) _str(s)
#define _str(s) #s
#define TIME_OF_BUILD _xstr(CURRENT_TIME_OF_BUILD)

void clock_str_to_local_time(const char *str, struct tm *ti);
void clock_init();
void clock_increment_local_time();
char *clock_get_local_time();
struct tm *clock_get_time();
void clock_print_time();
void clock_gfx_init();

#endif /* CLOCK__H */
