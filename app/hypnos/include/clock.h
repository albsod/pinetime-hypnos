/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CLOCK__H
#define CLOCK__H

#include <cts_sync.h>

/* Stringify build time included by cmake */
#define _xstr(s) _str(s)
#define _str(s) #s
#define TIME_OF_BUILD _xstr(CURRENT_TIME_OF_BUILD)

void clock_init(void);
void clock_increment_local_time(void);
void clock_sync_time(cts_datetime_t *cts);
void clock_show_time(void);

#endif /* CLOCK__H */
