/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#ifndef GFX__H
#define GFX__H

void gfx_init(void);
void gfx_battery_set_label(int);
void gfx_bt_set_label(int);
void gfx_time_set_label(char *);
void gfx_date_set_label(char *);
void lv_task_handler(void);

#endif /* GFX__H */
