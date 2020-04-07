
/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#ifndef BATTERY__H
#define BATTERY__H

void battery_status_init();
void battery_update_percentage();
void battery_update_charging_status(bool);
uint8_t battery_get_percentage();
bool battery_get_charging_status();
uint32_t battery_raw_to_mv(s16_t raw);
uint32_t battery_mv_to_ppt(uint32_t mv);

#endif /* BATTERY__H */
