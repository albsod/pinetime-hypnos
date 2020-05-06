/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <logging/log.h>

#include <device.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <stdlib.h>
#include <sys/printk.h> //tod have to remove this later on, since makes no sense
#include <drivers/counter.h>

#define DELAY 1000000 //should be 1 second
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL

LOG_MODULE_REGISTER(app);

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>

#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "cts_sync.h"

static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
		      0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

        
static void bt_ready(void)
{
        printk("Bluetooth initialized\n");

        if (IS_ENABLED(CONFIG_SETTINGS)) {
                settings_load();
        }
}

void bt_adv_start()
{
	printk("bt_le_adv_start\n");
	int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
                printk("Advertising failed to start (err %d)\n", err);
                return;
        }

        printk("Advertising successfully started\n");
}

void bt_init(void)
{
        int err = bt_enable(NULL);
        if (err) {
                printk("Bluetooth init failed (err %d)\n", err);
                return;
        }

        bt_ready();
	bt_adv_start();
        cts_sync_init();
}

void bt_adv_stop(void)
{
	k_sleep(K_MSEC(400));

	int err = bt_le_adv_stop();
	if (err) {
		printk("Advertising failed to stop (err %d)\n", err);
		return;
	}
}
