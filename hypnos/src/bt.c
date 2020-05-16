/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <logging/log.h>

#include <device.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <stdlib.h>
#include <drivers/counter.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL

LOG_MODULE_REGISTER(app);

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>

#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "cts_sync.h"

K_SEM_DEFINE(enable_bt_sem, 0, 1);
K_SEM_DEFINE(disable_bt_sem, 0, 1);


/* ********** variables ********** */
bool bt_enabled = false;
bool bt_initialized = false;

struct k_sem enable_bt_sem;
struct k_sem disable_bt_sem;

static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
		      0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};
/* ********** variables ********** */

        
static void bt_ready(void)
{
	LOG_DBG("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
}

void bt_adv_start()
{
	LOG_DBG("bt_le_adv_start");
	int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
                LOG_ERR("Advertising failed to start (err %d)", err);
                return;
        }

        LOG_DBG("Advertising successfully started");
}

void bt_init(void)
{
        int err = bt_enable(NULL);
        if (err) {
                LOG_ERR("Bluetooth init failed (err %d)", err);
                return;
        }

        bt_ready();
	bt_adv_start();
        cts_sync_init();
	bt_initialized = true;
}

void bt_adv_stop(void)
{
	k_sleep(K_MSEC(400));

	int err = bt_le_adv_stop();
	if (err) {
		LOG_ERR("Advertising failed to stop (err %d)", err);
		return;
	}
}

void bt_on(void)
{
	bt_enabled = true;
	k_sem_give(&enable_bt_sem);
}

void bt_await_on(void)
{
	k_sem_take(&enable_bt_sem, K_FOREVER);
}

void bt_off(void)
{
	bt_enabled = false;
	k_sem_give(&disable_bt_sem);
}

void bt_await_off(void)
{
	k_sem_take(&disable_bt_sem, K_FOREVER);
}

bool bt_mode(void)
{
	return bt_enabled;
}

bool bt_is_initialized(void)
{
	return bt_initialized;
}
