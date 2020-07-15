/*
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2020 Endian Technologies AB
 * Copyright (c) 2020 max00
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "cts_sync.h"
#include "gfx.h"
#include "log.h"

/* ********** Definitions ********** */
K_SEM_DEFINE(enable_bt_sem, 0, 1);
K_SEM_DEFINE(disable_bt_sem, 0, 1);

/* ********** Function prototypes ********** */
static void bt_ready(void);
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param);
static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout);

/* ********** Variables ********** */
bool bt_enabled = false;
bool bt_initialized = false;
bool bt_toggle_lock = false;

struct k_sem enable_bt_sem;
struct k_sem disable_bt_sem;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
				0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18),
};
static struct bt_conn_cb m_conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated
};

static struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(
	BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
	BT_GAP_ADV_SLOW_INT_MIN,
	BT_GAP_ADV_SLOW_INT_MAX,
	NULL
);

/* ********** Functions ********** */
static void bt_ready(void)
{
	bt_conn_cb_register(&m_conn_callbacks);

	int err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("BLE adv start failed (err %d)", err);
	}

	LOG_DBG("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		return;
	}
	LOG_INF("connected");
	cts_sync_enable(true);
	gfx_bt_set_label(BT_CONNECTED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("disconnected (reason: %u)", reason);
	cts_sync_enable(false);
	gfx_bt_set_label(BT_ADVERTISING_ON);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{

}

void bt_adv_start(void)
{
	LOG_DBG("bt_le_adv_start");
	int err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), NULL, 0);
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

bool bt_mode(void)
{
	return bt_enabled;
}

bool bt_is_initialized(void)
{
	return bt_initialized;
}

bool bt_toggle_is_locked(void)
{
	if (bt_toggle_lock) {
		return true;
	} else {
		return false;
	}
}

void bt_on(void)
{
	bt_enabled = true;
	bt_toggle_lock = true;
	k_sem_give(&enable_bt_sem);
}

void bt_await_on(void)
{
	k_sem_take(&enable_bt_sem, K_FOREVER);
}

void bt_toggle_unlock(void)
{
	bt_toggle_lock = false;
}

void bt_off(void)
{
	bt_enabled = false;
	bt_toggle_lock = true;
	k_sem_give(&disable_bt_sem);
}

void bt_await_off(void)
{
	k_sem_take(&disable_bt_sem, K_FOREVER);
}
