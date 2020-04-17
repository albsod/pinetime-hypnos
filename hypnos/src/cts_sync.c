/*
 * Modifications of najnesnaj's clock-cts sample:
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <time.h>
#include "log.h"
#include "cts_sync.h"

#define TIME_SYNC_WAIT 60
int time_sync_timeout = TIME_SYNC_WAIT;
static struct bt_gatt_discover_params cts_discovery_params;
static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_read_params read_params;

cts_datetime_t clock_datetime;

static void sync_cts_to_clock(cts_datetime_t* cts_datetime)
{
    LOG_INF("CTS sync to clock started.\n Y%04d D%03d T%2d:%2d:%2d",
        cts_datetime->year, cts_datetime->day,
        cts_datetime->hours, cts_datetime->minutes, cts_datetime->seconds);

    memset(&clock_datetime, 0, sizeof(cts_datetime_t));
    clock_datetime.year = cts_datetime->year;
    clock_datetime.month = cts_datetime->month;
    clock_datetime.day = cts_datetime->day;
    clock_datetime.hours = cts_datetime->hours;
    clock_datetime.minutes = cts_datetime->minutes;
    clock_datetime.seconds = cts_datetime->seconds;

    LOG_INF("CTS sync to clock complete.");

    time_sync_timeout = TIME_SYNC_WAIT;
}


void cts_get_datetime(struct tm *t)
{
	t->tm_year = clock_datetime.year -1900;
	t->tm_mon = clock_datetime.month -1;
	t->tm_mday = clock_datetime.day;
	t->tm_hour = clock_datetime.hours;
	t->tm_min = clock_datetime.minutes;
	t->tm_sec = clock_datetime.seconds;
}

int offset = 0;
cts_datetime_t datetime_buf;
u8_t cts_sync_read(struct bt_conn *conn, u8_t err,
				    struct bt_gatt_read_params *params,
				    const void *data, u16_t length)
{
    LOG_DBG("Reading CCC data: err %d, %d bytes, offset %d.", err, length, offset);

    if (!data || length <= 0) {
        sync_cts_to_clock(&datetime_buf);
        return BT_GATT_ITER_STOP;
    }

    memcpy(&datetime_buf + offset, data, length);
    offset += length;

    return BT_GATT_ITER_CONTINUE;
}

u8_t cts_sync_service_discovered(struct bt_conn* conn, const struct bt_gatt_attr* attr,
        struct bt_gatt_discover_params* params)
{
    if (!attr) {
        LOG_INF("CTS Service Discovery completed");
        return BT_GATT_ITER_STOP;
    }
    LOG_DBG("Discovered attribute, handle: %u", attr->handle);

    memset(&read_params, 0, sizeof(read_params));
    read_params.func = cts_sync_read;
    read_params.by_uuid.uuid = (struct bt_uuid *) &uuid;
    read_params.by_uuid.start_handle = attr->handle;
    read_params.by_uuid.end_handle = 0xffff;
    offset = 0;
    if (bt_gatt_read(conn, &read_params) < 0) {
        LOG_DBG("Could not initiate read of CCC data.");
    }

    return BT_GATT_ITER_STOP;
}

static void cts_sync_processor(struct bt_conn *conn, void *data)
{
    memcpy(&uuid, BT_UUID_CTS_CURRENT_TIME, sizeof(uuid));
    cts_discovery_params.func = cts_sync_service_discovered;
    cts_discovery_params.start_handle = 0x0001;
    cts_discovery_params.end_handle = 0xFFFF;
    cts_discovery_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
    cts_discovery_params.uuid = (struct bt_uuid *) &uuid;

    if (bt_gatt_discover(conn, &cts_discovery_params) != 0) {
        LOG_ERR("CTS Sync > GATT discovery FAILED.");
    }
}

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		return;
	}
    cts_sync_processor(conn, NULL);
}

// TODO: Remove, replace or fill it with something
static void disconnected(struct bt_conn *conn, u8_t reason)
{
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

void cts_sync_init()
{
	bt_conn_cb_register(&conn_callbacks);
}

void cts_sync_loop()
{
    if (time_sync_timeout > 0) {
        time_sync_timeout--;
    } else {
        bt_conn_foreach(BT_CONN_TYPE_ALL, cts_sync_processor, NULL);
        time_sync_timeout = TIME_SYNC_WAIT;
    }
}
