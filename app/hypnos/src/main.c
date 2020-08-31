/*
 * PineTime Hypnos: Smartwatch firmware for the PineTime dev kit
 * Copyright (c) 2020 Endian Technologies AB
 *
 * This is free software with ABSOLUTELY NO WARRANTY.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include "backlight.h"
#include "battery.h"
#include "bt.h"
#include "clock.h"
#include "cts_sync.h"
#include "display.h"
#include "event_handler.h"
#include "gfx.h"
#include "log.h"
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	#include "dfu/mcuboot.h"
#endif

/* DFU sample start */

#include <stats/stats.h>
#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
#include <device.h>
#include <fs/fs.h>
#include "fs_mgmt/fs_mgmt.h"
#include <fs/littlefs.h>
#endif
#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
#include "os_mgmt/os_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
#include "img_mgmt/img_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_STAT_MGMT
#include "stat_mgmt/stat_mgmt.h"
#endif

#include "common.h"

/* Define an example stats group; approximates seconds since boot. */
STATS_SECT_START(smp_svr_stats)
	STATS_SECT_ENTRY(ticks)
	STATS_SECT_END;

/* Assign a name to the `ticks` stat. */
STATS_NAME_START(smp_svr_stats)
	STATS_NAME(smp_svr_stats, ticks)
	STATS_NAME_END(smp_svr_stats);

/* Define an instance of the stats group. */
STATS_SECT_DECL(smp_svr_stats) smp_svr_stats;

#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(cstorage);
static struct fs_mount_t littlefs_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &cstorage,
	.storage_dev = (void *)FLASH_AREA_ID(storage),
	.mnt_point = "/lfs"
};
#endif


/* DFU sample ends */


/* ******** Thread prototypes, constants and macros ******** */
/* void main_thread(void); */
/* void bt_thread(void); */

/* #define STACKSIZE 1024 */
/* #define PRIORITY 7 */

/* K_THREAD_DEFINE(bt_id, STACKSIZE, bt_thread, NULL, NULL, NULL, */
/* 		PRIORITY, 0, 0); */
/* K_THREAD_DEFINE(main_id, STACKSIZE, main_thread, NULL, NULL, NULL, */
/* 		PRIORITY, 0, 0); */

/* ******** Functions ******** */
void main(void)
{
	LOG_INF("Welcome to PineTime Hypnos!");
	LOG_INF("This is free software with ABSOLUTELY NO WARRANTY.");

	gfx_init();
	clock_init();
	battery_init();
	display_init();
	event_handler_init();
	gfx_update();
	backlight_init();

#ifdef CONFIG_BOOTLOADER_MCUBOOT
	/* TODO: Ask the user to confirm the new image */
	if (!boot_is_img_confirmed()) {
		/* LOG_DBG("Confirming new firmware image."); */
		/* (void)boot_write_img_confirmed(); */
	}
#endif

	int rc = STATS_INIT_AND_REG(smp_svr_stats, STATS_SIZE_32,
				    "smp_svr_stats");

	if (rc < 0) {
		LOG_ERR("Error initializing stats system [%d]", rc);
	}

	/* Register the built-in mcumgr command handlers. */
#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
	rc = fs_mount(&littlefs_mnt);
	if (rc < 0) {
		LOG_ERR("Error mounting littlefs [%d]", rc);
	}

	fs_mgmt_register_group();
#endif
#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
	os_mgmt_register_group();
#endif
#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
	img_mgmt_register_group();
#endif
#ifdef CONFIG_MCUMGR_CMD_STAT_MGMT
	stat_mgmt_register_group();
#endif
#ifdef CONFIG_MCUMGR_SMP_BT
	start_smp_bluetooth();
#endif
/* #ifdef CONFIG_MCUMGR_SMP_UDP */
/* 	start_smp_udp(); */
/* #endif */
	/* using __TIME__ ensure that a new binary will be built on every
	 * compile which is convient when testing firmware upgrade.
	 */
	LOG_INF("build time: " __DATE__ " " __TIME__);

	while (1) {
		k_sleep(K_MSEC(1000));
		STATS_INC(smp_svr_stats, ticks);
	}
}

/* void main_thread(void) */
/* { */
/* 	while (true) { */
/* 		bt_await_off(); */
/* 		LOG_INF("Disabling BLE advertising..."); */
/* 		bt_adv_stop(); */
/* 		cts_sync_enable(false); */
/* 	} */
/* } */

/* void bt_thread(void) */
/* { */
/* 	while (true) { */
/* 		bt_await_on(); */
/* 		LOG_INF("Enabling BLE advertising..."); */
/* 		if (bt_is_initialized()) { */
/* 			bt_adv_start(); */
/* 		} else { */
/* 			bt_init(); */
/* 			cts_sync_init(); */
/* 		} */
/* 		cts_sync_enable(true); */
/* 	} */
/* } */
