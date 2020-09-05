/*
 * Hypnos: Smartwatch firmware for the PineTime
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

#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(cstorage);
static struct fs_mount_t littlefs_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &cstorage,
	.storage_dev = (void *)FLASH_AREA_ID(storage),
	.mnt_point = "/lfs"
};
#endif

/* ******** Functions ******** */
void main(void)
{
	LOG_INF("Welcome to Hypnos!");
	LOG_INF("This is free software with ABSOLUTELY NO WARRANTY.");

	gfx_init();
	clock_init();
	battery_init();
	display_init();
	event_handler_init();
	gfx_update();
	backlight_init();

	/* Register the built-in mcumgr command handlers. */
#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
	int rc = fs_mount(&littlefs_mnt);
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
	bt_init();

	while (1) {
		k_sleep(K_MSEC(1000));
		STATS_INC(smp_svr_stats, ticks);
	}
}
