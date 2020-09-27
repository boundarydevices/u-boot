// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Adrien Grassein <adrien.grassein@smile.fr>
 *
 * Fastboot specific functions for imx8m
 */

#include <linux/types.h>
#include <common.h>
#include <fastboot.h>

/**
 * fastboot_set_reboot_flag() - Set flag to indicate reboot-bootloader
 *
 * Set flag which indicates that we should reboot into the bootloader
 * following the reboot that fastboot executes after this function.
 */
int fastboot_set_reboot_flag(void) {
	char cmd[64];

	snprintf(cmd, sizeof(cmd), "bcb load %d misc",
		 CONFIG_FASTBOOT_FLASH_MMC_DEV);

	if (run_command(cmd, 0))
		return -ENODEV;

	if (run_command("bcb set command bootonce-bootloader", 0))
		return -ENOEXEC;

	if (run_command("bcb store", 0))
		return -EIO;

	return 0;
}
