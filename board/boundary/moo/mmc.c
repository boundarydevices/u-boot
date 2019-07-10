/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/arch/sys_proto.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <stdbool.h>

/* This should be defined for each board */
__weak int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

void board_late_mmc_env_init(void)
{
	env_set_ulong("mmcdev", 0);

	/* Set mmcblk env */
	env_set("mmcroot", "/dev/mmcblk0p2 rootwait rw");
	run_command("mmc dev 0", 0);
}
