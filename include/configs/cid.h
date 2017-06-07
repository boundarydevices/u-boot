/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices Nitrogen6X-Lite
 * board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3771

#define CONFIG_HW_WATCHDOG
#define CONFIG_SYS_BOOT_BOARD_POWER_CHECK
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7

#define BOOT_TARGET_DEVICES(func) func(MMC, mmc, 1)

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"cmd_custom= \0" \
	"magic_keys=12\0" \
	"key_magic1=pr\0" \
	"key_cmd1=echo Starting fastboot; fastboot 0\0" \
	"key_magic2=t\0" \
	"key_cmd2=echo Starting fastboot; fastboot 0\0" \

#endif	       /* __CONFIG_H */
