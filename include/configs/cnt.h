/*
 * Copyright (C) 2015 Boundary Devices, Inc.
 *
 * Configuration settings for the Boundary Devices CNT board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_BOOTDELAY	1

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3769

#define CONFIG_MXC_SPI_DISPLAY

#define CONFIG_FEC_MXC_PHYADDR		4
#define CONFIG_PHY_ATHEROS
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7

#define BOOT_TARGET_DEVICES(func) \
	DISTRO_BOOT_DEV_USB(func) \
	DISTRO_BOOT_DEV_MMC(func)

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \

#endif	       /* __CONFIG_H */
