/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices L-Shore
 * board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_BOOTDELAY	1

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3771

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(40 * 1024 * 1024)

#define CONFIG_VIDEO_LOGO

#define CONFIG_IMX_HDMI
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7
#define BD_MMC_UMS_DISKS "0"

#undef CONFIG_SYS_BOARD

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \

#endif	       /* __CONFIG_H */
