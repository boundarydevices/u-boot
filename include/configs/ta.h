/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices TA board
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_BOOTDELAY   1
#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3778

#define CONFIG_IMX_HDMI
#define CONFIG_PHY_MICREL_KSZ9021
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7
#define BD_MMC_UMS_DISKS "1 0"

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"bootdelay=1\0" \
	"disable_giga=1\0" \
	"fb_lvds=tm070jdhg30\0" \
	"panel=tm070jdhg30\0" \

#endif	       /* __CONFIG_H */
