/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices Nitrogen6X
 * and Freescale i.MX6Q Sabre Lite boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3774

#define CONFIG_MXC_SPI_DISPLAY

#define CONFIG_FEC_MXC_PHYADDR		7
#define CONFIG_IMX_HDMI
#define CONFIG_PREBOOT	"if itest.s x != x$splashsize ; then " \
				"sf probe && " \
				"sf read $splashimage $splashflash $splashsize" \
				" && bmp d $splashimage;" \
			"fi"
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7
#define BD_MMC_UMS_DISKS "1"
#define BD_SPLASH_FLASH "f0000"

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"netrecover=setenv ethact FEC; " \
		"setenv bootargs console=ttymxc1,115200; " \
		"dhcp 10800000 $serverip:uImage-${board}-recovery" \
		"&& dhcp 12800000 $serverip:uramdisk-${board}-recovery.img " \
		"&& bootm 10800000 12800000\0" \
	"novideo=1\0" \
	"savesplash=script=/savesplash; run runscript\0" \
	"splashpos=m,m\0" \

#endif	       /* __CONFIG_H */
