/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices MCS board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3769

#define CONFIG_PREBOOT	"if itest.s  \"\" != \"$bmpsize\" ; then " \
				"sf probe && " \
				"sf read ${splashimage} ${splashflash} $bmpsize" \
				" && bmp d 10008000;" \
			"fi"
#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7
#define BD_SPLASH_FLASH "f0000"

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"disable_giga=1\0" \
	"savesplash=script=/savesplash; run runscript\0" \

#endif	       /* __CONFIG_H */
