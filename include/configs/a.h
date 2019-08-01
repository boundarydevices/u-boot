/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices A board
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	0xffffffff

#define CONFIG_CONSOLE_MUX

#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_USB_MAX_CONTROLLER_COUNT 1
#define BD_I2C_MASK	2
#define BD_USB_START 	"setexpr otgstat *0x020c9030 \\\\& 0x08000000;" \
				"if itest.l ${otgstat} -eq 0 ; then " \
					"usb start;" \
				"fi ;"

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"cmd_custom= \0" \
	"disable_giga=1\0" \

#endif	       /* __CONFIG_H */
