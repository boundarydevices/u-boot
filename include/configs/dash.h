/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices DASH board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3769

#define CONFIG_ETHPRIME			"usb_ether"

#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \

#endif	       /* __CONFIG_H */
