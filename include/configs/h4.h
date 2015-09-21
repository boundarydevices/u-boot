/*
 * Copyright (C) 2015 Boundary Devices, Inc.
 *
 * Configuration settings for the Boundary Devices H4
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3778

#define CONFIG_MXC_SPI_DISPLAY

#define CONFIG_ETHPRIME			"usbnet"

#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \

#endif	       /* __CONFIG_H */
