/*
 * Copyright (C) 2015 Boundary Devices, Inc.
 *
 * Configuration settings for the Boundary Devices cob
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3769

#define CONFIG_ETHPRIME		"usbnet"

#define CONFIG_PWM_IMX
#define CONFIG_IMX6_PWM_PER_CLK  66000000

#define CONFIG_IMX_HDMI
#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7
#define BD_SKIP_FUSES

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \

#endif	       /* __CONFIG_H */
