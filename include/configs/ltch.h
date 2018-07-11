/*
 * Copyright (C) 2018 Boundary Devices <info@boundarydevices.com>
 *
 * Configuration settings for the Boundary Devices ltch
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_FEC_MXC_PHYADDR		4
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"usb_connect_wait=3000\0" \

#endif	       /* __CONFIG_H */
