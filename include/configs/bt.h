/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices bt
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3780

#define CONFIG_IMX_HDMI
#define CONFIG_PHY_MICREL_KSZ9021
#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7
#define BD_CMA		"384M"

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"cmd_custom=setenv bootargs $bootargs pci=nomsi pcie.force_gen=1 coherent_pool=32M\0" \


#endif	       /* __CONFIG_H */
