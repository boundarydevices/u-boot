/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices INSP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3769


#define CONFIG_IMX_HDMI
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"allow_noncea=1\0" \
	"bootdelay=1\0" \
	"preboot=if itest 0 != 0x$splashsize ; then sf probe && sf read ${splashimage} ${splashflash} $splashsize && bmp d ${splashimage} ; fi\0" \
	"splashimage=0x10008000\0" \
	"splashsize=2b7a\0" \

#endif	       /* __CONFIG_H */
