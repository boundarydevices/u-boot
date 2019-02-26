/*
 * Copyright (C) 2017 Boundary Devices, Inc.
 *
 * Configuration settings for the Boundary Devices YS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3769

#define CONFIG_SYS_CONSOLE_IS_IN_ENV

#define GP_RGMII2_PHY_RESET	IMX_GPIO_NR(1, 9)
#define CONFIG_SYS_FSL_ESDHC_GPIO_WP
#define ENET_MDIO_BASE			ENET2_BASE_ADDR
#define CONFIG_FEC_ENET2

#define CONFIG_MXC_UART_BASE		UART2_BASE
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_FEC_XCV_TYPE		RMII

#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_CONSOLE	"ttymxc1"
#define BD_I2C_MASK	0xb
#define BD_LOG_LEVEL	"7"
#define BD_CMA		"2M"

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"cmd_custom= \0"

#endif	       /* __CONFIG_H */
