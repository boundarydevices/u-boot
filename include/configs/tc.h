/*
 * Copyright (C) 2021 Boundary Devices, Inc.
 *
 * Configuration settings for the Boundary Devices YS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3769

#define CONFIG_IMX6_PWM_PER_CLK  66000000
#define CONFIG_SYS_FSL_ESDHC_GPIO_WP
#define CONFIG_MXC_UART_BASE		UART2_BASE

#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_CONSOLE	"ttymxc1"
#define BD_LOG_LEVEL	"7"

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS

#endif	       /* __CONFIG_H */
