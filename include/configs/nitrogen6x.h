/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices Nitrogen6X
 * and Freescale i.MX6Q Sabre Lite boards.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#ifndef CONFIG_BOARD_TYPE
#undef CONFIG_SYS_BOARD
#endif

#define CONFIG_MACH_TYPE	3769

#define CONFIG_MXC_SPI_DISPLAY

#define CONFIG_IMX_HDMI
/* Sabrelite has different reset pin */
#define GP_RGMII2_PHY_RESET	IMX_GPIO_NR(3, 23)

#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

#define BD_I2C_MASK	7
#ifndef CONFIG_EXTRA_ENV_SETTINGS_DEFCONFIG
#define CONFIG_EXTRA_ENV_SETTINGS_DEFCONFIG
#endif


#if defined(CONFIG_SABRELITE)
#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS \
	"console=ttymxc1\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_addr_r=0x18000000\0" \
	FDTFILE \
	"kernel_addr_r=" __stringify(CONFIG_LOADADDR) "\0"  \
	"pxefile_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"ramdisk_addr_r=0x13000000\0" \
	"ramdiskaddr=0x13000000\0" \
	"ip_dyn=yes\0" \
	"usb_pgood_delay=2000\0" \
	BOOTENV \
	CONFIG_EXTRA_ENV_SETTINGS_DEFCONFIG

#else
#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	CONFIG_EXTRA_ENV_SETTINGS_DEFCONFIG

#endif

#endif	       /* __CONFIG_H */
