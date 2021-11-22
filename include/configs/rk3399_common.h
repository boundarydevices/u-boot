/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 */

#ifndef __CONFIG_RK3399_COMMON_H
#define __CONFIG_RK3399_COMMON_H

#include "rockchip-common.h"

#define CONFIG_SYS_CBSIZE		1024

#define COUNTER_FREQUENCY               24000000
#define CONFIG_ROCKCHIP_STIMER_BASE	0xff8680a0

#define CONFIG_IRAM_BASE		0xff8c0000

#define CONFIG_SYS_INIT_SP_ADDR		0x00300000

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_TPL_BOOTROM_SUPPORT)
#define CONFIG_SPL_STACK		0x00400000
#define CONFIG_SPL_MAX_SIZE             0x40000
#define CONFIG_SPL_BSS_START_ADDR	0x00400000
#define CONFIG_SPL_BSS_MAX_SIZE         0x2000
#else
#define CONFIG_SPL_STACK		0xff8effff
#define CONFIG_SPL_MAX_SIZE		0x30000 - 0x2000
/*  BSS setup */
#define CONFIG_SPL_BSS_START_ADDR       0xff8e0000
#define CONFIG_SPL_BSS_MAX_SIZE         0x10000
#endif

#define CONFIG_SYS_BOOTM_LEN	(64 << 20)	/* 64M */

/* MMC/SD IP block */
#define CONFIG_ROCKCHIP_SDHCI_MAX_FREQ	200000000

/* RAW SD card / eMMC locations. */

/* FAT sd card locations. */
#define CONFIG_SYS_SDRAM_BASE		0
#define SDRAM_MAX_SIZE			0xf8000000

#ifndef CONFIG_SPL_BUILD

#define ENV_MEM_LAYOUT_SETTINGS \
	"scriptaddr=0x00500000\0" \
	"script_offset_f=0xffe000\0" \
	"script_size_f=0x2000\0" \
	"pxefile_addr_r=0x00600000\0" \
	"fdt_addr_r=0x01f00000\0" \
	"fdtoverlay_addr_r=0x02000000\0" \
	"kernel_addr_r=0x02080000\0" \
	"ramdisk_addr_r=0x06000000\0" \
	"kernel_comp_addr_r=0x08000000\0" \
	"kernel_comp_size=0x2000000\0"

#ifndef ROCKCHIP_DEVICE_SETTINGS
#define ROCKCHIP_DEVICE_SETTINGS
#endif

#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE 8192000
#define CONFIG_VIDEO_BMP_GZIP 1

#define CONSOLE_FONT_COLOR 14

#define PREBOOT_LOAD_LOGO \
   "ll=0; test $boot_source = spi && sf probe && sf read $loadaddr 0x170000 0x10000 && ll=1; " \
   "test $ll = 0 && ll=1 && " \
   "load mmc 1 $loadaddr splash.bmp || " \
   "load mmc 2 $loadaddr splash.bmp || " \
   "load mmc 1:2 $loadaddr /usr/share/fenix/logo/logo.bmp || " \
   "load mmc 2:2 $loadaddr /usr/share/fenix/logo/logo.bmp || " \
   "ll=0; " \
   "test $ll = 1 && bmp display $loadaddr m m || ll=0; " \
   "test $ll = 0 && fdt addr $fdtcontroladdr && fdt get addr logoaddr /logo data && bmp display $logoaddr m m; "

#define PREBOOT_CMD "run load_logo; usb start; kbi init; sleep 1;"

#include <config_distro_bootcmd.h>
#include <environment/distro/sf.h>
#define CONFIG_EXTRA_ENV_SETTINGS \
	ENV_MEM_LAYOUT_SETTINGS \
	"fdtfile=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"partitions=" PARTS_DEFAULT \
	ROCKCHIP_DEVICE_SETTINGS \
	BOOTENV \
	BOOTENV_SF \
	"altbootcmd=" \
		"setenv boot_syslinux_conf extlinux/extlinux-rollback.conf;" \
		"run distro_bootcmd\0" \
		"load_logo=" PREBOOT_LOAD_LOGO "\0" \
		"preboot=" PREBOOT_CMD "\0"

#endif

#endif
