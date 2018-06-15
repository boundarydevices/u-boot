/*
 * (C) Copyright 2015 Google, Inc
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef __CONFIG_RK3188_COMMON_H
#define __CONFIG_RK3188_COMMON_H

#define CONFIG_SYS_CACHELINE_SIZE	64

#include <asm/arch/hardware.h>
#include "rockchip-common.h"

#define CONFIG_SKIP_LOWLEVEL_INIT_ONLY
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_ENV_SIZE			0x2000
#define CONFIG_SYS_MAXARGS		16
#define CONFIG_SYS_MALLOC_LEN		(32 << 20)
#define CONFIG_SYS_CBSIZE		1024

#define CONFIG_SYS_TIMER_RATE		(24 * 1000 * 1000)
#define CONFIG_SYS_TIMER_BASE		0x2000e000 /* TIMER3 */
#define CONFIG_SYS_TIMER_COUNTER	(CONFIG_SYS_TIMER_BASE + 8)
#define CONFIG_SYS_TIMER_COUNTS_DOWN

#define CONFIG_SYS_NS16550_MEM32

#ifdef CONFIG_ROCKCHIP_SPL_BACK_TO_BROM
/* Bootrom will load u-boot binary to 0x60000000 once return from SPL */
#define CONFIG_SYS_TEXT_BASE		0x60000000
#else
#define CONFIG_SYS_TEXT_BASE		0x60100000
#endif
#define CONFIG_SYS_INIT_SP_ADDR		0x60100000
#define CONFIG_SYS_LOAD_ADDR		0x60800800

#define CONFIG_ROCKCHIP_MAX_INIT_SIZE	(0x8000 - 0x800)
#define CONFIG_ROCKCHIP_CHIP_TAG	"RK31"

#ifdef CONFIG_TPL_BUILD
#define CONFIG_SPL_TEXT_BASE		0x10080804
/* tpl size 1kb - 4byte RK31 header */
#define CONFIG_SPL_MAX_SIZE		(0x400 - 0x4)
#elif defined(CONFIG_SPL_BUILD)
/* spl size 32kb sram - 2kb bootrom - 1kb spl */
#define CONFIG_SPL_MAX_SIZE		(0x8000 - 0xC00)
#define CONFIG_SPL_TEXT_BASE		0x10080C00
#define CONFIG_SPL_FRAMEWORK		1
#define CONFIG_SPL_CLK			1
#define CONFIG_SPL_PINCTRL		1
#define CONFIG_SPL_REGMAP		1
#define CONFIG_SPL_SYSCON		1
#define CONFIG_SPL_RAM			1
#define CONFIG_SPL_DRIVERS_MISC_SUPPORT	1
#define CONFIG_ROCKCHIP_SERIAL		1
#endif

#define CONFIG_SPL_STACK		0x10087fff

/* MMC/SD IP block */
#define CONFIG_BOUNCE_BUFFER

#define CONFIG_SYS_SDRAM_BASE		0x60000000
#define CONFIG_NR_DRAM_BANKS		1
#define SDRAM_BANK_SIZE			(2UL << 30)

#define CONFIG_SPI_FLASH
#define CONFIG_SPI
#define CONFIG_SF_DEFAULT_SPEED 20000000

#ifndef CONFIG_SPL_BUILD
/* usb otg */
#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_GADGET_DWC2_OTG
#define CONFIG_ROCKCHIP_USB2_PHY
#define CONFIG_USB_GADGET_VBUS_DRAW	0

#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_G_DNL_MANUFACTURER	"Rockchip"
#define CONFIG_G_DNL_VENDOR_NUM		0x2207
#define CONFIG_G_DNL_PRODUCT_NUM	0x310a

/* usb host support */
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_DWC2
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_SMSC95XX
#define CONFIG_USB_ETHER_ASIX
#endif
#define ENV_MEM_LAYOUT_SETTINGS \
	"scriptaddr=0x60000000\0" \
	"pxefile_addr_r=0x60100000\0" \
	"fdt_addr_r=0x61f00000\0" \
	"kernel_addr_r=0x62000000\0" \
	"ramdisk_addr_r=0x64000000\0"

#include <config_distro_bootcmd.h>

/* Linux fails to load the fdt if it's loaded above 256M on a Rock board,
 * so limit the fdt reallocation to that */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0x6fffffff\0" \
	"initrd_high=0x6fffffff\0" \
	"partitions=" PARTS_DEFAULT \
	ENV_MEM_LAYOUT_SETTINGS \
	ROCKCHIP_DEVICE_SETTINGS \
	BOOTENV

#endif /* CONFIG_SPL_BUILD */

#define CONFIG_PREBOOT

#endif
