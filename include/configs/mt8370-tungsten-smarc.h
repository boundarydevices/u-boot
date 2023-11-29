/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for MT8370 Tungsten board
 *
 * Copyright (C) 2023 Boundary devices
 * Author: Simon Gaynor <simon.gaynor@lairdconnect.com>
 */

#ifndef __MT8370_TUNGSTEN_H
#define __MT8370_TUNGSTEN_H

#include <linux/sizes.h>

#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	-4
#define CONFIG_SYS_NS16550_MEM32
#define CONFIG_SYS_NS16550_COM1		0x11002000
#define CONFIG_SYS_NS16550_CLK		26000000

#define TUNGSTEN_510_SMARC_FIT_IMAGE_GUID \
	EFI_GUID(0x458ae454, 0xb228, 0x49eb, 0x80, 0xfc, \
		 0x5c, 0x68, 0x7f, 0x96, 0xc7, 0xc8)
#define TUNGSTEN_510_SMARC_FIP_IMAGE_GUID \
	EFI_GUID(0x0cb9a4dd, 0x8692, 0x425d, 0xa1, 0xdc, \
		 0xa3, 0x3b, 0xcf, 0x52, 0xad, 0x05)
#define TUNGSTEN_510_SMARC_BL2_IMAGE_GUID \
	EFI_GUID(0x546af9d8, 0x91e7, 0x4de0, 0xa8, 0xa3, \
		 0x35, 0x1b, 0xf1, 0x56, 0x9d, 0x64)
#define TUNGSTEN_510_SMARC_FW_IMAGE_GUID \
	EFI_GUID(0x5be6e67f, 0x8cec, 0x43f7, 0xb4, 0xe3, \
		 0x01, 0x33, 0x49, 0xdc, 0x49, 0xad)
#define TUNGSTEN_510_SMARC_ENV_IMAGE_GUID \
	EFI_GUID(0xf967dfc1, 0xbb75, 0x4439, 0x98, 0x8f, \
		 0xab, 0xe6, 0xc0, 0xa2, 0xd0, 0x19)

/* Environment settings */
#include <config_distro_bootcmd.h>

#ifdef CONFIG_CMD_MMC
#define BOOT_TARGET_MMC(func) func(MMC, mmc, 0)
#else
#define BOOT_TARGET_MMC(func)
#endif

#ifdef CONFIG_CMD_USB
#define BOOT_TARGET_USB(func) func(USB, usb, 0)
#else
#define BOOT_TARGET_USB(func)
#endif

#ifdef CONFIG_CMD_SCSI
#define BOOT_TARGET_SCSI(func) func(SCSI, scsi, 2)
#else
#define BOOT_TARGET_SCSI(func)
#endif

#define BOOT_TARGET_DEVICES(func) \
	BOOT_TARGET_MMC(func) \
	BOOT_TARGET_USB(func) \
	BOOT_TARGET_SCSI(func)

#if !defined(CONFIG_EXTRA_ENV_SETTINGS)
#define CONFIG_EXTRA_ENV_SETTINGS \
	"scriptaddr=0x40000000\0" \
	"fdt_addr_r=0x44000000\0" \
	"fdtoverlay_addr_r=0x44c00000\0" \
	"kernel_addr_r=0x45000000\0" \
	"ramdisk_addr_r=0x46000000\0" \
	"fdtfile=" CONFIG_DEFAULT_DEVICE_TREE ".dtb\0" \
	"splashimage=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
	"splashsource=mmc_fs\0" \
	"splashfile=logo.bmp\0" \
	"splashdevpart=0#bootassets\0" \
	"splashpos=m,m\0" \
	BOOTENV
#endif

#ifdef CONFIG_ARM64
#define MTK_SIP_PARTNAME_ID		0xC2000529
#endif

#endif
