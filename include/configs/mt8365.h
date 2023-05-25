/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for MT8365 based boards
 *
 * Copyright (C) 2021 BayLibre, SAS
 * Author: Fabien Parent <fparent@baylibre.com
 */

#ifndef __MT8365_H
#define __MT8365_H

#include <linux/sizes.h>

#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	-4
#define CONFIG_SYS_NS16550_MEM32
#define CONFIG_SYS_NS16550_COM1		0x11005200
#define CONFIG_SYS_NS16550_CLK		26000000


#define GENIO_350_EVK_FIT_IMAGE_GUID \
	EFI_GUID(0xa073e5d4, 0xafcb, 0x49de, 0x86, 0xa2, \
		 0xd6, 0x93, 0x5c, 0xf4, 0xce, 0xb0)

#define GENIO_350_EVK_FIP_IMAGE_GUID \
	EFI_GUID(0x82175ae7, 0xf97e, 0x44e0, 0x82, 0x98, \
		 0xdc, 0xf2, 0x23, 0xc9, 0x4e, 0x8f)

#define GENIO_350_EVK_BL2_IMAGE_GUID \
	EFI_GUID(0x221ccce5, 0xf62a, 0x4962, 0xb9, 0x41, \
		 0xef, 0x74, 0xf3, 0x06, 0x36, 0x2e)

#define GENIO_350_EVK_FW_IMAGE_GUID \
	EFI_GUID(0x13e9b687, 0x84f4, 0x492c, 0x84, 0xa1, \
		 0xfc, 0x7c, 0xa7, 0xd3, 0x9a, 0xc3)

#define GENIO_350_EVK_ENV_IMAGE_GUID \
	EFI_GUID(0x873bb255, 0x55cb, 0x4377, 0xa7, 0xc0, \
		 0x98, 0x73, 0x0f, 0x9d, 0x96, 0x80)

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

#define BOOT_TARGET_DEVICES(func) \
	BOOT_TARGET_MMC(func) \
	BOOT_TARGET_USB(func)

#define CONFIG_EXTRA_ENV_SETTINGS \
	"scriptaddr=0x40000000\0" \
	"fdt_addr_r=0x44000000\0" \
	"fdtoverlay_addr_r=0x44c00000\0" \
	"kernel_addr_r=0x45000000\0" \
	"ramdisk_addr_r=0x46000000\0" \
	"fdtfile=" CONFIG_DEFAULT_DEVICE_TREE ".dtb\0" \
	BOOTENV

#define CONFIG_SETUP_MEMORY_TAGS
#ifdef CONFIG_ARM64
#define MTK_SIP_PARTNAME_ID		0xC2000529
#endif

#endif
