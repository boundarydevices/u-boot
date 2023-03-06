/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for MT8195 based boards
 *
 * Copyright (C) 2021 BayLibre, SAS
 * Author: Fabien Parent <fparent@baylibre.com
 */

#ifndef __MT8195_H
#define __MT8195_H

#include <linux/sizes.h>

#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	-4
#define CONFIG_SYS_NS16550_MEM32
#define CONFIG_SYS_NS16550_COM1		0x11002000
#define CONFIG_SYS_NS16550_CLK		26000000

#define MT8195_DEMO_FIT_IMAGE_GUID \
	EFI_GUID(0x0538fd5d, 0x00dc, 0x47c5, 0x8d, 0x26, \
		 0x43, 0x48, 0x03, 0xb1, 0xe3, 0x27)

#define MT8195_DEMO_FIP_IMAGE_GUID \
	EFI_GUID(0xf65ec70e, 0x45ab, 0x40fb, 0x99, 0x52, \
		 0xe5, 0xbc, 0x30, 0x70, 0xda, 0x5c)

#define GENIO_1200_EVK_FIT_IMAGE_GUID \
	EFI_GUID(0x9fd30648, 0xb128, 0x440d, 0xbd, 0xd9, \
		 0x0e, 0x53, 0xeb, 0x2a, 0x3c, 0xb8)

#define GENIO_1200_EVK_FIP_IMAGE_GUID \
	EFI_GUID(0x39961e72, 0x5a8e, 0x445a, 0x90, 0xfe, \
		 0xed, 0x68, 0x33, 0x07, 0x44, 0xec)

#define GENIO_1200_EVK_UFS_FIT_IMAGE_GUID \
	EFI_GUID(0x49be7238, 0x9c9f, 0x4e4f, 0x94, 0x45, \
		 0x38, 0x9f, 0xcf, 0xd2, 0xa5, 0x12)

#define GENIO_1200_EVK_UFS_FIP_IMAGE_GUID \
	EFI_GUID(0x0c1603a7, 0x7f3d, 0x427c, 0x9e, 0xcc, \
		 0xf9, 0xb0, 0x2d, 0x2f, 0xc7, 0x29)

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
	BOOTENV
#endif

#endif
