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

#define MT8195_DEMO_BL2_IMAGE_GUID \
	EFI_GUID(0xac11d376, 0xfdd4, 0x419f, 0x88, 0x97, \
		 0x84, 0x93, 0xdd, 0x1b, 0x64, 0x60)

#define MT8195_DEMO_FW_IMAGE_GUID \
	EFI_GUID(0xea438b6b, 0x6f42, 0x422f, 0xb2, 0x39, \
		 0xa3, 0xfb, 0xaf, 0x0f, 0x76, 0xd5)

#define MT8195_DEMO_ENV_IMAGE_GUID \
	EFI_GUID(0xdd73443e, 0x4ab8, 0x4f96, 0xbf, 0x82, \
		 0xd3, 0x24, 0xff, 0x34, 0x6e, 0xff)

#define GENIO_1200_EVK_FIT_IMAGE_GUID \
	EFI_GUID(0x9fd30648, 0xb128, 0x440d, 0xbd, 0xd9, \
		 0x0e, 0x53, 0xeb, 0x2a, 0x3c, 0xb8)

#define GENIO_1200_EVK_FIP_IMAGE_GUID \
	EFI_GUID(0x39961e72, 0x5a8e, 0x445a, 0x90, 0xfe, \
		 0xed, 0x68, 0x33, 0x07, 0x44, 0xec)

#define GENIO_1200_EVK_BL2_IMAGE_GUID \
	EFI_GUID(0xa4a60c91, 0xffa3, 0x4cb8, 0x9b, 0x2a, \
		 0x4f, 0xf8, 0x13, 0x62, 0x0d, 0x22)

#define GENIO_1200_EVK_FW_IMAGE_GUID \
	EFI_GUID(0xe9fe6bbd, 0x30ba, 0x4c90, 0x82, 0xe2, \
		 0x2c, 0x3e, 0xa7, 0xed, 0xdd, 0x3d)

#define GENIO_1200_EVK_ENV_IMAGE_GUID \
	EFI_GUID(0x85ef9f43, 0xd9c9, 0x4817, 0xb5, 0x5e, \
		 0x3b, 0x31, 0x68, 0x9e, 0x0e, 0x7a)

#define GENIO_1200_EVK_UFS_FIT_IMAGE_GUID \
	EFI_GUID(0x49be7238, 0x9c9f, 0x4e4f, 0x94, 0x45, \
		 0x38, 0x9f, 0xcf, 0xd2, 0xa5, 0x12)

#define GENIO_1200_EVK_UFS_FIP_IMAGE_GUID \
	EFI_GUID(0x0c1603a7, 0x7f3d, 0x427c, 0x9e, 0xcc, \
		 0xf9, 0xb0, 0x2d, 0x2f, 0xc7, 0x29)

#define GENIO_1200_EVK_UFS_BL2_IMAGE_GUID \
	EFI_GUID(0xdcae4ff5, 0x53f2, 0x4664, 0xb0, 0x59, \
		 0x83, 0xeb, 0x33, 0xab, 0xf0, 0x6d)

#define GENIO_1200_EVK_UFS_FW_IMAGE_GUID \
	EFI_GUID(0xc6c7ba82, 0xe4ac, 0x49ce, 0xa7, 0x1f, \
		 0x10, 0xe1, 0x44, 0x74, 0xa7, 0x07)

#define GENIO_1200_EVK_UFS_ENV_IMAGE_GUID \
	EFI_GUID(0xc3bef54b, 0x80eb, 0x4ee4, 0xbf, 0xd1, \
		 0xb3, 0xfd, 0xc5, 0x50, 0x82, 0x66)

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
	"splashimage=0x60000000\0" \
	"splashsource=mmc_fs\0" \
	"splashfile=logo.bmp\0" \
	"splashdevpart=0#bootassets\0" \
	"splashpos=m,m\0" \
	BOOTENV
#endif

#ifdef CONFIG_ARM64
#define MTK_SIP_PARTNAME_ID		0xC2000529
#endif

#ifdef CONFIG_SYS_MALLOC_LEN
#undef CONFIG_SYS_MALLOC_LEN
#endif

#define CONFIG_SYS_MALLOC_LEN		SZ_64M

#endif
