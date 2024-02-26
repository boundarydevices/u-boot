/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for MT8188 based boards
 *
 * Copyright (C) 2022 MediaTek Inc.
 * Author: Macpaul Lin <macpaul.lin@mediatek.com>
 */

#ifndef __MT8390_TUNGSTEN_SMARC_H
#define __MT8390_TUNGSTEN_SMARC_H

#ifdef CONFIG_BOARD_TYPE_SET
#undef CONFIG_SYS_BOARD
#define CONFIG_SYS_BOARD CONFIG_BOARD_TYPE
#endif
#include <linux/sizes.h>

#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	-4
#define CONFIG_SYS_NS16550_MEM32
#define CONFIG_SYS_NS16550_COM1		0x11002000
#define CONFIG_SYS_NS16550_CLK		26000000
#define CONFIG_SYS_BOOTM_LEN		SZ_64M
#define PHYS_SDRAM			0x40000000

/* Monitor Command Prompt */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_CBSIZE		2048
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

#define MT8390_TUNGSTEN_SMARC_FIT_IMAGE_GUID \
	EFI_GUID(0x0019b800, 0x5555, 0x5555, 0x55, 0x55, \
		 0x55, 0x55, 0x55, 0x55, 0x55, 0x55)

#define MT8390_TUNGSTEN_SMARC_FIP_IMAGE_GUID \
	EFI_GUID(0x0019b801, 0x5555, 0x5555, 0x55, 0x55, \
		 0x55, 0x55, 0x55, 0x55, 0x55, 0x55)

#define MT8390_TUNGSTEN_SMARC_BL2_IMAGE_GUID \
	EFI_GUID(0x0019b802, 0x5555, 0x5555, 0x55, 0x55, \
		 0x55, 0x55, 0x55, 0x55, 0x55, 0x55)

#define MT8390_TUNGSTEN_SMARC_FW_IMAGE_GUID \
	EFI_GUID(0x0019b803, 0x5555, 0x5555, 0x55, 0x55, \
		 0x55, 0x55, 0x55, 0x55, 0x55, 0x55)

#define MT8390_TUNGSTEN_SMARC_ENV_IMAGE_GUID \
	EFI_GUID(0x0019b804, 0x5555, 0x5555, 0x55, 0x55, \
		 0x55, 0x55, 0x55, 0x55, 0x55, 0x55)

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
	"console=ttyS0\0" \
	"dtbos=gpu-mali apusys video\0" \
	"env_dev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"env_part=" __stringify(CONFIG_SYS_MMC_ENV_PART) "\0" \
	"scriptaddr=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
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
