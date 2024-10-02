/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 Ezurio LLC
 */

#ifndef __IMX95_NITROGEN_SMARC_H
#define __IMX95_NITROGEN_SMARC_H

#include <linux/sizes.h>
#include <linux/stringify.h>
#include <asm/arch/imx-regs.h>
#include "imx_env.h"

#ifdef CONFIG_AHAB_BOOT
#define AHAB_ENV "sec_boot=yes\0"
#else
#define AHAB_ENV "sec_boot=no\0"
#endif

#ifdef CONFIG_DISTRO_DEFAULTS
#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0) \
	func(MMC, mmc, 1) \
	func(USB, usb, 0)

#include <config_distro_bootcmd.h>
#else
#define BOOTENV
#endif

/* Initial environment variables */
#define CFG_EXTRA_ENV_SETTINGS		\
	BOOTENV \
	AHAB_ENV \
	"board=nitrogen-smarc\0" \
	"soc_type=imx95\0" \
	"env_dev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"env_part=" __stringify(CONFIG_SYS_MMC_ENV_PART) "\0" \
	"fastboot_raw_partition_bootloader=0x0 0x1ff0 mmcpart 1\0" \
	"fastboot_raw_partition_bootloader-env=0x1ff0 0x10 mmcpart 1\0" \
	"prepare_mcore=setenv mcore_args pd_ignore_unused;\0" \
	"cpuidle= \0" \
	"scriptaddr=0x93500000\0" \
	"kernel_addr_r=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
	"splashimage=0xA0000000\0" \
	"console=ttyLP0\0" \
	"fdt_addr_r=0x93000000\0"			\
	"fdt_addr=0x93000000\0"			\
	"fdt_high=0xffffffffffffffff\0"		\
	"cntr_addr=0xA8000000\0"			\
	"cntr_file=os_cntr_signed.bin\0" \
	"boot_fit=no\0" \
	"fdtfile=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"bootm_size=0x10000000\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"netargs=setenv bootargs ${cpuidle} ${mcore_args} console=${console},115200 " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${tftpserverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs;  " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${loadaddr} ${tftpserverip}:Image; " \
		"if ${get_cmd} ${fdt_addr} ${tftpserverip}:${fdt_file}; then " \
			"booti ${loadaddr} - ${fdt_addr}; " \
		"else " \
			"echo WARN: Cannot load the DT; " \
		"fi;\0" \
	"upgradeu=setenv boot_scripts upgrade.scr; boot;" \
                "echo Upgrade failed!; setenv boot_scripts boot.scr;\0" \
	"uboot_defconfig=imx95_nitrogen_smarc\0"
/* Link Definitions */

#define CFG_SYS_INIT_RAM_ADDR        0x90000000
#define CFG_SYS_INIT_RAM_SIZE        0x200000

#define CFG_SYS_SDRAM_BASE           0x90000000
#define PHYS_SDRAM                      0x90000000
/* Totally 8GB */
#define PHYS_SDRAM_SIZE			0x70000000UL /* 2GB  - 256MB DDR */
#define PHYS_SDRAM_2_SIZE 		0x180000000UL /* 6GB */

#define CFG_SYS_FSL_USDHC_NUM	2

/* Using ULP WDOG for reset */
#define WDOG_BASE_ADDR          WDG3_BASE_ADDR

/* USB configs */
#if defined(CONFIG_CMD_NET)
#define PHY_ANEG_TIMEOUT 20000
/* Number of Rx BD rings: 8 per ENETC instance */
#endif

#endif
