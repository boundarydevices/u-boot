/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2023 Boundary Devices
 */

#ifndef __IMX8ULP_NIROGEN_SOM_H
#define __IMX8ULP_NIROGEN_SOM_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#ifdef CONFIG_BOARD_TYPE_SET
#undef CONFIG_SYS_BOARD
#define CONFIG_SYS_BOARD CONFIG_BOARD_TYPE
#endif
#define CFG_SYS_UBOOT_BASE	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD
#define CFG_MALLOC_F_ADDR		0x22048000

#endif

/* ENET Config */
#if defined(CONFIG_FEC_MXC)
#define PHY_ANEG_TIMEOUT		20000

#define CFG_FEC_MXC_PHYADDR		7
#define CFG_FEC_XCV_TYPE		RMII
#define IMX_FEC_BASE			0x29950000
#endif

#ifdef CONFIG_CMD_MMC
#define DISTRO_BOOT_DEV_MMC(func) func(MMC, mmc, 0)
#else
#define DISTRO_BOOT_DEV_MMC(func)
#endif

#ifdef CONFIG_USB_STORAGE
#define DISTRO_BOOT_DEV_USB(func) func(USB, usb, 0)
#else
#define DISTRO_BOOT_DEV_USB(func)
#endif

#ifndef BOOT_TARGET_DEVICES
#define BOOT_TARGET_DEVICES(func) \
	DISTRO_BOOT_DEV_USB(func) \
	DISTRO_BOOT_DEV_MMC(func)
#endif

#ifdef CONFIG_DISTRO_DEFAULTS
#include <config_distro_bootcmd.h>
#else
#define BOOTENV
#endif

#define BD_CONSOLE	"ttyLP3"
#define BD_RAM_SCRIPT	"80020000"
#define BD_RAM_KERNEL	"80800000"
#define BD_RAM_RAMDISK	"82800000"
#define BD_RAM_FDT	"83000000"

/* Initial environment variables */
#define CONFIG_EXTRA_ENV_SETTINGS		\
	"console=" BD_CONSOLE "\0" \
	"env_dev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"env_part=" __stringify(CONFIG_SYS_MMC_ENV_PART) "\0" \
	"scriptaddr=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
	"fastboot_raw_partition_bootloader=0x0 0x1ff0 mmcpart 1\0" \
	"fastboot_raw_partition_bootloader-env=0x1ff0 0x10 mmcpart 1\0" \
	"fdt_addr=0x83000000\0" \
	"fdt_high=0xffffffffffffffff\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"kernel_addr_r=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
	"image=Image\0" \
	"boot_fit=no\0" \
	"fdtfile=imx8ulp-nitrogen-som.dtb\0" \
	"initrd_addr=0x83800000\0"		\
	"bootm_size=0x10000000\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"netargs=setenv bootargs console=${console},115200 root=/dev/nfs rw " \
		"ip=dhcp nfsroot=${tftpserverip}:${nfsroot},v3,tcp\0" \
	"netboot=run netargs; " \
		"if test -z \"${fdt_file}\" -a -n \"${soc}\"; then " \
			"setenv fdt_file ${soc}-${board}${boardver}.dtb; " \
		"fi; " \
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
	"net_upgradeu=dhcp " BD_RAM_SCRIPT " net_upgradeu.scr && source " BD_RAM_SCRIPT "\0" \
	"otg_upgradeu=run usbnetwork; tftp " BD_RAM_SCRIPT " net_upgradeu.scr && source " BD_RAM_SCRIPT "\0" \
	"upgradeu=setenv boot_scripts upgrade.scr; boot;" \
		"echo Upgrade failed!; setenv boot_scripts boot.scr\0" \
	"usbnet_devaddr=00:19:b8:00:00:02\0" \
	"usbnet_hostaddr=00:19:b8:00:00:01\0" \
	"usbnetwork=setenv ethact usb_ether; " \
		"setenv ipaddr 10.0.0.2; " \
		"setenv netmask 255.255.255.0; " \
		"setenv serverip 10.0.0.1;\0" \
	BOOTENV

/* Link Definitions */

#define CFG_SYS_INIT_RAM_ADDR	0x80000000
#define CFG_SYS_INIT_RAM_SIZE	0x80000

#define CFG_SYS_SDRAM_BASE		0x80000000
#define PHYS_SDRAM			0x80000000
#define PHYS_SDRAM_SIZE			0x80000000 /* 2GB DDR */

/* Using ULP WDOG for reset */
#define WDOG_BASE_ADDR			WDG3_RBASE

#endif
