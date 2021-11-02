/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for Amlogic Meson 64bits SoCs
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 */

#ifndef __MESON64_CONFIG_H
#define __MESON64_CONFIG_H

/* Generic Interrupt Controller Definitions */
#if (defined(CONFIG_MESON_AXG) || defined(CONFIG_MESON_G12A))
#define GICD_BASE			0xffc01000
#define GICC_BASE			0xffc02000
#else /* MESON GXL and GXBB */
#define GICD_BASE			0xc4301000
#define GICC_BASE			0xc4302000
#endif

/* For splashscreen */
#ifdef CONFIG_DM_VIDEO
#define STDOUT_CFG "vidconsole,serial"
#else
#define STDOUT_CFG "serial"
#endif

#ifdef CONFIG_USB_KEYBOARD
#define STDIN_CFG "usbkbd,serial"
#else
#define STDIN_CFG "serial"
#endif

#define CONFIG_CPU_ARMV8
#define CONFIG_REMAKE_ELF
#define CONFIG_SYS_MAXARGS		32
#define CONFIG_SYS_CBSIZE		1024

#define CONFIG_SYS_SDRAM_BASE		0
#define CONFIG_SYS_INIT_SP_ADDR		0x20000000
#define CONFIG_SYS_BOOTM_LEN		(64 << 20) /* 64 MiB */

/* ROM USB boot support, auto-execute boot.scr at scriptaddr */
#define BOOTENV_DEV_ROMUSB(devtypeu, devtypel, instance) \
	"bootcmd_romusb=" \
		"if test \"${boot_source}\" = \"usb\" && " \
				"test -n \"${scriptaddr}\"; then " \
			"echo '(ROM USB boot)'; " \
			"source ${scriptaddr}; " \
		"fi\0"

#define BOOTENV_DEV_NAME_ROMUSB(devtypeu, devtypel, instance)	\
		"romusb "

#ifdef CONFIG_CMD_USB
#define BOOT_TARGET_DEVICES_USB(func) func(USB, usb, 0)
#else
#define BOOT_TARGET_DEVICES_USB(func)
#endif

#ifdef CONFIG_CMD_NVME
	#define BOOT_TARGET_NVME(func) func(NVME, nvme, 0)
#else
	#define BOOT_TARGET_NVME(func)
#endif

#ifdef CONFIG_CMD_SCSI
	#define BOOT_TARGET_SCSI(func) func(SCSI, scsi, 0)
#else
	#define BOOT_TARGET_SCSI(func)
#endif

#ifndef BOOT_TARGET_DEVICES
#define BOOT_TARGET_DEVICES(func) \
	func(ROMUSB, romusb, na)  \
	func(MMC, mmc, 0) \
	func(MMC, mmc, 1) \
	func(MMC, mmc, 2) \
	BOOT_TARGET_DEVICES_USB(func) \
	BOOT_TARGET_NVME(func) \
	BOOT_TARGET_SCSI(func) \
	func(PXE, pxe, na) \
	func(DHCP, dhcp, na)
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
	"test $ll = 0 && fdt addr $fdtcontroladdr && fdt get addr logoaddr /logo data && bmp display $logoaddr m m && echo [i] display embed logo; "

#define PREBOOT_CMD "run load_logo; usb start; kbi init; sleep 1;"

#define CONFIG_HOSTNAME CONFIG_DEFAULT_DEVICE_TREE
#define CONFIG_BOOTP_SEND_HOSTNAME 1

#include <config_distro_bootcmd.h>

#ifndef CONFIG_EXTRA_ENV_SETTINGS
#define CONFIG_EXTRA_ENV_SETTINGS \
	"load_logo=" PREBOOT_LOAD_LOGO "\0" \
	"preboot=" PREBOOT_CMD "\0" \
	"stdin=" STDIN_CFG "\0" \
	"stdout=" STDOUT_CFG "\0" \
	"stderr=" STDOUT_CFG "\0" \
	"kernel_comp_addr_r=0x0d080000\0" \
	"kernel_comp_size=0x2000000\0" \
	"fdt_addr_r=0x08008000\0" \
	"loadaddr=0x01000000\0" \
	"scriptaddr=0x08000000\0" \
	"kernel_addr_r=0x08080000\0" \
	"pxefile_addr_r=0x01080000\0" \
	"fdtoverlay_addr_r=0x01000000\0" \
	"ramdisk_addr_r=0x13000000\0" \
	"fdtfile=amlogic/" CONFIG_DEFAULT_DEVICE_TREE ".dtb\0" \
	BOOTENV
#endif


#endif /* __MESON64_CONFIG_H */
