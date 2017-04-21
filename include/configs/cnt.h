/*
 * Copyright (C) 2015 Boundary Devices, Inc.
 *
 * Configuration settings for the Boundary Devices CNT board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_BOOTDELAY	1

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3769

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(12 * 1024 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MISC_INIT_R
#define CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_CDC
#define CONFIG_NETCONSOLE

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	       UART2_BASE

/* SPI flash. */
#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_SST
#define CONFIG_MXC_SPI
#define CONFIG_MXC_SPI_DISPLAY
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_CS   0
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_I2C_EDID

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR      0
#define CONFIG_SYS_FSL_USDHC_NUM       2
#define CONFIG_EFI_PARTITION

/* Ethernet Configs */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		4
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_MCS7830
#define CONFIG_USB_ETHER_SMSC95XX
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL_VIA_CONTROL_EP

/* Miscellaneous commands */
#define CONFIG_CMD_BMODE

/* Framebuffer and LCD */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE (6 * 1024 * 1024)
#define CONFIG_BMP_16BPP
#define CONFIG_IPUV3_CLK 264000000
#define CONFIG_CONSOLE_MUX
#define CONFIG_CMD_FBPANEL

#ifdef CONFIG_CMD_MMC
#define CONFIG_DRIVE_MMC "mmc "
#else
#define CONFIG_DRIVE_MMC
#endif

#ifdef CONFIG_USB_STORAGE
#define CONFIG_DRIVE_USB "usb "
#else
#define CONFIG_DRIVE_USB
#endif

#define CONFIG_DRIVE_TYPES CONFIG_DRIVE_USB CONFIG_DRIVE_MMC
#define CONFIG_UMSDEVS CONFIG_DRIVE_MMC

#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootdevs=" CONFIG_DRIVE_TYPES "\0" \
	"umsdevs=" CONFIG_UMSDEVS "\0" \
	"console=ttymxc1\0" \
	"clearenv=if sf probe || sf probe || sf probe 1 ; then " \
		"sf erase 0xc0000 0x2000 && " \
		"echo restored environment to factory default ; fi\0" \
	"bootcmd=for dtype in ${bootdevs}" \
		"; do " \
			"if itest.s \"xusb\" == \"x${dtype}\" ; then " \
				"usb start ;" \
			"fi; " \
			"for disk in 0 1 ; do ${dtype} dev ${disk} ;" \
				"load " \
					"${dtype} ${disk}:1 " \
					"10008000 " \
					"/6x_bootscript" \
					"&& source 10008000 ; " \
			"done ; " \
		"done; " \
		"setenv stdout serial,vga ; " \
		"echo ; echo 6x_bootscript not found ; " \
		"echo ; echo serial console at 115200, 8N1 ; echo ; " \
		"echo details at http://boundarydevices.com/6q_bootscript ; " \
		"setenv stdout serial;" \
		"setenv stdin serial,usbkbd;" \
		"for dtype in ${umsdevs} ; do " \
			"initcmd='mmc rescan' ;" \
			"for disk in 0 1 ; do " \
				"if $initcmd && $dtype dev $disk ; then " \
					"setenv stdout serial,vga; " \
					"echo expose ${dtype} ${disk} " \
						"over USB; " \
					"ums 0 $dtype $disk ;" \
				"fi; " \
		"	done; " \
		"done ;" \
		"setenv stdout serial,vga; " \
		"echo no block devices found;" \
		"\0" \
	"dfu_alt_info=u-boot raw 0x0 0xc0000\0" \
	"fdt_addr=0x13000000\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"loadsplash=if sf probe ; then sf read ${splashimage} c2000 ${splashsize} ; fi\0" \
	"rundfu=dfu 0 sf 0:0:25000000:0\0" \
	"uboot_defconfig=" CONFIG_DEFCONFIG "\0" \
	"upgradeu=for dtype in ${bootdevs}" \
		"; do " \
		"for disk in 0 1 ; do ${dtype} dev ${disk} ;" \
			"load ${dtype} ${disk}:1 10008000 " \
				"/6x_upgrade " \
				"&& source 10008000 ; " \
		"done ; " \
	"done\0" \
	"usbnet_devaddr=00:19:b8:00:00:02\0" \
	"usbnet_hostaddr=00:19:b8:00:00:01\0" \
	"usbrecover=setenv ethact usb_ether; " \
		"setenv ipaddr 10.0.0.2; " \
		"setenv netmask 255.255.255.0; " \
		"setenv serverip 10.0.0.1; " \
		"setenv bootargs console=ttymxc1,115200; " \
		"tftpboot 10800000 10.0.0.1:uImage-${board}-recovery" \
		"&& tftpboot 12800000 10.0.0.1:uramdisk-${board}-recovery.img " \
		"&& bootm 10800000 12800000\0" \

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END	       0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#define PHYS_SDRAM		       MMDC0_ARB_BASE_ADDR
#define CONFIG_RESET_CAUSE_ADDR	       (PHYS_SDRAM + 0x80)

#define CONFIG_SYS_SDRAM_BASE	       PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(8 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED

#define CONFIG_CMD_BMP

#define CONFIG_CMD_TIME
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_CMD_ELF

#define CONFIG_CMD_UNZIP

#define CONFIG_USB_GADGET
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USBDOWNLOAD_GADGET
#define CONFIG_USB_GADGET_VBUS_DRAW	2

/* Netchip IDs */
#define CONFIG_G_DNL_VENDOR_NUM 0x0525
#define CONFIG_G_DNL_PRODUCT_NUM 0xa4a5
#define CONFIG_G_DNL_MANUFACTURER "Boundary"

#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_USB_FASTBOOT_BUF_ADDR   CONFIG_SYS_LOAD_ADDR
#define CONFIG_USB_FASTBOOT_BUF_SIZE   0x07000000

/* USB Device Firmware Update support */
#define CONFIG_DFU_FUNCTION
#define CONFIG_DFU_SF
#define CONFIG_CMD_DFU
#define CONFIG_SYS_DFU_DATA_BUF_SIZE	0xc0000
#define DFU_MANIFEST_POLL_TIMEOUT	25000

#endif	       /* __CONFIG_H */
