/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices Nitrogen6X
 * and Freescale i.MX6Q Sabre Lite boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	0xffffffff

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

#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_SST
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_CS   0
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR      0
#define CONFIG_SYS_FSL_USDHC_NUM       1
#define CONFIG_EFI_PARTITION

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		6
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ9021

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_MAX_CONTROLLER_COUNT 1
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL_VIA_CONTROL_EP

/* Miscellaneous commands */
#define CONFIG_CMD_BMODE

/* Framebuffer and LCD */
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_CONSOLE_MUX

#define BOOTDEVS "usb mmc"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootcmd=script=/6x_bootscript; run runscript;" \
		"ums 0 mmc 0;\0" \
	"bootdevs=" BOOTDEVS "\0" \
	"clearenv=if sf probe || sf probe || sf probe 1 ; then " \
		"sf erase 0xc0000 0x2000 && " \
		"echo restored environment to factory default ; fi\0" \
	"console=ttymxc1\0" \
	"cmd_custom= \0" \
	"disable_giga=1\0" \
	"fdt_addr=0x13000000\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"usbnetwork=setenv ethact usb_ether; " \
		"setenv ipaddr 10.0.0.2; " \
		"setenv netmask 255.255.255.0; " \
		"setenv serverip 10.0.0.1;\0" \
	"uboot_defconfig=" CONFIG_DEFCONFIG "\0" \
	"upgradeu=script=/6x_upgrade; run runscript\0" \
	"disable_giga=1\0" \
	"runscript=disk=0;" \
		"for dtype in ${bootdevs} ; do " \
			"if itest.s ${dtype} == usb ; then " \
				"setexpr otgstat *0x020c9030 \\\\& 0x08000000;" \
				"if itest.l ${otgstat} -eq 0 ; then " \
					"usb start;" \
				"fi ;" \
			"fi;" \
			"load ${dtype} ${disk}:1 " \
				"10008000 $script " \
				"&& source 10008000 ; " \
		"done;" \
	"usbnet_devaddr=00:19:b8:00:00:02\0" \
	"usbnet_hostaddr=00:19:b8:00:00:01\0" \
	"usbrecover=run usbnetwork; " \
		"setenv bootargs console=ttymxc1,115200; " \
		"tftpboot 10800000 10.0.0.1:uImage-a-recovery" \
		"&& tftpboot 12800000 10.0.0.1:uramdisk-a-recovery.img " \
		"&& bootm 10800000 12800000\0" \
	"tftpu=run usbnetwork; " \
		"setenv uboot_file \"u-boot.amp\"; " \
		"tftp 12000000 10.0.0.1:$uboot_file && sf probe && " \
		"if sf read 0x12400000 0x400 $filesize ; then " \
			"if cmp.b 0x12000000 0x12400000 $filesize ; then " \
				"echo \"-- U-Boot versions match\" ; exit; " \
			"fi ; " \
			"echo \"----- (re)programming U-Boot\"; " \
			"sf erase 0 0xC0000 ; " \
			"sf write 0x12000000 0x400 $filesize && " \
			"sf read 12400000 0x400 $filesize && " \
			"if cmp.b 0x12000000 0x12400000 $filesize ; then " \
				"echo \"--- U-Boot updated\" ; echo \"cycle power\" ;" \
			"else " \
				"echo \"!! Error programming U-Boot\"; " \
			"fi; " \
			"exit; " \
		"fi; " \
		"echo \"U-Boot file ($uboot_file) not found on server\" \0" \

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END	       0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#define PHYS_SDRAM		       MMDC0_ARB_BASE_ADDR

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

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(8 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_CMD_TIME
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_CMD_GPIO
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

#endif	       /* __CONFIG_H */
