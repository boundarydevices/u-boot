/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the SolidRun mx6 based boards
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __MX6CUBOXI_CONFIG_H
#define __MX6CUBOXI_CONFIG_H

#include "mx6_common.h"

#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"

#define CONFIG_IMX6_THERMAL

#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_UART
#define CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR

/* Ethernet Configuration */
#define CONFIG_FEC_MXC
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_FEC_MXC_PHYADDR		0
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

/* Framebuffer */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_IPUV3_CLK		260000000
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_HDMI
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_IMX_VIDEO_SKIP
#define CONFIG_CONSOLE_MUX

/* USB */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL
#define CONFIG_PREBOOT \
	"if hdmidet; then " \
		"usb start; "		       \
		"setenv stdin  serial,usbkbd; "\
		"setenv stdout serial,vga; "   \
		"setenv stderr serial,vga; "   \
	"else " \
		"setenv stdin  serial; " \
		"setenv stdout serial; " \
		"setenv stderr serial; " \
	"fi;"

/* Command definition */

#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV	"ttymxc0"
#define CONFIG_MMCROOT		"/dev/mmcblk0p2"
#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC2 */

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"fdtfile=undefined\0" \
	"fdt_addr_r=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"bootm_size=0x10000000\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"update_sd_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if mmc dev ${mmcdev}; then "	\
			"if ${get_cmd} ${update_sd_firmware_filename}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr_r} ${fdtfile}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr_r}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr_r} ${fdtfile}; then " \
				"bootz ${loadaddr} - ${fdt_addr_r}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"findfdt="\
		"if test $board_name = HUMMINGBOARD && test $board_rev = MX6Q ; then " \
			"setenv fdtfile imx6q-hummingboard.dtb; fi; " \
		"if test $board_name = HUMMINGBOARD && test $board_rev = MX6DL ; then " \
			"setenv fdtfile imx6dl-hummingboard.dtb; fi; " \
		"if test $board_name = CUBOXI && test $board_rev = MX6Q ; then " \
			"setenv fdtfile imx6q-cubox-i.dtb; fi; " \
		"if test $board_name = CUBOXI && test $board_rev = MX6DL ; then " \
			"setenv fdtfile imx6dl-cubox-i.dtb; fi; " \
		"if test $fdtfile = undefined; then " \
			"echo WARNING: Could not determine dtb to use; fi; \0" \

#define CONFIG_BOOTCOMMAND \
	"run findfdt; " \
	"mmc dev ${mmcdev};" \
	"if mmc rescan; then " \
		"if run loadbootscript; then " \
		"run bootscript; " \
		"else " \
			"if run loadimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"fi; " \
	"else run netboot; fi"

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define CONFIG_SYS_SDRAM_BASE          MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(8 * 64 * 1024)

#endif                         /* __MX6CUBOXI_CONFIG_H */
