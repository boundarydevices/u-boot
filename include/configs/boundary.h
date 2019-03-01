/*
 * common configuration settings for the Boundary Devices boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __BOUNDARY_H
#define __BOUNDARY_H

#ifdef CONFIG_BOARD_TYPE
#undef CONFIG_SYS_BOARD
#define CONFIG_SYS_BOARD CONFIG_BOARD_TYPE
#endif

#define CONFIG_MISC_INIT_R
#define CONFIG_BOARD_LATE_INIT

#ifndef CONFIG_SYS_MALLOC_LEN
/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(12 * 1024 * 1024)
#endif

#define CONFIG_MXC_UART
#ifndef CONFIG_MXC_UART_BASE
#define CONFIG_MXC_UART_BASE		UART2_BASE
#endif

/* Secure boot (HAB) support */
#ifdef CONFIG_SECURE_BOOT
#define CONFIG_CSF_SIZE			0x2000
#define CONFIG_SYS_FSL_SEC_COMPAT	4
#define CONFIG_CMD_DEKBLOB
#endif

/* SPL magic */
#ifdef CONFIG_SPL
#define CONFIG_SPL_FRAMEWORK
#define CONFIG_SPL_TEXT_BASE		0x00908400
#define CONFIG_SPL_PAD_TO 0x400
#define CONFIG_SPL_START_S_PATH		"arch/arm/cpu/armv7"
#define CONFIG_SPL_STACK		0x0091FFB8

#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_USB
#define CONFIG_SPL_WATCHDOG_SUPPORT
#define CONFIG_SPL_GPIO_SUPPORT

/* SPI flash. */
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SPL_SPI_BUS		0
#define CONFIG_SPL_SPI_CS		0
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x20000

#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SYS_SPL_MALLOC_START	0x00916000
#define CONFIG_SYS_SPL_MALLOC_SIZE	0x2000
#endif

/* I2C Configs */
#ifdef BD_I2C_MASK
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#if (BD_I2C_MASK & 1)
#define CONFIG_SYS_I2C_MXC_I2C1
#endif
#if (BD_I2C_MASK & 2)
#define CONFIG_SYS_I2C_MXC_I2C2
#endif
#if (BD_I2C_MASK & 4)
#define CONFIG_SYS_I2C_MXC_I2C3
#endif
#if (BD_I2C_MASK & 8)
#define CONFIG_SYS_I2C_MXC_I2C4
#endif
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_I2C_EDID
#endif

/* MMC Configs */
#ifdef CONFIG_SYS_FSL_USDHC_NUM
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#endif

#ifdef CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_BUS  0
#ifndef CONFIG_SF_DEFAULT_CS
#define CONFIG_SF_DEFAULT_CS   0
#endif
#ifndef CONFIG_SF_DEFAULT_SPEED
#define CONFIG_SF_DEFAULT_SPEED 25000000
#endif
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

#ifdef CONFIG_FEC_MXC
#ifndef CONFIG_FEC_MXC_PHYADDR
#define CONFIG_FEC_MXC_PHYADDR		6
#endif

#define CONFIG_MII
#ifndef IMX_FEC_BASE
#define IMX_FEC_BASE			ENET_BASE_ADDR
#endif
#define CONFIG_FEC_XCV_TYPE		RGMII
#ifdef CONFIG_FEC_ENET2
#define CONFIG_ETHPRIME			"FEC0"
#else
#define CONFIG_ETHPRIME			"FEC"
#endif

#ifndef GP_RGMII_PHY_RESET
#ifdef CONFIG_MX6SX
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(2, 7)
#define GP_RGMII2_PHY_RESET	IMX_GPIO_NR(2, 6)
#elif defined(CONFIG_MX7D)
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(6, 10)
#elif defined(CONFIG_MX51)
#else
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(1, 27)
#endif
#endif

#endif

/*
 * SATA Configs
 */
#ifdef CONFIG_CMD_SATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#endif

/* USB gadget support */
#ifdef CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_NETCONSOLE
#endif

/* USB Configs */
#ifdef CONFIG_USB_MAX_CONTROLLER_COUNT
#if defined(CONFIG_MX51) || defined(CONFIG_MX53)
#define CONFIG_USB_EHCI_MX5
#endif
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */
#ifndef CONFIG_MXC_USB_PORTSC
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#endif
#ifndef CONFIG_MXC_USB_FLAGS
#define CONFIG_MXC_USB_FLAGS	0
#endif
#endif

/* Framebuffer and LCD */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE (6 * 1024 * 1024)
#define CONFIG_BMP_16BPP
#define CONFIG_CMD_FBPANEL
#if defined(CONFIG_MX6SX)
#define CONFIG_VIDEO_MXS
#define MXS_LCDIF_BASE MX6SX_LCDIF1_BASE_ADDR
#elif defined(CONFIG_MX7D)
#define CONFIG_VIDEO_MXS
#else
#define CONFIG_VIDEO_IPUV3
#endif
#endif

#ifndef CONFIG_PREBOOT
#define CONFIG_PREBOOT                 ""
#endif

#ifdef CONFIG_CMD_SATA
#define DISTRO_BOOT_DEV_SATA(func) func(SATA, sata, 0)
#else
#define DISTRO_BOOT_DEV_SATA(func)
#endif

#ifdef CONFIG_CMD_MMC
#if (CONFIG_SYS_FSL_USDHC_NUM == 1)
#define DISTRO_BOOT_DEV_MMC(func) func(MMC, mmc, 0)
#elif (CONFIG_SYS_FSL_USDHC_NUM == 2)
#define DISTRO_BOOT_DEV_MMC(func) func(MMC, mmc, 0) func(MMC, mmc, 1)
#else
#define DISTRO_BOOT_DEV_MMC(func) func(MMC, mmc, 0) func(MMC, mmc, 1) func(MMC, mmc, 2)
#endif
#else
#define DISTRO_BOOT_DEV_MMC(func)
#endif

#ifdef CONFIG_USB_STORAGE
#define DISTRO_BOOT_DEV_USB(func) func(USB, usb, 0)
#else
#define DISTRO_BOOT_DEV_USB(func)
#endif

#ifndef BD_CONSOLE
#if CONFIG_MXC_UART_BASE == UART2_BASE
#define BD_CONSOLE	"ttymxc1"
#elif CONFIG_MXC_UART_BASE == UART1_BASE
#define BD_CONSOLE	"ttymxc0"
#endif
#endif

#ifndef BD_CONSOLE_STR
#define BD_CONSOLE_STR	"console=" BD_CONSOLE "\0"
#endif

#if defined(CONFIG_VIDEO) && !defined(BD_NOVIDEO_CONSOLE)
#define BD_STDOUT_SERIAL	"setenv stdout serial;"
#define BD_STDOUT_VIDEO		"setenv stdout serial,vga; "
#define BD_STDIN_USBKBD		"setenv stdin serial,usbkbd;"
#else
#define BD_STDOUT_SERIAL
#define BD_STDOUT_VIDEO
#define BD_STDIN_USBKBD
#endif

#ifndef BD_SPLASH_FLASH
#define BD_SPLASH_FLASH	"c2000"
#endif

#if defined(CONFIG_MX6SX) || defined(CONFIG_MX7D)
#define BD_RAM_BASE	0x80000000
#define BD_RAM_SCRIPT	"80008000"
#define BD_RAM_KERNEL	"80800000"
#define BD_RAM_RAMDISK	"82800000"
#define BD_RAM_FDT	"83000000"
#elif defined(CONFIG_MX51)
#define BD_RAM_BASE	0x90000000
#define BD_RAM_SCRIPT	"90008000"
#define BD_RAM_KERNEL	"90800000"
#define BD_RAM_RAMDISK	"92800000"
#define BD_RAM_FDT	"93000000"
#elif defined(CONFIG_MX53)
#define BD_RAM_BASE	0x70000000
#define BD_RAM_SCRIPT	"70008000"
#define BD_RAM_KERNEL	"70800000"
#define BD_RAM_RAMDISK	"72800000"
#define BD_RAM_FDT	"73000000"
#else
#define BD_RAM_BASE	0x10000000
#define BD_RAM_SCRIPT	"10008000"
#define BD_RAM_KERNEL	"10800000"
#define BD_RAM_RAMDISK	"12800000"
#define BD_RAM_FDT	"13000000"
#endif

#ifndef BD_SKIP_FUSES
#ifndef BD_FUSE1
#if defined(CONFIG_MX6SX)
#define BD_FUSE1		"0 5"
#define BD_FUSE1_VAL		"08000030"	/* CS0 */
#define BD_FUSE2		"0 6"
#define BD_FUSE2_VAL		"00000010"
#elif defined(CONFIG_MX6Q) || defined(CONFIG_MX6S) || defined(CONFIG_MX6DL)
#define BD_FUSE1		"0 5"
#define BD_FUSE1_VAL		"18000030"	/* CS1 */
#define BD_FUSE2		"0 6"
#define BD_FUSE2_VAL		"00000010"
#elif defined(CONFIG_MX7D)
#define BD_FUSE1		"1 3"
#define BD_FUSE1_VAL		"10004000"	/* QSPI */
#endif
#endif

#ifndef BD_FUSE_MAC1A
#if defined(CONFIG_MX7D)
#define BD_FUSE_MAC1A		"9 1"
#define BD_FUSE_MAC1A_VAL	"00000019"
#define BD_FUSE_MAC1B		"9 0"
#else
#define BD_FUSE_MAC1A		"4 3"
#define BD_FUSE_MAC1A_VAL	"00000019"
#define BD_FUSE_MAC1B		"4 2"
#endif
#endif
#endif

#ifdef BD_FUSE1
#define BD_FUSE1_STR		"fuse1=" BD_FUSE1 "\0"
#define BD_FUSE1_VAL_STR	"fuse1_val=" BD_FUSE1_VAL "\0"
#else
#define BD_FUSE1_STR		""
#define BD_FUSE1_VAL_STR	""
#endif

#ifdef BD_FUSE2
#define BD_FUSE2_STR		"fuse2=" BD_FUSE2 "\0"
#define BD_FUSE2_VAL_STR	"fuse2_val=" BD_FUSE2_VAL "\0"
#else
#define BD_FUSE2_STR		""
#define BD_FUSE2_VAL_STR	""
#endif

#ifdef BD_FUSE_MAC1A
#define BD_FUSE_MAC1A_STR	"fuse_mac1a=" BD_FUSE_MAC1A "\0"
#define BD_FUSE_MAC1A_VAL_STR	"fuse_mac1a_val=" BD_FUSE_MAC1A_VAL "\0"
#define BD_FUSE_MAC1B_STR	"fuse_mac1b=" BD_FUSE_MAC1B "\0"
#else
#define BD_FUSE_MAC1A_STR	""
#define BD_FUSE_MAC1A_VAL_STR	""
#define BD_FUSE_MAC1B_STR	""
#endif

#ifndef BOOT_TARGET_DEVICES
#define BOOT_TARGET_DEVICES(func) \
	DISTRO_BOOT_DEV_MMC(func) \
	DISTRO_BOOT_DEV_SATA(func) \
	DISTRO_BOOT_DEV_USB(func)
#endif

#define BOOTENV_EXTRA_BOOT_SCRIPTS " 6x_bootscript "

#include <config_distro_bootcmd.h>

#ifdef BD_LOG_LEVEL
#define LOG_LEVEL_STR "loglevel=" BD_LOG_LEVEL "\0"
#else
#define LOG_LEVEL_STR ""
#endif

#ifdef BD_CMA
#define LOG_CMA_STR "cma=" BD_CMA "\0"
#else
#define LOG_CMA_STR ""
#endif

#ifndef CONFIG_ENV_OFFSET
#define CONFIG_ENV_OFFSET		0xc0000
#endif

#define BD_BOUNDARY_ENV_SETTINGS \
	"clearenv=if sf probe || sf probe ; then " \
		"sf erase " __stringify(CONFIG_ENV_OFFSET) " 0x2000 && " \
		"echo restored environment to factory default ; fi\0" \
	LOG_CMA_STR \
	BD_CONSOLE_STR \
	"dfu_alt_info=u-boot raw 0x0 0xc0000\0" \
	"fdt_addr=" BD_RAM_FDT "\0" \
	"fdt_high=0xffffffff\0" \
	BD_FUSE1_STR \
	BD_FUSE1_VAL_STR \
	BD_FUSE2_STR \
	BD_FUSE2_VAL_STR \
	BD_FUSE_MAC1A_STR \
	BD_FUSE_MAC1A_VAL_STR \
	BD_FUSE_MAC1B_STR \
	"initrd_high=0xffffffff\0" \
	"loadsplash=if sf probe ; then sf read ${splashimage} ${splashflash} ${splashsize} ; fi\0" \
	LOG_LEVEL_STR \
	"net_fuses=dhcp " BD_RAM_SCRIPT " prog_fuses.scr && source " BD_RAM_SCRIPT "\0" \
	"net_program=next=prog_fuses.scr; run net_upgradeu\0" \
	"net_upgradeu=dhcp " BD_RAM_SCRIPT " net_upgradeu.scr && source " BD_RAM_SCRIPT "\0" \
	"rundfu=dfu 0 sf 0:0:25000000:0\0" \
	"otg_fuses=run usbnetwork; tftp " BD_RAM_SCRIPT " prog_fuses.scr && source " BD_RAM_SCRIPT "\0" \
	"otg_program=next=prog_fuses.scr; run otg_upgradeu\0" \
	"otg_upgradeu=run usbnetwork; tftp " BD_RAM_SCRIPT " net_upgradeu.scr && source " BD_RAM_SCRIPT "\0" \
	"program=next=prog_fuses.scr; run upgradeu\0" \
	"scriptaddr=" BD_RAM_SCRIPT "\0" \
	"splashflash=" BD_SPLASH_FLASH "\0" \
	"uboot_defconfig=" CONFIG_DEFCONFIG "\0" \
	"upgradeu=setenv boot_scripts upgrade.scr; boot\0" \
	"usb_pgood_delay=2000\0" \
	"usbnet_devaddr=00:19:b8:00:00:02\0" \
	"usbnet_hostaddr=00:19:b8:00:00:01\0" \
	"usbnetwork=setenv ethact usb_ether; " \
		"setenv ipaddr 10.0.0.2; " \
		"setenv netmask 255.255.255.0; " \
		"setenv serverip 10.0.0.1;\0" \
	"usbrecover=run usbnetwork;" \
		"setenv bootargs console=${console},115200; " \
		"tftpboot " BD_RAM_KERNEL " 10.0.0.1:uImage-${board}-recovery && " \
		"tftpboot " BD_RAM_RAMDISK " 10.0.0.1:uramdisk-${board}-recovery.img && " \
		"bootm " BD_RAM_KERNEL " " BD_RAM_RAMDISK "\0" \
	BOOTENV

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START       BD_RAM_BASE
#define CONFIG_SYS_MEMTEST_END	       (BD_RAM_BASE + 0x00010000)
#define CONFIG_SYS_MEMTEST_SCRATCH     (BD_RAM_BASE + 0x00800000)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#if defined(CONFIG_MX51) || defined(CONFIG_MX53)
#define PHYS_SDRAM		       CSD0_BASE_ADDR
#else
#define PHYS_SDRAM		       MMDC0_ARB_BASE_ADDR
#endif

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

#if defined(CONFIG_ENV_IS_IN_MMC)
#undef CONFIG_ENV_OFFSET
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_SECT_SIZE		(8 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif

/*
 * PCI express
 */
/* #define CONFIG_CMD_PCI */
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#endif

#define CONFIG_CMD_UNZIP

#ifdef CONFIG_CI_UDC
#define CONFIG_USB_GADGET_VBUS_DRAW	2

/* USB Device Firmware Update support */
#define CONFIG_DFU_SF
#define CONFIG_SYS_DFU_DATA_BUF_SIZE	0xc0000
#define DFU_MANIFEST_POLL_TIMEOUT	25000
#endif

#endif
