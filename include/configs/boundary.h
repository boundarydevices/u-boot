/*
 * common configuration settings for the Boundary Devices boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __BOUNDARY_H
#define __BOUNDARY_H

#define CONFIG_BOARD_EARLY_INIT_F
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
#define CONFIG_FSL_CAAM
#define CONFIG_CMD_DEKBLOB
#define CONFIG_SYS_FSL_SEC_LE
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
#define CONFIG_CMD_I2C
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
#define CONFIG_EFI_PARTITION
#endif


#ifdef CONFIG_CMD_SF
#ifndef CONFIG_FSL_QSPI
#define CONFIG_MXC_SPI
#endif
#define CONFIG_SF_DEFAULT_BUS  0
#ifndef CONFIG_SF_DEFAULT_CS
#define CONFIG_SF_DEFAULT_CS   0
#endif
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

#ifdef CONFIG_PHY_MICREL_KSZ9021
#define CONFIG_PHY_MICREL
#endif

#if defined(CONFIG_PHY_ATHEROS) || defined(CONFIG_PHY_MICREL_KSZ9021)
#define CONFIG_PHYLIB
#define CONFIG_FEC_MXC

#ifndef CONFIG_FEC_MXC_PHYADDR
#define CONFIG_FEC_MXC_PHYADDR		6
#endif

#define CONFIG_CMD_MII
#define CONFIG_MII
#ifndef IMX_FEC_BASE
#define IMX_FEC_BASE			ENET_BASE_ADDR
#endif
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#endif


/*
 * SATA Configs
 */
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

/* USB gadget support */
#ifdef CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_CDC
#define CONFIG_NETCONSOLE
#endif

/* USB Configs */
#ifdef CONFIG_USB_MAX_CONTROLLER_COUNT
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#ifdef CONFIG_MX7D
#define CONFIG_USB_EHCI_MX7
#elif defined(CONFIG_MX51)
#define CONFIG_USB_EHCI_MX5
#else
#define CONFIG_USB_EHCI_MX6
#endif
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_MCS7830
#define CONFIG_USB_ETHER_SMSC95XX
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */
#ifndef CONFIG_MXC_USB_PORTSC
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#endif
#ifndef CONFIG_MXC_USB_FLAGS
#define CONFIG_MXC_USB_FLAGS	0
#endif
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL_VIA_CONTROL_EP
#endif

/* Miscellaneous commands */
#define CONFIG_CMD_BMODE
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP

/* Framebuffer and LCD */
#ifdef CONFIG_VIDEO
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
#ifdef CONFIG_IMX_HDMI
#define CONFIG_CMD_HDMIDETECT
#endif
#endif

#ifndef CONFIG_PREBOOT
#define CONFIG_PREBOOT                 ""
#endif

#ifdef CONFIG_CMD_SATA
#define BD_UMS_SATA_PREPARE \
	"if itest.s sata == ${dtype}; then " \
		"initcmd='sata init' ; disks=0;" \
	"fi; "
#else
#define BD_UMS_SATA_PREPARE
#endif

#ifndef BD_BOOT_DEV_SATA
#ifdef CONFIG_CMD_SATA
#define BD_BOOT_DEV_SATA "sata "
#else
#define BD_BOOT_DEV_SATA
#endif
#endif

#ifndef BD_BOOT_DEV_MMC
#ifdef CONFIG_CMD_MMC
#define BD_BOOT_DEV_MMC "mmc "
#else
#define BD_BOOT_DEV_MMC
#endif
#endif

#ifndef BD_BOOT_DEV_USB
#ifdef CONFIG_USB_STORAGE
#define BD_BOOT_DEV_USB "usb "
#else
#define BD_BOOT_DEV_USB
#endif
#endif

#ifndef BD_BOOT_DEVS
#define BD_BOOT_DEVS BD_BOOT_DEV_SATA BD_BOOT_DEV_MMC BD_BOOT_DEV_USB
#endif

#ifndef BD_MMC_DISKS
#if (CONFIG_SYS_FSL_USDHC_NUM == 1)
#define BD_MMC_DISKS "0"
#elif (CONFIG_SYS_FSL_USDHC_NUM == 2)
#define BD_MMC_DISKS "0 1"
#else
#define BD_MMC_DISKS "0 1 2"
#endif
#endif

#ifndef BD_FASTBOOT_FLASH_MMC_DEV
#if (CONFIG_SYS_FSL_USDHC_NUM == 1)
#define BD_FASTBOOT_FLASH_MMC_DEV   0
#elif (CONFIG_SYS_FSL_USDHC_NUM == 2)
#define BD_FASTBOOT_FLASH_MMC_DEV   1
#else
#define BD_FASTBOOT_FLASH_MMC_DEV   2
#endif
#endif

#ifndef BD_MMC_UMS_DISKS
#define BD_MMC_UMS_DISKS BD_MMC_DISKS
#endif

#ifndef BD_UMSDEVS
#define BD_UMSDEVS BD_BOOT_DEV_SATA BD_BOOT_DEV_MMC
#endif

#ifndef BD_USB_START
#define BD_USB_START "usb start ;"
#endif

#ifndef BD_CONSOLE
#define BD_CONSOLE "ttymxc1"
#endif

#ifdef CONFIG_VIDEO
#define CONFIG_CMD_BMP
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
#else
#define BD_RAM_BASE	0x10000000
#define BD_RAM_SCRIPT	"10008000"
#define BD_RAM_KERNEL	"10800000"
#define BD_RAM_RAMDISK	"12800000"
#define BD_RAM_FDT	"13000000"
#endif

#define BD_BOOTCMD_STD \
		"script=/6x_bootscript;" \
		"run runscript;" \
		BD_STDOUT_VIDEO \
		"echo ; echo 6x_bootscript not found ; " \
		"echo ; echo serial console at 115200, 8N1 ; echo ; " \
		"echo details at http://boundarydevices.com/6q_bootscript ; " \
		BD_STDOUT_SERIAL \
		BD_STDIN_USBKBD \
		"for dtype in ${umsdevs} ; do " \
			"initcmd='mmc rescan' ; disks=${mmc_ums_disks};" \
			BD_UMS_SATA_PREPARE \
			"for disk in ${disks} ; do " \
				"if $initcmd && $dtype dev $disk ; then " \
					BD_STDOUT_VIDEO \
					"echo expose ${dtype} ${disk} " \
						"over USB; " \
					"ums 0 $dtype $disk ;" \
				"fi; " \
			"done;" \
		"done;" \
		BD_STDOUT_VIDEO \
		"echo no block devices found;"

#ifndef BD_BOOTCMD
#define BD_BOOTCMD BD_BOOTCMD_STD
#endif

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

#define BD_BOUNDARY_ENV_SETTINGS \
	"active_partition=1\0" \
	"bootdevs=" BD_BOOT_DEVS "\0" \
	"bootcmd=" BD_BOOTCMD "\0" \
	"clearenv=if sf probe || sf probe ; then " \
		"sf erase 0xc0000 0x2000 && " \
		"echo restored environment to factory default ; fi\0" \
	LOG_CMA_STR \
	"console=" BD_CONSOLE "\0" \
	"dfu_alt_info=u-boot raw 0x0 0xc0000\0" \
	"fdt_addr=" BD_RAM_FDT "\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"loadsplash=if sf probe ; then sf read ${splashimage} ${splashflash} ${splashsize} ; fi\0" \
	LOG_LEVEL_STR \
	"mmc_disks=" BD_MMC_DISKS "\0" \
	"mmc_ums_disks=" BD_MMC_UMS_DISKS "\0" \
	"net_upgradeu=dhcp " BD_RAM_SCRIPT " net_upgradeu && source " BD_RAM_SCRIPT "\0" \
	"rundfu=dfu 0 sf 0:0:25000000:0\0" \
	"runscript=" \
		"for dtype in ${bootdevs}; do " \
			"if itest.s usb == ${dtype} ; then " \
				BD_USB_START \
			"fi; " \
			"disks=${mmc_disks};" \
			"if itest.s mmc != ${dtype} ; then " \
				"disks=0;" \
			"fi; " \
			"for disk in ${disks} ; do " \
				"${dtype} dev ${disk} ;" \
				"load ${dtype} ${disk}:${active_partition} " BD_RAM_SCRIPT " ${script} && " \
				"source " BD_RAM_SCRIPT ";" \
			"done ; " \
		"done;\0" \
	"otg_upgradeu=run usbnetwork; tftp " BD_RAM_SCRIPT " net_upgradeu && source " BD_RAM_SCRIPT "\0" \
	"splashflash=" BD_SPLASH_FLASH "\0" \
	"uboot_defconfig=" CONFIG_DEFCONFIG "\0" \
	"umsdevs=" BD_UMSDEVS "\0" \
	"upgradeu=script=/6x_upgrade; run runscript\0" \
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

/* Miscellaneous configurable options */
#define CONFIG_CMD_TIME
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START       BD_RAM_BASE
#define CONFIG_SYS_MEMTEST_END	       (BD_RAM_BASE + 0x00010000)
#define CONFIG_SYS_MEMTEST_SCRATCH     (BD_RAM_BASE + 0x00800000)
#define CONFIG_SYS_ALT_MEMTEST

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#if defined(CONFIG_MX51)
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

#ifndef CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_IS_IN_SPI_FLASH
#endif

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
#define CONFIG_USB_GADGET
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_FUNCTION_MASS_STORAGE
#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_USB_GADGET_VBUS_DRAW	2

/* Netchip IDs */
#define CONFIG_G_DNL_VENDOR_NUM 0x0525
#define CONFIG_G_DNL_PRODUCT_NUM 0xa4a5
#define CONFIG_G_DNL_MANUFACTURER "Boundary"

#define CONFIG_USB_FUNCTION_FASTBOOT
#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_FASTBOOT_BUF_ADDR   CONFIG_SYS_LOAD_ADDR
#define CONFIG_FASTBOOT_BUF_SIZE   0x26000000
#define CONFIG_FASTBOOT_FLASH
#define CONFIG_FASTBOOT_FLASH_MMC_DEV   BD_FASTBOOT_FLASH_MMC_DEV
#define CONFIG_CMD_GPT
#define CONFIG_CMD_PART
#define CONFIG_PARTITION_UUIDS

/* USB Device Firmware Update support */
#define CONFIG_USB_FUNCTION_DFU
#define CONFIG_DFU_SF
#define CONFIG_CMD_DFU
#define CONFIG_SYS_DFU_DATA_BUF_SIZE	0xc0000
#define DFU_MANIFEST_POLL_TIMEOUT	25000
#endif

#endif
