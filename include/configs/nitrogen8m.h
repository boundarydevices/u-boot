/*
 * Copyright 2018 Boundary Devices
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __NITROGEN8M_H
#define __NITROGEN8M_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#ifdef CONFIG_SECURE_BOOT
#define CONFIG_CSF_SIZE			0x2000 /* 8K region */
#endif

#define CONFIG_SPL_TEXT_BASE		0x7E1000
#define CONFIG_SPL_MAX_SIZE		(124 * 1024)
#define CONFIG_SYS_MONITOR_LEN		(512 * 1024)
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_USE_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	0x300
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION	1

#ifdef CONFIG_SPL_BUILD
/*#define CONFIG_ENABLE_DDR_TRAINING_DEBUG*/
#define CONFIG_SPL_LDSCRIPT		"arch/arm/cpu/armv8/u-boot-spl.lds"
#define CONFIG_SPL_STACK		0x187FF0
#define CONFIG_SPL_BSS_START_ADDR      0x00180000
#define CONFIG_SPL_BSS_MAX_SIZE        0x2000	/* 8 KB */
#define CONFIG_SYS_SPL_MALLOC_START    0x00182000
#define CONFIG_SYS_SPL_MALLOC_SIZE     0x2000	/* 8 KB */
#define CONFIG_SYS_ICACHE_OFF
#define CONFIG_SYS_DCACHE_OFF

#define CONFIG_SPL_ABORT_ON_RAW_IMAGE /* For RAW image gives a error info not panic */

#undef CONFIG_DM_MMC
#undef CONFIG_DM_PMIC
#undef CONFIG_DM_PMIC_PFUZE100

#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */

#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR 0x08
#endif

#define CONFIG_PREBOOT

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_REMAKE_ELF

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_POSTCLK_INIT
#define CONFIG_BOARD_LATE_INIT

/* Flat Device Tree Definitions */
#define CONFIG_OF_BOARD_SETUP

#undef CONFIG_CMD_EXPORTENV
#undef CONFIG_CMD_IMPORTENV
#undef CONFIG_CMD_IMLS

/* #define CONFIG_CMD_BMODE */
#undef CONFIG_CMD_CRC32
#undef CONFIG_BOOTM_NETBSD

/* ENET Config */
/* ENET1 */
#if defined(CONFIG_CMD_NET)
#define CONFIG_MII
#define CONFIG_ETHPRIME			"FEC"

#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_FEC_MXC_PHYADDR		4
#define FEC_QUIRK_ENET_MAC
#define GP_RGMII_PHY_RESET		IMX_GPIO_NR(1, 9)

#define CONFIG_PHY_GIGE
#define IMX_FEC_BASE			0x30BE0000

#endif

/* Link Definitions */
#define CONFIG_LOADADDR			0x40480000
#define CONFIG_SYS_TEXT_BASE		0x40200000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR

#define CONFIG_SYS_INIT_RAM_ADDR        0x40000000
#define CONFIG_SYS_INIT_RAM_SIZE        0x80000
#define CONFIG_SYS_INIT_SP_OFFSET \
        (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
        (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_SIZE			0x2000
#define CONFIG_ENV_OFFSET               (-CONFIG_ENV_SIZE)
#define CONFIG_SYS_MMC_ENV_DEV		0	/* USDHC1 */
#define CONFIG_SYS_MMC_ENV_PART         1	/* mmcblk0boot0 */

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		((CONFIG_ENV_SIZE + (2*1024) + (16*1024)) * 1024)

#define CONFIG_SYS_SDRAM_BASE           0x40000000
#define PHYS_SDRAM                      0x40000000
#define CONFIG_NR_DRAM_BANKS		1

#define CONFIG_BAUDRATE			115200

#define CONFIG_MXC_UART_BASE		UART1_BASE_ADDR

/* Monitor Command Prompt */
#define CONFIG_SYS_CBSIZE              2048
#define CONFIG_SYS_MAXARGS             64
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_IMX_BOOTAUX

#define CONFIG_FSL_USDHC

#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_SYS_FSL_ESDHC_ADDR       0

#define CONFIG_SUPPORT_EMMC_BOOT	/* eMMC specific */
#define CONFIG_SUPPORT_EMMC_RPMB
#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

#define CONFIG_MXC_GPIO

#define CONFIG_MXC_OCOTP
#define CONFIG_CMD_FUSE

/* I2C Configs */
#define CONFIG_SYS_I2C_SPEED		  100000

/* USB configs */
#ifndef CONFIG_SPL_BUILD
#define CONFIG_HAS_FSL_XHCI_USB

#ifdef CONFIG_HAS_FSL_XHCI_USB
#define CONFIG_USB_MAX_CONTROLLER_COUNT         2
#endif

#define CONFIG_USB_DWC3
#define CONFIG_USB_DWC3_GADGET
#define CONFIG_USBD_HS

#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USB_GADGET_VBUS_DRAW 2
#define CONFIG_USB_GADGET_DUALSPEED

#endif

#define CONFIG_OF_SYSTEM_SETUP

/* Framebuffer */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#endif

#ifndef BD_CONSOLE
#if CONFIG_MXC_UART_BASE == UART2_BASE_ADDR
#define BD_CONSOLE	"ttymxc1"
#elif CONFIG_MXC_UART_BASE == UART1_BASE_ADDR
#define BD_CONSOLE	"ttymxc0"
#endif
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

#ifndef BOOT_TARGET_DEVICES
#define BOOT_TARGET_DEVICES(func) \
	DISTRO_BOOT_DEV_MMC(func) \
	DISTRO_BOOT_DEV_USB(func)
#endif

#include <config_distro_bootcmd.h>
#define CONFIG_CMD_FBPANEL

#define BD_RAM_BASE	0x80000000
#define BD_RAM_SCRIPT	"40008000"
#define BD_RAM_KERNEL	"40800000"
#define BD_RAM_RAMDISK	"42800000"
#define BD_RAM_FDT	"43000000"

/* M4 specific */
#define SYS_AUXCORE_BOOTDATA_DDR	0x80000000
#define SYS_AUXCORE_BOOTDATA_TCM	0x007E0000

#define CONFIG_EXTRA_ENV_SETTINGS \
	"console=" BD_CONSOLE "\0" \
	"env_dev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"env_part=" __stringify(CONFIG_SYS_MMC_ENV_PART) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdt_addr=0x43000000\0" \
	"fdt_high=0xffffffffffffffff\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"m4boot=load ${devtype} ${devnum}:1 ${m4loadaddr} ${m4image}; " \
		"dcache flush; bootaux ${m4loadaddr}\0" \
	"m4image=m4_fw.bin\0" \
	"m4loadaddr="__stringify(SYS_AUXCORE_BOOTDATA_TCM)"\0" \
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

/*
 * PCI express
 */
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#endif
#endif
