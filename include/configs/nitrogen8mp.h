/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 NXP
 */

#ifndef __NITROGEN8MP_H
#define __NITROGEN8MP_H

#ifdef CONFIG_BOARD_TYPE
#undef CONFIG_SYS_BOARD
#define CONFIG_SYS_BOARD CONFIG_BOARD_TYPE
#endif
#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#define CONFIG_SPL_MAX_SIZE		(152 * 1024)
#define CONFIG_SYS_MONITOR_LEN		SZ_512K
#define CONFIG_SYS_UBOOT_BASE	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SPL_STACK		0x00960000
#define CONFIG_SPL_BSS_START_ADDR	0x0098FC00

#define CONFIG_SPL_BSS_MAX_SIZE		0x400	/* 1 KB */
#define CONFIG_SYS_SPL_MALLOC_START	0x42200000
#define CONFIG_SYS_SPL_MALLOC_SIZE	SZ_512K	/* 512 KB */

#define CONFIG_SPL_ABORT_ON_RAW_IMAGE

#undef CONFIG_CMD_USB
#undef CONFIG_CMD_USB_MASS_STORAGE
#undef CONFIG_USB_GADGET_MASS_STORAGE
#undef CONFIG_USB_FUNCTION_MASS_STORAGE

#undef CONFIG_DM_ETH
#undef CONFIG_DM_MMC
#undef CONFIG_DM_PMIC

#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PCA9450

#undef CONFIG_DM_I2C
#define CONFIG_SYS_I2C

#endif

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG
#define CONFIG_FASTBOOT_USB_DEV 0

#define CONFIG_REMAKE_ELF
/* ENET Config */
/* ENET1 */
#if defined(CONFIG_CMD_NET)
#define CONFIG_ETHPRIME                 "eth1" /* Set eqos to primary since we use its MDIO */

#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_FEC_MXC_PHYADDR          4
#define FEC_QUIRK_ENET_MAC

#define IMX_FEC_BASE			0x30BE0000

#define DWC_NET_PHYADDR			4
#ifdef CONFIG_DWC_ETH_QOS
#define CONFIG_SYS_NONCACHED_MEMORY     (1 * SZ_1M)     /* 1M */
#endif

#define PHY_ANEG_TIMEOUT 20000

#endif

/* Link Definitions */
#define CONFIG_LOADADDR			0x40480000

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_SYS_INIT_RAM_ADDR	0x40000000
#define CONFIG_SYS_INIT_RAM_SIZE	0x80000
#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_MMCROOT			"/dev/mmcblk2p2"  /* USDHC3 */

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		SZ_32M
/* bootm image length (Android) */
#define CONFIG_SYS_BOOTM_LEN		(96 * 1024 * 1024)

/* Totally 6GB DDR */
#define CONFIG_SYS_SDRAM_BASE		0x40000000
#define PHYS_SDRAM			0x40000000
#define PHYS_SDRAM_SIZE			(CONFIG_DDR_MB * 1024ULL * 1024ULL)

#define CONFIG_MXC_UART_BASE		UART2_BASE_ADDR

/* Monitor Command Prompt */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_CBSIZE		2048
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_IMX_BOOTAUX

#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

#ifdef CONFIG_FSL_FSPI
#define FSL_FSPI_FLASH_SIZE		SZ_32M
#define FSL_FSPI_FLASH_NUM		1
#define FSPI0_BASE_ADDR			0x30bb0000
#define FSPI0_AMBA_BASE			0x0
#define CONFIG_FSPI_QUAD_SUPPORT

#define CONFIG_SYS_FSL_FSPI_AHB
#endif

#ifdef CONFIG_NAND_MXS
#define CONFIG_CMD_NAND_TRIMFFS

/* NAND stuff */
#define CONFIG_SYS_MAX_NAND_DEVICE     1
#define CONFIG_SYS_NAND_BASE           0x20000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_SYS_NAND_USE_FLASH_BBT
#endif /* CONFIG_NAND_MXS */

#ifndef CONFIG_DM_I2C
#define CONFIG_SYS_I2C
#endif
#define CONFIG_SYS_I2C_SPEED		100000

/* USB configs */
#define CONFIG_USB_MAX_CONTROLLER_COUNT         2
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_VBUS_DRAW 2

#ifdef CONFIG_DM_VIDEO
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_BMP_24BPP
#define CONFIG_BMP_32BPP
#define CONFIG_VIDEO_BMP_RLE8
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
#define DISTRO_BOOT_DEV_MMC(func) func(MMC, mmc, 0) func(MMC, mmc, 2)
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
#define BD_RAM_SCRIPT	"40020000"
#define BD_RAM_KERNEL	"40800000"
#define BD_RAM_RAMDISK	"42800000"
#define BD_RAM_FDT	"43000000"

/* M4 specific */
#define SYS_AUXCORE_BOOTDATA_DDR	0x80000000
#define SYS_AUXCORE_BOOTDATA_TCM	0x007E0000

#define BD_FUSE1		"1 3"
#define BD_FUSE1_VAL		"10002000"	/* USDHC3 emmc */

#define BD_FUSE1_STR		"fuse1=" BD_FUSE1 "\0"
#define BD_FUSE1_VAL_STR	"fuse1_val=" BD_FUSE1_VAL "\0"
#ifdef BD_FUSE2
#define BD_FUSE2_STR		"fuse2=" BD_FUSE2 "\0"
#define BD_FUSE2_VAL_STR	"fuse2_val=" BD_FUSE2_VAL "\0"
#else
#define BD_FUSE2_STR		""
#define BD_FUSE2_VAL_STR	""
#endif

#define BD_FUSE_MAC1A		"9 1"
#define BD_FUSE_MAC1A_VAL	"00000019"
#define BD_FUSE_MAC1B		"9 0"

#define BD_FUSE_MAC1A_STR	"fuse_mac1a=" BD_FUSE_MAC1A "\0"
#define BD_FUSE_MAC1A_VAL_STR	"fuse_mac1a_val=" BD_FUSE_MAC1A_VAL "\0"
#define BD_FUSE_MAC1B_STR	"fuse_mac1b=" BD_FUSE_MAC1B "\0"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"console=" BD_CONSOLE "\0" \
	"env_dev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"env_part=" __stringify(CONFIG_SYS_MMC_ENV_PART) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"fastboot_raw_partition_bootloader=0x0 0x1ff0 mmcpart 1\0" \
	"fastboot_raw_partition_bootloader-env=0x1ff0 0x10 mmcpart 1\0" \
	"fdt_addr=0x43000000\0" \
	"fdt_high=0xffffffffffffffff\0" \
	BD_FUSE1_STR \
	BD_FUSE1_VAL_STR \
	BD_FUSE2_STR \
	BD_FUSE2_VAL_STR \
	BD_FUSE_MAC1A_STR \
	BD_FUSE_MAC1A_VAL_STR \
	BD_FUSE_MAC1B_STR \
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

#endif
