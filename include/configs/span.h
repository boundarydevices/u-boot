/*
 * Copyright 2018 Boundary Devices
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __SPAN_H
#define __SPAN_H

#ifdef CONFIG_BOARD_TYPE_SET
#undef CONFIG_SYS_BOARD
#define CONFIG_SYS_BOARD CONFIG_BOARD_TYPE
#endif
#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#define CONFIG_SPL_MAX_SIZE		(148 * 1024)
#define CONFIG_SYS_MONITOR_LEN		SZ_512K
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION	1
#define CONFIG_SYS_UBOOT_BASE		(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD
/*#define CONFIG_ENABLE_DDR_TRAINING_DEBUG*/
#ifdef CONFIG_IMX8MN
#define CONFIG_SPL_STACK		0x0095fff0
#define CONFIG_SPL_BSS_START_ADDR	0x00950000
#define CONFIG_MALLOC_F_ADDR		0x00940000	/* malloc f used before GD_FLG_FULL_MALLOC_INIT set */
#else
#define CONFIG_SPL_STACK		0x0091fff0
#define CONFIG_SPL_BSS_START_ADDR	0x00910000
#define CONFIG_MALLOC_F_ADDR		0x00930000	/* malloc f used before GD_FLG_FULL_MALLOC_INIT set */
#endif
#define CONFIG_SPL_BSS_MAX_SIZE		SZ_8K
#define CONFIG_SYS_SPL_MALLOC_START	0x42200000
#define CONFIG_SYS_SPL_MALLOC_SIZE	SZ_512K

#define CONFIG_SPL_ABORT_ON_RAW_IMAGE /* For RAW image gives a error info not panic */

#undef CONFIG_DM_ETH
#undef CONFIG_BLK
#undef CONFIG_DM_MMC
#undef CONFIG_DM_PMIC
#undef CONFIG_DM_PMIC_PFUZE100

#define CONFIG_SYS_I2C

#endif

#define CONFIG_CMD_READ
#define CONFIG_FASTBOOT_USB_DEV 0

#define CONFIG_BOARD_POSTCLK_INIT


#undef CONFIG_CMD_IMLS

#undef CONFIG_CMD_CRC32
#undef CONFIG_BOOTM_NETBSD

/* ENET Config */
/* ENET1 */
#if defined(CONFIG_CMD_NET)
#define CONFIG_ETHPRIME			"FEC"

#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_FEC_MXC_PHYADDR          4
#define FEC_QUIRK_ENET_MAC

#define IMX_FEC_BASE			0x30BE0000

#endif


/* Link Definitions */
#define CONFIG_SYS_INIT_RAM_ADDR        0x40000000
#define CONFIG_SYS_INIT_RAM_SIZE	0x200000
#define CONFIG_SYS_INIT_SP_OFFSET \
        (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
        (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_ENV_SIZE			0x2000

#define CONFIG_MMCROOT			"/dev/mmcblk1p2"  /* USDHC2 */

/* Size of malloc() pool */

#define CONFIG_SYS_SDRAM_BASE           0x40000000
#define PHYS_SDRAM                      0x40000000
#define PHYS_SDRAM_SIZE			(CONFIG_DDR_MB * 1024ULL * 1024ULL)

#define CONFIG_BAUDRATE			115200

#define CONFIG_MXC_UART_BASE		UART2_BASE_ADDR

/* Monitor Command Prompt */
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_CBSIZE              2048
#define CONFIG_SYS_MAXARGS             64
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

/* USDHC */
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR       0




#ifndef CONFIG_DM_I2C
#define CONFIG_SYS_I2C
#endif
#define CONFIG_SYS_I2C_SPEED		100000

/* USB configs */
#ifndef CONFIG_SPL_BUILD
#define CONFIG_USBD_HS


#endif

#define CONFIG_USB_GADGET_VBUS_DRAW 2

#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_USB_MAX_CONTROLLER_COUNT         2


#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_MXS
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE (6 * 1024 * 1024)
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#endif

#ifndef BD_CONSOLE
#if CONFIG_MXC_UART_BASE == UART2_BASE_ADDR
#define BD_CONSOLE	"ttymxc1"
#elif CONFIG_MXC_UART_BASE == UART1_BASE_ADDR
#define BD_CONSOLE	"ttymxc0"
#endif
#endif

#ifdef CONFIG_CMD_MMC
#define DISTRO_BOOT_DEV_MMC(func) func(MMC, mmc, 1) func(MMC, mmc, 0)
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

#define BD_RAM_BASE	0x80000000
#define BD_RAM_SCRIPT	"40020000"
#define BD_RAM_KERNEL	"40800000"
#define BD_RAM_RAMDISK	"42800000"
#define BD_RAM_FDT	"43000000"

#ifdef CONFIG_IMX8MM
#define BD_FUSE1		"1 3"
#define BD_FUSE1_VAL		"10002022"	/* USDHC1 emmc */
#endif

#ifdef CONFIG_IMX8MN
#define BD_FUSE1		"1 3"
#define BD_FUSE1_VAL		"10000200"	/* USDHC1 emmc */
#define BD_FUSE2		"2 1"
#define BD_FUSE2_VAL		"00000002"	/* 1.8V */
#endif

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
	"scriptaddr=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
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
	"m4loadaddr="__stringify(CONFIG_IMX_MCORE_TCM_ADDR)"\0" \
	"mcore_bootargs=clk-imx8mm.mcore_booted\0" \
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
