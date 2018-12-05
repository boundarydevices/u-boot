/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX7D SABRESD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __NITROGEN7_CONFIG_H
#define __NITROGEN7_CONFIG_H

#include "mx7_common.h"

#define CONFIG_DBG_MONITOR
#define PHYS_SDRAM_SIZE			SZ_1G
#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_IMX_THERMAL

#define CONFIG_DFU_MMC

/* ENET1 */
#define IMX_FEC_BASE			ENET_IPS_BASE_ADDR

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE3000
#define CONFIG_POWER_PFUZE3000_I2C_ADDR	0x08

#ifdef CONFIG_SPI_FLASH
/* #define CONFIG_SYS_FSL_QSPI_AHB */
#define FSL_QSPI_FLASH_NUM		1
#define FSL_QSPI_FLASH_SIZE		SZ_16M
#endif

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

#define CONFIG_SUPPORT_EMMC_BOOT	/* eMMC specific */
#define CONFIG_SYS_MMC_IMG_LOAD_PART	1
#define CONFIG_SYS_MMC_ENV_PART         1	/* mmcblk0boot0 */

#define CONFIG_FEC_MXC_PHYADDR		4
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_MXC_UART_BASE            UART1_IPS_BASE_ADDR
#define BD_CONSOLE	"ttymxc0"
#define BD_I2C_MASK	0xf

/* M4 specific */
#define SYS_AUXCORE_BOOTDATA_DDR	0x9ff00000
#define SYS_AUXCORE_BOOTDATA_OCRAM	0x00910000
#define SYS_AUXCORE_BOOTDATA_QSPI	0x601e0000
#define SYS_AUXCORE_BOOTDATA_TCM	0x007F8000
#define EXTRA_ENV_M4 \
	"loadm4image=load ${devtype} ${devnum}:1 ${loadaddr} ${m4image}\0" \
	"m4boot=run m4boot_nor\0" \
	"m4boot_ext=load ${devtype} ${devnum}:1 ${m4loadaddr} ${m4image}; " \
		"dcache flush; bootaux ${m4loadaddr}\0" \
	"m4boot_nor=sf probe; sf read ${m4loadaddr} ${m4offset} ${m4size}; " \
		"dcache flush; bootaux ${m4loadaddr}\0" \
	"m4boot_qspi=bootaux "__stringify(SYS_AUXCORE_BOOTDATA_QSPI)"\0" \
	"m4image=m4_fw.bin\0" \
	"m4loadaddr="__stringify(SYS_AUXCORE_BOOTDATA_TCM)"\0" \
	"m4loaddevs=mmc\0" \
	"m4offset=0x1e0000\0" \
	"m4size=0x8000\0" \
	"m4update=for devtype in ${m4loaddevs}; do " \
		"for devnum in 0 1 ; do ${devtype} dev ${devnum} ;" \
			"if run loadm4image; then " \
				"sf probe; " \
				"sf erase ${m4offset} ${m4size}; " \
				"sf write ${loadaddr} ${m4offset} ${filesize}; " \
				"exit; " \
			"fi; " \
		"done; " \
		"done\0"

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"rdinit=/linuxrc " \
		"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
		"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
		"g_mass_storage.iSerialNumber=\"\" "\
		"clk_ignore_unused "\
		"\0" \
	"initrd_addr=0x83800000\0" \
	"bootcmd_mfg=run mfgtool_args;bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0" \
	EXTRA_ENV_M4

#endif	/* __CONFIG_H */
