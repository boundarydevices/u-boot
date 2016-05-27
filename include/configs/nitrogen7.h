/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX7D SABRESD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __NITROGEN7_CONFIG_H
#define __NITROGEN7_CONFIG_H

/* Boot in secure mode for CAAM to work */
#define CONFIG_MX7_SEC

#include "mx7_common.h"

#define CONFIG_DBG_MONITOR
#define PHYS_SDRAM_SIZE			SZ_1G
#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000
#define CONFIG_STACKSIZE		SZ_128K
#define RAM_SIZE			PHYS_SDRAM_SIZE

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_IMX_THERMAL

#define CONFIG_DFU_MMC

/* Network */
#define CONFIG_FEC_DMA_MINALIGN		64
/* ENET1 */
#define IMX_FEC_BASE			ENET_IPS_BASE_ADDR

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE3000
#define CONFIG_POWER_PFUZE3000_I2C_ADDR	0x08

#ifdef CONFIG_SPI_FLASH
#define CONFIG_FSL_QSPI
#define CONFIG_CMD_SF
/* #define CONFIG_SYS_FSL_QSPI_AHB */
#define FSL_QSPI_FLASH_NUM		1
#define FSL_QSPI_FLASH_SIZE		SZ_16M
#endif

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

#define CONFIG_SUPPORT_EMMC_BOOT	/* eMMC specific */
#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

#define CONFIG_CI_UDC
#define CONFIG_FEC_MXC_PHYADDR          4
#define CONFIG_PHY_ATHEROS
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_CONSOLE	"ttymxc0"
#define BD_I2C_MASK	0xf

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
	"cmd_custom=echo\0" \

#endif	/* __CONFIG_H */
