/*
 * Copyright (C) 2018 Boundary Devices, Inc.
 *
 * Configuration settings for the Boundary Devices nitrogen51_e
 * board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/***********************************/

#define CONFIG_SYS_BOOTM_LEN	0x1000000

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>
#include <asm/mach-imx/gpio.h>

#define CONFIG_SYS_FSL_CLK

/* ATAGs */
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_LOADADDR		0x92000000
#define CONFIG_SYS_TEXT_BASE	0x97800000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR

#ifndef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY        3
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_CONS_INDEX       1
#define CONFIG_BAUDRATE         115200

/* Miscellaneous configurable options */
#undef CONFIG_CMD_IMLS
#define CONFIG_SYS_CBSIZE       512
#define CONFIG_SYS_MAXARGS      32
#define CONFIG_SYS_BARGSIZE     CONFIG_SYS_CBSIZE

/* MMC */
#define CONFIG_BOUNCE_BUFFER
#define ESDHCI_QUIRK_BROKEN_TIMEOUT_VALUE
#define CONFIG_SYS_FSL_ESDHC_ADDR	MMC_SDHC1_BASE_ADDR
#define CONFIG_SYS_FSL_ESDHC_NUM	1

/* Fuses */
/************************************/
#define CONFIG_FSL_IIM

#define CONFIG_SYS_DDR_CLKSEL	0

#define IMX_FEC_BASE		FEC_BASE_ADDR
#if 1
#define CONFIG_MXC_USB_PORT	1
#define CONFIG_MXC_USB_PORTSC	PORT_PTS_ULPI
#define CONFIG_MXC_USB_FLAGS	MXC_EHCI_POWER_PINS_ENABLED
#define CONFIG_USB_MAX_CONTROLLER_COUNT 1
#endif
#define CONFIG_SYS_CLKTL_CBCDR	0x59E35100

#define CONFIG_MACH_TYPE	3169

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(40 * 1024 * 1024)
#define CONFIG_MXC_UART_BASE		UART1_BASE
#define CONFIG_SYS_ARM_PODF		0
#define CONFIG_FSL_PMIC_BUS		0
#define CONFIG_FSL_PMIC_CS		0
#define CONFIG_FSL_PMIC_CLK		2500000
#define CONFIG_FSL_PMIC_MODE		(SPI_MODE_0 | SPI_CS_HIGH)
#define CONFIG_FSL_PMIC_BITLEN		32

#define CONFIG_FEC_MXC_PHYADDR		5
#define ETH_PHY_MASK			(0xf << 4)

/* PMIC Configs */
#define CONFIG_POWER
#define CONFIG_POWER_SPI
#define CONFIG_POWER_FSL
#define CONFIG_FSL_PMIC_BUS   0
#define CONFIG_FSL_PMIC_CS    0

#define CONFIG_FSL_PMIC_CLK   2500000
#define CONFIG_FSL_PMIC_MODE  (SPI_MODE_0 | SPI_CS_HIGH)
#define CONFIG_FSL_PMIC_BITLEN        32

#define CONFIG_POWER_FSL_MC13892
#define CONFIG_RTC_MC13XXX
#if 0
#define CONFIG_MMC_TRACE
#define CONFIG_SYS_MMC_MAX_BLK_COUNT 1
#endif

#if 1
#define CONFIG_VIDEO_LOGO
#endif
#define BD_I2C_MASK	3
#define BD_CONSOLE "ttymxc0"
#define BD_MMC_DISKS "0"
#define BD_LOG_LEVEL	"7"
#define BD_CMA		"2M"

#ifdef CONFIG_SPI_FLASH_ATMEL
/* only rev0 needs this for the small 512K at45db041d spi-nor */
#define CONFIG_ENV_OFFSET		0x7e000
#else
/* default to no console */
#define BD_CONSOLE_STR	""
#endif


#include "boundary.h"

#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"fb_lcd=off\0" \
	"fb_lcd2=off\0" \

#endif	       /* __CONFIG_H */
