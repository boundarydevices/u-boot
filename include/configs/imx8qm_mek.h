/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2018 NXP
 */

#ifndef __IMX8QM_MEK_H
#define __IMX8QM_MEK_H

#include <linux/sizes.h>
#include <linux/stringify.h>
#include <asm/arch/imx-regs.h>
#include "imx_env.h"

#ifdef CONFIG_SPL_BUILD
#define CFG_MALLOC_F_ADDR		0x00138000

/*
 * 0x08081000 - 0x08180FFF is for m4_0 xip image,
 * 0x08181000 - 0x008280FFF is for m4_1 xip image
  * So 3rd container image may start from 0x8281000
 */
#define CFG_SYS_UBOOT_BASE 0x08281000

#endif

#ifdef CONFIG_TARGET_IMX8QM_MEK_A53_ONLY
#define IMX_HDMI_FIRMWARE_LOAD_ADDR (CFG_SYS_SDRAM_BASE + SZ_64M)
#define IMX_HDMITX_FIRMWARE_SIZE 0x20000
#define IMX_HDMIRX_FIRMWARE_SIZE 0x20000
#endif

#define PHY_ANEG_TIMEOUT 20000

#ifdef CONFIG_AHAB_BOOT
#define AHAB_ENV "sec_boot=yes\0"
#else
#define AHAB_ENV "sec_boot=no\0"
#endif

/* Boot M4 */
#define M4_BOOT_ENV \
	"m4_0_image=m4_0.bin\0" \
	"m4_1_image=m4_1.bin\0" \
	"loadm4image_0=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${m4_0_image}\0" \
	"loadm4image_1=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${m4_1_image}\0" \
	"m4boot_0=run loadm4image_0; dcache flush; bootaux ${loadaddr} 0\0" \
	"m4boot_1=run loadm4image_1; dcache flush; bootaux ${loadaddr} 1\0" \

#ifdef CONFIG_NAND_BOOT
#define MFG_NAND_PARTITION "mtdparts=gpmi-nand:128m(boot),32m(kernel),16m(dtb),8m(misc),-(rootfs) "
#else
#define MFG_NAND_PARTITION ""
#endif

#ifdef CONFIG_TARGET_IMX8QM_MEK_A72_ONLY
#define HDP_LOAD_ENV
#define HDPRX_LOAD_ENV
#define INITRD_ADDR_ENV "initrd_addr=0xC3100000\0"
#else
#define HDP_LOAD_ENV \
	"if run loadhdp; then; hdp load ${hdp_addr}; fi;"
#define HDPRX_LOAD_ENV \
	"if test ${hdprx_enable} = yes; then if run loadhdprx; then; hdprx load ${hdprx_addr}; fi; fi; "
#define INITRD_ADDR_ENV "initrd_addr=0x83100000\0"
#endif

#define CFG_MFG_ENV_SETTINGS \
	CFG_MFG_ENV_SETTINGS_DEFAULT \
	INITRD_ADDR_ENV \
	"initrd_high=0xffffffffffffffff\0" \
	"emmc_dev=0\0" \
	"sd_dev=1\0"

/* Initial environment variables */
#define CFG_EXTRA_ENV_SETTINGS		\
	CFG_MFG_ENV_SETTINGS \
	M4_BOOT_ENV \
	AHAB_ENV \
	"script=boot.scr\0" \
	"image=Image\0" \
	SPLASH_IMAGE_ADDR \
	CFG_CONSOLE \
	FDT_ADDR \
	"fdt_high=0xffffffffffffffff\0"		\
	"cntr_addr=0x98000000\0"			\
	"cntr_file=os_cntr_signed.bin\0" \
	"boot_fdt=try\0" \
	FDT_FILE \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=1\0" \
	"mmcroot=" CFG_MMCROOT " rootwait rw\0" \
	"mmcautodetect=yes\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} earlycon root=${mmcroot} " \
		"cpufreq.default_governor=SCHEDUTIL\0" \
	"loadbootscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"hdp_addr=0x9c000000\0" \
	"hdprx_addr=0x9c800000\0" \
	"hdp_file=hdmitxfw.bin\0" \
	"hdprx_file=hdmirxfw.bin\0" \
	"hdprx_enable=no\0" \
	"loadhdp=fatload mmc ${mmcdev}:${mmcpart} ${hdp_addr} ${hdp_file}\0" \
	"loadhdprx=fatload mmc ${mmcdev}:${mmcpart} ${hdprx_addr} ${hdprx_file}\0" \
	"boot_os=booti ${loadaddr} - ${fdt_addr};\0" \
	"loadcntr=fatload mmc ${mmcdev}:${mmcpart} ${cntr_addr} ${cntr_file}\0" \
	"auth_os=booti ${cntr_addr}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		HDP_LOAD_ENV \
		HDPRX_LOAD_ENV \
		"run mmcargs; " \
		"if test ${sec_boot} = yes; then " \
			"run auth_os; " \
		"else " \
			"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
				"if run loadfdt; then " \
					"run boot_os; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"else " \
				"echo wait for boot; " \
			"fi;" \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} earlycon " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp " \
		"cpufreq.default_governor=SCHEDUTIL\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs;  " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if ${get_cmd} ${hdp_addr} ${hdp_file}; then; hdp load ${hdp_addr}; fi;" \
		"if test ${sec_boot} = yes; then " \
			"${get_cmd} ${cntr_addr} ${cntr_file}; " \
			"run auth_os; " \
		"else " \
			"${get_cmd} ${loadaddr} ${image}; " \
			"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
				"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
					"run boot_os; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"else " \
				"booti; " \
			"fi;" \
		"fi;\0"

#if defined(CONFIG_TARGET_IMX8QM_MEK_A72_ONLY)
	#define FDT_ADDR	"fdt_addr=0xC3000000\0"
	#define FDT_FILE	"fdt_file=imx8qm-mek-cockpit-a72.dtb\0"
#elif defined(CONFIG_TARGET_IMX8QM_MEK_A53_ONLY)
	#define FDT_ADDR	"fdt_addr=0x83000000\0"
	#define FDT_FILE	"fdt_file=imx8qm-mek-cockpit-a53.dtb\0"
#else
	#define FDT_ADDR	"fdt_addr=0x83000000\0"
	#define FDT_FILE	"fdt_file=undefined\0"
#endif

/* On LPDDR4 board, USDHC1 is for eMMC, USDHC2 is for SD on CPU board */
#if defined(CONFIG_TARGET_IMX8QM_MEK_A72_ONLY)
	#define CFG_MMCROOT			"/dev/mmcblk0p2"  /* USDHC1 */
#elif defined(CONFIG_TARGET_IMX8QM_MEK_A53_ONLY)
	#define CFG_MMCROOT			"/dev/mmcblk1p2"  /* USDHC2 */
#else
	#define CFG_MMCROOT			"/dev/mmcblk1p2"  /* USDHC2 */
#endif

#if defined(CONFIG_TARGET_IMX8QM_MEK_A72_ONLY)
#define CFG_CONSOLE "console=ttyLP2\0"
#define SPLASH_IMAGE_ADDR	"splashimage=0xde000000\0"
#else
#define CFG_CONSOLE "console=ttyLP0\0"
#define SPLASH_IMAGE_ADDR	"splashimage=0x9e000000\0"
#endif

#if defined(CONFIG_TARGET_IMX8QM_MEK_A53_ONLY)
#define CFG_SYS_SDRAM_BASE		0x80000000
#define PHYS_SDRAM_1			0x80000000
#define PHYS_SDRAM_2			0x880000000
#define PHYS_SDRAM_1_SIZE		0x40000000	/* 1 GB */
#define PHYS_SDRAM_2_SIZE		0x80000000	/* 2 GB */
#elif defined(CONFIG_TARGET_IMX8QM_MEK_A72_ONLY)
#define CFG_SYS_SDRAM_BASE		0xC0000000
#define PHYS_SDRAM_1			0xC0000000
#define PHYS_SDRAM_2			0x900000000
#define PHYS_SDRAM_1_SIZE		0x40000000	/* 1 GB */
#define PHYS_SDRAM_2_SIZE		0x80000000	/* 2 GB */
#else
#define CFG_SYS_SDRAM_BASE		0x80000000
#define PHYS_SDRAM_1			0x80000000
#define PHYS_SDRAM_2			0x880000000
#define PHYS_SDRAM_1_SIZE		0x80000000	/* 2 GB */
#define PHYS_SDRAM_2_SIZE		0x100000000	/* 4 GB */
#endif

#if defined(CONFIG_ANDROID_SUPPORT)
#include "imx8qm_mek_android.h"
#elif defined (CONFIG_ANDROID_AUTO_SUPPORT)
#include "imx8qm_mek_android_auto.h"
#endif

#endif /* __IMX8QM_MEK_H */
