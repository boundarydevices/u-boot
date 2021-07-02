/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for Mediatek Android based boards
 *
 * Copyright (C) 2021 BayLibre, SAS
 * Author: Guillaume La Roque <glaroque@baylibre.com
 */

#ifndef __MEDIATEK_ANDROID_H
#define __MEDIATEK_ANDROID_H

#ifdef CONFIG_TARGET_MT8183
#include <configs/mt8183.h>
#define SERIAL_ANDROID  "i500Pumpkin"
#elif CONFIG_TARGET_MT8516
#include <configs/mt8516.h>
#define SERIAL_ANDROID "i300aPumpkin"
#endif

#ifndef CONTROL_PARTITION
#define CONTROL_PARTITION "misc"
#endif

#ifndef BOOT_PARTITION
#define BOOT_PARTITION "boot"
#endif

#ifndef EXTRA_ANDROID_ENV_SETTINGS
#define EXTRA_ANDROID_ENV_SETTINGS ""
#endif

#ifdef CONFIG_SYS_MALLOC_LEN
#undef CONFIG_SYS_MALLOC_LEN
#endif

#define CONFIG_SYS_MALLOC_LEN		SZ_128M

#if CONFIG_IS_ENABLED(CMD_AB_SELECT)
#define ANDROIDBOOT_GET_CURRENT_SLOT_CMD "get_current_slot=" \
	"if part number mmc ${mmcdev} " CONTROL_PARTITION " control_part_number; " \
	"then " \
		"echo " CONTROL_PARTITION \
			" partition number:${control_part_number};" \
		"ab_select current_slot mmc ${mmcdev}:${control_part_number};" \
	"else " \
		"echo " CONTROL_PARTITION " partition not found;" \
	"fi;\0"

#define AB_SELECT_SLOT \
	"run get_current_slot; " \
	"if test -e \"${current_slot}\"; " \
	"then " \
		"setenv slot_suffix _${current_slot}; " \
	"else " \
		"echo current_slot not found;" \
		"exit;" \
	"fi;"

#define AB_SELECT_ARGS \
	"setenv bootargs_ab ${bootargs_ab} androidboot.slot_suffix=${slot_suffix}; " \
	"echo A/B cmdline addition: ${bootargs_ab};" \
	"setenv bootargs ${bootargs} ${bootargs_ab};"

#define AB_BOOTARGS " androidboot.force_normal_boot=1"
#define RECOVERY_PARTITION "boot"
#else
#define AB_SELECT_SLOT ""
#define AB_SELECT_ARGS " "
#define ANDROIDBOOT_GET_CURRENT_SLOT_CMD ""
#define AB_BOOTARGS " "
#define RECOVERY_PARTITION "recovery"
#endif

#if defined(CONFIG_CMD_AVB)
#define AVB_VERIFY_CHECK \
	"if run avb_verify; then " \
			"echo AVB verification OK.;" \
			"setenv bootargs \"$bootargs $avb_bootargs\";" \
	"else " \
		"setenv bootargs \"$bootargs androidboot.verifiedbootstate=orange\";" \
		"echo Running without AVB...; "\
	"fi;"

#define AVB_VERIFY_CMD "avb_verify=avb init ${mmcdev}; avb verify $slot_suffix;\0"
#else
#define AVB_VERIFY_CHECK ""
#define AVB_VERIFY_CMD ""

#endif

#if defined(CONFIG_CMD_ABOOTIMG)
/*
 * Prepares complete device tree blob for current board (for Android boot).
 *
 * Boot image or recovery image should be loaded into $loadaddr prior to running
 * these commands. The logic of these commnads is next:
 *
 *   1. Read correct DTB for current SoC/board from boot image in $loadaddr
 *      to $fdtaddr
 *   2. Merge all needed DTBOs for current board from 'dtbo' partition into read
 *      DTB
 *   3. User should provide $fdtaddr as 3rd argument to 'bootm'
 */
#define PREPARE_FDT \
	"echo Preparing FDT...; " \
	"abootimg get dtb --index=$dtb_index dtb_start dtb_size; " \
	"cp.b $dtb_start $fdt_addr_r $dtb_size; " \
	"fdt addr $fdt_addr_r $fdt_size; " \
	"part start mmc ${mmcdev} dtbo${slot_suffix} dtbo_start; " \
	"part size mmc ${mmcdev} dtbo${slot_suffix} dtbo_size; " \
	"mmc read ${dtboaddr} ${dtbo_start} ${dtbo_size}; " \
	"echo \"  Applying DTBOs...\"; " \
	"adtimg addr $dtboaddr; " \
	"dtbo_idx=''; " \
	"for index in $dtbo_index; do " \
		"adtimg get dt --index=$index dtbo_addr; " \
		"fdt apply $dtbo_addr; " \
		"if test $dtbo_idx = ''; then " \
			"dtbo_idx=${index}; " \
		"else " \
			"dtbo_idx=${dtbo_idx},${index}; " \
		"fi; " \
	"done; " \
	"setenv bootargs \"$bootargs androidboot.dtbo_idx=$dtbo_idx \"; "

#define BOOT_CMD "bootm ${loadaddr} ${loadaddr} ${fdt_addr_r};"

#else
#define PREPARE_FDT " "
#define BOOT_CMD "bootm ${loadaddr};"
#endif

#define BOOTENV_DEV_FASTBOOT(devtypeu, devtypel, instance) \
	"bootcmd_fastboot=" \
		"setenv run_fastboot 0;" \
		"if bcb load " __stringify(CONFIG_FASTBOOT_FLASH_MMC_DEV) " " \
		CONTROL_PARTITION "; then " \
			"if bcb test command = bootonce-bootloader; then " \
				"echo BCB: Bootloader boot...; " \
				"bcb clear command; bcb store; " \
				"setenv run_fastboot 1;" \
			"fi; " \
			"if bcb test command = boot-fastboot; then " \
				"echo BCB: fastboot userspace boot...; " \
				"setenv force_recovery 1;" \
			"fi; " \
		"else " \
			"echo Warning: BCB is corrupted or does not exist; " \
		"fi;" \
		"if test \"${run_fastboot}\" -eq 1; then " \
			"fastboot " __stringify(CONFIG_FASTBOOT_USB_DEV) "; " \
		"fi;\0"

#define BOOTENV_DEV_NAME_FASTBOOT(devtypeu, devtypel, instance)	\
		"fastboot "

#define BOOTENV_DEV_RECOVERY(devtypeu, devtypel, instance) \
	"bootcmd_recovery=" \
		"setenv run_recovery 0;" \
		"if bcb load " __stringify(CONFIG_FASTBOOT_FLASH_MMC_DEV) " " \
		CONTROL_PARTITION "; then " \
			"if bcb test command = boot-recovery; then " \
				"echo BCB: Recovery boot...; " \
				"setenv run_recovery 1;" \
			"fi;" \
		"fi;" \
		"if test \"${skip_recovery}\" -eq 1; then " \
			"echo Recovery skipped by environment;" \
			"setenv run_recovery 0;" \
		"fi;" \
		"if test \"${force_recovery}\" -eq 1; then " \
			"echo Recovery forced by environment;" \
			"setenv run_recovery 1;" \
		"fi;" \
		"if test \"${run_recovery}\" -eq 1; then " \
			"echo Running Recovery...;" \
			"mmc dev ${mmcdev};" \
			AB_SELECT_SLOT \
			AVB_VERIFY_CHECK \
			AB_SELECT_ARGS \
			"part start mmc ${mmcdev} " RECOVERY_PARTITION "${slot_suffix} boot_start;" \
			"part size mmc ${mmcdev} " RECOVERY_PARTITION "${slot_suffix} boot_size;" \
			"if mmc read ${loadaddr} ${boot_start} ${boot_size}; then " \
				PREPARE_FDT \
				"echo Running Android Recovery...;" \
				"setenv bootargs \"${bootargs} androidboot.serialno=${serial#} androidboot.hardware=${hardware} \" ; " \
				BOOT_CMD \
			"fi;" \
			"echo Failed to boot Android...;" \
			"reset;" \
		"fi;\0"

#define BOOTENV_DEV_NAME_RECOVERY(devtypeu, devtypel, instance)	\
		"recovery "

#define BOOTENV_DEV_SYSTEM(devtypeu, devtypel, instance) \
	"bootcmd_system=" \
		"echo Loading Android " BOOT_PARTITION " partition...;" \
		"mmc dev ${mmcdev};" \
		AB_SELECT_SLOT \
		AVB_VERIFY_CHECK \
		AB_SELECT_ARGS \
		"part start mmc ${mmcdev} " BOOT_PARTITION "${slot_suffix} boot_start;" \
		"part size mmc ${mmcdev} " BOOT_PARTITION "${slot_suffix} boot_size;" \
		"if mmc read ${loadaddr} ${boot_start} ${boot_size}; then " \
			PREPARE_FDT \
			"setenv bootargs \"${bootargs}" AB_BOOTARGS " androidboot.serialno=${serial#} androidboot.hardware=${hardware} \" ; " \
			"echo Running Android...;" \
			BOOT_CMD \
		"fi;" \
		"echo Failed to boot Android...;\0"

#define BOOTENV_DEV_NAME_SYSTEM(devtypeu, devtypel, instance)	\
		"system "

#define BOOTENV_DEV_PANIC(devtypeu, devtypel, instance) \
	"bootcmd_panic=" \
		"fastboot " __stringify(CONFIG_FASTBOOT_USB_DEV) "; " \
		"reset\0"

#define BOOTENV_DEV_NAME_PANIC(devtypeu, devtypel, instance)	\
		"panic "

#ifdef BOOT_TARGET_DEVICES
#undef BOOT_TARGET_DEVICES
#endif
/* Environment settings */
#include <config_distro_bootcmd.h>

#define BOOT_TARGET_DEVICES(func) \
	func(FASTBOOT, fastboot, na) \
	func(RECOVERY, recovery, na) \
	func(SYSTEM, system, na) \
	func(PANIC, panic, na) \


#ifdef CONFIG_EXTRA_ENV_SETTINGS
#undef CONFIG_EXTRA_ENV_SETTINGS
#endif
#define CONFIG_EXTRA_ENV_SETTINGS \
	"fastboot=fastboot usb 0\0"        \
	"board=" CONFIG_IDENT_STRING "\0"\
	"mmcdev=" __stringify(CONFIG_FASTBOOT_FLASH_MMC_DEV) "\0" \
	EXTRA_ANDROID_ENV_SETTINGS         \
	AVB_VERIFY_CMD                     \
	ANDROIDBOOT_GET_CURRENT_SLOT_CMD   \
	"dtb_index=0\0" \
	"dtbo_index=0\0" \
	"serial#=" SERIAL_ANDROID "\0" \
	"hardware=" CONFIG_SYS_BOARD "\0" \
	"loadaddr=0x50000000\0" \
	"dtbaddr=0x54000000\0" \
	"dtboaddr=0x54C00000\0" \
	"fdt_addr_r=0x56000000\0" \
	"fdt_size=0xC0000\0 " \
	BOOTENV

#endif
