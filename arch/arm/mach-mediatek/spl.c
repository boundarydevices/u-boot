// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 MediaTek Inc.
 * Author: Ryder Lee <ryder.lee@mediatek.com>
 */

#include <asm/cache.h>
#include <clk.h>
#include <cpu_func.h>
#include <hang.h>
#include <init.h>
#include <spl.h>

#include "init.h"

int spl_enable_dcache(void)
{
	int ret;

	ret = dram_init();
	if (ret) {
		printf("DRAM init failed\n");
		return ret;
	}

	gd->ram_top = gd->ram_base + get_effective_memsize();
	gd->relocaddr = gd->ram_top;

	ret = arch_reserve_mmu();
	if (ret) {
		printf("Reserve memory for MMU TLB table failed\n");
		return ret;
	}

	dram_init_banksize();

	dcache_enable();

	return 0;
}

void board_init_f(ulong dummy)
{
	int ret;

	icache_enable();

	ret = spl_early_init();
	if (ret)
		hang();

	/* enable console uart printing */
	preloader_console_init();

	/* soc early initialization */
	ret = mtk_soc_early_init();
	if (ret)
		hang();

	ret = spl_enable_dcache();
	if (ret)
		printf("Cannot enable dcache\n");
}

u32 spl_boot_device(void)
{
#if defined(CONFIG_SPL_SPI)
	return BOOT_DEVICE_SPI;
#elif defined(CONFIG_SPL_MMC)
	return BOOT_DEVICE_MMC1;
#elif defined(CONFIG_SPL_NAND_SUPPORT)
	return BOOT_DEVICE_NAND;
#elif defined(CONFIG_SPL_NOR_SUPPORT)
	return BOOT_DEVICE_NOR;
#else
	return BOOT_DEVICE_NONE;
#endif
}
