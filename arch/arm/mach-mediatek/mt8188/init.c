// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 MediaTek Inc.
 * Copyright (C) 2024 BayLibre, SAS
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <asm/global_data.h>
#include <asm/system.h>
#include <dm/uclass.h>
#include <wdt.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	int ret;

	ret = fdtdec_setup_memory_banksize();
	if (ret)
		return ret;

	return fdtdec_setup_mem_size_base();
}

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = gd->ram_base;
	gd->bd->bi_dram[0].size = gd->ram_size;

	return 0;
}

int mtk_soc_early_init(void)
{
	return 0;
}

void reset_cpu(void)
{
	struct udevice *wdt;

	if (IS_ENABLED(CONFIG_PSCI_RESET)) {
		psci_system_reset();
	} else {
		uclass_first_device(UCLASS_WDT, &wdt);
		if (wdt)
			wdt_expire_now(wdt, 0);
	}
}

int print_cpuinfo(void)
{
	printf("CPU:   MediaTek MT8188\n");
	return 0;
}
