// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 MediaTek Inc.
 * Author: Chris-QJ Chen <chris-qj.chen@mediatek.com>
 */

#include <clk.h>
#include <common.h>
#include <dm.h>
#include <init.h>
#include <ram.h>
#include <asm/arch/misc.h>
#include <asm/armv8/mmu.h>
#include <asm/sections.h>
#include <asm/system.h>
#include <dm/uclass.h>

DECLARE_GLOBAL_DATA_PTR;

struct mm_region *mem_map;

static struct mm_region * find_dram_entry_in_mem_map(void)
{
	struct mm_region *mm = mem_map;

	while (mm->size) {
		if (mm->phys == 0x40000000UL)
			return mm;
		mm++;
	}

	panic("ddr entry not found\n");	/* Entry not found, this must never happen. */
	return mm;
}

int dram_init(void)
{
	struct mm_region *mm = find_dram_entry_in_mem_map();
	long sdram_size = get_ram_size((void *)mm->phys, mm->size);

	gd->ram_base = mm->phys;
	gd->ram_size = sdram_size;
	mm->size = sdram_size;
	/*
	 * Limit gd->ram_top not exceeding SZ_4G.
	 * Because some periphals like mmc requires DMA buffer
	 * allocaed below SZ_4G.
	 *
	 * Note: SZ_1M is for adjusting gd->relocaddr,
	 *       the reserved memory for u-boot itself.
	 */
	if (gd->ram_base + gd->ram_size >= SZ_4G)
		gd->mon_len = (gd->ram_base + gd->ram_size + SZ_1M) - SZ_4G;

	return 0;
}

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = gd->ram_base;
	gd->bd->bi_dram[0].size = gd->ram_size;

	return 0;
}

int mtk_pll_early_init(void)
{
	return 0;
}

int mtk_soc_early_init(void)
{
	return 0;
}

#ifndef CONFIG_SYSRESET
void reset_cpu(ulong addr)
{
	psci_system_reset();
}
#endif

int print_cpuinfo(void)
{
	u32 part = mediatek_sip_part_name();

	if (part)
		printf("CPU:   MediaTek MT%.4x\n", part);
	else
		printf("CPU:   MediaTek MT8188\n");
	return 0;
}

static struct mm_region mt8188_mem_map[] = {
	{
		/* DDR */
		.virt = 0x40000000UL,
		.phys = 0x40000000UL,
		.size = 0x200000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) | PTE_BLOCK_OUTER_SHARE,
	}, {
		.virt = 0x00000000UL,
		.phys = 0x00000000UL,
		.size = 0x20000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		0,
	}
};
struct mm_region *mem_map = mt8188_mem_map;
