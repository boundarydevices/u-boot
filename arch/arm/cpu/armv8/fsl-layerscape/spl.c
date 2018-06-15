/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <asm/io.h>
#include <fsl_ifc.h>
#include <i2c.h>
#include <fsl_csu.h>
#include <asm/arch/fdt.h>
#include <asm/arch/ppa.h>

DECLARE_GLOBAL_DATA_PTR;

u32 spl_boot_device(void)
{
#ifdef CONFIG_SPL_MMC_SUPPORT
	return BOOT_DEVICE_MMC1;
#endif
#ifdef CONFIG_SPL_NAND_SUPPORT
	return BOOT_DEVICE_NAND;
#endif
	return 0;
}

u32 spl_boot_mode(const u32 boot_device)
{
	switch (spl_boot_device()) {
	case BOOT_DEVICE_MMC1:
#ifdef CONFIG_SPL_FAT_SUPPORT
		return MMCSD_MODE_FS;
#else
		return MMCSD_MODE_RAW;
#endif
	case BOOT_DEVICE_NAND:
		return 0;
	default:
		puts("spl: error: unsupported device\n");
		hang();
	}
}

#ifdef CONFIG_SPL_BUILD

void spl_board_init(void)
{
#if defined(CONFIG_SECURE_BOOT) && defined(CONFIG_FSL_LSCH2)
	/*
	 * In case of Secure Boot, the IBR configures the SMMU
	 * to allow only Secure transactions.
	 * SMMU must be reset in bypass mode.
	 * Set the ClientPD bit and Clear the USFCFG Bit
	*/
	u32 val;
	val = (in_le32(SMMU_SCR0) | SCR0_CLIENTPD_MASK) & ~(SCR0_USFCFG_MASK);
	out_le32(SMMU_SCR0, val);
	val = (in_le32(SMMU_NSCR0) | SCR0_CLIENTPD_MASK) & ~(SCR0_USFCFG_MASK);
	out_le32(SMMU_NSCR0, val);
#endif
#ifdef CONFIG_LAYERSCAPE_NS_ACCESS
	enable_layerscape_ns_access();
#endif
#ifdef CONFIG_SPL_FSL_LS_PPA
	ppa_init();
#endif
}

void board_init_f(ulong dummy)
{
	/* Clear global data */
	memset((void *)gd, 0, sizeof(gd_t));
	board_early_init_f();
	timer_init();
#ifdef CONFIG_ARCH_LS2080A
	env_init();
#endif
	get_clocks();

	preloader_console_init();

#ifdef CONFIG_SPL_I2C_SUPPORT
	i2c_init_all();
#endif
	dram_init();
#ifdef CONFIG_SPL_FSL_LS_PPA
#ifndef CONFIG_SYS_MEM_RESERVE_SECURE
#error Need secure RAM for PPA
#endif
	/*
	 * Secure memory location is determined in dram_init_banksize().
	 * gd->ram_size is deducted by the size of secure ram.
	 */
	dram_init_banksize();

	/*
	 * After dram_init_bank_size(), we know U-Boot only uses the first
	 * memory bank regardless how big the memory is.
	 */
	gd->ram_top = gd->bd->bi_dram[0].start + gd->bd->bi_dram[0].size;

	/*
	 * If PPA is loaded, U-Boot will resume running at EL2.
	 * Cache and MMU will be enabled. Need a place for TLB.
	 * U-Boot will be relocated to the end of available memory
	 * in first bank. At this point, we cannot know how much
	 * memory U-Boot uses. Put TLB table lower by SPL_TLB_SETBACK
	 * to avoid overlapping. As soon as the RAM version U-Boot sets
	 * up new MMU, this space is no longer needed.
	 */
	gd->ram_top -= SPL_TLB_SETBACK;
	gd->arch.tlb_size = PGTABLE_SIZE;
	gd->arch.tlb_addr = (gd->ram_top - gd->arch.tlb_size) & ~(0x10000 - 1);
	gd->arch.tlb_allocated = gd->arch.tlb_addr;
#endif	/* CONFIG_SPL_FSL_LS_PPA */
}
#endif /* CONFIG_SPL_BUILD */
