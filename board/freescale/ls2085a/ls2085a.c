/*
 * Copyright 2014 Freescale Semiconductor
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <netdev.h>
#include <fsl_ifc.h>
#include <fsl_ddr.h>
#include <asm/io.h>
#include <fdt_support.h>
#include <libfdt.h>
#include <fsl_debug_server.h>
#include <fsl-mc/fsl_mc.h>
#include <environment.h>
#include <asm/arch-fsl-lsch3/soc.h>

DECLARE_GLOBAL_DATA_PTR;

int board_init(void)
{
	init_final_memctl_regs();

#ifdef CONFIG_ENV_IS_NOWHERE
	gd->env_addr = (ulong)&default_environment[0];
#endif

	return 0;
}

int board_early_init_f(void)
{
	fsl_lsch3_early_init_f();
	return 0;
}

void detail_board_ddr_info(void)
{
	puts("\nDDR    ");
	print_size(gd->bd->bi_dram[0].size + gd->bd->bi_dram[1].size, "");
	print_ddr_info(0);
	if (gd->bd->bi_dram[2].size) {
		puts("\nDP-DDR ");
		print_size(gd->bd->bi_dram[2].size, "");
		print_ddr_info(CONFIG_DP_DDR_CTRL);
	}
}

int dram_init(void)
{
	gd->ram_size = initdram(0);

	return 0;
}

#if defined(CONFIG_ARCH_MISC_INIT)
int arch_misc_init(void)
{
#ifdef CONFIG_FSL_DEBUG_SERVER
	debug_server_init();
#endif

	return 0;
}
#endif

unsigned long get_dram_size_to_hide(void)
{
	unsigned long dram_to_hide = 0;

/* Carve the Debug Server private DRAM block from the end of DRAM */
#ifdef CONFIG_FSL_DEBUG_SERVER
	dram_to_hide += debug_server_get_dram_block_size();
#endif

/* Carve the MC private DRAM block from the end of DRAM */
#ifdef CONFIG_FSL_MC_ENET
	dram_to_hide += mc_get_dram_block_size();
#endif

	return dram_to_hide;
}

int board_eth_init(bd_t *bis)
{
	int error = 0;

#ifdef CONFIG_SMC91111
	error = smc91111_initialize(0, CONFIG_SMC91111_BASE);
#endif

#ifdef CONFIG_FSL_MC_ENET
	error = cpu_eth_init(bis);
#endif
	return error;
}

#ifdef CONFIG_FSL_MC_ENET
void fdt_fixup_board_enet(void *fdt)
{
	int offset;

	offset = fdt_path_offset(fdt, "/fsl-mc");

	/*
	 * TODO: Remove this when backward compatibility
	 * with old DT node (fsl,dprc@0) is no longer needed.
	 */
	if (offset < 0)
		offset = fdt_path_offset(fdt, "/fsl,dprc@0");

	if (offset < 0) {
		printf("%s: ERROR: fsl-mc node not found in device tree (error %d)\n",
		       __func__, offset);
		return;
	}

	if (get_mc_boot_status() == 0)
		fdt_status_okay(fdt, offset);
	else
		fdt_status_fail(fdt, offset);
}
#endif

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	phys_addr_t base;
	phys_size_t size;

	ft_cpu_setup(blob, bd);

	/* limit the memory size to bank 1 until Linux can handle 40-bit PA */
	base = getenv_bootm_low();
	size = getenv_bootm_size();
	fdt_fixup_memory(blob, (u64)base, (u64)size);

#ifdef CONFIG_FSL_MC_ENET
	fdt_fixup_board_enet(blob);
	fsl_mc_ldpaa_exit(bd);
#endif

	return 0;
}
#endif
