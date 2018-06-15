/*
 * (C)Copyright 2016 Rockchip Electronics Co., Ltd
 * Authors: Andy Yan <andy.yan@rock-chips.com>
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <fdtdec.h>
#include <asm/arch/grf_rv1108.h>
#include <asm/arch/hardware.h>

DECLARE_GLOBAL_DATA_PTR;

int mach_cpu_init(void)
{
	int node;
	struct rv1108_grf *grf;

	node = fdt_node_offset_by_compatible(gd->fdt_blob, -1, "rockchip,rv1108-grf");
	grf = (struct rv1108_grf *)fdtdec_get_addr(gd->fdt_blob, node, "reg");

	/*evb board use UART2 m0 for debug*/
	rk_clrsetreg(&grf->gpio2d_iomux,
		     GPIO2D2_MASK | GPIO2D1_MASK,
		     GPIO2D2_UART2_SOUT_M0 << GPIO2D2_SHIFT |
		     GPIO2D1_UART2_SIN_M0 << GPIO2D1_SHIFT);
	rk_clrreg(&grf->gpio3c_iomux, GPIO3C3_MASK | GPIO3C2_MASK);

	return 0;
}


int board_init(void)
{
	return 0;
}

int dram_init(void)
{
	gd->ram_size = 0x8000000;

	return 0;
}

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = 0x60000000;
	gd->bd->bi_dram[0].size = 0x8000000;

	return 0;
}
