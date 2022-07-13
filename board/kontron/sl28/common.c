// SPDX-License-Identifier: GPL-2.0+

#include <common.h>
#include <asm/global_data.h>

DECLARE_GLOBAL_DATA_PTR;

u32 get_lpuart_clk(u32 base)
{
	return gd->bus_clk / CONFIG_SYS_FSL_LPUART_CLK_DIV;
}
