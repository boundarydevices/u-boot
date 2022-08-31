// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Chris-qj Chen <chris-qj.chen@mediatek.com>
 */

#include <common.h>
#include <dm.h>
#include <asm/io.h>
#include <dt-bindings/clock/mt8195-clk.h>
#include <linux/bitops.h>

#include "clk-mtk.h"

extern const struct mtk_pll_data apmixed_plls[];
extern const struct mtk_fixed_clk top_fixed_clks[];
extern const struct mtk_fixed_factor top_fixed_divs[];
extern const struct mtk_composite top_muxes[];

const struct mtk_clk_tree mt8195_clk_tree = {
	.xtal_rate = 26 * MHZ,
	.xtal2_rate = 26 * MHZ,
	.fdivs_offs = CLK_TOP_CLK26M_D2,
	.muxes_offs = CLK_TOP_AXI,
	.plls = apmixed_plls,
	.fclks = top_fixed_clks,
	.fdivs = top_fixed_divs,
	.muxes = top_muxes,
};
