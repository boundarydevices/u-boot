// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017~2018 NXP
 *
 */

#include <common.h>
#include <log.h>
#include <asm/io.h>
#include <malloc.h>
#include <clk-uclass.h>
#include <dm/device.h>
#include <dm/devres.h>
#include <dm/uclass.h>
#include <linux/clk-provider.h>
#include <clk.h>
#include "clk.h"
#include <linux/err.h>
#include "../clk-fractional-divider.h"

#define PCG_PCS_SHIFT	24
#define PCG_PCS_MASK	0x7
#define PCG_CGC_SHIFT	30
#define PCG_FRAC_SHIFT	3
#define PCG_FRAC_WIDTH	1
#define PCG_FRAC_MASK	BIT(3)
#define PCG_PCD_SHIFT	0
#define PCG_PCD_WIDTH	3
#define PCG_PCD_MASK	0x7

#define SW_RST		BIT(28)

static int pcc_gate_enable(struct clk *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	u32 val;
	int ret;

	ret = clk_gate_ops.enable(hw);
	if (ret)
		return ret;

	/*
	 * release the sw reset for peripherals associated with
	 * with this pcc clock.
	 */
	val = readl(gate->reg);
	val |= SW_RST;
	writel(val, gate->reg);

	return 0;
}

static int pcc_gate_disable(struct clk *hw)
{
	clk_gate_ops.disable(hw);
	return 0;
}

static const struct clk_ops pcc_gate_ops = {
	.enable = pcc_gate_enable,
	.disable = pcc_gate_disable,
};

struct clk *imx8ulp_clk_composite(const char *name,
				     const char * const *parent_names,
				     int num_parents, bool mux_present,
				     bool rate_present, bool gate_present,
				     void __iomem *reg, bool has_swrst)
{
	struct clk *mux_hw = NULL, *fd_hw = NULL, *gate_hw = NULL;
	struct clk_fractional_divider *fd = NULL;
	struct clk_gate *gate = NULL;
	struct clk_mux *mux = NULL;
	struct clk *hw;
	int i;
	u32 val;

	if (mux_present) {
		mux = kzalloc(sizeof(*mux), GFP_KERNEL);
		if (!mux)
			return ERR_PTR(-ENOMEM);
		mux_hw = &mux->clk;
		mux->reg = reg;
		mux->shift = PCG_PCS_SHIFT;
		mux->mask = PCG_PCS_MASK;
		mux->parent_names = parent_names;
		mux->num_parents = num_parents;
	}

	if (rate_present) {
		fd = kzalloc(sizeof(*fd), GFP_KERNEL);
		if (!fd) {
			kfree(mux);
			return ERR_PTR(-ENOMEM);
		}
		fd_hw = &fd->clk;
		fd->reg = reg;
		fd->mshift = PCG_FRAC_SHIFT;
		fd->mwidth = PCG_FRAC_WIDTH;
		fd->mmask  = PCG_FRAC_MASK;
		fd->nshift = PCG_PCD_SHIFT;
		fd->nwidth = PCG_PCD_WIDTH;
		fd->nmask = PCG_PCD_MASK;
		fd->flags = CLK_FRAC_DIVIDER_ZERO_BASED;
	}

	if (gate_present) {
		gate = kzalloc(sizeof(*gate), GFP_KERNEL);
		if (!gate) {
			kfree(mux);
			kfree(fd);
			return ERR_PTR(-ENOMEM);
		}
		gate_hw = &gate->clk;
		gate->reg = reg;
		gate->bit_idx = PCG_CGC_SHIFT;
		/*
		 * make sure clock is gated during clock tree initialization,
		 * the HW ONLY allow clock parent/rate changed with clock gated,
		 * during clock tree initialization, clocks could be enabled
		 * by bootloader, so the HW status will mismatch with clock tree
		 * prepare count, then clock core driver will allow parent/rate
		 * change since the prepare count is zero, but HW actually
		 * prevent the parent/rate change due to the clock is enabled.
		 */
		val = readl_relaxed(reg);
		val &= ~(1 << PCG_CGC_SHIFT);
		writel_relaxed(val, reg);
	}

	for (i = 0; i < num_parents; i++) {
		const char *parent_name = parent_names[i];
		struct udevice *parent;
		int ret;

		ret = uclass_get_device_by_name(UCLASS_CLK, parent_name, &parent);
		if (ret)
			printf("%s: %s not found %d\n", __func__, parent_name, ret);
	}

	hw = clk_register_composite(NULL, name,
		parent_names, num_parents,
		mux_hw, &clk_mux_ops,
		fd_hw, &clk_fractional_divider_ops,
		gate_hw, has_swrst ? &pcc_gate_ops : &clk_gate_ops,
		CLK_SET_RATE_GATE | CLK_SET_PARENT_GATE |
		CLK_SET_RATE_NO_REPARENT);
	if (IS_ERR(hw)) {
		kfree(mux);
		kfree(fd);
		kfree(gate);
	}

	return hw;
}

struct clk *imx7ulp_clk_composite(const char *name, const char * const *parent_names,
				int num_parents, bool mux_present, bool rate_present,
				bool gate_present, void __iomem *reg)
{
	return imx8ulp_clk_composite(name, parent_names, num_parents, mux_present, rate_present,
					gate_present, reg, false);
}
