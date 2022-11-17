// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017~2018 NXP
 *
 * Author: Dong Aisheng <aisheng.dong@nxp.com>
 *
 */

#include <common.h>
#include <asm/io.h>
#include <malloc.h>
#include <clk-uclass.h>
#include <dm/device.h>
#include <dm/devres.h>
#include <linux/clk-provider.h>
#include <div64.h>
#include <clk.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include "clk.h"


/**
 * struct clk_pfdv2 - IMX PFD clock
 * @hw:		clock source
 * @reg:	PFD register address
 * @gate_bit:	Gate bit offset
 * @vld_bit:	Valid bit offset
 * @frac_off:	PLL Fractional Divider offset
 */

struct clk_pfdv2 {
	struct clk	hw;
	void __iomem	*reg;
	u8		gate_bit;
	u8		vld_bit;
	u8		frac_off;
};

#define to_clk_pfdv2(_hw) container_of(_hw, struct clk_pfdv2, hw)

#define CLK_PFDV2_FRAC_MASK 0x3f

#define LOCK_TIMEOUT_US		1000

static int clk_pfdv2_wait(struct clk_pfdv2 *pfd)
{
	u32 val;

	return readl_poll_timeout(pfd->reg, val, val & (1 << pfd->vld_bit),
				  LOCK_TIMEOUT_US);
}

static int clk_pfdv2_enable(struct clk *hw)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u32 val;

	val = readl_relaxed(pfd->reg);
	val &= ~(1 << pfd->gate_bit);
	writel_relaxed(val, pfd->reg);

	return clk_pfdv2_wait(pfd);
}

static int clk_pfdv2_disable(struct clk *hw)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u32 val;

	val = readl_relaxed(pfd->reg);
	val |= (1 << pfd->gate_bit);
	writel_relaxed(val, pfd->reg);
	return 0;
}

static unsigned long clk_pfdv2_recalc_rate(struct clk *hw)
{
	unsigned long parent_rate = clk_get_parent_rate(hw);
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u64 tmp = parent_rate;
	u8 frac;

	frac = (readl_relaxed(pfd->reg) >> pfd->frac_off)
		& CLK_PFDV2_FRAC_MASK;

	if (!frac) {
		pr_debug("clk_pfdv2: %s invalid pfd frac value 0\n",
			hw->dev->name);
		return 0;
	}

	tmp *= 18;
	do_div(tmp, frac);

	return tmp;
}

static ulong clk_pfdv2_determine_rate(struct clk *hw, ulong rate)
{
	unsigned long parent_rate = clk_get_parent_rate(hw);
	u64 tmp;
	unsigned frac;

	tmp = parent_rate;
	tmp = tmp * 18 + rate / 2;
	do_div(tmp, rate);
	frac = tmp;

	if (frac < 12)
		frac = 12;
	else if (frac > 35)
		frac = 35;

	tmp = parent_rate;
	tmp *= 18;
	do_div(tmp, frac);

	return tmp;
}

static int clk_pfdv2_is_enabled(struct clk *hw)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);

	if (readl_relaxed(pfd->reg) & (1 << pfd->gate_bit))
		return 0;

	return 1;
}

static ulong clk_pfdv2_set_rate(struct clk *hw, ulong rate)
{
	unsigned long parent_rate = clk_get_parent_rate(hw);
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u64 tmp = parent_rate;
	u32 val;
	u8 frac;

	if (!rate)
		return -EINVAL;

	/*
	 * PFD can NOT change rate without gating.
	 * as the PFDs may enabled in HW by default but no
	 * consumer used it, the enable count is '0', so the
	 * 'SET_RATE_GATE' can NOT help on blocking the set_rate
	 * ops especially for 'assigned-clock-xxx'. In order
	 * to simplify the case, just disable the PFD if it is
	 * enabled in HW but not in SW.
	 */
	if (clk_pfdv2_is_enabled(hw))
		clk_pfdv2_disable(hw);

	tmp = tmp * 18 + rate / 2;
	do_div(tmp, rate);
	frac = tmp;
	if (frac < 12)
		frac = 12;
	else if (frac > 35)
		frac = 35;

	val = readl_relaxed(pfd->reg);
	val &= ~(CLK_PFDV2_FRAC_MASK << pfd->frac_off);
	val |= frac << pfd->frac_off;
	writel_relaxed(val, pfd->reg);

	tmp = parent_rate;
	tmp = tmp * 18;
	do_div(tmp, frac);

	return tmp;
}

static const struct clk_ops clk_pfdv2_ops = {
	.get_rate	= clk_pfdv2_recalc_rate,
	.set_rate	= clk_pfdv2_set_rate,
	.enable		= clk_pfdv2_enable,
	.disable	= clk_pfdv2_disable,
	.round_rate	= clk_pfdv2_determine_rate,
};

struct clk *imx_clk_pfdv2(enum imx_pfdv2_type type, const char *name,
			     const char *parent_name, void __iomem *reg, u8 idx)
{
	struct clk_pfdv2 *pfd;
	struct clk *hw;
	int ret;

	if (idx > 3)
		return ERR_PTR(-EINVAL);

	pfd = kzalloc(sizeof(*pfd), GFP_KERNEL);
	if (!pfd)
		return ERR_PTR(-ENOMEM);

	pfd->reg = reg;
	pfd->gate_bit = (idx + 1) * 8 - 1;
	pfd->vld_bit = pfd->gate_bit - 1;
	pfd->frac_off = idx * 8;

	hw = &pfd->hw;
	if (type == IMX_PFDV2_IMX7ULP)
		hw->flags = CLK_SET_RATE_GATE | CLK_SET_RATE_PARENT;
	else
		hw->flags = CLK_SET_RATE_GATE;

	ret = clk_register(hw, "imx_clk_pfdv2", name, parent_name);
	if (ret) {
		kfree(pfd);
		hw = ERR_PTR(ret);
	}

	return hw;
}

U_BOOT_DRIVER(clk_pfdv2) = {
	.name	= "imx_clk_pfdv2",
	.id	= UCLASS_CLK,
	.ops	= &clk_pfdv2_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
