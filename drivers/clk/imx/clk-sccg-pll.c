// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Copyright 2018 NXP.
 *
 * This driver supports the SCCG plls found in the imx8m SOCs
 *
 * Documentation for this SCCG pll can be found at:
 *   https://www.nxp.com/docs/en/reference-manual/IMX8MDQLQRM.pdf#page=834
 */

#include <common.h>
#include <asm/io.h>
#include <malloc.h>
#include <clk-uclass.h>
#include <dm/device.h>
#include <dm/devres.h>
#include <dm/uclass.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <clk.h>
#include <div64.h>

#include "clk.h"

struct clk_rate_request {
	unsigned long rate;
	unsigned long min_rate;
	unsigned long max_rate;
	unsigned long best_parent_rate;
	struct clk *best_parent_hw;
};

#define UBOOT_DM_CLK_IMX_SCCG_PLL "imx_clk_sccg_pll"

/* PLL CFGs */
#define PLL_CFG0		0x0
#define PLL_CFG1		0x4
#define PLL_CFG2		0x8

#define PLL_DIVF1_MASK		GENMASK(18, 13)
#define PLL_DIVF2_MASK		GENMASK(12, 7)
#define PLL_DIVR1_MASK		GENMASK(27, 25)
#define PLL_DIVR2_MASK		GENMASK(24, 19)
#define PLL_DIVQ_MASK           GENMASK(6, 1)
#define PLL_REF_MASK		GENMASK(2, 0)

#define PLL_LOCK_MASK		BIT(31)
#define PLL_PD_MASK		BIT(7)

/* These are the specification limits for the SSCG PLL */
#define PLL_REF_MIN_FREQ		25000000UL
#define PLL_REF_MAX_FREQ		235000000UL

#define PLL_STAGE1_MIN_FREQ		1600000000UL
#define PLL_STAGE1_MAX_FREQ		2400000000UL

#define PLL_STAGE1_REF_MIN_FREQ		25000000UL
#define PLL_STAGE1_REF_MAX_FREQ		54000000UL

#define PLL_STAGE2_MIN_FREQ		1200000000UL
#define PLL_STAGE2_MAX_FREQ		2400000000UL

#define PLL_STAGE2_REF_MIN_FREQ		54000000UL
#define PLL_STAGE2_REF_MAX_FREQ		75000000UL

#define PLL_OUT_MIN_FREQ		20000000UL
#define PLL_OUT_MAX_FREQ		1200000000UL

#define PLL_DIVR1_MAX			7
#define PLL_DIVR2_MAX			63
#define PLL_DIVF1_MAX			63
#define PLL_DIVF2_MAX			63
#define PLL_DIVQ_MAX			63

#define PLL_BYPASS_NONE			0x0
#define PLL_BYPASS1			0x2
#define PLL_BYPASS2			0x1

#define SSCG_PLL_BYPASS1_MASK           BIT(5)
#define SSCG_PLL_BYPASS2_MASK           BIT(4)
#define SSCG_PLL_BYPASS_MASK		GENMASK(5, 4)

#define PLL_SCCG_LOCK_TIMEOUT		70

struct clk_sccg_pll_setup {
	int divr1, divf1;
	int divr2, divf2;
	int divq;
	int bypass;

	uint64_t vco1;
	uint64_t vco2;
	uint64_t fout;
	uint64_t ref;
	uint64_t ref_div1;
	uint64_t ref_div2;
	uint64_t fout_request;
	int fout_error;
};

struct clk_sccg_pll {
	struct clk	hw;

	void __iomem *base;
	struct clk *parents[3];

	struct clk_sccg_pll_setup setup;

	u8 parent;
	u8 bypass1;
	u8 bypass2;
	u8 flags;
};

#define to_clk_sccg_pll(_hw) container_of(_hw, struct clk_sccg_pll, hw)

static int clk_sccg_pll_wait_lock(struct clk_sccg_pll *pll)
{
	u32 val;

	val = readl_relaxed(pll->base + PLL_CFG0);

	/* don't wait for lock if all plls are bypassed */
	if (!(val & SSCG_PLL_BYPASS2_MASK))
		return readl_poll_timeout(pll->base, val, val & PLL_LOCK_MASK,
						PLL_SCCG_LOCK_TIMEOUT);

	return 0;
}

static int clk_sccg_pll2_check_match(struct clk_sccg_pll_setup *setup,
					struct clk_sccg_pll_setup *temp_setup)
{
	int new_diff = temp_setup->fout - temp_setup->fout_request;
	int diff = temp_setup->fout_error;

	if (abs(diff) > abs(new_diff)) {
		temp_setup->fout_error = new_diff;
		memcpy(setup, temp_setup, sizeof(struct clk_sccg_pll_setup));

		if (temp_setup->fout_request == temp_setup->fout)
			return 0;
	}
	return -1;
}

static int clk_sccg_divq_lookup(struct clk_sccg_pll_setup *setup,
				struct clk_sccg_pll_setup *temp_setup)
{
	int ret = -EINVAL;

	for (temp_setup->divq = 0; temp_setup->divq <= PLL_DIVQ_MAX;
	     temp_setup->divq++) {
		temp_setup->vco2 = temp_setup->vco1;
		do_div(temp_setup->vco2, temp_setup->divr2 + 1);
		temp_setup->vco2 *= 2;
		temp_setup->vco2 *= temp_setup->divf2 + 1;
		if (temp_setup->vco2 >= PLL_STAGE2_MIN_FREQ &&
				temp_setup->vco2 <= PLL_STAGE2_MAX_FREQ) {
			temp_setup->fout = temp_setup->vco2;
			do_div(temp_setup->fout, 2 * (temp_setup->divq + 1));

			ret = clk_sccg_pll2_check_match(setup, temp_setup);
			if (!ret) {
				temp_setup->bypass = PLL_BYPASS1;
				return ret;
			}
		}
	}

	return ret;
}

static int clk_sccg_divf2_lookup(struct clk_sccg_pll_setup *setup,
					struct clk_sccg_pll_setup *temp_setup)
{
	int ret = -EINVAL;

	for (temp_setup->divf2 = 0; temp_setup->divf2 <= PLL_DIVF2_MAX;
	     temp_setup->divf2++) {
		ret = clk_sccg_divq_lookup(setup, temp_setup);
		if (!ret)
			return ret;
	}

	return ret;
}

static int clk_sccg_divr2_lookup(struct clk_sccg_pll_setup *setup,
				struct clk_sccg_pll_setup *temp_setup)
{
	int ret = -EINVAL;

	for (temp_setup->divr2 = 0; temp_setup->divr2 <= PLL_DIVR2_MAX;
	     temp_setup->divr2++) {
		temp_setup->ref_div2 = temp_setup->vco1;
		do_div(temp_setup->ref_div2, temp_setup->divr2 + 1);
		if (temp_setup->ref_div2 >= PLL_STAGE2_REF_MIN_FREQ &&
		    temp_setup->ref_div2 <= PLL_STAGE2_REF_MAX_FREQ) {
			ret = clk_sccg_divf2_lookup(setup, temp_setup);
			if (!ret)
				return ret;
		}
	}

	return ret;
}

static int clk_sccg_pll2_find_setup(struct clk_sccg_pll_setup *setup,
					struct clk_sccg_pll_setup *temp_setup,
					uint64_t ref)
{

	int ret = -EINVAL;

	if (ref < PLL_STAGE1_MIN_FREQ || ref > PLL_STAGE1_MAX_FREQ)
		return ret;

	temp_setup->vco1 = ref;

	ret = clk_sccg_divr2_lookup(setup, temp_setup);
	return ret;
}

static int clk_sccg_divf1_lookup(struct clk_sccg_pll_setup *setup,
				struct clk_sccg_pll_setup *temp_setup)
{
	int ret = -EINVAL;

	for (temp_setup->divf1 = 0; temp_setup->divf1 <= PLL_DIVF1_MAX;
	     temp_setup->divf1++) {
		uint64_t vco1 = temp_setup->ref;

		do_div(vco1, temp_setup->divr1 + 1);
		vco1 *= 2;
		vco1 *= temp_setup->divf1 + 1;

		ret = clk_sccg_pll2_find_setup(setup, temp_setup, vco1);
		if (!ret) {
			temp_setup->bypass = PLL_BYPASS_NONE;
			return ret;
		}
	}

	return ret;
}

static int clk_sccg_divr1_lookup(struct clk_sccg_pll_setup *setup,
				struct clk_sccg_pll_setup *temp_setup)
{
	int ret = -EINVAL;

	for (temp_setup->divr1 = 0; temp_setup->divr1 <= PLL_DIVR1_MAX;
	     temp_setup->divr1++) {
		temp_setup->ref_div1 = temp_setup->ref;
		do_div(temp_setup->ref_div1, temp_setup->divr1 + 1);
		if (temp_setup->ref_div1 >= PLL_STAGE1_REF_MIN_FREQ &&
		    temp_setup->ref_div1 <= PLL_STAGE1_REF_MAX_FREQ) {
			ret = clk_sccg_divf1_lookup(setup, temp_setup);
			if (!ret)
				return ret;
		}
	}

	return ret;
}

static int clk_sccg_pll1_find_setup(struct clk_sccg_pll_setup *setup,
					struct clk_sccg_pll_setup *temp_setup,
					uint64_t ref)
{

	int ret = -EINVAL;

	if (ref < PLL_REF_MIN_FREQ || ref > PLL_REF_MAX_FREQ)
		return ret;

	temp_setup->ref = ref;

	ret = clk_sccg_divr1_lookup(setup, temp_setup);

	return ret;
}

static int clk_sccg_pll_find_setup(struct clk_sccg_pll_setup *setup,
					uint64_t prate,
					uint64_t rate, int try_bypass)
{
	struct clk_sccg_pll_setup temp_setup;
	int ret = -EINVAL;

	memset(&temp_setup, 0, sizeof(struct clk_sccg_pll_setup));
	memset(setup, 0, sizeof(struct clk_sccg_pll_setup));

	temp_setup.fout_error = PLL_OUT_MAX_FREQ;
	temp_setup.fout_request = rate;

	switch (try_bypass) {

	case PLL_BYPASS2:
		if (prate == rate) {
			setup->bypass = PLL_BYPASS2;
			setup->fout = rate;
			ret = 0;
		}
		break;

	case PLL_BYPASS1:
		ret = clk_sccg_pll2_find_setup(setup, &temp_setup, prate);
		break;

	case PLL_BYPASS_NONE:
		ret = clk_sccg_pll1_find_setup(setup, &temp_setup, prate);
		break;
	}

	return ret;
}

static int clk_sccg_pll_prepare(struct clk *hw)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CFG0);
	val &= ~PLL_PD_MASK;
	writel_relaxed(val, pll->base + PLL_CFG0);

	return clk_sccg_pll_wait_lock(pll);
}

static int clk_sccg_pll_unprepare(struct clk *hw)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CFG0);
	val |= PLL_PD_MASK;
	writel_relaxed(val, pll->base + PLL_CFG0);
	return 0;
}

static unsigned long clk_sccg_pll_recalc_rate(struct clk *hw)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val, divr1, divf1, divr2, divf2, divq;
	u64 temp64;
	unsigned long parent_rate = clk_get_parent_rate(hw);

	val = readl_relaxed(pll->base + PLL_CFG2);
	divr1 = FIELD_GET(PLL_DIVR1_MASK, val);
	divr2 = FIELD_GET(PLL_DIVR2_MASK, val);
	divf1 = FIELD_GET(PLL_DIVF1_MASK, val);
	divf2 = FIELD_GET(PLL_DIVF2_MASK, val);
	divq = FIELD_GET(PLL_DIVQ_MASK, val);

	temp64 = parent_rate;

	val = readl(pll->base + PLL_CFG0);
	if (val & SSCG_PLL_BYPASS2_MASK) {
		temp64 = parent_rate;
	} else if (val & SSCG_PLL_BYPASS1_MASK) {
		temp64 *= divf2;
		do_div(temp64, (divr2 + 1) * (divq + 1));
	} else {
		temp64 *= 2;
		temp64 *= (divf1 + 1) * (divf2 + 1);
		do_div(temp64, (divr1 + 1) * (divr2 + 1) * (divq + 1));
	}

	return temp64;
}

static u8 clk_sccg_pll_get_parent(struct clk *hw)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val;
	u8 ret = pll->parent;

	val = readl(pll->base + PLL_CFG0);
	if (val & SSCG_PLL_BYPASS2_MASK)
		ret = pll->bypass2;
	else if (val & SSCG_PLL_BYPASS1_MASK)
		ret = pll->bypass1;
	return ret;
}

static int clk_sccg_pll_set_parent(struct clk *hw, struct clk *parent)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val;

	val = readl(pll->base + PLL_CFG0);
	val &= ~SSCG_PLL_BYPASS_MASK;
	val |= FIELD_PREP(SSCG_PLL_BYPASS_MASK, pll->setup.bypass);
	writel(val, pll->base + PLL_CFG0);

	return clk_sccg_pll_wait_lock(pll);
}

static int __clk_sccg_pll_determine_rate(struct clk *hw,
					struct clk_rate_request *req,
					uint64_t min,
					uint64_t max,
					uint64_t rate,
					int bypass)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	struct clk_sccg_pll_setup *setup = &pll->setup;
	struct clk *parent_clk = NULL;
	int bypass_parent_index;
	int ret = -EINVAL;

	req->max_rate = max;
	req->min_rate = min;

	switch (bypass) {
	case PLL_BYPASS2:
		bypass_parent_index = pll->bypass2;
		break;
	case PLL_BYPASS1:
		bypass_parent_index = pll->bypass1;
		break;
	default:
		bypass_parent_index = pll->parent;
		break;
	}

	parent_clk = pll->parents[bypass_parent_index];
	req->rate = clk_get_rate(parent_clk);
	if (!ret) {
		ret = clk_sccg_pll_find_setup(setup, req->rate,
						rate, bypass);
	}

	req->best_parent_hw = parent_clk;
	req->best_parent_rate = req->rate;
	req->rate = setup->fout;

	return ret;
}

static int clk_sccg_pll_determine_rate(struct clk *hw,
				       struct clk_rate_request *req)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	struct clk_sccg_pll_setup *setup = &pll->setup;
	uint64_t rate = req->rate;
	uint64_t min = req->min_rate;
	uint64_t max = req->max_rate;
	int ret = -EINVAL;

	if (rate < PLL_OUT_MIN_FREQ || rate > PLL_OUT_MAX_FREQ)
		return ret;

	ret = __clk_sccg_pll_determine_rate(hw, req, req->rate, req->rate,
						rate, PLL_BYPASS2);
	if (!ret)
		return ret;

	ret = __clk_sccg_pll_determine_rate(hw, req, PLL_STAGE1_REF_MIN_FREQ,
						PLL_STAGE1_REF_MAX_FREQ, rate,
						PLL_BYPASS1);
	if (!ret)
		return ret;

	ret = __clk_sccg_pll_determine_rate(hw, req, PLL_REF_MIN_FREQ,
						PLL_REF_MAX_FREQ, rate,
						PLL_BYPASS_NONE);
	if (!ret)
		return ret;

	if (setup->fout >= min && setup->fout <= max)
		ret = 0;

	return ret;
}

static ulong clk_sccg_pll_set_rate(struct clk *hw, unsigned long rate)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	struct clk_sccg_pll_setup *setup = &pll->setup;
	u32 val;
	struct clk_rate_request req;

	req.rate = rate;
	req.min_rate = rate >> 1;
	req.max_rate = rate << 1;
	req.best_parent_rate = 0;
	req.best_parent_hw = NULL;
	clk_sccg_pll_determine_rate(hw, &req);

	/* set bypass here too since the parent might be the same */
	val = readl(pll->base + PLL_CFG0);
	val &= ~SSCG_PLL_BYPASS_MASK;
	val |= FIELD_PREP(SSCG_PLL_BYPASS_MASK, setup->bypass);
	writel(val, pll->base + PLL_CFG0);

	val = readl_relaxed(pll->base + PLL_CFG2);
	val &= ~(PLL_DIVF1_MASK | PLL_DIVF2_MASK);
	val &= ~(PLL_DIVR1_MASK | PLL_DIVR2_MASK | PLL_DIVQ_MASK);
	val |= FIELD_PREP(PLL_DIVF1_MASK, setup->divf1);
	val |= FIELD_PREP(PLL_DIVF2_MASK, setup->divf2);
	val |= FIELD_PREP(PLL_DIVR1_MASK, setup->divr1);
	val |= FIELD_PREP(PLL_DIVR2_MASK, setup->divr2);
	val |= FIELD_PREP(PLL_DIVQ_MASK, setup->divq);
	writel_relaxed(val, pll->base + PLL_CFG2);

	return clk_sccg_pll_wait_lock(pll);
}

static const struct clk_ops clk_sccg_pll_ops = {
	.enable		= clk_sccg_pll_prepare,
	.disable	= clk_sccg_pll_unprepare,
	.get_rate	= clk_sccg_pll_recalc_rate,
	.set_rate	= clk_sccg_pll_set_rate,
	.set_parent	= clk_sccg_pll_set_parent,
};

struct clk *imx_clk_sccg_pll(const char *name,
				const char * const *parent_names,
				u8 num_parents,
				u8 parent, u8 bypass1, u8 bypass2,
				void __iomem *base,
				unsigned long flags)
{
	struct clk_sccg_pll *pll;
	struct clk *clk;
	int ret;
	int cur_parent;
	int i;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->parent = parent;
	pll->bypass1 = bypass1;
	pll->bypass2 = bypass2;

	pll->base = base;
	pll->flags = flags;
	clk = &pll->hw;

	if (num_parents > ARRAY_SIZE(pll->parents))
		num_parents = ARRAY_SIZE(pll->parents);

	for (i = 0; i < num_parents; i++) {
		struct udevice *parent;

		ret = uclass_get_device_by_name(UCLASS_CLK, parent_names[i], &parent);
		if (ret)
			return ERR_PTR(ret);
		pll->parents[i] = dev_get_clk_ptr(parent);
	}


	cur_parent = clk_sccg_pll_get_parent(clk);
	ret = clk_register(clk, UBOOT_DM_CLK_IMX_SCCG_PLL, name, parent_names[cur_parent]);
	if (ret) {
		pr_err("%s: failed to register pll %s %d\n",
		       __func__, name, ret);
		kfree(pll);
		return ERR_PTR(ret);
	}

	return clk;
}

U_BOOT_DRIVER(clk_sccg_pll) = {
	.name	= UBOOT_DM_CLK_IMX_SCCG_PLL,
	.id	= UCLASS_CLK,
	.ops	= &clk_sccg_pll_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
