// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014 Intel Corporation
 *
 * Adjustable fractional divider clock implementation.
 * Uses rational best approximation algorithm.
 *
 * Output is calculated as
 *
 *	rate = (m / n) * parent_rate				(1)
 *
 * This is useful when we have a prescaler block which asks for
 * m (numerator) and n (denominator) values to be provided to satisfy
 * the (1) as much as possible.
 *
 * Since m and n have the limitation by a range, e.g.
 *
 *	n >= 1, n < N_width, where N_width = 2^nwidth		(2)
 *
 * for some cases the output may be saturated. Hence, from (1) and (2),
 * assuming the worst case when m = 1, the inequality
 *
 *	floor(log2(parent_rate / rate)) <= nwidth		(3)
 *
 * may be derived. Thus, in cases when
 *
 *	(parent_rate / rate) >> N_width				(4)
 *
 * we might scale up the rate by 2^scale (see the description of
 * CLK_FRAC_DIVIDER_POWER_OF_TWO_PS for additional information), where
 *
 *	scale = floor(log2(parent_rate / rate)) - nwidth	(5)
 *
 * and assume that the IP, that needs m and n, has also its own
 * prescaler, which is capable to divide by 2^scale. In this way
 * we get the denominator to satisfy the desired range (2) and
 * at the same time much much better result of m and n than simple
 * saturated values.
 */

#include <common.h>
#include <asm/io.h>
#include <clk-uclass.h>
#include <div64.h>
#include <dm.h>
#include <log.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/rational.h>

#include "clk-fractional-divider.h"

#define to_clk_fd(__hw) container_of(__hw, struct clk_fractional_divider, clk)

static inline u32 clk_fd_readl(struct clk_fractional_divider *fd)
{
	return readl(fd->reg);
}

static inline void clk_fd_writel(struct clk_fractional_divider *fd, u32 val)
{
	writel(val, fd->reg);
}

static ulong clk_fd_recalc_rate(struct clk *hw)
{
	struct clk_fractional_divider *fd = to_clk_fd(hw);
	unsigned long parent_rate = clk_get_parent_rate(hw);
	unsigned long m, n;
	u32 val;
	u64 ret;

	val = clk_fd_readl(fd);

	m = (val & fd->mmask) >> fd->mshift;
	n = (val & fd->nmask) >> fd->nshift;

	if (fd->flags & CLK_FRAC_DIVIDER_ZERO_BASED) {
		m++;
		n++;
	}

	if (!n || !m)
		return parent_rate;

	ret = (u64)parent_rate * m;
	do_div(ret, n);

	debug("%s: %s: parent_rate=%ld rate=%ld m=%ld n=%ld\n", __func__, hw->dev->name, parent_rate, (unsigned long)ret, m, n);
	return ret;
}

void clk_fractional_divider_general_approximation(struct clk *hw,
						  unsigned long rate,
						  unsigned long *parent_rate,
						  unsigned long *m, unsigned long *n)
{
	struct clk_fractional_divider *fd = to_clk_fd(hw);
	int min = (fd->flags & CLK_FRAC_DIVIDER_ZERO_BASED) ? 1 : 0;

	/*
	 * Get rate closer to *parent_rate to guarantee there is no overflow
	 * for m and n. In the result it will be the nearest rate left shifted
	 * by (scale - fd->nwidth) bits.
	 *
	 * For the detailed explanation see the top comment in this file.
	 */
	if (fd->flags & CLK_FRAC_DIVIDER_POWER_OF_TWO_PS) {
		unsigned long scale = fls_long(*parent_rate / rate - 1);

		if (scale > fd->nwidth)
			rate <<= scale - fd->nwidth;
	}

#if 0
	*m = rate;
	*n = *parent_rate;
	rational_best_ratio_bigger(m, n,
			GENMASK(fd->mwidth - 1, 0) + min, GENMASK(fd->nwidth - 1, 0) + min);
#else
	rational_best_approximation(rate, *parent_rate,
			GENMASK(fd->mwidth - 1, 0) + min, GENMASK(fd->nwidth - 1, 0) + min,
			m, n);
#endif
}

static ulong clk_fd_round_rate(struct clk *hw, ulong rate)
{
	struct clk_fractional_divider *fd = to_clk_fd(hw);
	unsigned long parent_rate = clk_get_parent_rate(hw);
	unsigned long m, n;
	u64 ret;

	if (!rate || (!(hw->flags & CLK_SET_RATE_PARENT) && rate >= parent_rate))
		return parent_rate;

	if (fd->approximation)
		fd->approximation(hw, rate, &parent_rate, &m, &n);
	else
		clk_fractional_divider_general_approximation(hw, rate, &parent_rate, &m, &n);

	ret = (u64)parent_rate * m;
	do_div(ret, n);
	debug("%s: %s :parent_rate=%ld rate=%ld m=%ld n=%ld\n", __func__, hw->dev->name, parent_rate, (unsigned long)ret, m, n);

	return ret;
}

static ulong clk_fd_set_rate(struct clk *hw, unsigned long rate)
{
	struct clk_fractional_divider *fd = to_clk_fd(hw);
	unsigned long parent_rate = clk_get_parent_rate(hw);
	unsigned long m, n;
	u64 temp64;
	u32 val;
	int min = (fd->flags & CLK_FRAC_DIVIDER_ZERO_BASED) ? 1 : 0;

#if 0
	m = rate;
	n = parent_rate;
	rational_best_ratio_bigger(&m, &n,
			GENMASK(fd->mwidth - 1, 0) + min, GENMASK(fd->nwidth - 1, 0) + min);
#else
	rational_best_approximation(rate, parent_rate,
			GENMASK(fd->mwidth - 1, 0) + min, GENMASK(fd->nwidth - 1, 0) + min,
			&m, &n);
#endif
	debug("%s: %s: rate=%ld parent_rate=%ld m=%ld n=%ld\n", __func__, hw->dev->name, rate, parent_rate, m, n);

	if (fd->flags & CLK_FRAC_DIVIDER_ZERO_BASED) {
		if (m)
			m--;
		if (n)
			n--;
	}

	val = clk_fd_readl(fd);
	val &= ~(fd->mmask | fd->nmask);
	val |= (m << fd->mshift) | (n << fd->nshift);
	clk_fd_writel(fd, val);

	if (fd->flags & CLK_FRAC_DIVIDER_ZERO_BASED) {
		m++;
		n++;
	}
	temp64 = (u64)parent_rate * m;
	do_div(temp64, n);
	debug("%s: %s: parent_rate=%ld rate=%ld m=%ld n=%ld\n", __func__, hw->dev->name, parent_rate, (ulong)temp64, m, n);

	return (ulong)temp64;
}

const struct clk_ops clk_fractional_divider_ops = {
	.get_rate = clk_fd_recalc_rate,
	.round_rate = clk_fd_round_rate,
	.set_rate = clk_fd_set_rate,
};

struct clk *clk_register_fractional_divider(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 mshift, u8 mwidth, u8 nshift, u8 nwidth,
		u8 clk_divider_flags, spinlock_t *lock)
{
	struct clk_fractional_divider *fd;
	struct clk *hw;
	int ret;

	fd = kzalloc(sizeof(*fd), GFP_KERNEL);
	if (!fd)
		return ERR_PTR(-ENOMEM);

	hw = &fd->clk;
	hw->flags = flags;

	fd->reg = reg;
	fd->mshift = mshift;
	fd->mwidth = mwidth;
	fd->mmask = GENMASK(mwidth - 1, 0) << mshift;
	fd->nshift = nshift;
	fd->nwidth = nwidth;
	fd->nmask = GENMASK(nwidth - 1, 0) << nshift;
	fd->flags = clk_divider_flags;

	ret = clk_register(hw, "ccf_clk_fractional_divider", name,
			parent_name);
	if (ret) {
		kfree(fd);
		hw = ERR_PTR(ret);
	}

	return hw;
}

U_BOOT_DRIVER(imx_clk_fractional_divider) = {
	.name	= "ccf_clk_fractional_divider",
	.id	= UCLASS_CLK,
	.ops	= &clk_fractional_divider_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
