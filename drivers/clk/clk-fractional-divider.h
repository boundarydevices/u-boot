/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _CLK_FRACTIONAL_DIV_H
#define _CLK_FRACTIONAL_DIV_H

struct clk;

struct clk_fractional_divider {
	struct clk	clk;
	void __iomem	*reg;
	u8		mshift;
	u8		mwidth;
	u32		mmask;
	u8		nshift;
	u8		nwidth;
	u32		nmask;
	u8		flags;
	void		(*approximation)(struct clk *hw,
				unsigned long rate, unsigned long *parent_rate,
				unsigned long *m, unsigned long *n);
};

#define CLK_FRAC_DIVIDER_ZERO_BASED		BIT(0)
#define CLK_FRAC_DIVIDER_BIG_ENDIAN		BIT(1)
#define CLK_FRAC_DIVIDER_POWER_OF_TWO_PS	BIT(2)

extern const struct clk_ops clk_fractional_divider_ops;

void clk_fractional_divider_general_approximation(struct clk *hw,
						  unsigned long rate,
						  unsigned long *parent_rate,
						  unsigned long *m,
						  unsigned long *n);

#endif
