// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 Lucas Stach <l.stach@pengutronix.de>, Pengutronix
 */

#include <common.h>
#include <asm/io.h>
#include <malloc.h>
#include <clk-uclass.h>
#include <dm/device.h>
#include <dm/devres.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <clk.h>
#include <div64.h>

#define UBOOT_DM_CLK_IMX_CPU "imx_clk_cpu"

struct clk_cpu {
	struct clk	hw;
	struct clk	*div;
	struct clk	*mux;
	struct clk	*pll;
	struct clk	*step;
};

static inline struct clk_cpu *to_clk_cpu(struct clk *hw)
{
	return container_of(hw, struct clk_cpu, hw);
}

static unsigned long clk_cpu_recalc_rate(struct clk *hw)
{
	struct clk_cpu *cpu = to_clk_cpu(hw);

	return clk_get_rate(cpu->div);
}

static ulong clk_cpu_set_rate(struct clk *hw, unsigned long rate)
{
	struct clk_cpu *cpu = to_clk_cpu(hw);
	int ret;

	/* switch to PLL bypass clock */
	ret = clk_set_parent(cpu->mux, cpu->step);
	if (ret)
		return ret;

	/* reprogram PLL */
	ret = clk_set_rate(cpu->pll, rate);
	if (ret) {
		clk_set_parent(cpu->mux, cpu->pll);
		return ret;
	}
	/* switch back to PLL clock */
	clk_set_parent(cpu->mux, cpu->pll);

	/* Ensure the divider is what we expect */
	clk_set_rate(cpu->div, rate);

	return 0;
}

static const struct clk_ops clk_cpu_ops = {
	.get_rate	= clk_cpu_recalc_rate,
	.set_rate	= clk_cpu_set_rate,
};

struct clk *imx_clk_cpu(const char *name, const char *parent_name,
		struct clk *div, struct clk *mux, struct clk *pll,
		struct clk *step)
{
	struct clk_cpu *cpu;
	struct clk *hw;
	int ret;

	cpu = kzalloc(sizeof(*cpu), GFP_KERNEL);
	if (!cpu)
		return ERR_PTR(-ENOMEM);

	cpu->div = div;
	cpu->mux = mux;
	cpu->pll = pll;
	cpu->step = step;
	hw = &cpu->hw;

	ret = clk_register(hw, UBOOT_DM_CLK_IMX_CPU, name, parent_name);
	if (ret) {
		pr_err("%s: failed to register pll %s %d\n",
		       __func__, name, ret);
		kfree(pll);
		return ERR_PTR(ret);
	}

	return hw;
}

U_BOOT_DRIVER(clk_cpu) = {
	.name	= UBOOT_DM_CLK_IMX_CPU,
	.id	= UCLASS_CLK,
	.ops	= &clk_cpu_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
