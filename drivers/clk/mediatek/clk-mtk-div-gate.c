// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: James Liao <jamesjj.liao@mediatek.com>
 */

#include <common.h>
#include <clk.h>
#include <linux/clk-provider.h>
#include "clk-mtk-div-gate.h"

struct clk *mtk_clk_register_div_gate(const char *name,
		const char * const *parent_names, u8 num_parents,
		unsigned long flags,
		void __iomem *gate_reg, u8 gate_bit,
		void __iomem *div_reg, u8 div_shift, u8 div_width)
{
	struct clk *clk;
	struct clk *clk_gate = NULL;
	struct clk *clk_div = NULL;
	struct clk_divider *div = NULL;
	struct clk_gate *gate = NULL;
	const struct clk_ops *gate_ops = NULL, *div_ops = NULL;
	int ret;

	if (gate_reg) {
		gate = kzalloc(sizeof(*gate), GFP_KERNEL);
		if (!gate) {
			ret = -ENOMEM;
			goto err_out;
		}

		gate->reg = gate_reg;
		gate->bit_idx = gate_bit;
		gate->flags = CLK_GATE_SET_TO_DISABLE;
		clk_gate = &gate->clk;
		gate_ops = &clk_gate_ops;
	}

	if (div_reg) {
		div = kzalloc(sizeof(*div), GFP_KERNEL);
		if (!div) {
			ret = -ENOMEM;
			goto err_out;
		}

		div->reg = div_reg;
		div->shift = div_shift;
		div->width = div_width;
		clk_div = &div->clk;
		div_ops = &clk_divider_ops;
	}

	clk = clk_register_composite(NULL, name, parent_names, num_parents,
		NULL, NULL,
		clk_div, div_ops,
		clk_gate, gate_ops,
		flags);

	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto err_out;
	}

	return clk;
err_out:
	kfree(div);
	kfree(gate);

	return ERR_PTR(ret);
}
