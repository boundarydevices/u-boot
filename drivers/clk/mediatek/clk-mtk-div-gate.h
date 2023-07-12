/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __DRV_CLK_MTK_DIV_GATE_H
#define __DRV_CLK_MTK_DIV_GATE_H

#include <linux/types.h>

struct clk *mtk_clk_register_div_gate(const char *name,
		const char * const *parent_names, u8 num_parents,
		unsigned long flags,
		void __iomem *gate_reg, u8 gate_bit,
		void __iomem *div_reg, u8 div_shift, u8 div_width);
#endif
