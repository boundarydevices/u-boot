/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __DRV_CLK_MTK_GATE_H
#define __DRV_CLK_MTK_GATE_H

#include <linux/types.h>

#define MTKF_CLK_GATE_INV	BIT(0)
#define MTKF_CLK_GATE_SETCLR	BIT(1)

struct clk *mtk_clk_register_gate(const char *name, const char *parent,
		unsigned long flags, void __iomem *reg, u8 gate_bit,
		u8 mtk_flags);
#endif
