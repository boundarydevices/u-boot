/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __DRV_CLK_MTK_MUX_H
#define __DRV_CLK_MTK_MUX_H

#include <linux/types.h>

#define MTKF_CLK_MUX_SETCLR_UPD	1

struct clk *mtk_clk_register_mux(const char *name,
		const char * const *parent_names, u8 num_parents,
		unsigned long flags,
		void __iomem *reg, u8 mux_shift, u8 mux_width,
		u8 gate_bit, void __iomem *upd_reg, u8 upd_bit,
		u8 mtk_flags);
#endif
