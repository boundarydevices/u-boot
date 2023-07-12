/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __DRV_CLK_MTK_PLL_H
#define __DRV_CLK_MTK_PLL_H

#include <linux/types.h>

#define MTKF_RST_BAR	BIT(0)
#define MTKF_PWR_REG1	BIT(1)
#define MTKF_PLL_AO	BIT(2)

struct clk *mtk_clk_register_pll(const char *name,
		const char *parent,
		unsigned long flags,
		void __iomem *base,
		unsigned reg_ofs,
		unsigned tuner_ofs,
		unsigned tuner_en_ofs,
		u8 tuner_en_bit, u8 pll_en_bit, u8 mtk_flags);
#endif /* __DRV_CLK_MTK_PLL_H */
