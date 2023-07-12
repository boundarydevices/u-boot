// SPDX-License-Identifier: GPL-2.0-only

#include <common.h>
#include <asm/io.h>
#include <clk.h>
#include <dm/device.h>
#include <linux/clk-provider.h>
#include "clk-mtk-gate.h"

struct mtk_clk_gate {
	struct clk	clk;
	void __iomem	*reg;
	u8		gate_bit;
	u8		mtk_flags;
};

#define MTK_SET_OFFSET	0
#define MTK_CLR_OFFSET	4
#define UBOOT_DM_CLK_CCF_MTK_GATE "ccf_clk_mtk_gate"

static inline struct mtk_clk_gate *to_mtk_clk_gate(struct clk *_clk)
{
	return container_of(_clk, struct mtk_clk_gate, clk);
}

static int mtk_cg_enable(struct clk *clk)
{
	struct mtk_clk_gate *cg = to_mtk_clk_gate(clk);

	if (cg->mtk_flags & MTKF_CLK_GATE_SETCLR) {
		writel(BIT(cg->gate_bit), cg->reg +
			((cg->mtk_flags & MTKF_CLK_GATE_INV) ?
					MTK_SET_OFFSET : MTK_CLR_OFFSET));
	} else if (cg->mtk_flags & MTKF_CLK_GATE_INV) {
		setbits_le32(cg->reg, BIT(cg->gate_bit));
	} else {
		clrbits_le32(cg->reg, BIT(cg->gate_bit));
	}
	return 0;
}

static int mtk_cg_disable(struct clk *clk)
{
	struct mtk_clk_gate *cg = to_mtk_clk_gate(clk);

	if (cg->mtk_flags & MTKF_CLK_GATE_SETCLR) {
		writel(BIT(cg->gate_bit), cg->reg +
			((cg->mtk_flags & MTKF_CLK_GATE_INV) ?
					MTK_CLR_OFFSET : MTK_SET_OFFSET));
	} else if (cg->mtk_flags & MTKF_CLK_GATE_INV) {
		clrbits_le32(cg->reg, BIT(cg->gate_bit));
	} else {
		setbits_le32(cg->reg, BIT(cg->gate_bit));
	}
	return 0;
}

static ulong mtk_cg_recalc_rate(struct clk *clk)
{
	unsigned long parent_rate = clk_get_parent_rate(clk);

	return (ulong)parent_rate;
}

struct clk *mtk_clk_register_gate(const char *name, const char *parent,
		unsigned long flags, void __iomem *reg, u8 gate_bit,
		u8 mtk_flags)
{
	struct mtk_clk_gate *cg;
	struct clk *clk;
	int ret;

	cg = kzalloc(sizeof(*cg), GFP_KERNEL);
	if (!cg)
		return ERR_PTR(-ENOMEM);

	cg->reg = reg;
	cg->gate_bit = gate_bit;
	cg->mtk_flags = mtk_flags;
	clk = &cg->clk;
	clk->flags = flags;

	ret = clk_register(clk, UBOOT_DM_CLK_CCF_MTK_GATE,
		name, parent);
	if (ret) {
		kfree(cg);
		return ERR_PTR(ret);
	}
	return clk;
}

static const struct clk_ops mtk_clk_gate_ops = {
	.enable		= mtk_cg_enable,
	.disable	= mtk_cg_disable,
	.get_rate	= mtk_cg_recalc_rate,
};

U_BOOT_DRIVER(mtk_clk_gate) = {
	.name	= UBOOT_DM_CLK_CCF_MTK_GATE,
	.id	= UCLASS_CLK,
	.ops	= &mtk_clk_gate_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
