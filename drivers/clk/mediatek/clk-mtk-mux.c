// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 DENX Software Engineering
 * Lukasz Majewski, DENX Software Engineering, lukma@denx.de
 *
 * Copyright (C) 2011 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 * Copyright (C) 2011 Richard Zhao, Linaro <richard.zhao@linaro.org>
 * Copyright (C) 2011-2012 Mike Turquette, Linaro Ltd <mturquette@linaro.org>
 *
 * Simple multiplexer clock implementation
 */

/*
 * U-Boot CCF porting node:
 *
 * The Linux kernel - as of tag: 5.0-rc3 is using also the imx_clk_fixup_mux()
 * version of CCF mux. It is used on e.g. imx6q to provide fixes (like
 * imx_cscmr1_fixup) for broken HW.
 *
 * At least for IMX6Q (but NOT IMX6QP) it is important when we set the parent
 * clock.
 */

#define LOG_CATEGORY UCLASS_CLK

#include <common.h>
#include <clk.h>
#include <clk-uclass.h>
#include <log.h>
#include <malloc.h>
#include <asm/io.h>
#include <dm/device.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <dm/uclass.h>
#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include "clk-mtk-mux.h"

#include "clk.h"

#define UBOOT_DM_CLK_CCF_MTK_MUX "ccf_clk_mtk_mux"
#define UBOOT_DM_CLK_CCF_MTK_MUX_GATE "ccf_clk_mtk_mux_gate"
#define to_clk_mtk_mux(_clk) container_of(_clk, struct clk_mtk_mux, clk)

struct clk_mtk_mux {
	struct clk	clk;
	void __iomem	*reg;
	void __iomem	*upd_reg;
	u8		mux_shift;
	u8		mux_width;
	u8		gate_bit;
	u8		upd_bit;
	u8		mtk_flags;
	u8		reparent;
	u8		num_parents;
	const char * const *parent_names;
};

static int mtk_clk_mux_get_parent(struct clk *clk)
{
	struct clk_mtk_mux *mux = to_clk_mtk_mux(clk);
	u32 val;

	val = readl(mux->reg);
	val >>= mux->mux_shift;
	val &= (1 << mux->mux_width) - 1;
	if (val >= mux->num_parents)
		return -EINVAL;

	return val;
}

static int clk_fetch_parent_index(struct clk *clk,
				  struct clk *parent)
{
	struct clk_mtk_mux *mux = to_clk_mtk_mux(clk);

	int i;

	if (!parent)
		return -EINVAL;

	for (i = 0; i < mux->num_parents; i++) {
		if (!strcmp(parent->dev->name, mux->parent_names[i])) {
			debug("%s: %s, parent %s found, index %d\n", __func__, clk->dev->name, parent->dev->name, i);
			return i;
		}
	}

	printf("%s: %s, parent %s not found\n", __func__, clk->dev->name, parent->dev->name);
	return -EINVAL;
}

#define MTK_SET_OFFSET	4
#define MTK_CLR_OFFSET	8

static int mtk_clk_mux_set_parent(struct clk *clk, struct clk *parent)
{
	struct clk_mtk_mux *mux = to_clk_mtk_mux(clk);
	int index;
	u32 val;
	u32 mask = ((1 << mux->mux_width) - 1);

	debug("%s: %s, parent %s\n", __func__, clk->dev->name, parent->dev->name);
	index = clk_fetch_parent_index(clk, parent);
	if (index < 0) {
		log_err("Could not fetch index\n");
		return index;
	}

	if (mux->mtk_flags & MTKF_CLK_MUX_SETCLR_UPD) {
		val = (mask << mux->mux_shift);
		writel(val, mux->reg + MTK_CLR_OFFSET);

		val = (index << mux->mux_shift);
		writel(val, mux->reg + MTK_SET_OFFSET);

		if (mux->upd_reg)
			writel(BIT(mux->upd_bit), mux->upd_reg);
	} else {
		val = readl(mux->reg);
		val &= ~(mask << mux->mux_shift);
		val |= (index << mux->mux_shift);
		writel(val, mux->reg);
	}
	return 0;
}

static int mtk_clk_mux_enable(struct clk *clk)
{
	struct clk_mtk_mux *mux = to_clk_mtk_mux(clk);

	writel(BIT(mux->gate_bit), mux->reg + MTK_CLR_OFFSET);
	/*
	 * If the parent has been changed when the clock was disabled, it will
	 * not be effective yet. Set the update bit to ensure the mux gets
	 * updated.
	 */
	if (mux->reparent && mux->upd_reg) {
		writel(BIT(mux->upd_bit), mux->upd_reg);
		mux->reparent = false;
	}

	return 0;
}

static int mtk_clk_mux_disable(struct clk *clk)
{
	struct clk_mtk_mux *mux = to_clk_mtk_mux(clk);

	writel(BIT(mux->gate_bit), mux->reg + MTK_SET_OFFSET);

	return 0;
}

struct clk *mtk_clk_register_mux(const char *name,
		const char * const *parent_names, u8 num_parents,
		unsigned long flags,
		void __iomem *reg, u8 mux_shift, u8 mux_width,
		u8 gate_bit, void __iomem *upd_reg, u8 upd_bit,
		u8 mtk_flags)
{
	struct clk_mtk_mux *mux;
	struct clk *clk;
	int index;
	int ret;

	/* allocate the mux */
	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return ERR_PTR(-ENOMEM);

	/* U-boot specific assignments */
	mux->parent_names = parent_names;
	mux->num_parents = num_parents;

	/* struct clk_mtk_mux assignments */
	mux->reg = reg;
	mux->mux_shift = mux_shift;
	mux->mux_width = mux_width;
	mux->gate_bit = gate_bit;
	mux->upd_reg = upd_reg;
	mux->upd_bit = upd_bit;
	mux->mtk_flags = mtk_flags;

	clk = &mux->clk;
	clk->flags = flags;

	index = mtk_clk_mux_get_parent(clk);
	if (index < 0)
		index = 0;
	ret = clk_register(clk,
		(gate_bit < 32) ? UBOOT_DM_CLK_CCF_MTK_MUX_GATE :
				UBOOT_DM_CLK_CCF_MTK_MUX,
		name, parent_names[index]);
	if (ret) {
		kfree(mux);
		return ERR_PTR(ret);
	}

	return clk;
}

static const struct clk_ops mtk_mux_ops = {
	.get_rate = clk_generic_get_rate,
	.set_parent = mtk_clk_mux_set_parent,
};

U_BOOT_DRIVER(mtk_clk_mux) = {
	.name	= UBOOT_DM_CLK_CCF_MTK_MUX,
	.id	= UCLASS_CLK,
	.ops	= &mtk_mux_ops,
	.flags = DM_FLAG_PRE_RELOC,
};

static const struct clk_ops mtk_mux_gate_ops  = {
	.get_rate = clk_generic_get_rate,
	.enable = mtk_clk_mux_enable,
	.disable = mtk_clk_mux_disable,
	.set_parent = mtk_clk_mux_set_parent,
};

U_BOOT_DRIVER(mtk_clk_mux_gate) = {
	.name	= UBOOT_DM_CLK_CCF_MTK_MUX_GATE,
	.id	= UCLASS_CLK,
	.ops	= &mtk_mux_gate_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
