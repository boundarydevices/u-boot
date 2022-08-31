// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Chris-qj Chen <chris-qj.chen@mediatek.com>
 */

#include <common.h>
#include <dm.h>
#include <asm/io.h>
#include <dt-bindings/clock/mt8195-clk.h>
#include <linux/bitops.h>

#include "clk-mtk.h"

static const struct mtk_gate_regs imp_iic_wrap_cg_regs = {
	.set_ofs = 0xe08,
	.clr_ofs = 0xe04,
	.sta_ofs = 0xe00,
};

#define GATE_IMP_IIC_WRAP(_id, _parent, _shift)				\
	GATE_MTK_FLAGS(_id, _parent, &imp_iic_wrap_cg_regs, _shift,	\
		CLK_PARENT_TOPCKGEN | CLK_GATE_SETCLR)

static const struct mtk_gate imp_iic_wrap_s_clks[] = {
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_S_I2C5, CLK_TOP_I2C, 0),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_S_I2C6, CLK_TOP_I2C, 1),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_S_I2C7, CLK_TOP_I2C, 2),
};

static const struct mtk_gate imp_iic_wrap_w_clks[] = {
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_W_I2C0, CLK_TOP_I2C, 0),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_W_I2C1, CLK_TOP_I2C, 1),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_W_I2C2, CLK_TOP_I2C, 2),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_W_I2C3, CLK_TOP_I2C, 3),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_W_I2C4, CLK_TOP_I2C, 4),
};

extern const struct mtk_clk_tree mt8195_clk_tree;
static int mt8195_imp_iic_wrap_probe(struct udevice *dev)
{
	return mtk_common_clk_gate_init(dev, &mt8195_clk_tree, dev->driver_data);
}

static const struct udevice_id of_match_clk_mt8195_imp_iic_wrap[] = {
	{
		.compatible = "mediatek,mt8195-imp_iic_wrap_s",
		.data = &imp_iic_wrap_s_clks,
	}, {
		.compatible = "mediatek,mt8195-imp_iic_wrap_w",
		.data = &imp_iic_wrap_w_clks,
	}, {
		/* sentinel */
	}
};

U_BOOT_DRIVER(mtk_clk_imp_iic_wrap) = {
	.name = "mt8195-imp_iic_wrap",
	.id = UCLASS_CLK,
	.of_match = of_match_clk_mt8195_imp_iic_wrap,
	.probe = mt8195_imp_iic_wrap_probe,
	.priv_auto = sizeof(struct mtk_clk_priv),
	.ops = &mtk_clk_gate_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
