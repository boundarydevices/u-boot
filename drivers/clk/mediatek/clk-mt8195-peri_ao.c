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

static const struct mtk_gate_regs peri_ao_cg_regs = {
	.set_ofs = 0x10,
	.clr_ofs = 0x14,
	.sta_ofs = 0x18,
};

#define GATE_PERI_AO(_id, _parent, _shift)			\
	GATE_MTK_FLAGS(_id, _parent, &peri_ao_cg_regs, _shift,	\
		       CLK_PARENT_TOPCKGEN | CLK_GATE_SETCLR)

static const struct mtk_gate peri_ao_clks[] = {
	GATE_PERI_AO(CLK_PERI_AO_ETHERNET, CLK_TOP_AXI, 0),
	GATE_PERI_AO(CLK_PERI_AO_ETHERNET_BUS, CLK_TOP_AXI, 1),
	GATE_PERI_AO(CLK_PERI_AO_FLASHIF_BUS, CLK_TOP_AXI, 3),
	GATE_PERI_AO(CLK_PERI_AO_FLASHIF_FLASH, CLK_TOP_SPINOR, 5),
	GATE_PERI_AO(CLK_PERI_AO_SSUSB_1P_BUS, CLK_TOP_USB_TOP_1P, 7),
	GATE_PERI_AO(CLK_PERI_AO_SSUSB_1P_XHCI, CLK_TOP_SSUSB_XHCI_1P, 8),
	GATE_PERI_AO(CLK_PERI_AO_SSUSB_2P_BUS, CLK_TOP_USB_TOP_2P, 9),
	GATE_PERI_AO(CLK_PERI_AO_SSUSB_2P_XHCI, CLK_TOP_SSUSB_XHCI_2P, 10),
	GATE_PERI_AO(CLK_PERI_AO_SSUSB_3P_BUS, CLK_TOP_USB_TOP_3P, 11),
	GATE_PERI_AO(CLK_PERI_AO_SSUSB_3P_XHCI, CLK_TOP_SSUSB_XHCI_3P, 12),
	GATE_PERI_AO(CLK_PERI_AO_SPINFI, CLK_TOP_SPINFI_BCLK, 15),
	GATE_PERI_AO(CLK_PERI_AO_ETHERNET_MAC, CLK_TOP_SNPS_ETH_250M, 16),
	GATE_PERI_AO(CLK_PERI_AO_NFI_H, CLK_TOP_AXI, 19),
	GATE_PERI_AO(CLK_PERI_AO_FNFI1X, CLK_TOP_NFI1X, 20),
	GATE_PERI_AO(CLK_PERI_AO_PCIE_P0_MEM, CLK_TOP_MEM_466M, 24),
	GATE_PERI_AO(CLK_PERI_AO_PCIE_P1_MEM, CLK_TOP_MEM_466M, 25),
};

extern const struct mtk_clk_tree mt8195_clk_tree;
static int mt8195_peri_ao_probe(struct udevice *dev)
{
	return mtk_common_clk_gate_init(dev, &mt8195_clk_tree, peri_ao_clks);
}

static const struct udevice_id of_match_clk_mt8195_peri_ao[] = {
	{ .compatible = "mediatek,mt8195-pericfg_ao", },
	{ }
};

U_BOOT_DRIVER(mtk_clk_peri_ao) = {
	.name = "mt8195-peri_ao",
	.id = UCLASS_CLK,
	.of_match = of_match_clk_mt8195_peri_ao,
	.probe = mt8195_peri_ao_probe,
	.priv_auto = sizeof(struct mtk_clk_priv),
	.ops = &mtk_clk_gate_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
