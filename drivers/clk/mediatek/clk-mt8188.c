// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (c) 2022 MediaTek Inc.
// Author: Garmin Chang <garmin.chang@mediatek.com>

#include <common.h>
#include <dm.h>
#include <dt-bindings/clock/mediatek,mt8188-clk.h>
#include <linux/clk-provider.h>
#include "clk-mtk-div-gate.h"
#include "clk-mtk-gate.h"
#include "clk-mtk-mux.h"
#include "clk-mtk-pll.h"

#define CLK_TOP_BASE	0
#define CLK_INFRA_AO_BASE	(CLK_TOP_BASE + CLK_TOP_NR_CLK)
#define CLK_APMIXED_BASE	(CLK_INFRA_AO_BASE + CLK_INFRA_AO_NR_CLK)
#if 0
#define CLK_AUDIODSP_BASE	(CLK_APMIXED_BASE + CLK_APMIXED_NR_CLK)
#define CLK_PERI_AO_BASE	(CLK_AUDIODSP_BASE + CLK_AUDIODSP_NR_CLK)
#define CLK_IMP_IIC_WRAP_C_BASE	(CLK_PERI_AO_BASE + CLK_PERI_AO_NR_CLK)
#define CLK_IMP_IIC_WRAP_W_BASE	(CLK_IMP_IIC_WRAP_C_BASE + CLK_IMP_IIC_WRAP_C_NR_CLK)
#define CLK_IMP_IIC_WRAP_EN_BASE (CLK_IMP_IIC_WRAP_W_BASE + CLK_IMP_IIC_WRAP_W_NR_CLK)
#define CLK_MFGCFG_BASE		(CLK_IMP_IIC_WRAP_EN_BASE + CLK_IMP_IIC_WRAP_EN_NR_CLK)
#define CLK_VPP0_BASE		(CLK_MFGCFG_BASE + CLK_MFGCFG_NR_CLK)
#define CLK_WPE_TOP_BASE	(CLK_VPP0_BASE + CLK_VPP0_NR_CLK)
#define CLK_WPE_VPP0_BASE	(CLK_WPE_TOP_BASE + CLK_WPE_TOP_NR_CLK)
#define CLK_VPP1_BASE		(CLK_WPE_VPP0_BASE + CLK_WPE_VPP0_NR_CLK)
#define CLK_IMGSYS_MAIN_BASE	(CLK_VPP1_BASE + CLK_VPP1_NR_CLK)
#define CLK_IMGSYS1_DIP_TOP_BASE (CLK_IMGSYS_MAIN_BASE + CLK_IMGSYS_MAIN_NR_CLK)
#define CLK_IMGSYS1_DIP_NR_BASE	(CLK_IMGSYS1_DIP_TOP_BASE + CLK_IMGSYS1_DIP_TOP_NR_CLK)
#define CLK_IMGSYS_WPE1_BASE	(CLK_IMGSYS1_DIP_NR_BASE + CLK_IMGSYS1_DIP_NR_NR_CLK)
#define CLK_IPE_BASE		(CLK_IMGSYS_WPE1_BASE + CLK_IMGSYS_WPE1_NR_CLK)
#define CLK_IMGSYS_WPE2_BASE	(CLK_IPE_BASE + CLK_IPE_NR_CLK)
#define CLK_IMGSYS_WPE3_BASE	(CLK_IMGSYS_WPE2_BASE + CLK_IMGSYS_WPE2_NR_CLK)
#define CLK_CAM_MAIN_BASE	(CLK_IMGSYS_WPE3_BASE + CLK_IMGSYS_WPE3_NR_CLK)
#define CLK_CAM_RAWA_BASE	(CLK_CAM_MAIN_BASE + CLK_CAM_MAIN_NR_CLK)
#define CLK_CAM_YUVA_BASE	(CLK_CAM_RAWA_BASE + CLK_CAM_RAWA_NR_CLK)
#define CLK_CAM_RAWB_BASE	(CLK_CAM_YUVA_BASE + CLK_CAM_YUVA_NR_CLK)
#define CLK_CAM_YUVB_BASE	(CLK_CAM_RAWB_BASE + CLK_CAM_RAWB_NR_CLK)
#define CLK_CCU_NR_BASE		(CLK_CAM_YUVB_BASE + CLK_CAM_YUVB_NR_CLK)
#define CLK_VDE1_BASE		(CLK_CCU_NR_BASE + CLK_CCU_NR_CLK)
#define CLK_VDE2_BASE		(CLK_VDE1_BASE + CLK_VDE1_NR_CLK)
#define CLK_VEN1_BASE		(CLK_VDE2_BASE + CLK_VDE2_NR_CLK)
#define CLK_VDO0_BASE		(CLK_VEN1_BASE + CLK_VEN1_NR_CLK)
#define CLK_VDO1_BASE		(CLK_VDO0_BASE + CLK_VDO0_NR_CLK)
#define CLK_TOTAL		(CLK_VDO1_BASE + CLK_VDO1_NR_CLK)
#else
#define CLK_TOTAL		(CLK_APMIXED_BASE + CLK_APMIXED_NR_CLK)
#endif

static const u16 mt8188_bases[] = {CLK_INFRA_AO_BASE, CLK_APMIXED_BASE, CLK_TOTAL};

enum {
	TYPE_mt_clk_external = 1,
	TYPE_mt_clk_factor,
	TYPE_mt_clk_mux_gate,
	TYPE_mt_clk_mux,
	TYPE_mt_clk_div_gate,
	TYPE_mt_clk_gate,
	TYPE_mt_clk_pll,
};

struct mt8188_factor {
	u8 mult;
	u8 div;
};

struct mt8188_mux_gate {
	u16 mux_ofs;	/* mux_set_ofs is +4, mux_clr_ofs is +8 */
	u8 shift;
	u8 width;
	u8 gate_bit;
	u8 upd_ofs;
	u8 upd_bit;
};

struct mt8188_mux {
	u16 mux_ofs;
	u8 shift;
	u8 width;
};

struct mt8188_div_gate {
	u16 gate_ofs;
	u16 div_ofs;
	u8 gate_bit;
	u8 div_shift;
	u8 div_width;
};

struct mt8188_gate {
	u16 gate_ofs;
	u8 gate_bit;
	u8 mtk_flags;
};

struct mt8188_pll {
	u16 reg_ofs;
	u16 tuner_ofs;
	u8 tuner_en_bit;
	u8 pll_en_bit;
	u8 mtk_flags;
};

struct mt8188_clk_probe {
	u8 type;
	u8 num_parents;
	u8 *name;
	unsigned flags;
	union {
		const u16 parent;
		const u16 *parents;
	};
	union {
		struct mt8188_factor f;
		struct mt8188_mux_gate mg;
		struct mt8188_mux m;
		struct mt8188_div_gate dg;
		struct mt8188_gate g;
		struct mt8188_pll pll;
	};
};

struct mt8188_clk_priv {
	unsigned base_id;
	unsigned dev_id;
	void __iomem *base;
	ofnode np;
	const struct mt8188_clk_probe *cps;
	unsigned num_clks;
	struct clk **clks;
};

enum {
	dev_top,
	dev_infracfg_ao,
	dev_apmixedsys,
	dev_end
};
struct mt8188_clk_priv *mt8188_clk_priv_devices[dev_end];

static const short axi_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D4_D4,
	CLK_TOP_MAINPLL_D7_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_MAINPLL_D6_D2,
	CLK_TOP_ULPOSC1_D4
};

static const short spm_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_ULPOSC1_D10,
	CLK_TOP_MAINPLL_D7_D4,
	CLK_TOP_CLK32K
};

static const short scp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_UNIVPLL_D3,
	CLK_TOP_MAINPLL_D3
};

static const short bus_aximem_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D7_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_MAINPLL_D6
};

static const short vpp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_MMPLL_D6_D2,
	CLK_TOP_UNIVPLL_D5_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MMPLL_D4_D2,
	CLK_TOP_MMPLL_D7,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_MMPLL_D5,
	CLK_TOP_TVDPLL1,
	CLK_TOP_TVDPLL2,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MMPLL_D4
};

static const short ethdr_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_MMPLL_D6_D2,
	CLK_TOP_UNIVPLL_D5_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MMPLL_D4_D2,
	CLK_TOP_MMPLL_D7,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_MMPLL_D5_D4,
	CLK_TOP_TVDPLL1,
	CLK_TOP_TVDPLL2,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MMPLL_D4
};

static const short ipe_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_IMGPLL,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_MMPLL_D6,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_MMPLL_D4_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_MMPLL_D6_D2,
	CLK_TOP_UNIVPLL_D5_D2,
	CLK_TOP_MAINPLL_D7
};

static const short cam_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_TVDPLL1,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_MMPLL_D4,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MMPLL_D7,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_IMGPLL
};

static const short ccu_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_MMPLL_D6,
	CLK_TOP_MMPLL_D7,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_UNIVPLL_D7
};

static const short ccu_ahb_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_MMPLL_D6,
	CLK_TOP_MMPLL_D7,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_UNIVPLL_D7
};

static const short img_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_IMGPLL,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_MMPLL_D6,
	CLK_TOP_MMPLL_D7,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_MMPLL_D4_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_UNIVPLL_D5_D2
};

static const short camtm_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D4_D4,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_UNIVPLL_D6_D4
};

static const short dsp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MMPLL_D4,
	CLK_TOP_MAINPLL_D3,
	CLK_TOP_UNIVPLL_D3
};

static const short dsp1_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_MMPLL_D5,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MAINPLL_D3,
	CLK_TOP_UNIVPLL_D3
};

static const short dsp2_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_MMPLL_D5,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MAINPLL_D3,
	CLK_TOP_UNIVPLL_D3
};

static const short dsp3_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_MMPLL_D5,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MAINPLL_D3,
	CLK_TOP_UNIVPLL_D3
};

static const short dsp4_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MMPLL_D4,
	CLK_TOP_MAINPLL_D3,
	CLK_TOP_UNIVPLL_D3
};

static const short dsp5_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MMPLL_D4,
	CLK_TOP_MAINPLL_D3,
	CLK_TOP_UNIVPLL_D3
};

static const short dsp6_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MMPLL_D4,
	CLK_TOP_MAINPLL_D3,
	CLK_TOP_UNIVPLL_D3
};

static const short dsp7_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_MMPLL_D4,
	CLK_TOP_MAINPLL_D3,
	CLK_TOP_UNIVPLL_D3
};

static const short mfg_core_tmp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_UNIVPLL_D7
};

static const short camtg_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_192M_D8,
	CLK_TOP_UNIVPLL_D6_D8,
	CLK_TOP_UNIVPLL_192M_D4,
	CLK_TOP_UNIVPLL_192M_D10,
	CLK_TOP_CLK13M,
	CLK_TOP_UNIVPLL_192M_D16,
	CLK_TOP_UNIVPLL_192M_D32
};

static const short camtg2_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_192M_D8,
	CLK_TOP_UNIVPLL_D6_D8,
	CLK_TOP_UNIVPLL_192M_D4,
	CLK_TOP_UNIVPLL_192M_D10,
	CLK_TOP_CLK13M,
	CLK_TOP_UNIVPLL_192M_D16,
	CLK_TOP_UNIVPLL_192M_D32
};

static const short camtg3_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_192M_D8,
	CLK_TOP_UNIVPLL_D6_D8,
	CLK_TOP_UNIVPLL_192M_D4,
	CLK_TOP_UNIVPLL_192M_D10,
	CLK_TOP_CLK13M,
	CLK_TOP_UNIVPLL_192M_D16,
	CLK_TOP_UNIVPLL_192M_D32
};

static const short uart_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D8
};

static const short spi_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D5_D4,
	CLK_TOP_MAINPLL_D6_D4,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D6_D2,
	CLK_TOP_MAINPLL_D4_D4,
	CLK_TOP_UNIVPLL_D5_D4
};

static const short msdc5hclk_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_MAINPLL_D6_D2
};

static const short msdc50_0_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MSDCPLL,
	CLK_TOP_MSDCPLL_D2,
	CLK_TOP_UNIVPLL_D4_D4,
	CLK_TOP_MAINPLL_D6_D2,
	CLK_TOP_UNIVPLL_D4_D2
};

static const short msdc30_1_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D6_D2,
	CLK_TOP_MAINPLL_D7_D2,
	CLK_TOP_MSDCPLL_D2
};

static const short msdc30_2_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D6_D2,
	CLK_TOP_MAINPLL_D7_D2,
	CLK_TOP_MSDCPLL_D2
};

static const short intdir_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_UNIVPLL_D4
};

static const short aud_intbus_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D4_D4,
	CLK_TOP_MAINPLL_D7_D4
};

static const short audio_h_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D7,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL1,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL2
};

static const short pwrap_ulposc_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_ULPOSC1_D10,
	CLK_TOP_ULPOSC1_D7,
	CLK_TOP_ULPOSC1_D8,
	CLK_TOP_ULPOSC1_D16,
	CLK_TOP_MAINPLL_D4_D8,
	CLK_TOP_UNIVPLL_D5_D8,
	CLK_TOP_TVDPLL1_D16
};

static const short atb_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_MAINPLL_D5_D2
};

static const short sspm_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D7_D2,
	CLK_TOP_MAINPLL_D6_D2,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_MAINPLL_D9,
	CLK_TOP_MAINPLL_D4_D2
};

static const short dp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_TVDPLL1_D2,
	CLK_TOP_TVDPLL2_D2,
	CLK_TOP_TVDPLL1_D4,
	CLK_TOP_TVDPLL2_D4,
	CLK_TOP_TVDPLL1_D8,
	CLK_TOP_TVDPLL2_D8,
	CLK_TOP_TVDPLL1_D16,
	CLK_TOP_TVDPLL2_D16
};

static const short edp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_TVDPLL1_D2,
	CLK_TOP_TVDPLL2_D2,
	CLK_TOP_TVDPLL1_D4,
	CLK_TOP_TVDPLL2_D4,
	CLK_TOP_TVDPLL1_D8,
	CLK_TOP_TVDPLL2_D8,
	CLK_TOP_TVDPLL1_D16,
	CLK_TOP_TVDPLL2_D16
};

static const short dpi_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_TVDPLL1_D2,
	CLK_TOP_TVDPLL2_D2,
	CLK_TOP_TVDPLL1_D4,
	CLK_TOP_TVDPLL2_D4,
	CLK_TOP_TVDPLL1_D8,
	CLK_TOP_TVDPLL2_D8,
	CLK_TOP_TVDPLL1_D16,
	CLK_TOP_TVDPLL2_D16
};

static const short disp_pwm0_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_ULPOSC1_D2,
	CLK_TOP_ULPOSC1_D4,
	CLK_TOP_ULPOSC1_D16,
	CLK_TOP_ETHPLL_D4
};

static const short disp_pwm1_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_ULPOSC1_D2,
	CLK_TOP_ULPOSC1_D4,
	CLK_TOP_ULPOSC1_D16
};

static const short usb_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D5_D4,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_UNIVPLL_D5_D2
};

static const short ssusb_xhci_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D5_D4,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_UNIVPLL_D5_D2
};

static const short usb_2p_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D5_D4,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_UNIVPLL_D5_D2
};

static const short ssusb_xhci_2p_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D5_D4,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_UNIVPLL_D5_D2
};

static const short usb_3p_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D5_D4,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_UNIVPLL_D5_D2
};

static const short ssusb_xhci_3p_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D5_D4,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_UNIVPLL_D5_D2
};

static const short i2c_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D4_D8,
	CLK_TOP_UNIVPLL_D5_D4
};

static const short seninf_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D4_D4,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_UNIVPLL_D7,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MMPLL_D6,
	CLK_TOP_UNIVPLL_D5
};

static const short seninf1_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D4_D4,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_UNIVPLL_D7,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MMPLL_D6,
	CLK_TOP_UNIVPLL_D5
};

static const short gcpu_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MMPLL_D5_D2,
	CLK_TOP_UNIVPLL_D5_D2
};

static const short venc_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MMPLL_D4_D2,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MMPLL_D6,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_MAINPLL_D6_D2,
	CLK_TOP_MMPLL_D9,
	CLK_TOP_UNIVPLL_D4_D4,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_UNIVPLL_D5_D2,
	CLK_TOP_MAINPLL_D5
};

static const short vdec_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_MMPLL_D6_D2,
	CLK_TOP_UNIVPLL_D5_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MMPLL_D4_D2,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MAINPLL_D5,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_MMPLL_D6,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_TVDPLL2,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_IMGPLL,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MMPLL_D9
};

static const short pwm_parents[] = {
	CLK_TOP_CLK32K,
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D4_D8,
	CLK_TOP_UNIVPLL_D6_D4
};

static const short mcupm_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D6_D2,
	CLK_TOP_MAINPLL_D7_D4
};

static const short spmi_p_mst_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_CLK13M,
	CLK_TOP_ULPOSC1_D8,
	CLK_TOP_ULPOSC1_D10,
	CLK_TOP_ULPOSC1_D16,
	CLK_TOP_ULPOSC1_D7,
	CLK_TOP_CLK32K,
	CLK_TOP_MAINPLL_D7_D8,
	CLK_TOP_MAINPLL_D6_D8,
	CLK_TOP_MAINPLL_D5_D8
};

static const short spmi_m_mst_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_CLK13M,
	CLK_TOP_ULPOSC1_D8,
	CLK_TOP_ULPOSC1_D10,
	CLK_TOP_ULPOSC1_D16,
	CLK_TOP_ULPOSC1_D7,
	CLK_TOP_CLK32K,
	CLK_TOP_MAINPLL_D7_D8,
	CLK_TOP_MAINPLL_D6_D8,
	CLK_TOP_MAINPLL_D5_D8
};

static const short dvfsrc_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_ULPOSC1_D10,
	CLK_TOP_UNIVPLL_D6_D8,
	CLK_TOP_MSDCPLL_D16
};

static const short tl_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D5_D4,
	CLK_TOP_MAINPLL_D4_D4
};

static const short aes_msdcfde_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_MAINPLL_D4_D4,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_UNIVPLL_D6
};

static const short dsi_occ_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_UNIVPLL_D5_D2,
	CLK_TOP_UNIVPLL_D4_D2
};

static const short wpe_vpp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_MMPLL_D6_D2,
	CLK_TOP_UNIVPLL_D5_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_UNIVPLL_D4_D2,
	CLK_TOP_MMPLL_D4_D2,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_MMPLL_D7,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_MAINPLL_D5,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_TVDPLL1,
	CLK_TOP_UNIVPLL_D4
};

static const short hdcp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D4_D8,
	CLK_TOP_MAINPLL_D5_D8,
	CLK_TOP_UNIVPLL_D6_D4
};

static const short hdcp_24m_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_192M_D4,
	CLK_TOP_UNIVPLL_192M_D8,
	CLK_TOP_UNIVPLL_D6_D8
};

static const short hdmi_apb_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_MSDCPLL_D2
};

static const short snps_eth_250m_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_ETHPLL_D2
};

static const short snps_eth_62p4m_ptp_parents[] = {
	CLK_TOP_APLL2_D3,
	CLK_TOP_APLL1_D3,
	CLK_TOP_CLK26M,
	CLK_TOP_ETHPLL_D8
};

static const short snps_eth_50m_rmii_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_ETHPLL_D10
};

static const short adsp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_CLK13M,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_UNIVPLL_D4_D4,
	CLK_TOP_UNIVPLL_D4,
	CLK_TOP_ULPOSC1_D2,
	CLK_TOP_ULPOSC1,
	CLK_TOP_ADSPPLL,
	CLK_TOP_ADSPPLL_D2,
	CLK_TOP_ADSPPLL_D4,
	CLK_TOP_ADSPPLL_D8
};

static const short audio_local_bus_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_CLK13M,
	CLK_TOP_MAINPLL_D4_D4,
	CLK_TOP_MAINPLL_D7_D2,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_MAINPLL_D7,
	CLK_TOP_MAINPLL_D4,
	CLK_TOP_UNIVPLL_D6,
	CLK_TOP_ULPOSC1,
	CLK_TOP_ULPOSC1_D4,
	CLK_TOP_ULPOSC1_D2
};

static const short asm_h_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D5_D2
};

static const short asm_l_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D6_D4,
	CLK_TOP_UNIVPLL_D6_D2,
	CLK_TOP_MAINPLL_D5_D2
};

static const short apll1_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL1_D4
};

static const short apll2_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL2_D4
};

static const short apll3_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL3_D4
};

static const short apll4_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL4_D4
};

static const short apll5_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL5_D4
};

static const short i2so1_parents[] = {
	CLK_TOP_CLK26M,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL1,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL2,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL3,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL4,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL5
};

static const short i2so2_parents[] = {
	CLK_TOP_CLK26M,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL1,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL2,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL3,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL4,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL5
};

static const short i2si1_parents[] = {
	CLK_TOP_CLK26M,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL1,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL2,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL3,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL4,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL5
};

static const short i2si2_parents[] = {
	CLK_TOP_CLK26M,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL1,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL2,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL3,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL4,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL5
};

static const short dptx_parents[] = {
	CLK_TOP_CLK26M,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL1,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL2,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL3,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL4,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL5
};

static const short aud_iec_parents[] = {
	CLK_TOP_CLK26M,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL1,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL2,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL3,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL4,
	CLK_APMIXED_BASE + CLK_APMIXED_APLL5
};

static const short a1sys_hp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL1_D4
};

static const short a2sys_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL2_D4
};

static const short a3sys_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL3_D4,
	CLK_TOP_APLL4_D4,
	CLK_TOP_APLL5_D4
};

static const short a4sys_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL3_D4,
	CLK_TOP_APLL4_D4,
	CLK_TOP_APLL5_D4
};

static const short ecc_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MAINPLL_D4_D4,
	CLK_TOP_MAINPLL_D5_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_MAINPLL_D6,
	CLK_TOP_UNIVPLL_D6
};

static const short spinor_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_CLK13M,
	CLK_TOP_MAINPLL_D7_D8,
	CLK_TOP_UNIVPLL_D6_D8
};

static const short ulposc_parents[] = {
	CLK_TOP_ULPOSC1,
	CLK_TOP_ETHPLL_D2,
	CLK_TOP_MAINPLL_D4_D2,
	CLK_TOP_ETHPLL_D10
};

static const short srck_parents[] = {
	CLK_TOP_ULPOSC1_D10,
	CLK_TOP_CLK26M
};

static const short mfg_fast_ref_parents[] = {
	CLK_TOP_MFG_CORE_TMP,
	CLK_TOP_MFGPLL
};

#define MTK_EXTERNAL(_name) \
	.type = TYPE_mt_clk_external,	\
	.name = _name,			\
	.parent = 0xffff

#define MTK_FACTOR(_name, _parent, _mult, _div) \
	.type = TYPE_mt_clk_factor,		\
	.name = _name,				\
	.parent = _parent,			\
	.f = {.mult = _mult, .div = _div}

#define MTK_MUX_GATE(_name, _p, _mux_ofs, s, w, g, _upd_ofs, _upd_bit) \
	.type = TYPE_mt_clk_mux_gate,		\
	.name = _name,				\
	.parents = _p, .num_parents = ARRAY_SIZE(_p), \
	.mg = {.mux_ofs = _mux_ofs, .shift = s, .width = w, g, \
		.upd_ofs = _upd_ofs, .upd_bit = _upd_bit}

#define MTK_MUX(_name, _p, _mux_ofs, _s, _w) \
	.type = TYPE_mt_clk_mux,		\
	.name = _name,				\
	.parents = _p, .num_parents = ARRAY_SIZE(_p), \
	.mg = {.mux_ofs = _mux_ofs, .shift = _s, .width = _w}

#define MTK_DIV_GATE(_name, _parent, _gate_ofs, _gate_bit, _div_ofs, _div_width, _div_shift) \
	.type = TYPE_mt_clk_div_gate,		\
	.name = _name,				\
	.parent = _parent,			\
	.dg = {.gate_ofs = _gate_ofs, .gate_bit = _gate_bit, \
		.div_ofs = _div_ofs, .div_width = _div_width, .div_shift = _div_shift}

#define MTK_GATE(_name, _parent, _gate_ofs, _gate_bit, _mtk_flags) \
	.type = TYPE_mt_clk_gate,		\
	.name = _name,				\
	.parent = _parent,			\
	.g = {.gate_ofs = _gate_ofs, .gate_bit = _gate_bit, \
		.mtk_flags = _mtk_flags}

static const struct mt8188_clk_probe mt8188_top_clk_init[] = {
[CLK_TOP_ULPOSC1] = { MTK_EXTERNAL("ulposc_ck1")},
[CLK_TOP_MPHONE_SLAVE_BCK] = { MTK_EXTERNAL("mphone_slave_bck")},
[CLK_TOP_PAD_FPC] = { MTK_EXTERNAL("pad_fpc_ck")},
[CLK_TOP_466M_FMEM] = { MTK_EXTERNAL("hd_466m_fmem_ck")},
[CLK_TOP_PEXTP_PIPE] = { MTK_EXTERNAL("pextp_pipe")},
[CLK_TOP_DSI_PHY] = { MTK_EXTERNAL("dsi_phy")},
[CLK_TOP_CLK13M] = { MTK_EXTERNAL("clk13m")},
[CLK_TOP_CLK26M] = { MTK_EXTERNAL("clk26m")},
[CLK_TOP_CLK32K] = { MTK_EXTERNAL("clk32k")},

[CLK_TOP_MFGPLL] = { MTK_FACTOR("mfgpll_ck", CLK_APMIXED_BASE + CLK_APMIXED_MFGPLL, 1, 1)},
[CLK_TOP_MAINPLL] = { MTK_FACTOR("mainpll_ck", CLK_APMIXED_BASE + CLK_APMIXED_MAINPLL, 1, 1)},
[CLK_TOP_MAINPLL_D3] = { MTK_FACTOR("mainpll_d3", CLK_TOP_MAINPLL, 1, 3)},
[CLK_TOP_MAINPLL_D4] = { MTK_FACTOR("mainpll_d4", CLK_TOP_MAINPLL, 1, 4)},
[CLK_TOP_MAINPLL_D4_D2] = { MTK_FACTOR("mainpll_d4_d2", CLK_TOP_MAINPLL_D4, 1, 2)},
[CLK_TOP_MAINPLL_D4_D4] = { MTK_FACTOR("mainpll_d4_d4", CLK_TOP_MAINPLL_D4, 1, 4)},
[CLK_TOP_MAINPLL_D4_D8] = { MTK_FACTOR("mainpll_d4_d8", CLK_TOP_MAINPLL_D4, 1, 8)},
[CLK_TOP_MAINPLL_D5] = { MTK_FACTOR("mainpll_d5", CLK_TOP_MAINPLL, 1, 5)},
[CLK_TOP_MAINPLL_D5_D2] = { MTK_FACTOR("mainpll_d5_d2", CLK_TOP_MAINPLL_D5, 1, 2)},
[CLK_TOP_MAINPLL_D5_D4] = { MTK_FACTOR("mainpll_d5_d4", CLK_TOP_MAINPLL_D5, 1, 4)},
[CLK_TOP_MAINPLL_D5_D8] = { MTK_FACTOR("mainpll_d5_d8", CLK_TOP_MAINPLL_D5, 1, 8)},
[CLK_TOP_MAINPLL_D6] = { MTK_FACTOR("mainpll_d6", CLK_TOP_MAINPLL, 1, 6)},
[CLK_TOP_MAINPLL_D6_D2] = { MTK_FACTOR("mainpll_d6_d2", CLK_TOP_MAINPLL_D6, 1, 2)},
[CLK_TOP_MAINPLL_D6_D4] = { MTK_FACTOR("mainpll_d6_d4", CLK_TOP_MAINPLL_D6, 1, 4)},
[CLK_TOP_MAINPLL_D6_D8] = { MTK_FACTOR("mainpll_d6_d8", CLK_TOP_MAINPLL_D6, 1, 8)},
[CLK_TOP_MAINPLL_D7] = { MTK_FACTOR("mainpll_d7", CLK_TOP_MAINPLL, 1, 7)},
[CLK_TOP_MAINPLL_D7_D2] = { MTK_FACTOR("mainpll_d7_d2", CLK_TOP_MAINPLL_D7, 1, 2)},
[CLK_TOP_MAINPLL_D7_D4] = { MTK_FACTOR("mainpll_d7_d4", CLK_TOP_MAINPLL_D7, 1, 4)},
[CLK_TOP_MAINPLL_D7_D8] = { MTK_FACTOR("mainpll_d7_d8", CLK_TOP_MAINPLL_D7, 1, 8)},
[CLK_TOP_MAINPLL_D9] = { MTK_FACTOR("mainpll_d9", CLK_TOP_MAINPLL, 1, 9)},
[CLK_TOP_UNIVPLL] = { MTK_FACTOR("univpll_ck", CLK_APMIXED_BASE + CLK_APMIXED_UNIVPLL, 1, 1)},
[CLK_TOP_UNIVPLL_D2] = { MTK_FACTOR("univpll_d2", CLK_TOP_UNIVPLL, 1, 2)},
[CLK_TOP_UNIVPLL_D3] = { MTK_FACTOR("univpll_d3", CLK_TOP_UNIVPLL, 1, 3)},
[CLK_TOP_UNIVPLL_D4] = { MTK_FACTOR("univpll_d4", CLK_TOP_UNIVPLL, 1, 4)},
[CLK_TOP_UNIVPLL_D4_D2] = { MTK_FACTOR("univpll_d4_d2", CLK_TOP_UNIVPLL_D4, 1, 2)},
[CLK_TOP_UNIVPLL_D4_D4] = { MTK_FACTOR("univpll_d4_d4", CLK_TOP_UNIVPLL_D4, 1, 4)},
[CLK_TOP_UNIVPLL_D4_D8] = { MTK_FACTOR("univpll_d4_d8", CLK_TOP_UNIVPLL_D4, 1, 8)},
[CLK_TOP_UNIVPLL_D5] = { MTK_FACTOR("univpll_d5", CLK_TOP_UNIVPLL, 1, 5)},
[CLK_TOP_UNIVPLL_D5_D2] = { MTK_FACTOR("univpll_d5_d2", CLK_TOP_UNIVPLL_D5, 1, 2)},
[CLK_TOP_UNIVPLL_D5_D4] = { MTK_FACTOR("univpll_d5_d4", CLK_TOP_UNIVPLL_D5, 1, 4)},
[CLK_TOP_UNIVPLL_D5_D8] = { MTK_FACTOR("univpll_d5_d8", CLK_TOP_UNIVPLL_D5, 1, 8)},
[CLK_TOP_UNIVPLL_D6] = { MTK_FACTOR("univpll_d6", CLK_TOP_UNIVPLL, 1, 6)},
[CLK_TOP_UNIVPLL_D6_D2] = { MTK_FACTOR("univpll_d6_d2", CLK_TOP_UNIVPLL_D6, 1, 2)},
[CLK_TOP_UNIVPLL_D6_D4] = { MTK_FACTOR("univpll_d6_d4", CLK_TOP_UNIVPLL_D6, 1, 4)},
[CLK_TOP_UNIVPLL_D6_D8] = { MTK_FACTOR("univpll_d6_d8", CLK_TOP_UNIVPLL_D6, 1, 8)},
[CLK_TOP_UNIVPLL_D7] = { MTK_FACTOR("univpll_d7", CLK_TOP_UNIVPLL, 1, 7)},
[CLK_TOP_UNIVPLL_192M] = { MTK_FACTOR("univpll_192m", CLK_TOP_UNIVPLL, 1, 13)},
[CLK_TOP_UNIVPLL_192M_D4] = { MTK_FACTOR("univpll_192m_d4", CLK_TOP_UNIVPLL_192M, 1, 4)},
[CLK_TOP_UNIVPLL_192M_D8] = { MTK_FACTOR("univpll_192m_d8", CLK_TOP_UNIVPLL_192M, 1, 8)},
[CLK_TOP_UNIVPLL_192M_D10] = { MTK_FACTOR("univpll_192m_d10", CLK_TOP_UNIVPLL_192M, 1, 10)},
[CLK_TOP_UNIVPLL_192M_D16] = { MTK_FACTOR("univpll_192m_d16", CLK_TOP_UNIVPLL_192M, 1, 16)},
[CLK_TOP_UNIVPLL_192M_D32] = { MTK_FACTOR("univpll_192m_d32", CLK_TOP_UNIVPLL_192M, 1, 32)},
[CLK_TOP_IMGPLL] = { MTK_FACTOR("imgpll_ck", CLK_APMIXED_BASE + CLK_APMIXED_IMGPLL, 1, 1)},
[CLK_TOP_APLL1_D3] = { MTK_FACTOR("apll1_d3", CLK_APMIXED_BASE + CLK_APMIXED_APLL1, 1, 3)},
[CLK_TOP_APLL1_D4] = { MTK_FACTOR("apll1_d4", CLK_APMIXED_BASE + CLK_APMIXED_APLL1, 1, 4)},
[CLK_TOP_APLL2_D3] = { MTK_FACTOR("apll2_d3", CLK_APMIXED_BASE + CLK_APMIXED_APLL2, 1, 3)},
[CLK_TOP_APLL2_D4] = { MTK_FACTOR("apll2_d4", CLK_APMIXED_BASE + CLK_APMIXED_APLL2, 1, 4)},
[CLK_TOP_APLL3_D4] = { MTK_FACTOR("apll3_d4", CLK_APMIXED_BASE + CLK_APMIXED_APLL3, 1, 4)},
[CLK_TOP_APLL4_D4] = { MTK_FACTOR("apll4_d4", CLK_APMIXED_BASE + CLK_APMIXED_APLL4, 1, 4)},
[CLK_TOP_APLL5_D4] = { MTK_FACTOR("apll5_d4", CLK_APMIXED_BASE + CLK_APMIXED_APLL5, 1, 4)},
[CLK_TOP_MMPLL] = { MTK_FACTOR("mmpll_ck", CLK_APMIXED_BASE + CLK_APMIXED_MMPLL, 1, 1)},
[CLK_TOP_MMPLL_D4] = { MTK_FACTOR("mmpll_d4", CLK_TOP_MMPLL, 1, 4)},
[CLK_TOP_MMPLL_D4_D2] = { MTK_FACTOR("mmpll_d4_d2", CLK_TOP_MMPLL_D4, 1, 2)},
[CLK_TOP_MMPLL_D5] = { MTK_FACTOR("mmpll_d5", CLK_TOP_MMPLL, 1, 5)},
[CLK_TOP_MMPLL_D5_D2] = { MTK_FACTOR("mmpll_d5_d2", CLK_TOP_MMPLL_D5, 1, 2)},
[CLK_TOP_MMPLL_D5_D4] = { MTK_FACTOR("mmpll_d5_d4", CLK_TOP_MMPLL_D5, 1, 4)},
[CLK_TOP_MMPLL_D6] = { MTK_FACTOR("mmpll_d6", CLK_TOP_MMPLL, 1, 6)},
[CLK_TOP_MMPLL_D6_D2] = { MTK_FACTOR("mmpll_d6_d2", CLK_TOP_MMPLL_D6, 1, 2)},
[CLK_TOP_MMPLL_D7] = { MTK_FACTOR("mmpll_d7", CLK_TOP_MMPLL, 1, 7)},
[CLK_TOP_MMPLL_D9] = { MTK_FACTOR("mmpll_d9", CLK_TOP_MMPLL, 1, 9)},
[CLK_TOP_TVDPLL1] = { MTK_FACTOR("tvdpll1_ck", CLK_APMIXED_BASE + CLK_APMIXED_TVDPLL1, 1, 1)},
[CLK_TOP_TVDPLL1_D2] = { MTK_FACTOR("tvdpll1_d2", CLK_TOP_TVDPLL1, 1, 2)},
[CLK_TOP_TVDPLL1_D4] = { MTK_FACTOR("tvdpll1_d4", CLK_TOP_TVDPLL1, 1, 4)},
[CLK_TOP_TVDPLL1_D8] = { MTK_FACTOR("tvdpll1_d8", CLK_TOP_TVDPLL1, 1, 8)},
[CLK_TOP_TVDPLL1_D16] = { MTK_FACTOR("tvdpll1_d16", CLK_TOP_TVDPLL1, 1, 16)},
[CLK_TOP_TVDPLL2] = { MTK_FACTOR("tvdpll2_ck", CLK_APMIXED_BASE + CLK_APMIXED_TVDPLL2, 1, 1)},
[CLK_TOP_TVDPLL2_D2] = { MTK_FACTOR("tvdpll2_d2", CLK_TOP_TVDPLL2, 1, 2)},
[CLK_TOP_TVDPLL2_D4] = { MTK_FACTOR("tvdpll2_d4", CLK_TOP_TVDPLL2, 1, 4)},
[CLK_TOP_TVDPLL2_D8] = { MTK_FACTOR("tvdpll2_d8", CLK_TOP_TVDPLL2, 1, 8)},
[CLK_TOP_TVDPLL2_D16] = { MTK_FACTOR("tvdpll2_d16", CLK_TOP_TVDPLL2, 1, 16)},
[CLK_TOP_MSDCPLL] = { MTK_FACTOR("msdcpll_ck", CLK_APMIXED_BASE + CLK_APMIXED_MSDCPLL, 1, 1)},
[CLK_TOP_MSDCPLL_D2] = { MTK_FACTOR("msdcpll_d2", CLK_TOP_MSDCPLL, 1, 2)},
[CLK_TOP_MSDCPLL_D16] = { MTK_FACTOR("msdcpll_d16", CLK_TOP_MSDCPLL, 1, 16)},
[CLK_TOP_ETHPLL] = { MTK_FACTOR("ethpll_ck", CLK_APMIXED_BASE + CLK_APMIXED_ETHPLL, 1, 1)},
[CLK_TOP_ETHPLL_D2] = { MTK_FACTOR("ethpll_d2", CLK_TOP_ETHPLL, 1, 2)},
[CLK_TOP_ETHPLL_D4] = { MTK_FACTOR("ethpll_d4", CLK_TOP_ETHPLL, 1, 4)},
[CLK_TOP_ETHPLL_D8] = { MTK_FACTOR("ethpll_d8", CLK_TOP_ETHPLL, 1, 8)},
[CLK_TOP_ETHPLL_D10] = { MTK_FACTOR("ethpll_d10", CLK_TOP_ETHPLL, 1, 10)},
[CLK_TOP_ADSPPLL] = { MTK_FACTOR("adsppll_ck", CLK_APMIXED_BASE + CLK_APMIXED_ADSPPLL, 1, 1)},
[CLK_TOP_ADSPPLL_D2] = { MTK_FACTOR("adsppll_d2", CLK_TOP_ADSPPLL, 1, 2)},
[CLK_TOP_ADSPPLL_D4] = { MTK_FACTOR("adsppll_d4", CLK_TOP_ADSPPLL, 1, 4)},
[CLK_TOP_ADSPPLL_D8] = { MTK_FACTOR("adsppll_d8", CLK_TOP_ADSPPLL, 1, 8)},
[CLK_TOP_ULPOSC1_D2] = { MTK_FACTOR("ulposc1_d2", CLK_TOP_ULPOSC1, 1, 2)},
[CLK_TOP_ULPOSC1_D4] = { MTK_FACTOR("ulposc1_d4", CLK_TOP_ULPOSC1, 1, 4)},
[CLK_TOP_ULPOSC1_D8] = { MTK_FACTOR("ulposc1_d8", CLK_TOP_ULPOSC1, 1, 8)},
[CLK_TOP_ULPOSC1_D7] = { MTK_FACTOR("ulposc1_d7", CLK_TOP_ULPOSC1, 1, 7)},
[CLK_TOP_ULPOSC1_D10] = { MTK_FACTOR("ulposc1_d10", CLK_TOP_ULPOSC1, 1, 10)},
[CLK_TOP_ULPOSC1_D16] = { MTK_FACTOR("ulposc1_d16", CLK_TOP_ULPOSC1, 1, 16)},
/*
 * CLK_CFG_0
 * axi_sel and bus_aximem_sel are bus clocks, should not be closed by Linux.
 * spm_sel and scp_sel are main clocks in always-on co-processor.
 */
[CLK_TOP_AXI] = { MTK_MUX_GATE("top_axi", axi_parents, 0x020, 0, 4, 7, 0x04, 0), .flags = CLK_IS_CRITICAL},
[CLK_TOP_SPM] = { MTK_MUX_GATE("top_spm", spm_parents, 0x020, 8, 4, 15, 0x04, 1), .flags = CLK_IS_CRITICAL},
[CLK_TOP_SCP] = { MTK_MUX_GATE("top_scp", scp_parents, 0x020, 16, 4, 23, 0x04, 2), .flags = CLK_IS_CRITICAL},
[CLK_TOP_BUS_AXIMEM] = { MTK_MUX_GATE("top_bus_aximem", bus_aximem_parents, 0x020, 24, 4, 31, 0x04, 3), .flags = CLK_IS_CRITICAL},
	/* CLK_CFG_1 */
[CLK_TOP_VPP] = { MTK_MUX_GATE("top_vpp", vpp_parents, 0x02C, 0, 4, 7, 0x04, 4)},
[CLK_TOP_ETHDR] = { MTK_MUX_GATE("top_ethdr", ethdr_parents, 0x02C, 8, 4, 15, 0x04, 5)},
[CLK_TOP_IPE] = { MTK_MUX_GATE("top_ipe", ipe_parents, 0x02C, 16, 4, 23, 0x04, 6)},
[CLK_TOP_CAM] = { MTK_MUX_GATE("top_cam", cam_parents, 0x02C, 24, 4, 31, 0x04, 7)},
	/* CLK_CFG_2 */
[CLK_TOP_CCU] = { MTK_MUX_GATE("top_ccu", ccu_parents, 0x038, 0, 4, 7, 0x04, 8)},
[CLK_TOP_CCU_AHB] = { MTK_MUX_GATE("top_ccu_ahb", ccu_ahb_parents, 0x038, 8, 4, 15, 0x04, 9)},
[CLK_TOP_IMG] = { MTK_MUX_GATE("top_img", img_parents, 0x038, 16, 4, 23, 0x04, 10)},
[CLK_TOP_CAMTM] = { MTK_MUX_GATE("top_camtm", camtm_parents, 0x038, 24, 4, 31, 0x04, 11)},
	/* CLK_CFG_3 */
[CLK_TOP_DSP] = { MTK_MUX_GATE("top_dsp", dsp_parents, 0x044, 0, 4, 7, 0x04, 12)},
[CLK_TOP_DSP1] = { MTK_MUX_GATE("top_dsp1", dsp1_parents, 0x044, 8, 4, 15, 0x04, 13)},
[CLK_TOP_DSP2] = { MTK_MUX_GATE("top_dsp2", dsp2_parents, 0x044, 16, 4, 23, 0x04, 14)},
[CLK_TOP_DSP3] = { MTK_MUX_GATE("top_dsp3", dsp3_parents, 0x044, 24, 4, 31, 0x04, 15)},
	/* CLK_CFG_4 */
[CLK_TOP_DSP4] = { MTK_MUX_GATE("top_dsp4", dsp4_parents, 0x050, 0, 4, 7, 0x04, 16)},
[CLK_TOP_DSP5] = { MTK_MUX_GATE("top_dsp5", dsp5_parents, 0x050, 8, 4, 15, 0x04, 17)},
[CLK_TOP_DSP6] = { MTK_MUX_GATE("top_dsp6", dsp6_parents, 0x050, 16, 4, 23, 0x04, 18)},
[CLK_TOP_DSP7] = { MTK_MUX_GATE("top_dsp7", dsp7_parents, 0x050, 24, 4, 31, 0x04, 19)},
	/* CLK_CFG_5 */
[CLK_TOP_MFG_CORE_TMP] = { MTK_MUX_GATE("top_mfg_core_tmp", mfg_core_tmp_parents, 0x05C, 0, 4, 7, 0x04, 20)},
[CLK_TOP_CAMTG] = { MTK_MUX_GATE("top_camtg", camtg_parents, 0x05C, 8, 4, 15, 0x04, 21)},
[CLK_TOP_CAMTG2] = { MTK_MUX_GATE("top_camtg2", camtg2_parents, 0x05C, 16, 4, 23, 0x04, 22)},
[CLK_TOP_CAMTG3] = { MTK_MUX_GATE("top_camtg3", camtg3_parents, 0x05C, 24, 4, 31, 0x04, 23)},
	/* CLK_CFG_6 */
[CLK_TOP_UART] = { MTK_MUX_GATE("top_uart", uart_parents, 0x068, 0, 4, 7, 0x04, 24)},
[CLK_TOP_SPI] = { MTK_MUX_GATE("top_spi", spi_parents, 0x068, 8, 4, 15, 0x04, 25)},
[CLK_TOP_MSDC50_0_HCLK] = { MTK_MUX_GATE("top_msdc5hclk", msdc5hclk_parents, 0x068, 16, 4, 23, 0x04, 26)},
[CLK_TOP_MSDC50_0] = { MTK_MUX_GATE("top_msdc50_0", msdc50_0_parents, 0x068, 24, 4, 31, 0x04, 27)},
	/* CLK_CFG_7 */
[CLK_TOP_MSDC30_1] = { MTK_MUX_GATE("top_msdc30_1", msdc30_1_parents, 0x074, 0, 4, 7, 0x04, 28)},
[CLK_TOP_MSDC30_2] = { MTK_MUX_GATE("top_msdc30_2", msdc30_2_parents, 0x074, 8, 4, 15, 0x04, 29)},
[CLK_TOP_INTDIR] = { MTK_MUX_GATE("top_intdir", intdir_parents, 0x074, 16, 4, 23, 0x04, 30)},
[CLK_TOP_AUD_INTBUS] = { MTK_MUX_GATE("top_aud_intbus", aud_intbus_parents, 0x074, 24, 4, 31, 0x04, 31)},
	/* CLK_CFG_8 */
[CLK_TOP_AUDIO_H] = { MTK_MUX_GATE("top_audio_h", audio_h_parents, 0x080, 0, 4, 7, 0x08, 0)},
[CLK_TOP_PWRAP_ULPOSC] = { MTK_MUX_GATE("top_pwrap_ulposc", pwrap_ulposc_parents, 0x080, 8, 4, 15, 0x08, 1)},
[CLK_TOP_ATB] = { MTK_MUX_GATE("top_atb", atb_parents, 0x080, 16, 4, 23, 0x08, 2)},
[CLK_TOP_SSPM] = { MTK_MUX_GATE("top_sspm", sspm_parents, 0x080, 24, 4, 31, 0x08, 3)},
	/* CLK_CFG_9 */
[CLK_TOP_DP] = { MTK_MUX_GATE("top_dp", dp_parents, 0x08C, 0, 4, 7, 0x08, 4)},
[CLK_TOP_EDP] = { MTK_MUX_GATE("top_edp", edp_parents, 0x08C, 8, 4, 15, 0x08, 5)},
[CLK_TOP_DPI] = { MTK_MUX_GATE("top_dpi", dpi_parents, 0x08C, 16, 4, 23, 0x08, 6)},
[CLK_TOP_DISP_PWM0] = { MTK_MUX_GATE("top_disp_pwm0", disp_pwm0_parents, 0x08C, 24, 4, 31, 0x08, 7)},
	/* CLK_CFG_10 */
[CLK_TOP_DISP_PWM1] = { MTK_MUX_GATE("top_disp_pwm1", disp_pwm1_parents, 0x098, 0, 4, 7, 0x08, 8)},
[CLK_TOP_USB_TOP] = { MTK_MUX_GATE("top_usb_top", usb_parents, 0x098, 8, 4, 15, 0x08, 9)},
[CLK_TOP_SSUSB_XHCI] = { MTK_MUX_GATE("top_ssusb_xhci", ssusb_xhci_parents, 0x098, 16, 4, 23, 0x08, 10)},
[CLK_TOP_USB_TOP_2P] = { MTK_MUX_GATE("top_usb_top_2p", usb_2p_parents, 0x098, 24, 4, 31, 0x08, 11)},
	/* CLK_CFG_11 */
[CLK_TOP_SSUSB_XHCI_2P] = { MTK_MUX_GATE("top_ssusb_xhci_2p", ssusb_xhci_2p_parents, 0x0A4, 0, 4, 7, 0x08, 12)},
[CLK_TOP_USB_TOP_3P] = { MTK_MUX_GATE("top_usb_top_3p", usb_3p_parents, 0x0A4, 8, 4, 15, 0x08, 13)},
[CLK_TOP_SSUSB_XHCI_3P] = { MTK_MUX_GATE("top_ssusb_xhci_3p", ssusb_xhci_3p_parents, 0x0A4, 16, 4, 23, 0x08, 14)},
[CLK_TOP_I2C] = { MTK_MUX_GATE("top_i2c", i2c_parents, 0x0A4, 24, 4, 31, 0x08, 15)},
	/* CLK_CFG_12 */
[CLK_TOP_SENINF] = { MTK_MUX_GATE("top_seninf", seninf_parents, 0x0B0, 0, 4, 7, 0x08, 16)},
[CLK_TOP_SENINF1] = { MTK_MUX_GATE("top_seninf1", seninf1_parents, 0x0B0, 8, 4, 15, 0x08, 17)},
[CLK_TOP_GCPU] = { MTK_MUX_GATE("top_gcpu", gcpu_parents, 0x0B0, 16, 4, 23, 0x08, 18)},
[CLK_TOP_VENC] = { MTK_MUX_GATE("top_venc", venc_parents, 0x0B0, 24, 4, 31, 0x08, 19)},
	/*
	 * CLK_CFG_13
	 * top_mcupm is main clock in co-processor, should not be handled by Linux.
	 */
[CLK_TOP_VDEC] = { MTK_MUX_GATE("top_vdec", vdec_parents, 0x0BC, 0, 4, 7, 0x08, 20)},
[CLK_TOP_PWM] = { MTK_MUX_GATE("top_pwm", pwm_parents, 0x0BC, 8, 4, 15, 0x08, 21)},
[CLK_TOP_MCUPM] = { MTK_MUX_GATE("top_mcupm", mcupm_parents, 0x0BC, 16, 4, 23, 0x08, 22), .flags = CLK_IS_CRITICAL},
[CLK_TOP_SPMI_P_MST] = { MTK_MUX_GATE("top_spmi_p_mst", spmi_p_mst_parents, 0x0BC, 24, 4, 31, 0x08, 23)},
	/*
	 * CLK_CFG_14
	 * dvfsrc_sel is for internal DVFS usage, should not be handled by Linux.
	 */
[CLK_TOP_SPMI_M_MST] = { MTK_MUX_GATE("top_spmi_m_mst", spmi_m_mst_parents, 0x0C8, 0, 4, 7, 0x08, 24)},
[CLK_TOP_DVFSRC] = { MTK_MUX_GATE("top_dvfsrc", dvfsrc_parents, 0x0C8, 8, 4, 15, 0x08, 25), .flags = CLK_IS_CRITICAL},
[CLK_TOP_TL] = { MTK_MUX_GATE("top_tl", tl_parents, 0x0C8, 16, 4, 23, 0x08, 26)},
[CLK_TOP_AES_MSDCFDE] = { MTK_MUX_GATE("top_aes_msdcfde", aes_msdcfde_parents, 0x0C8, 24, 4, 31, 0x08, 27)},
	/* CLK_CFG_15 */
[CLK_TOP_DSI_OCC] = { MTK_MUX_GATE("top_dsi_occ", dsi_occ_parents, 0x0D4, 0, 4, 7, 0x08, 28)},
[CLK_TOP_WPE_VPP] = { MTK_MUX_GATE("top_wpe_vpp", wpe_vpp_parents, 0x0D4, 8, 4, 15, 0x08, 29)},
[CLK_TOP_HDCP] = { MTK_MUX_GATE("top_hdcp", hdcp_parents, 0x0D4, 16, 4, 23, 0x08, 30)},
[CLK_TOP_HDCP_24M] = { MTK_MUX_GATE("top_hdcp_24m", hdcp_24m_parents, 0x0D4, 24, 4, 31, 0x08, 31)},
	/* CLK_CFG_16 */
[CLK_TOP_HDMI_APB] = { MTK_MUX_GATE("top_hdmi_apb", hdmi_apb_parents, 0x0E0, 0, 4, 7, 0x0C, 0)},
[CLK_TOP_SNPS_ETH_250M] = { MTK_MUX_GATE("top_snps_eth_250m", snps_eth_250m_parents, 0x0E0, 8, 4, 15, 0x0C, 1)},
[CLK_TOP_SNPS_ETH_62P4M_PTP] = { MTK_MUX_GATE("top_snps_eth_62p4m_ptp", snps_eth_62p4m_ptp_parents, 0x0E0, 16, 4, 23, 0x0C, 2)},
[CLK_TOP_SNPS_ETH_50M_RMII] = { MTK_MUX_GATE("snps_eth_50m_rmii", snps_eth_50m_rmii_parents, 0x0E0, 24, 4, 31, 0x0C, 3)},
	/* CLK_CFG_17 */
[CLK_TOP_ADSP] = { MTK_MUX_GATE("top_adsp", adsp_parents, 0x0EC, 0, 4, 7, 0x0C, 4)},
[CLK_TOP_AUDIO_LOCAL_BUS] = { MTK_MUX_GATE("top_audio_local_bus", audio_local_bus_parents, 0x0EC, 8, 4, 15, 0x0C, 5)},
[CLK_TOP_ASM_H] = { MTK_MUX_GATE("top_asm_h", asm_h_parents, 0x0EC, 16, 4, 23, 0x0C, 6)},
[CLK_TOP_ASM_L] = { MTK_MUX_GATE("top_asm_l", asm_l_parents, 0x0EC, 24, 4, 31, 0x0C, 7)},
	/* CLK_CFG_18 */
[CLK_TOP_APLL1] = { MTK_MUX_GATE("top_apll1", apll1_parents, 0x0F8, 0, 4, 7, 0x0C, 8)},
[CLK_TOP_APLL2] = { MTK_MUX_GATE("top_apll2", apll2_parents, 0x0F8, 8, 4, 15, 0x0C, 9)},
[CLK_TOP_APLL3] = { MTK_MUX_GATE("top_apll3", apll3_parents, 0x0F8, 16, 4, 23, 0x0C, 10)},
[CLK_TOP_APLL4] = { MTK_MUX_GATE("top_apll4", apll4_parents, 0x0F8, 24, 4, 31, 0x0C, 11)},
	/* CLK_CFG_19 */
[CLK_TOP_APLL5] = { MTK_MUX_GATE("top_apll5", apll5_parents, 0x0104, 0, 4, 7, 0x0C, 12)},
[CLK_TOP_I2SO1] = { MTK_MUX_GATE("top_i2so1", i2so1_parents, 0x0104, 8, 4, 15, 0x0C, 13)},
[CLK_TOP_I2SO2] = { MTK_MUX_GATE("top_i2so2", i2so2_parents, 0x0104, 16, 4, 23, 0x0C, 14)},
[CLK_TOP_I2SI1] = { MTK_MUX_GATE("top_i2si1", i2si1_parents, 0x0104, 24, 4, 31, 0x0C, 15)},
	/* CLK_CFG_20 */
[CLK_TOP_I2SI2] = { MTK_MUX_GATE("top_i2si2", i2si2_parents, 0x0110, 0, 4, 7, 0x0C, 16)},
[CLK_TOP_DPTX] = { MTK_MUX_GATE("top_dptx", dptx_parents, 0x0110, 8, 4, 15, 0x0C, 17)},
[CLK_TOP_AUD_IEC] = { MTK_MUX_GATE("top_aud_iec", aud_iec_parents, 0x0110, 16, 4, 23, 0x0C, 18)},
[CLK_TOP_A1SYS_HP] = { MTK_MUX_GATE("top_a1sys_hp", a1sys_hp_parents, 0x0110, 24, 4, 31, 0x0C, 19)},
	/* CLK_CFG_21 */
[CLK_TOP_A2SYS] = { MTK_MUX_GATE("top_a2sys", a2sys_parents, 0x011C, 0, 4, 7, 0x0C, 20)},
[CLK_TOP_A3SYS] = { MTK_MUX_GATE("top_a3sys", a3sys_parents, 0x011C, 8, 4, 15, 0x0C, 21)},
[CLK_TOP_A4SYS] = { MTK_MUX_GATE("top_a4sys", a4sys_parents, 0x011C, 16, 4, 23, 0x0C, 22)},
[CLK_TOP_ECC] = { MTK_MUX_GATE("top_ecc", ecc_parents, 0x011C, 24, 4, 31, 0x0C, 23)},
	/*
	 * CLK_CFG_22
	 * top_ulposc/top_srck are clock source of always on co-processor,
	 * should not be closed by Linux.
	 */
[CLK_TOP_SPINOR] = { MTK_MUX_GATE("top_spinor", spinor_parents, 0x0128, 0, 4, 7, 0x0C, 24)},
[CLK_TOP_ULPOSC] = { MTK_MUX_GATE("top_ulposc", ulposc_parents, 0x0128, 8, 4, 15, 0x0C, 25), .flags = CLK_IS_CRITICAL},
[CLK_TOP_SRCK] = { MTK_MUX_GATE("top_srck", srck_parents, 0x0128, 16, 4, 23, 0x0C, 26), .flags = CLK_IS_CRITICAL},
	/* CLK_MISC_CFG_3 */
[CLK_TOP_MFG_CK_FAST_REF] = { MTK_MUX("mfg_ck_fast_ref", mfg_fast_ref_parents, 0x0250, 8, 1)},
[CLK_TOP_APLL12_CK_DIV0] = { MTK_DIV_GATE("apll12_div0", CLK_TOP_I2SI1, 0x0320, 0, 0x0328, 8, 0)},
[CLK_TOP_APLL12_CK_DIV1] = { MTK_DIV_GATE("apll12_div1", CLK_TOP_I2SI2, 0x0320, 1, 0x0328, 8, 8)},
[CLK_TOP_APLL12_CK_DIV2] = { MTK_DIV_GATE("apll12_div2", CLK_TOP_I2SO1, 0x0320, 2, 0x0328, 8, 16)},
[CLK_TOP_APLL12_CK_DIV3] = { MTK_DIV_GATE("apll12_div3", CLK_TOP_I2SO2, 0x0320, 3, 0x0328, 8, 24)},
[CLK_TOP_APLL12_CK_DIV4] = { MTK_DIV_GATE("apll12_div4", CLK_TOP_AUD_IEC, 0x0320, 4, 0x0334, 8, 0)},
[CLK_TOP_APLL12_CK_DIV9] = { MTK_DIV_GATE("apll12_div9", CLK_TOP_DPTX, 0x0320, 9, 0x0338, 8, 8)},
	/* TOP0 */
[CLK_TOP_CFGREG_CLOCK_EN_VPP0] = { MTK_GATE("cfgreg_clock_vpp0", CLK_TOP_VPP, 0x238, 0, 0)},
[CLK_TOP_CFGREG_CLOCK_EN_VPP1] = { MTK_GATE("cfgreg_clock_vpp1", CLK_TOP_VPP, 0x238, 1, 0)},
[CLK_TOP_CFGREG_CLOCK_EN_VDO0] = { MTK_GATE("cfgreg_clock_vdo0", CLK_TOP_VPP, 0x238, 2, 0)},
[CLK_TOP_CFGREG_CLOCK_EN_VDO1] = { MTK_GATE("cfgreg_clock_vdo1", CLK_TOP_VPP, 0x238, 3, 0)},
[CLK_TOP_CFGREG_CLOCK_ISP_AXI_GALS] = { MTK_GATE("cfgreg_clock_isp_axi_gals", CLK_TOP_VPP, 0x238, 4, 0)},
[CLK_TOP_CFGREG_F26M_VPP0] = { MTK_GATE("cfgreg_f26m_vpp0", CLK_TOP_CLK26M, 0x238, 5, 0)},
[CLK_TOP_CFGREG_F26M_VPP1] = { MTK_GATE("cfgreg_f26m_vpp1", CLK_TOP_CLK26M, 0x238, 6, 0)},
[CLK_TOP_CFGREG_F26M_VDO0] = { MTK_GATE("cfgreg_f26m_vdo0", CLK_TOP_CLK26M, 0x238, 7, 0)},
[CLK_TOP_CFGREG_F26M_VDO1] = { MTK_GATE("cfgreg_f26m_vdo1", CLK_TOP_CLK26M, 0x238, 8, 0)},
[CLK_TOP_CFGREG_AUD_F26M_AUD] = { MTK_GATE("cfgreg_aud_f26m_aud", CLK_TOP_CLK26M, 0x238, 9, 0)},
[CLK_TOP_CFGREG_UNIPLL_SES] = { MTK_GATE("cfgreg_unipll_ses", CLK_TOP_UNIVPLL_D2, 0x238, 15, 0)},
[CLK_TOP_CFGREG_F_PCIE_PHY_REF] = { MTK_GATE("cfgreg_f_pcie_phy_ref", CLK_TOP_CLK26M, 0x238, 18, 0)},
	/* TOP1 */
[CLK_TOP_SSUSB_TOP_REF] = { MTK_GATE("ssusb_ref", CLK_TOP_CLK26M, 0x250, 0, 0)},
[CLK_TOP_SSUSB_PHY_REF] = { MTK_GATE("ssusb_phy_ref", CLK_TOP_CLK26M, 0x250, 1, 0)},
[CLK_TOP_SSUSB_TOP_P1_REF] = { MTK_GATE("ssusb_p1_ref", CLK_TOP_CLK26M, 0x250, 2, 0)},
[CLK_TOP_SSUSB_PHY_P1_REF] = { MTK_GATE("ssusb_phy_p1_ref", CLK_TOP_CLK26M, 0x250, 3, 0)},
[CLK_TOP_SSUSB_TOP_P2_REF] = { MTK_GATE("ssusb_p2_ref", CLK_TOP_CLK26M, 0x250, 4, 0)},
[CLK_TOP_SSUSB_PHY_P2_REF] = { MTK_GATE("ssusb_phy_p2_ref", CLK_TOP_CLK26M, 0x250, 5, 0)},
[CLK_TOP_SSUSB_TOP_P3_REF] = { MTK_GATE("ssusb_p3_ref", CLK_TOP_CLK26M, 0x250, 6, 0)},
[CLK_TOP_SSUSB_PHY_P3_REF] = { MTK_GATE("ssusb_phy_p3_ref", CLK_TOP_CLK26M, 0x250, 7, 0)},
};

static inline void clk_dm1(struct mt8188_clk_priv *priv, ulong id, struct clk *clk)
{
	if (id >= priv->num_clks) {
		printf("%s:id (%ld) too large\n", __func__, id);
		return;
	}
	if (!IS_ERR(clk))
		clk->id = id + priv->base_id;
	priv->clks[id] = clk;
}

static inline struct clk* clk_obtain_external(ofnode np, const char *name)
{
	struct clk clk_tmp;
	struct clk *pclk;
	int ret = clk_get_by_name_nodev(np, name, &clk_tmp);

	if (!ret) {
		pclk = dev_get_clk_ptr(clk_tmp.dev);
		return pclk;
	}
	return NULL;
}

static void mt8188_probe_single(struct mt8188_clk_priv *priv, unsigned clk_id);

const char* mt8188_get_name(struct mt8188_clk_priv *priv, const struct mt8188_clk_probe *cp, u16 dci)
{
	unsigned dev_id = 0;
	unsigned clk_id;
	struct mt8188_clk_priv *p;
	unsigned base = 0;

	while (1) {
		if (dci < mt8188_bases[dev_id])
			break;
		base = mt8188_bases[dev_id];
		dev_id++;
		if (dev_id >= ARRAY_SIZE(mt8188_bases))
			return NULL;
	}
	clk_id = dci - base;

	p = mt8188_clk_priv_devices[dev_id];

	if (!p) {
		printf("%s: add dependence on dev %d:%d for %d\n", __func__, dev_id, clk_id, priv->dev_id);
		return NULL;
	}
	if (clk_id >= p->num_clks)
		goto parent_error;

	if (!p->clks[clk_id])
		mt8188_probe_single(p, clk_id);
	if (p->clks[clk_id])
		return p->clks[clk_id]->dev->name;
	printf("%s: parent clk missing %d:%d\n", __func__, dev_id, clk_id);
	return NULL;

parent_error:
	printf("%s: invalid parent id 0x%x %d:%d for %s\n", __func__, dci, dev_id, clk_id, cp->name);
	return NULL;
}

static void mt8188_probe_single(struct mt8188_clk_priv *priv, unsigned clk_id)
{
	struct clk *clk = NULL;
	const struct mt8188_clk_probe *cp = &priv->cps[clk_id];
	const char *parent = NULL;
	const char **parents = NULL;
	int num_parents;

	debug("%s: %s number of parents %d, base=%lx\n", __func__, cp->name, cp->num_parents, (long)priv->base);
	if (priv->clks[clk_id]) {
		printf("%s: %s probe not needed\n", __func__, cp->name);
		return;
	}
	if (cp->num_parents) {
		int i;

		parents = kzalloc(cp->num_parents * sizeof(char *), GFP_KERNEL);
		if (!parents) {
			printf("%s: %s oom %d\n", __func__, cp->name, cp->num_parents);
			return;
		}
		for (i = 0; i < cp->num_parents; i++) {
			parent = mt8188_get_name(priv, cp, cp->parents[i]);
			if (!parent)
				return;
			parents[i] = parent;
		}
		parent = NULL;
		num_parents = cp->num_parents;
	} else if (cp->parent != 0xffff) {
		parent = mt8188_get_name(priv, cp, cp->parent);
		if (!parent)
			return;
	}
	if (!parents) {
		switch (cp->type) {
		case TYPE_mt_clk_mux_gate:
		case TYPE_mt_clk_mux:
		case TYPE_mt_clk_div_gate:
			parents = kzalloc(1 * sizeof(char *), GFP_KERNEL);
			parents[0] = parent;
			num_parents = 1;
		}
	}

	switch (cp->type) {
	case TYPE_mt_clk_external:
		clk = clk_obtain_external(priv->np, cp->name);
		break;
	case TYPE_mt_clk_factor:
		clk = clk_register_fixed_factor(NULL, cp->name, parent, CLK_SET_RATE_PARENT, cp->f.mult, cp->f.div);
		break;
	case TYPE_mt_clk_mux_gate:
		clk = mtk_clk_register_mux(cp->name, parents, num_parents,
			cp->flags | CLK_SET_RATE_NO_REPARENT, priv->base + cp->mg.mux_ofs,
			cp->mg.shift, cp->mg.width,
			cp->mg.gate_bit, priv->base + cp->mg.upd_ofs, cp->mg.upd_bit,
			MTKF_CLK_MUX_SETCLR_UPD);
		break;
	case TYPE_mt_clk_mux:
		clk = clk_register_mux(NULL, cp->name, parents, num_parents,
			cp->flags, priv->base + cp->m.mux_ofs,
			cp->m.shift, cp->m.width, 0);
		break;
	case TYPE_mt_clk_div_gate:
		clk = mtk_clk_register_div_gate(cp->name, parents, num_parents,
			cp->flags,
			priv->base + cp->dg.gate_ofs, cp->dg.gate_bit,
			priv->base + cp->dg.div_ofs, cp->dg.div_shift, cp->dg.div_width);
		break;
	case TYPE_mt_clk_gate:
		clk = mtk_clk_register_gate(cp->name, parent,
			cp->flags,
			priv->base + cp->g.gate_ofs, cp->g.gate_bit, cp->g.mtk_flags);
		break;
	case TYPE_mt_clk_pll:
		clk = mtk_clk_register_pll(cp->name, parent,
			0,
			priv->base, cp->pll.reg_ofs, cp->pll.tuner_ofs, 0x0000,
			cp->pll.tuner_en_bit, cp->pll.pll_en_bit, cp->pll.mtk_flags);
		break;
	default:
		printf("%s: clk %d:%d undefined\n", __func__, priv->dev_id, clk_id);
		break;
	}
	if (clk)
		clk_dm1(priv, clk_id, clk);
	return;
}

struct mt8188_clk_init {
	int	dev_id;
	int	base_id;
	int	num_clks;
	const struct mt8188_clk_probe *cps;
};

const struct mt8188_clk_init mt8188_topckgen = {
	.dev_id = dev_top,
	.base_id = CLK_TOP_BASE,
	.num_clks = CLK_TOP_NR_CLK,
	.cps = mt8188_top_clk_init,
};

#define MTK_PLL(_name, _parent, _reg_ofs, _tuner_ofs, _tuner_en_bit, _pll_en_bit, _flags) \
	.type = TYPE_mt_clk_pll,	\
	.name = _name,			\
	.parent = _parent,		\
	.pll = {.reg_ofs = _reg_ofs,		\
		.tuner_ofs = _tuner_ofs,	\
		.tuner_en_bit = _tuner_en_bit,	\
		.pll_en_bit = _pll_en_bit,	\
		.mtk_flags = _flags},		\

static const struct mt8188_clk_probe mt8188_apmixedsys_init[] = {
[CLK_APMIXED_CLK26M] = { MTK_EXTERNAL("clk26m")},
[CLK_APMIXED_ETHPLL]  = { MTK_PLL("ethpll",  CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x044C,    0,  0, 9, 0)},
[CLK_APMIXED_MSDCPLL] = { MTK_PLL("msdcpll", CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0514,    0,  0, 9, 0)},
[CLK_APMIXED_TVDPLL1] = { MTK_PLL("tvdpll1", CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0524,    0,  0, 9, 0)},
[CLK_APMIXED_TVDPLL2] = { MTK_PLL("tvdpll2", CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0534,    0,  0, 9, 0)},
[CLK_APMIXED_MMPLL]   = { MTK_PLL("mmpll",   CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0544,    0,  0, 9, MTKF_RST_BAR)},
[CLK_APMIXED_MAINPLL] = { MTK_PLL("mainpll", CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x045C,    0,  0, 9, MTKF_RST_BAR)},
[CLK_APMIXED_IMGPLL]  = { MTK_PLL("imgpll",  CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0554,    0,  0, 9, 0)},
[CLK_APMIXED_UNIVPLL] = { MTK_PLL("univpll", CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0504,    0,  0, 9, MTKF_RST_BAR)},
[CLK_APMIXED_ADSPPLL] = { MTK_PLL("adsppll", CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x042C,    0,  0, 9, 0)},
[CLK_APMIXED_MFGPLL]  = { MTK_PLL("mfgpll",  CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0340,    0,  0, 9, 0)},
[CLK_APMIXED_APLL1]   = { MTK_PLL("apll1",   CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0304, 0x34, 12, 9, MTKF_PWR_REG1)},
[CLK_APMIXED_APLL2]   = { MTK_PLL("apll2",   CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0318, 0x38, 13, 9, MTKF_PWR_REG1)},
[CLK_APMIXED_APLL3]   = { MTK_PLL("apll3",   CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x032C, 0x3C, 14, 9, MTKF_PWR_REG1)},
[CLK_APMIXED_APLL4]   = { MTK_PLL("apll4",   CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0404, 0x40, 15, 9, MTKF_PWR_REG1)},
[CLK_APMIXED_APLL5]   = { MTK_PLL("apll5",   CLK_APMIXED_BASE + CLK_APMIXED_CLK26M, 0x0418, 0x44, 16, 9, MTKF_PWR_REG1)},
[CLK_APMIXED_PLL_SSUSB26M_EN] = { MTK_GATE("pll_ssusb26m_en", CLK_TOP_CLK26M, 0x8, 1, MTKF_CLK_GATE_INV)},
};

static const struct mt8188_clk_init mt8188_apmixedsys = {
	.dev_id = dev_apmixedsys,
	.base_id = CLK_APMIXED_BASE,
	.num_clks = CLK_APMIXED_NR_CLK,
	.cps = mt8188_apmixedsys_init,
};

static const struct mt8188_clk_probe mt8188_infracfg_ao_init[] = {
/* INFRA_AO0 */
[CLK_INFRA_AO_PMIC_TMR] = { MTK_GATE("infra_ao_pmic_tmr", CLK_TOP_PWRAP_ULPOSC, 0x80, 0, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PMIC_AP] = { MTK_GATE("infra_ao_pmic_ap", CLK_TOP_PWRAP_ULPOSC, 0x80, 1, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PMIC_MD] = { MTK_GATE("infra_ao_pmic_md", CLK_TOP_PWRAP_ULPOSC, 0x80, 2, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PMIC_CONN] = { MTK_GATE("infra_ao_pmic_conn", CLK_TOP_PWRAP_ULPOSC, 0x80, 3, MTKF_CLK_GATE_SETCLR)},
/* infra_ao_sej is main clock is for secure engine with JTAG support */
[CLK_INFRA_AO_SEJ] = { MTK_GATE("infra_ao_sej", CLK_TOP_AXI, 0x80, 5, MTKF_CLK_GATE_SETCLR), .flags = CLK_IS_CRITICAL},
[CLK_INFRA_AO_APXGPT] = { MTK_GATE("infra_ao_apxgpt", CLK_TOP_AXI, 0x80, 6, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_GCE] = { MTK_GATE("infra_ao_gce", CLK_TOP_AXI, 0x80, 8, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_GCE2] = { MTK_GATE("infra_ao_gce2", CLK_TOP_AXI, 0x80, 9, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_THERM] = { MTK_GATE("infra_ao_therm", CLK_TOP_AXI, 0x80, 10, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PWM_HCLK] = { MTK_GATE("infra_ao_pwm_h", CLK_TOP_AXI, 0x80, 15, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PWM1] = { MTK_GATE("infra_ao_pwm1", CLK_TOP_PWM, 0x80, 16, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PWM2] = { MTK_GATE("infra_ao_pwm2", CLK_TOP_PWM, 0x80, 17, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PWM3] = { MTK_GATE("infra_ao_pwm3", CLK_TOP_PWM, 0x80, 18, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PWM4] = { MTK_GATE("infra_ao_pwm4", CLK_TOP_PWM, 0x80, 19, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PWM] = { MTK_GATE("infra_ao_pwm", CLK_TOP_PWM, 0x80, 21, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_UART0] = { MTK_GATE("infra_ao_uart0", CLK_TOP_UART, 0x80, 22, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_UART1] = { MTK_GATE("infra_ao_uart1", CLK_TOP_UART, 0x80, 23, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_UART2] = { MTK_GATE("infra_ao_uart2", CLK_TOP_UART, 0x80, 24, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_UART3] = { MTK_GATE("infra_ao_uart3", CLK_TOP_UART, 0x80, 25, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_UART4] = { MTK_GATE("infra_ao_uart4", CLK_TOP_UART, 0x80, 26, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_GCE_26M] = { MTK_GATE("infra_ao_gce_26m", CLK_TOP_CLK26M, 0x80, 27, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_CQ_DMA_FPC] = { MTK_GATE("infra_ao_dma", CLK_TOP_PAD_FPC, 0x80, 28, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_UART5] = { MTK_GATE("infra_ao_uart5", CLK_TOP_UART, 0x80, 29, MTKF_CLK_GATE_SETCLR)},

/* INFRA_AO1 */
[CLK_INFRA_AO_HDMI_26M] = { MTK_GATE("infra_ao_hdmi_26m", CLK_TOP_CLK26M, 0x88, 0, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_SPI0] = { MTK_GATE("infra_ao_spi0", CLK_TOP_SPI, 0x88, 1, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_MSDC0] = { MTK_GATE("infra_ao_msdc0", CLK_TOP_MSDC50_0_HCLK, 0x88, 2, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_MSDC1] = { MTK_GATE("infra_ao_msdc1", CLK_TOP_AXI, 0x88, 4, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_MSDC2] = { MTK_GATE("infra_ao_msdc2", CLK_TOP_AXI, 0x88, 5, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_MSDC0_SRC] = { MTK_GATE("infra_ao_msdc0_clk", CLK_TOP_MSDC50_0, 0x88, 6, MTKF_CLK_GATE_SETCLR)},
/* infra_ao_dvfsrc is for internal DVFS usage, should not be handled by Linux. */
[CLK_INFRA_AO_DVFSRC] = { MTK_GATE("infra_ao_dvfsrc", CLK_TOP_CLK26M, 0x88, 7, MTKF_CLK_GATE_SETCLR), .flags = CLK_IS_CRITICAL},
[CLK_INFRA_AO_TRNG] = { MTK_GATE("infra_ao_trng", CLK_TOP_AXI, 0x88, 9, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_AUXADC] = { MTK_GATE("infra_ao_auxadc", CLK_TOP_CLK26M, 0x88, 10, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_CPUM] = { MTK_GATE("infra_ao_cpum", CLK_TOP_AXI, 0x88, 11, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_HDMI_32K] = { MTK_GATE("infra_ao_hdmi_32k", CLK_TOP_CLK32K, 0x88, 12, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_CEC_66M_HCLK] = { MTK_GATE("infra_ao_cec_66m_hclk", CLK_TOP_AXI, 0x88, 13, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PCIE_TL_26M] = { MTK_GATE("infra_ao_pcie_tl_26m", CLK_TOP_CLK26M, 0x88, 15, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_MSDC1_SRC] = { MTK_GATE("infra_ao_msdc1_clk", CLK_TOP_MSDC30_1, 0x88, 16, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_CEC_66M_BCLK] = { MTK_GATE("infra_ao_cec_66m_bclk", CLK_TOP_AXI, 0x88, 17, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PCIE_TL_96M] = { MTK_GATE("infra_ao_pcie_tl_96m", CLK_TOP_TL, 0x88, 18, MTKF_CLK_GATE_SETCLR)},
/* infra_ao_dapc is for device access permission control module */
[CLK_INFRA_AO_DEVICE_APC] = { MTK_GATE("infra_ao_dapc", CLK_TOP_AXI, 0x88, 20, MTKF_CLK_GATE_SETCLR), .flags = CLK_IS_CRITICAL},
[CLK_INFRA_AO_ECC_66M_HCLK] = { MTK_GATE("infra_ao_ecc_66m_hclk", CLK_TOP_AXI, 0x88, 23, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_DEBUGSYS] = { MTK_GATE("infra_ao_debugsys", CLK_TOP_AXI, 0x88, 24, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_AUDIO] = { MTK_GATE("infra_ao_audio", CLK_TOP_AXI, 0x88, 25, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PCIE_TL_32K] = { MTK_GATE("infra_ao_pcie_tl_32k", CLK_TOP_CLK32K, 0x88, 26, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_DBG_TRACE] = { MTK_GATE("infra_ao_dbg_trace", CLK_TOP_AXI, 0x88, 29, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_DRAMC_F26M] = { MTK_GATE("infra_ao_dramc26", CLK_TOP_CLK26M, 0x88, 31, MTKF_CLK_GATE_SETCLR)},

/* INFRA_AO2 */
[CLK_INFRA_AO_IRTX] = { MTK_GATE("infra_ao_irtx", CLK_TOP_AXI, 0xa4, 0, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_DISP_PWM] = { MTK_GATE("infra_ao_disp_pwm", CLK_TOP_DISP_PWM0, 0xa4, 2, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_CLDMA_BCLK] = { MTK_GATE("infra_ao_cldmabclk", CLK_TOP_AXI, 0xa4, 3, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_AUDIO_26M_BCLK] = { MTK_GATE("infra_ao_audio26m", CLK_TOP_CLK26M, 0xa4, 4, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_SPI1] = { MTK_GATE("infra_ao_spi1", CLK_TOP_SPI, 0xa4, 6, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_SPI2] = { MTK_GATE("infra_ao_spi2", CLK_TOP_SPI, 0xa4, 9, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_SPI3] = { MTK_GATE("infra_ao_spi3", CLK_TOP_SPI, 0xa4, 10, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_FSSPM] = { MTK_GATE("infra_ao_fsspm", CLK_TOP_SSPM, 0xa4, 15, MTKF_CLK_GATE_SETCLR), .flags = CLK_IS_CRITICAL},
[CLK_INFRA_AO_SSPM_BUS_HCLK] = { MTK_GATE("infra_ao_sspm_hclk", CLK_TOP_AXI, 0xa4, 17, MTKF_CLK_GATE_SETCLR), .flags = CLK_IS_CRITICAL},
[CLK_INFRA_AO_APDMA_BCLK] = { MTK_GATE("infra_ao_apdma_bclk", CLK_TOP_AXI, 0xa4, 18, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_SPI4] = { MTK_GATE("infra_ao_spi4", CLK_TOP_SPI, 0xa4, 25, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_SPI5] = { MTK_GATE("infra_ao_spi5", CLK_TOP_SPI, 0xa4, 26, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_CQ_DMA] = { MTK_GATE("infra_ao_cq_dma", CLK_TOP_AXI, 0xa4, 27, MTKF_CLK_GATE_SETCLR)},

/* INFRA_AO3 */
[CLK_INFRA_AO_MSDC0_SELF] = { MTK_GATE("infra_ao_msdc0sf", CLK_TOP_MSDC50_0, 0xc0, 0, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_MSDC1_SELF] = { MTK_GATE("infra_ao_msdc1sf", CLK_TOP_MSDC50_0, 0xc0, 1, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_MSDC2_SELF] = { MTK_GATE("infra_ao_msdc2sf", CLK_TOP_MSDC50_0, 0xc0, 2, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_I2S_DMA] = { MTK_GATE("infra_ao_i2s_dma", CLK_TOP_AXI, 0xc0, 5, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_AP_MSDC0] = { MTK_GATE("infra_ao_ap_msdc0", CLK_TOP_MSDC50_0, 0xc0, 7, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_MD_MSDC0] = { MTK_GATE("infra_ao_md_msdc0", CLK_TOP_MSDC50_0, 0xc0, 8, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_MSDC30_2] = { MTK_GATE("infra_ao_msdc30_2", CLK_TOP_MSDC30_2, 0xc0, 9, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_GCPU] = { MTK_GATE("infra_ao_gcpu", CLK_TOP_GCPU, 0xc0, 10, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_PCIE_PERI_26M] = { MTK_GATE("infra_ao_pcie_peri_26m", CLK_TOP_CLK26M, 0xc0, 15, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_GCPU_66M_BCLK] = { MTK_GATE("infra_ao_gcpu_66m_bclk", CLK_TOP_AXI, 0xc0, 16, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_GCPU_133M_BCLK] = { MTK_GATE("infra_ao_gcpu_133m_bclk", CLK_TOP_AXI, 0xc0, 17, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_DISP_PWM1] = { MTK_GATE("infra_ao_disp_pwm1", CLK_TOP_DISP_PWM1, 0xc0, 20, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_FBIST2FPC] = { MTK_GATE("infra_ao_fbist2fpc", CLK_TOP_MSDC50_0, 0xc0, 24, MTKF_CLK_GATE_SETCLR)},
/* infra_ao_dapc_sync is for device access permission control module */
[CLK_INFRA_AO_DEVICE_APC_SYNC] = { MTK_GATE("infra_ao_dapc_sync", CLK_TOP_AXI, 0xc0, 25, MTKF_CLK_GATE_SETCLR), .flags = CLK_IS_CRITICAL},
[CLK_INFRA_AO_PCIE_P1_PERI_26M] = { MTK_GATE("infra_ao_pcie_p1_peri_26m", CLK_TOP_CLK26M, 0xc0, 26, MTKF_CLK_GATE_SETCLR)},

/* INFRA_AO4 */
/* infra_ao_133m_mclk_set/infra_ao_66m_mclk_set are main clocks of peripheral */
[CLK_INFRA_AO_133M_MCLK_CK] = { MTK_GATE("infra_ao_133m_mclk_set", CLK_TOP_AXI, 0xe0, 0, MTKF_CLK_GATE_SETCLR), .flags = CLK_IS_CRITICAL},
[CLK_INFRA_AO_66M_MCLK_CK] = { MTK_GATE("infra_ao_66m_mclk_set", CLK_TOP_AXI, 0xe0, 1, MTKF_CLK_GATE_SETCLR), .flags = CLK_IS_CRITICAL},
[CLK_INFRA_AO_PCIE_PL_P_250M_P0] = { MTK_GATE("infra_ao_pcie_pl_p_250m_p0", CLK_TOP_PEXTP_PIPE, 0xe0, 7, MTKF_CLK_GATE_SETCLR)},
[CLK_INFRA_AO_RG_AES_MSDCFDE_CK_0P] = { MTK_GATE("infra_ao_aes_msdcfde_0p", CLK_TOP_AES_MSDCFDE, 0xe0, 18, MTKF_CLK_GATE_SETCLR)},
};

static const struct mt8188_clk_init mt8188_infracfg_ao = {
	.dev_id = dev_infracfg_ao,
	.base_id = CLK_INFRA_AO_BASE,
	.num_clks = CLK_INFRA_AO_NR_CLK,
	.cps = mt8188_infracfg_ao_init,
};

static const struct udevice_id mt8188_clk_dt_ids[] = {
	{ .compatible = "mediatek,mt8188-topckgen", (long)&mt8188_topckgen},
	{ .compatible = "mediatek,mt8188-apmixedsys", (long)&mt8188_apmixedsys},
	{ .compatible = "mediatek,mt8188-infracfg_ao", (long)&mt8188_infracfg_ao},
	{ }
};

static int mt8188_probe(struct udevice *dev)
{
	struct clk_bulk clks;
	int ret;

	ret = clk_get_bulk(dev, &clks);

	return 0;
}

static int mt8188_of_xlate(struct clk *clk,
				struct ofnode_phandle_args *args)
{
	struct mt8188_clk_priv *priv;
	unsigned i;

	if (!clk)
		return -EINVAL;
	if (!clk->dev)
		return -EINVAL;
	priv = dev_get_priv(clk->dev);
	if (!priv)
		return -EINVAL;
	if (args->args_count > 1) {
		debug("Invalid args_count: %d\n", args->args_count);
		return -EINVAL;
	}

	clk->id = priv->base_id;
	if (args->args_count) {
		i = args->args[0];
		if (i < priv->num_clks) {
			clk->id += i;
			if (!priv->clks)
				return -EINVAL;
			if (!priv->clks[i])
				mt8188_probe_single(priv, i);
		} else if (priv->num_clks) {
			printf("%s: error, too big %d >= %d %s\n", __func__, args->args[0], priv->num_clks, dev_read_name(clk->dev));
			return -EINVAL;
		}
	}
	clk->data = 0;
	return 0;
}

static const struct clk_ops mt8188_clk_ops = {
	.of_xlate = mt8188_of_xlate,
	.round_rate = ccf_clk_round_rate,
	.set_rate = ccf_clk_set_rate,
	.get_rate = ccf_clk_get_rate,
	.set_parent = ccf_clk_set_parent,
	.enable = ccf_clk_enable,
	.disable = ccf_clk_disable,
};

static int mt8188_clk_ofdata_to_platdata(struct udevice *dev)
{
	struct mt8188_clk_priv *priv = dev_get_priv(dev);
	struct mt8188_clk_init *pdata;

	pdata = (struct mt8188_clk_init *)dev->driver_data;
	priv->np = dev_ofnode(dev);
	priv->base = (void *)dev_read_addr(dev);
	debug("%s: base=%lx dev_id=%d\n", __func__, (long)priv->base, pdata->dev_id);
	priv->base_id = pdata->base_id;
	priv->dev_id = pdata->dev_id;
	priv->num_clks = pdata->num_clks;
	priv->cps = pdata->cps;
	if (priv->clks)
		printf("%s: already allocated\n", __func__);
	else
		priv->clks = kzalloc(sizeof(struct clk *) * priv->num_clks, GFP_KERNEL);

	if (mt8188_clk_priv_devices[priv->dev_id])
		printf("%s: already set %d %p %p\n", __func__, priv->dev_id,
			mt8188_clk_priv_devices[priv->dev_id], priv);
	mt8188_clk_priv_devices[priv->dev_id] = priv;

	return 0;
}

U_BOOT_DRIVER(mtk_clk) = {
	.name = "mt8188-clk",
	.id = UCLASS_CLK,
	.of_match = mt8188_clk_dt_ids,
	.of_to_plat = mt8188_clk_ofdata_to_platdata,
	.ops = &mt8188_clk_ops,
	.probe = mt8188_probe,
	.priv_auto = sizeof(struct mt8188_clk_priv),
	.flags = DM_FLAG_PRE_RELOC,
};
