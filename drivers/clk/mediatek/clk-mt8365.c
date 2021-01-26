// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek clock driver for MT8365 SoC
 *
 * Copyright (C) 2021 BayLibre, SAS
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Fabien Parent <fparent@baylibre.com>
 * Author: Weiyi Lu <weiyi.lu@mediatek.com>
 */

#include <common.h>
#include <dm.h>
#include <asm/io.h>
#include <dt-bindings/clock/mt8365-clk.h>

#include "clk-mtk.h"

#define MT8365_PLL_FMAX		(3800UL * MHZ)
#define MT8365_PLL_FMIN		(1500UL * MHZ)

/* apmixedsys */
#define PLL(_id, _reg, _pwr_reg, _en_mask, _flags, _pcwbits, _pd_reg,	\
	    _pd_shift, _pcw_reg, _pcw_shift) {				\
		.id = _id,						\
		.reg = _reg,						\
		.pwr_reg = _pwr_reg,					\
		.en_mask = _en_mask,					\
		.rst_bar_mask = BIT(23),				\
		.fmax = MT8365_PLL_FMAX,				\
		.flags = _flags,					\
		.pcwbits = _pcwbits,					\
		.pcwibits = 8,						\
		.pd_reg = _pd_reg,					\
		.pd_shift = _pd_shift,					\
		.pcw_reg = _pcw_reg,					\
		.pcw_shift = _pcw_shift,				\
	}

static const struct mtk_pll_data apmixed_plls[] = {
	PLL(CLK_APMIXED_ARMPLL, 0x030C, 0x0318, 0x00000001, 0, 22, 0x0310, 24,
	    0x0310, 0),
	PLL(CLK_APMIXED_MAINPLL, 0x0228, 0x0234, 0xFF000001, HAVE_RST_BAR, 22,
	    0x022C, 24, 0x022C, 0),
	PLL(CLK_APMIXED_UNIVPLL, 0x0208, 0x0214, 0xFF000001, HAVE_RST_BAR, 22,
	    0x020C, 24, 0x020C, 0),
	PLL(CLK_APMIXED_MFGPLL, 0x0218, 0x0224, 0x00000001, 0, 22, 0x021C, 24,
	    0x021C, 0),
	PLL(CLK_APMIXED_MSDCPLL, 0x0350, 0x035C, 0x00000001, 0, 22, 0x0354, 24,
	    0x0354, 0),
	PLL(CLK_APMIXED_MMPLL, 0x0330, 0x033C, 0x00000001, 0, 22, 0x0334, 24,
	    0x0334, 0),
	PLL(CLK_APMIXED_APLL1, 0x031C, 0x032C, 0x00000001, 0, 32, 0x0320, 24,
	    0x0324, 0),
	PLL(CLK_APMIXED_APLL2, 0x0360, 0x0370, 0x00000001, 0, 32, 0x0364, 24,
	    0x0368, 0),
	PLL(CLK_APMIXED_LVDSPLL, 0x0374, 0x0380, 0x00000001, 0, 22, 0x0378, 24,
	    0x0378, 0),
	PLL(CLK_APMIXED_DSPPLL, 0x0390, 0x039C, 0x00000001, 0, 22, 0x0394, 24,
	    0x0394, 0),
	PLL(CLK_APMIXED_APUPLL, 0x03A0, 0x03AC, 0x00000001, 0, 22, 0x03A4, 24,
	    0x03A4, 0),
};

static const struct mtk_fixed_clk top_fixed_clks[] = {
	FIXED_CLK(CLK_TOP_CLK_NULL, CLK_XTAL, 0),
	FIXED_CLK(CLK_TOP_DSI0_LNTC_DSICK, CLK_TOP_CLK26M, 75000000),
	FIXED_CLK(CLK_TOP_VPLL_DPIX, CLK_TOP_CLK26M, 75000000),
	FIXED_CLK(CLK_TOP_LVDSTX_CLKDIG_CTS, CLK_TOP_CLK26M, 52500000),
	FIXED_CLK(CLK_TOP_CLK26M, CLK_XTAL, 26000000),
	FIXED_CLK(CLK_TOP_CLK32K, CLK_XTAL, 32000),
};

static const struct mtk_fixed_factor top_fixed_divs[] = {
	FACTOR(CLK_TOP_SYSPLL_D2, CLK_APMIXED_MAINPLL, 1, 2,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL1_D2, CLK_APMIXED_MAINPLL, 1, 4,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL1_D4, CLK_APMIXED_MAINPLL, 1, 8,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL1_D8, CLK_APMIXED_MAINPLL, 1, 16,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL1_D16, CLK_APMIXED_MAINPLL, 1, 32,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL_D3, CLK_APMIXED_MAINPLL, 1, 3,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL2_D2, CLK_APMIXED_MAINPLL, 1, 6,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL2_D4, CLK_APMIXED_MAINPLL, 1, 12,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL2_D8, CLK_APMIXED_MAINPLL, 1, 24,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL_D5, CLK_APMIXED_MAINPLL, 1, 5,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL3_D2, CLK_APMIXED_MAINPLL, 1, 10,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL3_D4, CLK_APMIXED_MAINPLL, 1, 20,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL_D7, CLK_APMIXED_MAINPLL, 1, 7,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL4_D2, CLK_APMIXED_MAINPLL, 1, 14,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_SYSPLL4_D4, CLK_APMIXED_MAINPLL, 1, 28,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_UNIVPLL, CLK_APMIXED_UNIVPLL, 1, 2, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_UNIVPLL_D2, CLK_TOP_UNIVPLL, 1, 2, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL1_D2, CLK_TOP_UNIVPLL, 1, 4, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL1_D4, CLK_TOP_UNIVPLL, 1, 8, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL_D3, CLK_TOP_UNIVPLL, 1, 3, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL2_D2, CLK_TOP_UNIVPLL, 1, 6, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL2_D4, CLK_TOP_UNIVPLL, 1, 12,
	       CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL2_D8, CLK_TOP_UNIVPLL, 1, 24,
	       CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL2_D32, CLK_TOP_UNIVPLL, 1, 96,
	       CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL_D5, CLK_TOP_UNIVPLL, 1, 5, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL3_D2, CLK_TOP_UNIVPLL, 1, 10,
	       CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL3_D4, CLK_TOP_UNIVPLL, 1, 20,
	       CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_MMPLL, CLK_APMIXED_MMPLL, 1, 1,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_MMPLL_D2, CLK_APMIXED_MMPLL, 1, 2,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_MFGPLL, CLK_APMIXED_MFGPLL, 1, 1,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_LVDSPLL_D2, CLK_APMIXED_LVDSPLL, 1, 2,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_LVDSPLL_D4, CLK_APMIXED_LVDSPLL, 1, 4,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_LVDSPLL_D8, CLK_APMIXED_LVDSPLL, 1, 8,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_LVDSPLL_D16, CLK_APMIXED_LVDSPLL, 1, 16,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_USB20_192M, CLK_TOP_UNIVPLL, 1, 1, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_USB20_192M_D4, CLK_TOP_USB20_192M, 1, 4,
	       CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_USB20_192M_D8, CLK_TOP_USB20_192M, 1, 8,
	       CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_USB20_192M_D16, CLK_TOP_USB20_192M, 1, 16,
	       CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_USB20_192M_D32, CLK_TOP_USB20_192M, 1, 32,
	       CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_APLL1, CLK_APMIXED_APLL1, 1, 1, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_APLL1_D2, CLK_TOP_APLL1, 1, 2, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_APLL1_D4, CLK_TOP_APLL1, 1, 4, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_APLL1_D8, CLK_TOP_APLL1, 1, 8, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_APLL2, CLK_APMIXED_APLL2, 1, 1, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_APLL2_D2, CLK_TOP_APLL2, 1, 2, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_APLL2_D4, CLK_TOP_APLL2, 1, 4, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_APLL2_D8, CLK_TOP_APLL2, 1, 8, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_MSDCPLL, CLK_APMIXED_MSDCPLL, 1, 1, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_MSDCPLL_D2, CLK_APMIXED_MSDCPLL, 1, 2,
	       CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_DSPPLL, CLK_APMIXED_DSPPLL, 1, 1, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_DSPPLL_D2, CLK_APMIXED_DSPPLL, 1, 2, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_DSPPLL_D4, CLK_APMIXED_DSPPLL, 1, 4, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_DSPPLL_D8, CLK_APMIXED_DSPPLL, 1, 8, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_APUPLL, CLK_APMIXED_APUPLL, 1, 1, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_CLK26M_D52, CLK_TOP_CLK26M, 1, 52, CLK_PARENT_TOPCKGEN),
};

static const int axi_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYSPLL_D7,
	CLK_TOP_SYSPLL1_D4,
	CLK_TOP_SYSPLL3_D2
};

static const int mem_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MMPLL,
	CLK_TOP_SYSPLL_D3,
	CLK_TOP_SYSPLL1_D2
};

static const int mm_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MMPLL,
	CLK_TOP_SYSPLL1_D2,
	CLK_TOP_SYSPLL_D5,
	CLK_TOP_SYSPLL1_D4,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_UNIVPLL1_D2,
	CLK_TOP_MMPLL_D2,
};

static const int scp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYSPLL4_D2,
	CLK_TOP_UNIVPLL2_D2,
	CLK_TOP_SYSPLL1_D2,
	CLK_TOP_UNIVPLL1_D2,
	CLK_TOP_SYSPLL_D3,
	CLK_TOP_UNIVPLL_D3
};

static const int mfg_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MFGPLL,
	CLK_TOP_SYSPLL_D3,
	CLK_TOP_UNIVPLL_D3
};

static const int atb_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYSPLL1_D4,
	CLK_TOP_SYSPLL1_D2
};

static const int camtg_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_USB20_192M_D8,
	CLK_TOP_UNIVPLL2_D8,
	CLK_TOP_USB20_192M_D4,
	CLK_TOP_UNIVPLL2_D32,
	CLK_TOP_USB20_192M_D16,
	CLK_TOP_USB20_192M_D32,
};

static const int uart_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL2_D8
};

static const int spi_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL2_D2,
	CLK_TOP_UNIVPLL2_D4,
	CLK_TOP_UNIVPLL2_D8
};

static const int msdc50_0_hc_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYSPLL1_D2,
	CLK_TOP_UNIVPLL1_D4,
	CLK_TOP_SYSPLL2_D2
};

static const int msdc50_0_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MSDCPLL,
	CLK_TOP_UNIVPLL1_D2,
	CLK_TOP_SYSPLL1_D2,
	CLK_TOP_UNIVPLL_D5,
	CLK_TOP_SYSPLL2_D2,
	CLK_TOP_UNIVPLL1_D4,
	CLK_TOP_SYSPLL4_D2
};

static const int msdc50_2_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MSDCPLL,
	CLK_TOP_UNIVPLL_D3,
	CLK_TOP_UNIVPLL1_D2,
	CLK_TOP_SYSPLL1_D2,
	CLK_TOP_UNIVPLL2_D2,
	CLK_TOP_SYSPLL2_D2,
	CLK_TOP_UNIVPLL1_D4
};

static const int msdc30_1_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MSDCPLL_D2,
	CLK_TOP_UNIVPLL2_D2,
	CLK_TOP_SYSPLL2_D2,
	CLK_TOP_UNIVPLL1_D4,
	CLK_TOP_SYSPLL1_D4,
	CLK_TOP_SYSPLL2_D4,
	CLK_TOP_UNIVPLL2_D8
};

static const int audio_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYSPLL3_D4,
	CLK_TOP_SYSPLL4_D4,
	CLK_TOP_SYSPLL1_D16
};

static const int aud_intbus_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYSPLL1_D4,
	CLK_TOP_SYSPLL4_D2
};

static const int aud_1_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL1
};

static const int aud_2_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL2
};

static const int aud_engen1_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL1_D2,
	CLK_TOP_APLL1_D4,
	CLK_TOP_APLL1_D8
};

static const int aud_engen2_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_APLL2_D2,
	CLK_TOP_APLL2_D4,
	CLK_TOP_APLL2_D8,
};

static const int aud_spdif_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D2
};

static const int disp_pwm_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL2_D4
};

static const int dxcc_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYSPLL1_D2,
	CLK_TOP_SYSPLL1_D4,
	CLK_TOP_SYSPLL1_D8
};

static const int ssusb_sys_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL3_D4,
	CLK_TOP_UNIVPLL2_D4,
	CLK_TOP_UNIVPLL3_D2
};

static const int spm_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYSPLL1_D8
};

static const int i2c_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL3_D4,
	CLK_TOP_UNIVPLL3_D2,
	CLK_TOP_SYSPLL1_D8,
	CLK_TOP_SYSPLL2_D8
};

static const int pwm_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL3_D4,
	CLK_TOP_SYSPLL1_D8
};

static const int senif_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL1_D4,
	CLK_TOP_UNIVPLL1_D2,
	CLK_TOP_UNIVPLL2_D2
};

static const int aes_fde_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_MSDCPLL,
	CLK_TOP_UNIVPLL_D3,
	CLK_TOP_UNIVPLL2_D2,
	CLK_TOP_UNIVPLL1_D2,
	CLK_TOP_SYSPLL1_D2
};

static const int dpi0_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_LVDSPLL_D2,
	CLK_TOP_LVDSPLL_D4,
	CLK_TOP_LVDSPLL_D8,
	CLK_TOP_LVDSPLL_D16
};

static const int dsp_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYS_26M_D2,
	CLK_TOP_DSPPLL,
	CLK_TOP_DSPPLL_D2,
	CLK_TOP_DSPPLL_D4,
	CLK_TOP_DSPPLL_D8
};

static const int nfi2x_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYSPLL2_D2,
	CLK_TOP_SYSPLL_D7,
	CLK_TOP_SYSPLL_D3,
	CLK_TOP_SYSPLL2_D4,
	CLK_TOP_MSDCPLL_D2,
	CLK_TOP_UNIVPLL1_D2,
	CLK_TOP_UNIVPLL_D5
};

static const int nfiecc_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_SYSPLL4_D2,
	CLK_TOP_UNIVPLL2_D4,
	CLK_TOP_SYSPLL_D7,
	CLK_TOP_UNIVPLL1_D2,
	CLK_TOP_SYSPLL1_D2,
	CLK_TOP_UNIVPLL2_D2,
	CLK_TOP_SYSPLL_D5
};

static const int ecc_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL2_D2,
	CLK_TOP_UNIVPLL1_D2,
	CLK_TOP_UNIVPLL_D3,
	CLK_TOP_SYSPLL_D2
};

static const int eth_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL2_D8,
	CLK_TOP_SYSPLL4_D4,
	CLK_TOP_SYSPLL1_D8,
	CLK_TOP_SYSPLL4_D2
};

static const int gcpu_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D3,
	CLK_TOP_UNIVPLL2_D2,
	CLK_TOP_SYSPLL_D3,
	CLK_TOP_SYSPLL2_D2
};

static const int gcpu_cpm_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL2_D2,
	CLK_TOP_SYSPLL2_D2
};

static const int apu_parents[] = {
	CLK_TOP_CLK26M,
	CLK_TOP_UNIVPLL_D2,
	CLK_APMIXED_APUPLL,
	CLK_TOP_MMPLL,
	CLK_TOP_SYSPLL_D3,
	CLK_TOP_UNIVPLL1_D2,
	CLK_TOP_SYSPLL1_D2,
	CLK_TOP_SYSPLL1_D4
};

static const struct mtk_composite top_muxes[] = {
	/* CLK_CFG_0 */
	MUX(CLK_TOP_AXI_SEL, axi_parents, 0x040, 0, 2),
	MUX(CLK_TOP_MEM_SEL, mem_parents, 0x040, 8, 2),
	MUX(CLK_TOP_MM_SEL, mm_parents, 0x040, 16, 3),
	MUX(CLK_TOP_SCP_SEL, scp_parents, 0x040, 24, 3),
	/* CLK_CFG_1 */
	MUX(CLK_TOP_MFG_SEL, mfg_parents, 0x050, 0, 2),
	MUX(CLK_TOP_ATB_SEL, atb_parents, 0x050, 8, 2),
	MUX(CLK_TOP_CAMTG_SEL, camtg_parents, 0x050, 16, 3),
	MUX(CLK_TOP_CAMTG1_SEL, camtg_parents, 0x050, 24, 3),
	/* CLK_CFG_2 */
	MUX(CLK_TOP_UART_SEL, uart_parents, 0x060, 0, 1),
	MUX(CLK_TOP_SPI_SEL, spi_parents, 0x060, 8, 2),
	MUX(CLK_TOP_MSDC50_0_HC_SEL, msdc50_0_hc_parents, 0x060, 16, 2),
	MUX(CLK_TOP_MSDC2_2_HC_SEL, msdc50_0_hc_parents, 0x060, 24, 2),
	/* CLK_CFG_3 */
	MUX(CLK_TOP_MSDC50_0_SEL, msdc50_0_parents, 0x070, 0, 3),
	MUX(CLK_TOP_MSDC50_2_SEL, msdc50_2_parents, 0x070, 8, 3),
	MUX(CLK_TOP_MSDC30_1_SEL, msdc30_1_parents, 0x070, 16, 3),
	MUX(CLK_TOP_AUDIO_SEL, audio_parents, 0x070, 24, 2),
	/* CLK_CFG_4 */
	MUX(CLK_TOP_AUD_INTBUS_SEL, aud_intbus_parents, 0x080, 0, 2),
	MUX(CLK_TOP_AUD_1_SEL, aud_1_parents, 0x080, 8, 1),
	MUX(CLK_TOP_AUD_2_SEL, aud_2_parents, 0x080, 16, 1),
	MUX(CLK_TOP_AUD_ENGEN1_SEL, aud_engen1_parents, 0x080, 24, 2),
	/* CLK_CFG_5 */
	MUX(CLK_TOP_AUD_ENGEN2_SEL, aud_engen2_parents, 0x090, 0, 2),
	MUX(CLK_TOP_AUD_SPDIF_SEL, aud_spdif_parents, 0x090, 8, 1),
	MUX(CLK_TOP_DISP_PWM_SEL, disp_pwm_parents, 0x090, 16, 2),
	/* CLK_CFG_6 */
	MUX(CLK_TOP_DXCC_SEL, dxcc_parents, 0x0a0, 0, 2),
	MUX(CLK_TOP_SSUSB_SYS_SEL, ssusb_sys_parents, 0x0a0, 8, 2),
	MUX(CLK_TOP_SSUSB_XHCI_SEL, ssusb_sys_parents, 0x0a0, 16, 2),
	MUX(CLK_TOP_SPM_SEL, spm_parents, 0x0a0, 24, 1),
	/* CLK_CFG_7 */
	MUX(CLK_TOP_I2C_SEL, i2c_parents, 0x0b0, 0, 3),
	MUX(CLK_TOP_PWM_SEL, pwm_parents, 0x0b0, 8, 2),
	MUX(CLK_TOP_SENIF_SEL, senif_parents, 0x0b0, 16, 2),
	MUX(CLK_TOP_AES_FDE_SEL, aes_fde_parents, 0x0b0, 24, 3),
	/* CLK_CFG_8 */
	MUX(CLK_TOP_CAMTM_SEL, senif_parents, 0x0c0, 0, 2),
	MUX(CLK_TOP_DPI0_SEL, dpi0_parents, 0x0c0, 8, 3),
	MUX(CLK_TOP_DPI1_SEL, dpi0_parents, 0x0c0, 16, 3),
	MUX(CLK_TOP_DSP_SEL, dsp_parents, 0x0c0, 24, 3),
	/* CLK_CFG_9 */
	MUX(CLK_TOP_NFI2X_SEL, nfi2x_parents, 0x0d0, 0, 3),
	MUX(CLK_TOP_NFIECC_SEL, nfiecc_parents, 0x0d0, 8, 3),
	MUX(CLK_TOP_ECC_SEL, ecc_parents, 0x0d0, 16, 3),
	MUX(CLK_TOP_ETH_SEL, eth_parents, 0x0d0, 24, 3),
	/* CLK_CFG_10 */
	MUX(CLK_TOP_GCPU_SEL, gcpu_parents, 0x0e0, 0, 3),
	MUX(CLK_TOP_GCPU_CPM_SEL, gcpu_cpm_parents, 0x0e0, 8, 2),
	MUX(CLK_TOP_APU_SEL, apu_parents, 0x0e0, 16, 3),
	MUX(CLK_TOP_APU_IF_SEL, apu_parents, 0x0e0, 24, 3),
};

static const struct mtk_clk_tree mt8365_clk_tree = {
	.xtal_rate = 26 * MHZ,
	.xtal2_rate = 26 * MHZ,
	.fdivs_offs = CLK_TOP_SYSPLL_D2,
	.muxes_offs = CLK_TOP_AXI_SEL,
	.plls = apmixed_plls,
	.fclks = top_fixed_clks,
	.fdivs = top_fixed_divs,
	.muxes = top_muxes,
};

static const struct mtk_gate_regs infra2_cg_regs = {
	.set_ofs = 0x80,
	.clr_ofs = 0x84,
	.sta_ofs = 0x90,
};

static const struct mtk_gate_regs infra3_cg_regs = {
	.set_ofs = 0x88,
	.clr_ofs = 0x8c,
	.sta_ofs = 0x94,
};

static const struct mtk_gate_regs infra4_cg_regs = {
	.set_ofs = 0xa4,
	.clr_ofs = 0xa8,
	.sta_ofs = 0xac,
};

static const struct mtk_gate_regs infra5_cg_regs = {
	.set_ofs = 0xc0,
	.clr_ofs = 0xc4,
	.sta_ofs = 0xc8,
};

static const struct mtk_gate_regs infra6_cg_regs = {
	.set_ofs = 0xd0,
	.clr_ofs = 0xd4,
	.sta_ofs = 0xd8,
};

#define GATE_INFRA2(_id, _parent, _shift) {		\
		.id = _id,						\
		.parent = _parent,					\
		.regs = &infra2_cg_regs,				\
		.shift = _shift,					\
		.flags = CLK_GATE_SETCLR | CLK_PARENT_TOPCKGEN,		\
	}

#define GATE_INFRA3(_id, _parent, _shift) {		\
		.id = _id,						\
		.parent = _parent,					\
		.regs = &infra3_cg_regs,				\
		.shift = _shift,					\
		.flags = CLK_GATE_SETCLR | CLK_PARENT_TOPCKGEN,		\
	}

#define GATE_INFRA4(_id, _parent, _shift) {		\
		.id = _id,						\
		.parent = _parent,					\
		.regs = &infra4_cg_regs,				\
		.shift = _shift,					\
		.flags = CLK_GATE_SETCLR | CLK_PARENT_TOPCKGEN,		\
	}

#define GATE_INFRA5(_id, _parent, _shift) {		\
		.id = _id,						\
		.parent = _parent,					\
		.regs = &infra5_cg_regs,				\
		.shift = _shift,					\
		.flags = CLK_GATE_SETCLR | CLK_PARENT_TOPCKGEN,		\
	}

#define GATE_INFRA6(_id, _parent, _shift) {		\
		.id = _id,						\
		.parent = _parent,					\
		.regs = &infra6_cg_regs,				\
		.shift = _shift,					\
		.flags = CLK_GATE_SETCLR | CLK_PARENT_TOPCKGEN,		\
	}

static const struct mtk_gate infra_clks[] = {
	/* IFR2 */
	GATE_INFRA2(CLK_IFR_PMIC_TMR, CLK_TOP_CLK26M, 0),
	GATE_INFRA2(CLK_IFR_PMIC_AP, CLK_TOP_CLK26M, 1),
	GATE_INFRA2(CLK_IFR_PMIC_MD, CLK_TOP_CLK26M, 2),
	GATE_INFRA2(CLK_IFR_PMIC_CONN, CLK_TOP_CLK26M, 3),
	GATE_INFRA2(CLK_IFR_ICUSB, CLK_TOP_AXI_SEL, 8),
	GATE_INFRA2(CLK_IFR_GCE, CLK_TOP_AXI_SEL, 9),
	GATE_INFRA2(CLK_IFR_THERM, CLK_TOP_AXI_SEL, 10),
	GATE_INFRA2(CLK_IFR_PWM_HCLK, CLK_TOP_AXI_SEL, 15),
	GATE_INFRA2(CLK_IFR_PWM1, CLK_TOP_PWM_SEL, 16),
	GATE_INFRA2(CLK_IFR_PWM2, CLK_TOP_PWM_SEL, 17),
	GATE_INFRA2(CLK_IFR_PWM3, CLK_TOP_PWM_SEL, 18),
	GATE_INFRA2(CLK_IFR_PWM4, CLK_TOP_PWM_SEL, 19),
	GATE_INFRA2(CLK_IFR_PWM5, CLK_TOP_PWM_SEL, 20),
	GATE_INFRA2(CLK_IFR_PWM, CLK_TOP_PWM_SEL, 21),
	GATE_INFRA2(CLK_IFR_UART0, CLK_TOP_UART_SEL, 22),
	GATE_INFRA2(CLK_IFR_UART1, CLK_TOP_UART_SEL, 23),
	GATE_INFRA2(CLK_IFR_UART2, CLK_TOP_UART_SEL, 24),
	GATE_INFRA2(CLK_IFR_DSP_UART, CLK_TOP_UART_SEL, 26),
	GATE_INFRA2(CLK_IFR_GCE_26M, CLK_TOP_CLK26M, 27),
	GATE_INFRA2(CLK_IFR_CQ_DMA_FPC, CLK_TOP_AXI_SEL, 28),
	GATE_INFRA2(CLK_IFR_BTIF, CLK_TOP_AXI_SEL, 31),
	/* IFR3 */
	GATE_INFRA3(CLK_IFR_SPI0, CLK_TOP_SPI_SEL, 1),
	GATE_INFRA3(CLK_IFR_MSDC0_HCLK, CLK_TOP_MSDC50_0_HC_SEL, 2),
	GATE_INFRA3(CLK_IFR_MSDC2_HCLK, CLK_TOP_MSDC2_2_HC_SEL, 3),
	GATE_INFRA3(CLK_IFR_MSDC1_HCLK, CLK_TOP_AXI_SEL, 4),
	GATE_INFRA3(CLK_IFR_DVFSRC, CLK_TOP_CLK26M, 7),
	GATE_INFRA3(CLK_IFR_GCPU, CLK_TOP_AXI_SEL, 8),
	GATE_INFRA3(CLK_IFR_TRNG, CLK_TOP_AXI_SEL, 9),
	GATE_INFRA3(CLK_IFR_AUXADC, CLK_TOP_CLK26M, 10),
	GATE_INFRA3(CLK_IFR_AUXADC_MD, CLK_TOP_CLK26M, 14),
	GATE_INFRA3(CLK_IFR_AP_DMA, CLK_TOP_AXI_SEL, 18),
	GATE_INFRA3(CLK_IFR_DEBUGSYS, CLK_TOP_AXI_SEL, 24),
	GATE_INFRA3(CLK_IFR_AUDIO, CLK_TOP_AXI_SEL, 25),
	/* IFR4 */
	GATE_INFRA4(CLK_IFR_PWM_FBCLK6, CLK_TOP_PWM_SEL, 0),
	GATE_INFRA4(CLK_IFR_DISP_PWM, CLK_TOP_DISP_PWM_SEL, 2),
	GATE_INFRA4(CLK_IFR_AUD_26M_BK, CLK_TOP_CLK26M, 4),
	GATE_INFRA4(CLK_IFR_CQ_DMA, CLK_TOP_AXI_SEL, 27),
	/* IFR5 */
	GATE_INFRA5(CLK_IFR_MSDC0_SF, CLK_TOP_MSDC50_0_SEL, 0),
	GATE_INFRA5(CLK_IFR_MSDC1_SF, CLK_TOP_MSDC50_0_SEL, 1),
	GATE_INFRA5(CLK_IFR_MSDC2_SF, CLK_TOP_MSDC50_0_SEL, 2),
	GATE_INFRA5(CLK_IFR_AP_MSDC0, CLK_TOP_MSDC50_0_SEL, 7),
	GATE_INFRA5(CLK_IFR_MD_MSDC0, CLK_TOP_MSDC50_0_SEL, 8),
	GATE_INFRA5(CLK_IFR_MSDC0_SRC, CLK_TOP_MSDC50_0_SEL, 9),
	GATE_INFRA5(CLK_IFR_MSDC1_SRC, CLK_TOP_MSDC30_1_SEL, 10),
	GATE_INFRA5(CLK_IFR_MSDC2_SRC, CLK_TOP_MSDC50_2_SEL, 11),
	GATE_INFRA5(CLK_IFR_PWRAP_TMR, CLK_TOP_CLK26M, 12),
	GATE_INFRA5(CLK_IFR_PWRAP_SPI, CLK_TOP_CLK26M, 13),
	GATE_INFRA5(CLK_IFR_PWRAP_SYS, CLK_TOP_CLK26M, 14),
	GATE_INFRA5(CLK_IFR_IRRX_26M, CLK_TOP_CLK26M, 22),
	GATE_INFRA5(CLK_IFR_IRRX_32K, CLK_TOP_CLK32K, 23),
	GATE_INFRA5(CLK_IFR_I2C0_AXI, CLK_TOP_I2C_SEL, 24),
	GATE_INFRA5(CLK_IFR_I2C1_AXI, CLK_TOP_I2C_SEL, 25),
	GATE_INFRA5(CLK_IFR_I2C2_AXI, CLK_TOP_I2C_SEL, 26),
	GATE_INFRA5(CLK_IFR_I2C3_AXI, CLK_TOP_I2C_SEL, 27),
	GATE_INFRA5(CLK_IFR_NIC_AXI, CLK_TOP_AXI_SEL, 28),
	GATE_INFRA5(CLK_IFR_NIC_SLV_AXI, CLK_TOP_AXI_SEL, 29),
	GATE_INFRA5(CLK_IFR_APU_AXI, CLK_TOP_AXI_SEL, 30),
	/* IFR6 */
	GATE_INFRA6(CLK_IFR_NFIECC, CLK_TOP_NFIECC_SEL, 0),
	GATE_INFRA6(CLK_IFR_NFI1X_BK, CLK_TOP_NFI2X_SEL, 1),
	GATE_INFRA6(CLK_IFR_NFIECC_BK, CLK_TOP_NFI2X_SEL, 2),
	GATE_INFRA6(CLK_IFR_NFI_BK, CLK_TOP_AXI_SEL, 3),
	GATE_INFRA6(CLK_IFR_MSDC2_AP_BK, CLK_TOP_AXI_SEL, 4),
	GATE_INFRA6(CLK_IFR_MSDC2_MD_BK, CLK_TOP_AXI_SEL, 5),
	GATE_INFRA6(CLK_IFR_MSDC2_BK, CLK_TOP_AXI_SEL, 6),
	GATE_INFRA6(CLK_IFR_SUSB_133_BK, CLK_TOP_AXI_SEL, 7),
	GATE_INFRA6(CLK_IFR_SUSB_66_BK, CLK_TOP_AXI_SEL, 8),
	GATE_INFRA6(CLK_IFR_SSUSB_SYS, CLK_TOP_SSUSB_SYS_SEL, 9),
	GATE_INFRA6(CLK_IFR_SSUSB_REF, CLK_TOP_SSUSB_SYS_SEL, 10),
	GATE_INFRA6(CLK_IFR_SSUSB_XHCI, CLK_TOP_SSUSB_XHCI_SEL, 11),
};

static const struct mtk_gate_regs top0_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x0,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs top1_cg_regs = {
	.set_ofs = 0x104,
	.clr_ofs = 0x104,
	.sta_ofs = 0x104,
};

#define GATE_TOP0(_id, _parent, _shift) {	\
		.id = _id,				\
		.parent = _parent,			\
		.regs = &top0_cg_regs,			\
		.shift = _shift,			\
		.flags = CLK_GATE_NO_SETCLR | CLK_PARENT_TOPCKGEN,	\
	}

#define GATE_TOP1_I(_id, _parent, _shift) {	\
		.id = _id,				\
		.parent = _parent,			\
		.regs = &top1_cg_regs,			\
		.shift = _shift,			\
		.flags = CLK_GATE_NO_SETCLR_INV | CLK_PARENT_TOPCKGEN,	\
	}

static const struct mtk_gate top_clks[] = {
	/* TOP0 */
	GATE_TOP0(CLK_TOP_CONN_32K, CLK_TOP_CLK32K, 10),
	GATE_TOP0(CLK_TOP_CONN_26M, CLK_TOP_CLK26M, 11),
	GATE_TOP0(CLK_TOP_DSP_32K, CLK_TOP_CLK32K, 16),
	GATE_TOP0(CLK_TOP_DSP_26M, CLK_TOP_CLK26M, 17),
	/* TOP1 */
	GATE_TOP1_I(CLK_TOP_USB20_48M_EN, CLK_TOP_USB20_192M_D4, 8),
	GATE_TOP1_I(CLK_TOP_UNIVPLL_48M_EN, CLK_TOP_USB20_192M_D4, 9),
	GATE_TOP1_I(CLK_TOP_LVDSTX_CLKDIG_EN, CLK_TOP_LVDSTX_CLKDIG_CTS, 20),
	GATE_TOP1_I(CLK_TOP_VPLL_DPIX_EN, CLK_TOP_VPLL_DPIX, 21),
	GATE_TOP1_I(CLK_TOP_SSUSB_TOP_CK_EN, CLK_TOP_CLK_NULL, 22),
	GATE_TOP1_I(CLK_TOP_SSUSB_PHY_CK_EN, CLK_TOP_CLK_NULL, 23),
};

static int mt8365_apmixedsys_probe(struct udevice *dev)
{
	return mtk_common_clk_init(dev, &mt8365_clk_tree);
}

static int mt8365_topckgen_probe(struct udevice *dev)
{
	return mtk_common_clk_init(dev, &mt8365_clk_tree);
}

static int mt8365_topckgen_cg_probe(struct udevice *dev)
{
	return mtk_common_clk_gate_init(dev, &mt8365_clk_tree, top_clks);
}

static int mt8365_infracfg_probe(struct udevice *dev)
{
	return mtk_common_clk_gate_init(dev, &mt8365_clk_tree, infra_clks);
}

static const struct udevice_id mt8365_apmixed_compat[] = {
	{ .compatible = "mediatek,mt8365-apmixedsys", },
	{ }
};

static const struct udevice_id mt8365_topckgen_compat[] = {
	{ .compatible = "mediatek,mt8365-topckgen", },
	{ }
};

static const struct udevice_id mt8365_topckgen_cg_compat[] = {
	{ .compatible = "mediatek,mt8365-topckgen-cg", },
	{ }
};

static const struct udevice_id mt8365_infracfg_compat[] = {
	{ .compatible = "mediatek,mt8365-infracfg", },
	{ }
};

U_BOOT_DRIVER(mtk_clk_apmixedsys) = {
	.name = "mt8365-apmixedsys",
	.id = UCLASS_CLK,
	.of_match = mt8365_apmixed_compat,
	.probe = mt8365_apmixedsys_probe,
	.priv_auto = sizeof(struct mtk_clk_priv),
	.ops = &mtk_clk_apmixedsys_ops,
	.flags = DM_FLAG_PRE_RELOC,
};

U_BOOT_DRIVER(mtk_clk_topckgen) = {
	.name = "mt8365-topckgen",
	.id = UCLASS_CLK,
	.of_match = mt8365_topckgen_compat,
	.probe = mt8365_topckgen_probe,
	.priv_auto = sizeof(struct mtk_clk_priv),
	.ops = &mtk_clk_topckgen_ops,
	.flags = DM_FLAG_PRE_RELOC,
};

U_BOOT_DRIVER(mtk_clk_topckgen_cg) = {
	.name = "mt8365-topckgen-cg",
	.id = UCLASS_CLK,
	.of_match = mt8365_topckgen_cg_compat,
	.probe = mt8365_topckgen_cg_probe,
	.priv_auto = sizeof(struct mtk_clk_priv),
	.ops = &mtk_clk_gate_ops,
	.flags = DM_FLAG_PRE_RELOC,
};

U_BOOT_DRIVER(mtk_clk_infracfg) = {
	.name = "mt8365-infracfg",
	.id = UCLASS_CLK,
	.of_match = mt8365_infracfg_compat,
	.probe = mt8365_infracfg_probe,
	.priv_auto = sizeof(struct mtk_clk_priv),
	.ops = &mtk_clk_gate_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
