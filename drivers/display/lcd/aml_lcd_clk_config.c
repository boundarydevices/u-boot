/*
 * drivers/display/lcd/lcd_clk_config.c
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
*/

#include <common.h>
#include <malloc.h>
#include <asm/arch/gpio.h>
#include <amlogic/aml_lcd.h>
#include "aml_lcd_reg.h"
#include "aml_lcd_common.h"

static const unsigned int od_fb_table[2] = {1, 2};

static const unsigned int od_table[6] = {
	1, 2, 4, 8, 16, 32
};
static const unsigned int div_pre_table[6] = {
	1, 2, 3, 4, 5, 6
};
static const unsigned int pi_div_table[2] = {2, 4};

static char *lcd_clk_div_sel_table[] = {
	"1",
	"2",
	"3",
	"3.5",
	"3.75",
	"4",
	"5",
	"6",
	"6.25",
	"7",
	"7.5",
	"12",
	"14",
	"15",
	"2.5",
	"invalid",
};

static char *lcd_pll_ss_table_gxtvbb[] = {
	"0, disable",
	"1, +/-0.3%",
	"2, +/-0.5%",
	"3, +/-0.9%",
	"4, +/-1.2%",
};

static char *lcd_pll_ss_table_txl[] = {
	"0, disable",
	"1, +/-0.3%",
	"2, +/-0.4%",
	"3, +/-0.9%",
	"4, +/-1.2%",
};

static char *lcd_pll_ss_table_txlx[] = {
	"0, disable",
	"1, +/-0.3%",
	"2, +/-0.5%",
	"3, +/-1.0%",
	"4, +/-1.6%",
	"5, +/-3.0%",
};

static char *lcd_pll_ss_table_txhd[] = {
	"0, disable",
	"1, +/-0.3%",
	"2, +/-0.5%",
	"3, +/-1.0%",
	"4, +/-2.0%",
	"5, +/-3.0%",
};

static struct lcd_clk_config_s clk_conf = { /* unit: kHz */
	/* IN-OUT parameters */
	.fin = FIN_FREQ,
	.fout = 0,

	/* pll parameters */
	.pll_mode = 0, /* txl */
	.od_fb = 0,
	.pll_m = 0,
	.pll_n = 0,
	.pll_od1_sel = 0,
	.pll_od2_sel = 0,
	.pll_od3_sel = 0,
	.pll_pi_div_sel = 0, /* txhd */
	.pll_level = 0,
	.ss_level = 0,
	.div_sel = 0,
	.xd = 0,
	.pll_fout = 0,

	/* clk path node parameters */
	.pll_m_max = 0,
	.pll_m_min = 0,
	.pll_n_max = 0,
	.pll_n_min = 0,
	.pll_frac_range = 0,
	.pll_od_sel_max = 0,
	.ss_level_max = 0,
	.div_sel_max = 0,
	.xd_max = 0,
	.pll_ref_fmax = 0,
	.pll_ref_fmin = 0,
	.pll_vco_fmax = 0,
	.pll_vco_fmin = 0,
	.pll_out_fmax = 0,
	.pll_out_fmin = 0,
	.div_in_fmax = 0,
	.div_out_fmax = 0,
	.xd_out_fmax = 0,
};

struct lcd_clk_config_s *get_lcd_clk_config(void)
{
	return &clk_conf;
}

static void lcd_clk_config_init_print(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	switch (lcd_drv->chip_type) {
	case LCD_CHIP_AXG:
	case LCD_CHIP_G12A:
	case LCD_CHIP_G12B:
	case LCD_CHIP_SM1:
		LCDPR("lcd clk config init:\n"
			"pll_m_max:         %d\n"
			"pll_m_min:         %d\n"
			"pll_n_max:         %d\n"
			"pll_n_min:         %d\n"
			"pll_frac_range:    %d\n"
			"pll_od_sel_max:    %d\n"
			"ss_level_max:      %d\n"
			"pll_ref_fmax:      %d\n"
			"pll_ref_fmin:      %d\n"
			"pll_vco_fmax:      %d\n"
			"pll_vco_fmin:      %d\n"
			"pll_out_fmax:      %d\n"
			"pll_out_fmin:      %d\n"
			"xd_out_fmax:       %d\n\n",
			clk_conf.pll_m_max, clk_conf.pll_m_min,
			clk_conf.pll_n_max, clk_conf.pll_n_min,
			clk_conf.pll_frac_range,
			clk_conf.pll_od_sel_max, clk_conf.ss_level_max,
			clk_conf.pll_ref_fmax, clk_conf.pll_ref_fmin,
			clk_conf.pll_vco_fmax, clk_conf.pll_vco_fmin,
			clk_conf.pll_out_fmax, clk_conf.pll_out_fmin,
			 clk_conf.xd_out_fmax);
		break;
	default:
		LCDPR("lcd clk config:\n"
			"pll_m_max:         %d\n"
			"pll_m_min:         %d\n"
			"pll_n_max:         %d\n"
			"pll_n_min:         %d\n"
			"pll_frac_range:    %d\n"
			"pll_od_sel_max:    %d\n"
			"ss_level_max:      %d\n"
			"pll_ref_fmax:      %d\n"
			"pll_ref_fmin:      %d\n"
			"pll_vco_fmax:      %d\n"
			"pll_vco_fmin:      %d\n"
			"pll_out_fmax:      %d\n"
			"pll_out_fmin:      %d\n"
			"div_in_fmax:       %d\n"
			"div_out_fmax:      %d\n"
			"xd_out_fmax:       %d\n\n",
			clk_conf.pll_m_max, clk_conf.pll_m_min,
			clk_conf.pll_n_max, clk_conf.pll_n_min,
			clk_conf.pll_frac_range,
			clk_conf.pll_od_sel_max, clk_conf.ss_level_max,
			clk_conf.pll_ref_fmax, clk_conf.pll_ref_fmin,
			clk_conf.pll_vco_fmax, clk_conf.pll_vco_fmin,
			clk_conf.pll_out_fmax, clk_conf.pll_out_fmin,
			clk_conf.div_in_fmax, clk_conf.div_out_fmax,
			clk_conf.xd_out_fmax);
		break;
	}
}

void lcd_clk_config_print(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	switch (lcd_drv->chip_type) {
	case LCD_CHIP_G12A:
	case LCD_CHIP_G12B:
	case LCD_CHIP_SM1:
		if (lcd_drv->lcd_config->lcd_clk_path) {
			LCDPR("lcd clk config:\n"
				"clk_path      %d\n"
				"pll_m:        %d\n"
				"pll_n:        %d\n"
				"pll_frac:     0x%03x\n"
				"pll_fvco:     %dkHz\n"
				"pll_od:       %d\n"
				"pll_out:      %dkHz\n"
				"xd:           %d\n"
				"fout:         %dkHz\n"
				"ss_level:     %d\n\n",
				lcd_drv->lcd_config->lcd_clk_path,
				clk_conf.pll_m, clk_conf.pll_n,
				clk_conf.pll_frac, clk_conf.pll_fvco,
				clk_conf.pll_od1_sel, clk_conf.pll_fout,
				clk_conf.xd, clk_conf.fout, clk_conf.ss_level);
		} else {
			LCDPR("lcd clk config:\n"
				"clk_path        %d\n"
				"pll_m:          %d\n"
				"pll_n:          %d\n"
				"pll_frac:       0x%03x\n"
				"pll_fvco:       %dkHz\n"
				"pll_od1:        %d\n"
				"pll_od2:        %d\n"
				"pll_od3:        %d\n"
				"pll_out:        %dkHz\n"
				"div_sel:        %s(index %d)\n"
				"xd:             %d\n"
				"fout:           %dkHz\n"
				"ss_level:       %d\n\n",
				lcd_drv->lcd_config->lcd_clk_path,
				clk_conf.pll_m, clk_conf.pll_n,
				clk_conf.pll_frac, clk_conf.pll_fvco,
				clk_conf.pll_od1_sel, clk_conf.pll_od2_sel,
				clk_conf.pll_od3_sel, clk_conf.pll_fout,
				lcd_clk_div_sel_table[clk_conf.div_sel],
				clk_conf.div_sel, clk_conf.xd,
				clk_conf.fout, clk_conf.ss_level);
		}
		break;
	case LCD_CHIP_AXG:
		LCDPR("lcd clk config:\n"
			"pll_m:        %d\n"
			"pll_n:        %d\n"
			"pll_frac:     0x%03x\n"
			"pll_fvco:     %dkHz\n"
			"pll_od:       %d\n"
			"pll_out:      %dkHz\n"
			"xd:           %d\n"
			"fout:         %dkHz\n"
			"ss_level:     %d\n\n",
			clk_conf.pll_m, clk_conf.pll_n,
			clk_conf.pll_frac, clk_conf.pll_fvco,
			clk_conf.pll_od1_sel, clk_conf.pll_fout,
			clk_conf.xd, clk_conf.fout, clk_conf.ss_level);
		break;
	default:
		LCDPR("lcd clk config:\n"
			"pll_mode:       %d\n"
			"pll_m:          %d\n"
			"pll_n:          %d\n"
			"pll_frac:       0x%03x\n"
			"pll_fvco:       %dkHz\n"
			"pll_od1:        %d\n"
			"pll_od2:        %d\n"
			"pll_od3:        %d\n"
			"pll_pi_div_sel: %d\n"
			"pll_out:        %dkHz\n"
			"div_sel:        %s(index %d)\n"
			"xd:             %d\n"
			"fout:           %dkHz\n"
			"ss_level:       %d\n\n",
			clk_conf.pll_mode, clk_conf.pll_m, clk_conf.pll_n,
			clk_conf.pll_frac, clk_conf.pll_fvco,
			clk_conf.pll_od1_sel, clk_conf.pll_od2_sel,
			clk_conf.pll_od3_sel, clk_conf.pll_pi_div_sel,
			clk_conf.pll_fout,
			lcd_clk_div_sel_table[clk_conf.div_sel],
			clk_conf.div_sel, clk_conf.xd,
			clk_conf.fout, clk_conf.ss_level);
		break;
	}
}

static void lcd_clk_config_chip_init(void)
{
	struct lcd_clk_config_s *cConf;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	cConf = get_lcd_clk_config();
	switch (lcd_drv->chip_type) {
	case LCD_CHIP_GXTVBB:
		cConf->od_fb = PLL_FRAC_OD_FB_GXTVBB;
		cConf->ss_level_max = SS_LEVEL_MAX_GXTVBB;
		cConf->pll_m_max = PLL_M_MAX_GXTVBB;
		cConf->pll_m_min = PLL_M_MIN_GXTVBB;
		cConf->pll_n_max = PLL_N_MAX_GXTVBB;
		cConf->pll_n_min = PLL_N_MIN_GXTVBB;
		cConf->pll_frac_range = PLL_FRAC_RANGE_GXTVBB;
		cConf->pll_od_sel_max = PLL_OD_SEL_MAX_GXTVBB;
		cConf->pll_ref_fmax = PLL_FREF_MAX_GXTVBB;
		cConf->pll_ref_fmin = PLL_FREF_MIN_GXTVBB;
		cConf->pll_vco_fmax = PLL_VCO_MAX_GXTVBB;
		cConf->pll_vco_fmin = PLL_VCO_MIN_GXTVBB;
		cConf->pll_out_fmax = CLK_DIV_IN_MAX_GXTVBB;
		cConf->pll_out_fmin = cConf->pll_vco_fmin / 16;
		cConf->div_in_fmax = CLK_DIV_IN_MAX_GXTVBB;
		cConf->div_out_fmax = CRT_VID_CLK_IN_MAX_GXTVBB;
		cConf->xd_out_fmax = ENCL_CLK_IN_MAX_GXTVBB;
		break;
	case LCD_CHIP_GXL:
		cConf->od_fb = PLL_FRAC_OD_FB_GXL;
		cConf->ss_level_max = SS_LEVEL_MAX_GXL;
		cConf->pll_m_max = PLL_M_MAX_GXL;
		cConf->pll_m_min = PLL_M_MIN_GXL;
		cConf->pll_n_max = PLL_N_MAX_GXL;
		cConf->pll_n_min = PLL_N_MIN_GXL;
		cConf->pll_frac_range = PLL_FRAC_RANGE_GXL;
		cConf->pll_od_sel_max = PLL_OD_SEL_MAX_GXL;
		cConf->pll_ref_fmax = PLL_FREF_MAX_GXL;
		cConf->pll_ref_fmin = PLL_FREF_MIN_GXL;
		cConf->pll_vco_fmax = PLL_VCO_MAX_GXL;
		cConf->pll_vco_fmin = PLL_VCO_MIN_GXL;
		cConf->pll_out_fmax = CLK_DIV_IN_MAX_GXL;
		cConf->pll_out_fmin = cConf->pll_vco_fmin / 16;
		cConf->div_in_fmax = CLK_DIV_IN_MAX_GXL;
		cConf->div_out_fmax = CRT_VID_CLK_IN_MAX_GXL;
		cConf->xd_out_fmax = ENCL_CLK_IN_MAX_GXL;
		break;
	case LCD_CHIP_GXM:
		cConf->od_fb = PLL_FRAC_OD_FB_GXM;
		cConf->ss_level_max = SS_LEVEL_MAX_GXM;
		cConf->pll_m_max = PLL_M_MAX_GXM;
		cConf->pll_m_min = PLL_M_MIN_GXM;
		cConf->pll_n_max = PLL_N_MAX_GXM;
		cConf->pll_n_min = PLL_N_MIN_GXM;
		cConf->pll_frac_range = PLL_FRAC_RANGE_GXM;
		cConf->pll_od_sel_max = PLL_OD_SEL_MAX_GXM;
		cConf->pll_ref_fmax = PLL_FREF_MAX_GXM;
		cConf->pll_ref_fmin = PLL_FREF_MIN_GXM;
		cConf->pll_vco_fmax = PLL_VCO_MAX_GXM;
		cConf->pll_vco_fmin = PLL_VCO_MIN_GXM;
		cConf->pll_out_fmax = CLK_DIV_IN_MAX_GXM;
		cConf->pll_out_fmin = cConf->pll_vco_fmin / 16;
		cConf->div_in_fmax = CLK_DIV_IN_MAX_GXM;
		cConf->div_out_fmax = CRT_VID_CLK_IN_MAX_GXM;
		cConf->xd_out_fmax = ENCL_CLK_IN_MAX_GXM;
		break;
	case LCD_CHIP_TXL:
		cConf->od_fb = PLL_FRAC_OD_FB_TXL;
		cConf->ss_level_max = SS_LEVEL_MAX_TXL;
		cConf->pll_m_max = PLL_M_MAX_TXL;
		cConf->pll_m_min = PLL_M_MIN_TXL;
		cConf->pll_n_max = PLL_N_MAX_TXL;
		cConf->pll_n_min = PLL_N_MIN_TXL;
		cConf->pll_frac_range = PLL_FRAC_RANGE_TXL;
		cConf->pll_od_sel_max = PLL_OD_SEL_MAX_TXL;
		cConf->pll_ref_fmax = PLL_FREF_MAX_TXL;
		cConf->pll_ref_fmin = PLL_FREF_MIN_TXL;
		cConf->pll_vco_fmax = PLL_VCO_MAX_TXL;
		cConf->pll_vco_fmin = PLL_VCO_MIN_TXL;
		cConf->pll_out_fmax = CLK_DIV_IN_MAX_TXL;
		cConf->pll_out_fmin = cConf->pll_vco_fmin / 16;
		cConf->div_in_fmax = CLK_DIV_IN_MAX_TXL;
		cConf->div_out_fmax = CRT_VID_CLK_IN_MAX_TXL;
		cConf->xd_out_fmax = ENCL_CLK_IN_MAX_TXL;
		break;
	case LCD_CHIP_TXLX:
		cConf->od_fb = PLL_FRAC_OD_FB_TXLX;
		cConf->ss_level_max = SS_LEVEL_MAX_TXLX;
		cConf->pll_m_max = PLL_M_MAX_TXLX;
		cConf->pll_m_min = PLL_M_MIN_TXLX;
		cConf->pll_n_max = PLL_N_MAX_TXLX;
		cConf->pll_n_min = PLL_N_MIN_TXLX;
		cConf->pll_frac_range = PLL_FRAC_RANGE_TXLX;
		cConf->pll_od_sel_max = PLL_OD_SEL_MAX_TXLX;
		cConf->pll_ref_fmax = PLL_FREF_MAX_TXLX;
		cConf->pll_ref_fmin = PLL_FREF_MIN_TXLX;
		cConf->pll_vco_fmax = PLL_VCO_MAX_TXLX;
		cConf->pll_vco_fmin = PLL_VCO_MIN_TXLX;
		cConf->pll_out_fmax = CLK_DIV_IN_MAX_TXLX;
		cConf->pll_out_fmin = cConf->pll_vco_fmin / 16;
		cConf->div_in_fmax = CLK_DIV_IN_MAX_TXLX;
		cConf->div_out_fmax = CRT_VID_CLK_IN_MAX_TXLX;
		cConf->xd_out_fmax = ENCL_CLK_IN_MAX_TXLX;
		break;
	case LCD_CHIP_AXG:
		cConf->od_fb = PLL_FRAC_OD_FB_AXG;
		cConf->ss_level_max = SS_LEVEL_MAX_AXG;
		cConf->pll_m_max = PLL_M_MAX_AXG;
		cConf->pll_m_min = PLL_M_MIN_AXG;
		cConf->pll_n_max = PLL_N_MAX_AXG;
		cConf->pll_n_min = PLL_N_MIN_AXG;
		cConf->pll_frac_range = PLL_FRAC_RANGE_AXG;
		cConf->pll_od_sel_max = PLL_OD_SEL_MAX_AXG;
		cConf->pll_ref_fmax = PLL_FREF_MAX_AXG;
		cConf->pll_ref_fmin = PLL_FREF_MIN_AXG;
		cConf->pll_vco_fmax = PLL_VCO_MAX_AXG;
		cConf->pll_vco_fmin = PLL_VCO_MIN_AXG;
		cConf->pll_out_fmax = CRT_VID_CLK_IN_MAX_AXG;
		cConf->pll_out_fmin = cConf->pll_vco_fmin /
			od_table[cConf->pll_od_sel_max - 1];
		cConf->div_out_fmax = CRT_VID_CLK_IN_MAX_AXG;
		cConf->xd_out_fmax = ENCL_CLK_IN_MAX_AXG;
		break;
	case LCD_CHIP_TXHD:
		cConf->od_fb = PLL_FRAC_OD_FB_TXHD;
		cConf->ss_level_max = SS_LEVEL_MAX_TXHD;
		cConf->pll_m_max = PLL_M_MAX_TXHD;
		cConf->pll_m_min = PLL_M_MIN_TXHD;
		cConf->pll_n_max = PLL_N_MAX_TXHD;
		cConf->pll_n_min = PLL_N_MIN_TXHD;
		cConf->pll_frac_range = PLL_FRAC_RANGE_TXHD;
		cConf->pll_od_sel_max = PLL_OD_SEL_MAX_TXHD;
		cConf->pll_ref_fmax = PLL_FREF_MAX_TXHD;
		cConf->pll_ref_fmin = PLL_FREF_MIN_TXHD;
		cConf->pll_vco_fmax = PLL_VCO_MAX_TXHD;
		cConf->pll_vco_fmin = PLL_VCO_MIN_TXHD;
		cConf->pll_out_fmax = CLK_DIV_IN_MAX_TXHD;
		cConf->pll_out_fmin = cConf->pll_vco_fmin / 16;
		cConf->div_in_fmax = CLK_DIV_IN_MAX_TXHD;
		cConf->div_out_fmax = CRT_VID_CLK_IN_MAX_TXHD;
		cConf->xd_out_fmax = ENCL_CLK_IN_MAX_TXHD;
		break;
	case LCD_CHIP_G12A:
	case LCD_CHIP_G12B:
	case LCD_CHIP_SM1:
		if (lcd_drv->lcd_config->lcd_clk_path) {
			cConf->od_fb = PLL_FRAC_OD_FB_GP0_G12A;
			cConf->ss_level_max = SS_LEVEL_MAX_GP0_G12A;
			cConf->pll_frac_range = PLL_FRAC_RANGE_GP0_G12A;
			cConf->pll_od_sel_max = PLL_OD_SEL_MAX_GP0_G12A;
			cConf->pll_vco_fmax = PLL_VCO_MAX_GP0_G12A;
			cConf->pll_vco_fmin = PLL_VCO_MIN_GP0_G12A;
		} else {
			cConf->od_fb = PLL_FRAC_OD_FB_HPLL_G12A;
			cConf->ss_level_max = SS_LEVEL_MAX_HPLL_G12A;
			cConf->pll_frac_range = PLL_FRAC_RANGE_HPLL_G12A;
			cConf->pll_od_sel_max = PLL_OD_SEL_MAX_HPLL_G12A;
			cConf->pll_vco_fmax = PLL_VCO_MAX_HPLL_G12A;
			cConf->pll_vco_fmin = PLL_VCO_MIN_HPLL_G12A;
		}
		cConf->pll_m_max = PLL_M_MAX_G12A;
		cConf->pll_m_min = PLL_M_MIN_G12A;
		cConf->pll_n_max = PLL_N_MAX_G12A;
		cConf->pll_n_min = PLL_N_MIN_G12A;
		cConf->pll_ref_fmax = PLL_FREF_MAX_G12A;
		cConf->pll_ref_fmin = PLL_FREF_MIN_G12A;
		cConf->pll_out_fmax = CRT_VID_CLK_IN_MAX_G12A;
		cConf->pll_out_fmin = cConf->pll_vco_fmin / 16;
		cConf->div_out_fmax = CRT_VID_CLK_IN_MAX_G12A;
		cConf->xd_out_fmax = ENCL_CLK_IN_MAX_G12A;
		break;
	default:
		LCDPR("%s invalid chip type\n", __func__);
		break;
	}
	if (lcd_debug_print_flag > 0)
		lcd_clk_config_init_print();
}

/* **********************************
 * lcd controller operation
 * ********************************** */
static int lcd_pll_wait_lock(unsigned int reg, unsigned int lock_bit)
{
	unsigned int pll_lock;
	int wait_loop = PLL_WAIT_LOCK_CNT; /* 200 */
	int ret = 0;

	do {
		udelay(50);
		pll_lock = lcd_hiu_getb(reg, lock_bit, 1);
		wait_loop--;
	} while ((pll_lock == 0) && (wait_loop > 0));
	if (pll_lock == 0)
		ret = -1;
	LCDPR("%s: pll_lock=%d, wait_loop=%d\n",
		__func__, pll_lock, (PLL_WAIT_LOCK_CNT - wait_loop));
	return ret;
}

#define PLL_WAIT_LOCK_CNT_G12A    1000
static int lcd_pll_wait_lock_g12a(int path)
{
	unsigned int pll_ctrl, pll_ctrl3, pll_ctrl6;
	unsigned int pll_lock;
	int wait_loop = PLL_WAIT_LOCK_CNT_G12A; /* 200 */
	int ret = 0;

	if (path) {
		pll_ctrl = HHI_GP0_PLL_CNTL0;
		pll_ctrl3 = HHI_GP0_PLL_CNTL3;
		pll_ctrl6 = HHI_GP0_PLL_CNTL6;
	} else {
		pll_ctrl = HHI_HDMI_PLL_CNTL0;
		pll_ctrl3 = HHI_HDMI_PLL_CNTL3;
		pll_ctrl6 = HHI_HDMI_PLL_CNTL6;
	}
	do {
		udelay(50);
		pll_lock = lcd_hiu_getb(pll_ctrl, 31, 1);
		wait_loop--;
	} while ((pll_lock != 1) && (wait_loop > 0));

	if (pll_lock == 1) {
		goto pll_lock_end_g12a;
	} else {
		LCDPR("path: %d, pll try 1, lock: %d\n", path, pll_lock);
		lcd_hiu_setb(pll_ctrl3, 1, 31, 1);
		wait_loop = PLL_WAIT_LOCK_CNT_G12A;
		do {
			udelay(50);
			pll_lock = lcd_hiu_getb(pll_ctrl, 31, 1);
			wait_loop--;
		} while ((pll_lock != 1) && (wait_loop > 0));
	}

	if (pll_lock == 1) {
		goto pll_lock_end_g12a;
	} else {
		LCDPR("path: %d, pll try 2, lock: %d\n", path, pll_lock);
		lcd_hiu_write(pll_ctrl6, 0x55540000);
		wait_loop = PLL_WAIT_LOCK_CNT_G12A;
		do {
			udelay(50);
			pll_lock = lcd_hiu_getb(pll_ctrl, 31, 1);
			wait_loop--;
		} while ((pll_lock != 1) && (wait_loop > 0));
	}

	if (pll_lock != 1)
		ret = -1;

pll_lock_end_g12a:
	LCDPR("%s: path=%d, pll_lock=%d, wait_loop=%d\n",
		__func__, path, pll_lock, (PLL_WAIT_LOCK_CNT_G12A - wait_loop));
	return ret;
}

static void lcd_set_pll_ss_gxtvbb(struct lcd_clk_config_s *cConf)
{
	if ((cConf->pll_fvco >= 5500000) && (cConf->pll_fvco <= 6000000)) {
		switch (cConf->ss_level) {
		case 1: /* +/- 0.3% */
			lcd_hiu_write(HHI_HPLL_CNTL3, 0x12dc5080);
			lcd_hiu_write(HHI_HPLL_CNTL4, 0xb01da72c);
			lcd_hiu_write(HHI_HPLL_CNTL5, 0x51486980);
			lcd_hiu_write(HHI_HPLL_CNTL6, 0x00082a55);
			break;
		case 2: /* +/- 0.5% */
			lcd_hiu_write(HHI_HPLL_CNTL3, 0x12dc5080);
			lcd_hiu_write(HHI_HPLL_CNTL4, 0xa85da72c);
			lcd_hiu_write(HHI_HPLL_CNTL5, 0x51486980);
			lcd_hiu_write(HHI_HPLL_CNTL6, 0x00082a55);
			break;
		case 3: /* +/- 0.9% */
			lcd_hiu_write(HHI_HPLL_CNTL3, 0x12dc5080);
			lcd_hiu_write(HHI_HPLL_CNTL4, 0xb09da72c);
			lcd_hiu_write(HHI_HPLL_CNTL5, 0x51486980);
			lcd_hiu_write(HHI_HPLL_CNTL6, 0x00082a55);
			break;
		case 4: /* +/- 1.2% */
			lcd_hiu_write(HHI_HPLL_CNTL3, 0x12dc5080);
			lcd_hiu_write(HHI_HPLL_CNTL4, 0xb0dda72c);
			lcd_hiu_write(HHI_HPLL_CNTL5, 0x51486980);
			lcd_hiu_write(HHI_HPLL_CNTL6, 0x00082a55);
			break;
		default: /* disable */
			lcd_hiu_write(HHI_HPLL_CNTL3, 0x12dc5081);
			lcd_hiu_write(HHI_HPLL_CNTL4, 0x801da72c);
			lcd_hiu_write(HHI_HPLL_CNTL5, 0x71486980);
			lcd_hiu_write(HHI_HPLL_CNTL6, 0x00002a55);
			break;
		}
	} else {
		switch (cConf->ss_level) {
		case 1: /* +/- 0.3% */
			lcd_hiu_write(HHI_HPLL_CNTL3, 0x0d1c5090);
			lcd_hiu_write(HHI_HPLL_CNTL4, 0xb01da72c);
			lcd_hiu_write(HHI_HPLL_CNTL5, 0x51486980);
			lcd_hiu_write(HHI_HPLL_CNTL6, 0x00082a55);
			break;
		case 2: /* +/- 0.5% */
			lcd_hiu_write(HHI_HPLL_CNTL3, 0x0d1c5090);
			lcd_hiu_write(HHI_HPLL_CNTL4, 0xa85da72c);
			lcd_hiu_write(HHI_HPLL_CNTL5, 0x51486980);
			lcd_hiu_write(HHI_HPLL_CNTL6, 0x00082a55);
			break;
		case 3: /* +/- 0.9% */
			lcd_hiu_write(HHI_HPLL_CNTL3, 0x0d1c5090);
			lcd_hiu_write(HHI_HPLL_CNTL4, 0xb09da72c);
			lcd_hiu_write(HHI_HPLL_CNTL5, 0x51486980);
			lcd_hiu_write(HHI_HPLL_CNTL6, 0x00082a55);
			break;
		case 4: /* +/- 1.2% */
			lcd_hiu_write(HHI_HPLL_CNTL3, 0x0d1c5090);
			lcd_hiu_write(HHI_HPLL_CNTL4, 0xb0dda72c);
			lcd_hiu_write(HHI_HPLL_CNTL5, 0x51486980);
			lcd_hiu_write(HHI_HPLL_CNTL6, 0x00082a55);
			break;
		default: /* disable */
			lcd_hiu_write(HHI_HPLL_CNTL3, 0x0d5c5091);
			lcd_hiu_write(HHI_HPLL_CNTL4, 0x801da72c);
			lcd_hiu_write(HHI_HPLL_CNTL5, 0x71486980);
			lcd_hiu_write(HHI_HPLL_CNTL6, 0x00002a55);
			break;
		}
	}
	if ((lcd_debug_print_flag > 0) || (cConf->ss_level > 0)) {
		LCDPR("set pll spread spectrum: %s\n",
			lcd_pll_ss_table_gxtvbb[cConf->ss_level]);
	}
}

static void lcd_set_pll_gxtvbb(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl2;
	int ret;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);
	pll_ctrl = ((1 << LCD_PLL_EN_GXTVBB) |
		(1 << 27) | /* DPLL_BGP_EN */
		(cConf->pll_n << LCD_PLL_N_GXTVBB) |
		(cConf->pll_m << LCD_PLL_M_GXTVBB));

	pll_ctrl2 = ((cConf->pll_od3_sel << LCD_PLL_OD3_GXTVBB) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_GXTVBB) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_GXTVBB));
	pll_ctrl2 |= ((1 << 14) | (cConf->pll_frac << 0));

	lcd_hiu_write(HHI_HPLL_CNTL, pll_ctrl | (1 << LCD_PLL_RST_GXTVBB));
	lcd_hiu_write(HHI_HPLL_CNTL2, pll_ctrl2);
	if ((cConf->pll_fvco >= 5500000) && (cConf->pll_fvco <= 6000000)) {
		lcd_hiu_write(HHI_HPLL_CNTL3, 0x12dc5081);
		lcd_hiu_write(HHI_HPLL_CNTL4, 0x801da72c);
		lcd_hiu_write(HHI_HPLL_CNTL5, 0x71486980);
		lcd_hiu_write(HHI_HPLL_CNTL6, 0x00002a55);
	} else {
		lcd_hiu_write(HHI_HPLL_CNTL3, 0x0d5c5091);
		lcd_hiu_write(HHI_HPLL_CNTL4, 0x801da72c);
		lcd_hiu_write(HHI_HPLL_CNTL5, 0x71486980);
		lcd_hiu_write(HHI_HPLL_CNTL6, 0x00002a55);
	}
	lcd_hiu_write(HHI_HPLL_CNTL, pll_ctrl);

	ret = lcd_pll_wait_lock(HHI_HPLL_CNTL, LCD_PLL_LOCK_GXTVBB);
	if (ret)
		LCDERR("hpll lock failed\n");

	if (cConf->ss_level > 0)
		lcd_set_pll_ss_gxtvbb(cConf);
}

static void lcd_update_pll_frac_gxtvbb(struct lcd_clk_config_s *cConf)
{
	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	if (cConf->pll_frac > 0)
		lcd_hiu_setb(HHI_HPLL_CNTL2, cConf->pll_frac, 0, 12);
	else
		lcd_hiu_setb(HHI_HPLL_CNTL2, 0, 0, 12);
}

static void lcd_set_pll_ss_txl(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl3, pll_ctrl4;

	pll_ctrl3 = lcd_hiu_read(HHI_HPLL_CNTL3);
	pll_ctrl4 = lcd_hiu_read(HHI_HPLL_CNTL4);

	switch (cConf->ss_level) {
	case 1: /* +/- 0.3% */
		pll_ctrl3 &= ~(0xf << 10);
		pll_ctrl3 |= ((1 << 14) | (0xc << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		break;
	case 2: /* +/- 0.4% */
		pll_ctrl3 &= ~(0xf << 10);
		pll_ctrl3 |= ((1 << 14) | (0x8 << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (0x1 << 2);
		break;
	case 3: /* +/- 0.9% */
		pll_ctrl3 &= ~(0xf << 10);
		pll_ctrl3 |= ((1 << 14) | (0xc << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (0x2 << 2);
		break;
	case 4: /* +/- 1.2% */
		pll_ctrl3 &= ~(0xf << 10);
		pll_ctrl3 |= ((1 << 14) | (0xc << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (0x3 << 2);
		break;
	default: /* disable */
		pll_ctrl3 &= ~((0xf << 10) | (1 << 14));
		pll_ctrl4 &= ~(0x3 << 2);
		break;
	}
	lcd_hiu_write(HHI_HPLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HPLL_CNTL4, pll_ctrl4);

	LCDPR("set pll spread spectrum: %s\n",
		lcd_pll_ss_table_txl[cConf->ss_level]);
}

static void lcd_set_pll_txl(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl2, pll_ctrl3;
	int ret;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);
	pll_ctrl = ((1 << LCD_PLL_EN_TXL) |
		(cConf->pll_n << LCD_PLL_N_TXL) |
		(cConf->pll_m << LCD_PLL_M_TXL));
	pll_ctrl2 = 0x800ca000;
	pll_ctrl2 |= ((1 << 12) | (cConf->pll_frac << 0));
	pll_ctrl3 = 0x860330c4 | (cConf->od_fb << 30);
	pll_ctrl3 |= ((cConf->pll_od3_sel << LCD_PLL_OD3_TXL) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_TXL) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_TXL));

	lcd_hiu_write(HHI_HPLL_CNTL, pll_ctrl);
	lcd_hiu_write(HHI_HPLL_CNTL2, pll_ctrl2);
	lcd_hiu_write(HHI_HPLL_CNTL3, pll_ctrl3);
	if (cConf->pll_mode)
		lcd_hiu_write(HHI_HPLL_CNTL4, 0x0d160000);
	else
		lcd_hiu_write(HHI_HPLL_CNTL4, 0x0c8e0000);
	lcd_hiu_write(HHI_HPLL_CNTL5, 0x001fa729);
	lcd_hiu_write(HHI_HPLL_CNTL6, 0x01a31500);
	lcd_hiu_setb(HHI_HPLL_CNTL, 1, LCD_PLL_RST_TXL, 1);
	lcd_hiu_setb(HHI_HPLL_CNTL, 0, LCD_PLL_RST_TXL, 1);

	ret = lcd_pll_wait_lock(HHI_HPLL_CNTL, LCD_PLL_LOCK_TXL);
	if (ret)
		LCDERR("hpll lock failed\n");

	if (cConf->ss_level > 0)
		lcd_set_pll_ss_txl(cConf);
}

static void lcd_update_pll_frac_txl(struct lcd_clk_config_s *cConf)
{
	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	lcd_hiu_setb(HHI_HPLL_CNTL2, cConf->pll_frac, 0, 12);
}

static void lcd_set_pll_ss_custom_txlx(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl3, pll_ctrl4, pll_ctrl5;
	unsigned int dep_sel = 0, str_m = 0, m = 0, n = 0, temp;

	pll_ctrl3 = lcd_hiu_read(HHI_HPLL_CNTL3);
	pll_ctrl4 = lcd_hiu_read(HHI_HPLL_CNTL4);
	pll_ctrl5 = lcd_hiu_read(HHI_HPLL_CNTL5);

	if (cConf->ss_level > 60000)
		goto lcd_ss_txlx_custom;

	m = cConf->ss_level / LCD_SS_STEP_BASE;
	for (str_m = 0; str_m < 15; str_m += 2) {
		temp = ((str_m == 0) ? 1 : str_m);
		for (dep_sel = 1; dep_sel <= 15; dep_sel++) {
			if ((temp * dep_sel) == m) {
				n = 1;
				goto lcd_ss_txlx_custom;
			}
		}
	}
lcd_ss_txlx_custom:
	if (n == 0) {
		LCDERR("custom ss %dppm invalid, default disable\n",
			cConf->ss_level);
		cConf->ss_level = 0;
		pll_ctrl3 &= ~((0xf << 10) | (1 << 14));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl5 &= ~(0x3 << 30);
	} else {
		LCDPR("set pll spread spectrum: %dppm\n", cConf->ss_level);
		pll_ctrl3 &= ~(0xf << 10);
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
	}

	lcd_hiu_write(HHI_HPLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HPLL_CNTL4, pll_ctrl4);
	lcd_hiu_write(HHI_HPLL_CNTL5, pll_ctrl5);
}

static void lcd_set_pll_ss_txlx(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl3, pll_ctrl4, pll_ctrl5;
	unsigned int dep_sel = 0, str_m = 0;

	if (cConf->ss_level >= 1000) { /* custom */
		lcd_set_pll_ss_custom_txlx(cConf);
		return;
	}

	pll_ctrl3 = lcd_hiu_read(HHI_HPLL_CNTL3);
	pll_ctrl4 = lcd_hiu_read(HHI_HPLL_CNTL4);
	pll_ctrl5 = lcd_hiu_read(HHI_HPLL_CNTL5);

	cConf->ss_level = (cConf->ss_level >= cConf->ss_level_max) ?
		0 : cConf->ss_level;
	switch (cConf->ss_level) {
	case 1: /* +/- 0.3% */
		/* 12 * 500ppm * 1 */
		dep_sel = 12;
		str_m = 0;
		pll_ctrl3 &= ~(0xf << 10);
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
		break;
	case 2: /* +/- 0.5% */
		/* 10 * 500ppm * 2 */
		dep_sel = 10;
		str_m = 2;
		pll_ctrl3 &= ~(0xf << 10);
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
		break;
	case 3: /* +/- 1.0% */
		/* 10 * 500ppm * 4 */
		dep_sel = 10;
		str_m = 4;
		pll_ctrl3 &= ~(0xf << 10);
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
		break;
	case 4: /* +/- 1.6% */
		/* 8 * 500ppm * 8 */
		dep_sel = 8;
		str_m = 8;
		pll_ctrl3 &= ~(0xf << 10);
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
		break;
	case 5: /* +/- 3.0% */
		/* 12 * 500ppm * 10 */
		dep_sel = 12;
		str_m = 10;
		pll_ctrl3 &= ~(0xf << 10);
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
		break;
	default: /* disable */
		pll_ctrl3 &= ~((0xf << 10) | (1 << 14));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		break;
	}
	lcd_hiu_write(HHI_HPLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HPLL_CNTL4, pll_ctrl4);
	lcd_hiu_write(HHI_HPLL_CNTL5, pll_ctrl5);

	LCDPR("set pll spread spectrum: %s\n",
		lcd_pll_ss_table_txlx[cConf->ss_level]);
}

static void lcd_set_pll_txlx(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl2, pll_ctrl3;
	int ret;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);
	pll_ctrl = ((1 << LCD_PLL_EN_TXLX) |
		(cConf->pll_n << LCD_PLL_N_TXLX) |
		(cConf->pll_m << LCD_PLL_M_TXLX));
	pll_ctrl2 = 0x800ca000;
	pll_ctrl2 |= ((1 << 12) | (cConf->pll_frac << 0));
	pll_ctrl3 = 0x860030c4 | (cConf->od_fb << 30);
	pll_ctrl3 |= ((cConf->pll_od3_sel << LCD_PLL_OD3_TXLX) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_TXLX) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_TXLX));

	lcd_hiu_write(HHI_HPLL_CNTL, pll_ctrl);
	lcd_hiu_write(HHI_HPLL_CNTL2, pll_ctrl2);
	lcd_hiu_write(HHI_HPLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HPLL_CNTL4, 0x0c8e0000);
	lcd_hiu_write(HHI_HPLL_CNTL5, 0x001fa729);
	lcd_hiu_write(HHI_HPLL_CNTL6, 0x01a31500);
	lcd_hiu_setb(HHI_HPLL_CNTL, 1, LCD_PLL_RST_TXLX, 1);
	lcd_hiu_setb(HHI_HPLL_CNTL, 0, LCD_PLL_RST_TXLX, 1);

	ret = lcd_pll_wait_lock(HHI_HPLL_CNTL, LCD_PLL_LOCK_TXLX);
	if (ret)
		LCDERR("hpll lock failed\n");

	if (cConf->ss_level > 0)
		lcd_set_pll_ss_txlx(cConf);
}

static void lcd_set_pll_ss_txhd(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl3, pll_ctrl4, pll_ctrl5;
	unsigned int dep_sel = 0, str_m = 0;

	pll_ctrl3 = lcd_hiu_read(HHI_HPLL_CNTL3);
	pll_ctrl4 = lcd_hiu_read(HHI_HPLL_CNTL4);
	pll_ctrl5 = lcd_hiu_read(HHI_HPLL_CNTL5);

	switch (cConf->ss_level) {
	case 1: /* +/- 0.3% */
		/* 6 * 500ppm * 1 */
		dep_sel = 6;
		str_m = 1;
		pll_ctrl3 &= ~((0xf << 10) | (1 << 18));
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
		break;
	case 2: /* +/- 0.5% */
		/* 10 * 500ppm * 2 */
		dep_sel = 10;
		str_m = 2;
		pll_ctrl3 &= ~((0xf << 10) | (1 << 18));
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
		break;
	case 3: /* +/- 1.0% */
		/* 10 * 500ppm * 4 */
		dep_sel = 10;
		str_m = 4;
		pll_ctrl3 &= ~((0xf << 10) | (1 << 18));
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
		break;
	case 4: /* +/- 2% */
		/* 10 * 500ppm * 8 */
		dep_sel = 8;
		str_m = 8;
		pll_ctrl3 &= ~((0xf << 10) | (1 << 18));
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
		break;
	case 5: /* +/- 3.0% */
		/* 12 * 500ppm * 10 */
		dep_sel = 12;
		str_m = 10;
		pll_ctrl3 &= ~((0xf << 10) | (1 << 18));
		pll_ctrl3 |= ((1 << 14) | ((dep_sel & 0xf) << 10));
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl4 |= (((str_m >> 0) & 0x3) << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		pll_ctrl5 |= (((str_m >> 2) & 0x3) << 30);
		break;
	default: /* disable */
		pll_ctrl3 &= ~((0xf << 10) | (1 << 14));
		pll_ctrl3 |= (1 << 18);
		pll_ctrl4 &= ~(0x3 << 2);
		pll_ctrl5 &= ~(0x3 << 30);
		break;
	}
	lcd_hiu_write(HHI_HPLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HPLL_CNTL4, pll_ctrl4);
	lcd_hiu_write(HHI_HPLL_CNTL5, pll_ctrl5);

	LCDPR("set pll spread spectrum: %s\n",
		lcd_pll_ss_table_txhd[cConf->ss_level]);
}

static void lcd_set_pll_txhd(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl2, pll_ctrl3, pll_ctrl6;
	int ret;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);
	pll_ctrl = ((1 << LCD_PLL_EN_TXHD) |
		(cConf->pll_n << LCD_PLL_N_TXHD) |
		(cConf->pll_m << LCD_PLL_M_TXHD));
	if (cConf->pll_fvco > 4500000)
		pll_ctrl2 = 0x800ca000; /* bit[18]=1 */
	else
		pll_ctrl2 = 0x8008a000; /* bit[18]=0 */
	pll_ctrl2 |= ((1 << 12) | (cConf->pll_frac << 0));
	pll_ctrl3 = 0x860730c4 | (cConf->od_fb << 30);
	pll_ctrl3 |= ((cConf->pll_od3_sel << LCD_PLL_OD3_TXHD) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_TXHD) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_TXHD));
	pll_ctrl6 = (0x01a31500 | (cConf->pll_pi_div_sel << 30));

	lcd_hiu_write(HHI_HPLL_CNTL, pll_ctrl);
	lcd_hiu_write(HHI_HPLL_CNTL2, pll_ctrl2);
	lcd_hiu_write(HHI_HPLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HPLL_CNTL4, 0x0a960000); /* 0x0c8e0000 */
	lcd_hiu_write(HHI_HPLL_CNTL5, 0x001fa729);
	lcd_hiu_write(HHI_HPLL_CNTL6, pll_ctrl6);
	lcd_hiu_setb(HHI_HPLL_CNTL, 1, LCD_PLL_RST_TXHD, 1);
	lcd_hiu_setb(HHI_HPLL_CNTL, 0, LCD_PLL_RST_TXHD, 1);

	ret = lcd_pll_wait_lock(HHI_HPLL_CNTL, LCD_PLL_LOCK_TXHD);
	if (ret)
		LCDERR("hpll lock failed\n");

	if (cConf->ss_level > 0)
		lcd_set_pll_ss_txhd(cConf);
}

/*GP0_PLL_CNTL	= 0x40010250  ********Enable PLL
  GP0_PLL_CNTL1 = 0xc084a000  ********Set PLL parameter
  GP0_PLL_CNTL2 = 0xb75020be  ********Set PLL parameter
  GP0_PLL_CNTL3 = 0x0a59a288  ********Set PLL parameter
  GP0_PLL_CNTL4 = 0xc000004d  ********Set PLL parameter
  GP0_PLL_CNTL5 = 0x00058000  ********Set PLL parameter
  GP0_PLL_CNTL	= 0x60010250  ********Enable PLL and Reset
  GP0_PLL_CNTL	= 0x40010250  ********Get rid of Reset*/
static void lcd_set_pll_axg(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl1, pll_ctrl2;
	int ret;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	pll_ctrl = ((1 << LCD_PLL_EN_AXG) |
		(cConf->pll_n << LCD_PLL_N_AXG) |
		(cConf->pll_m << LCD_PLL_M_AXG) |
		(cConf->pll_od1_sel << LCD_PLL_OD_AXG));
	pll_ctrl1 = 0xc084a000;
	pll_ctrl1 |= ((1 << 12) | (cConf->pll_frac << 0));
	pll_ctrl2 = 0xb75020be | (cConf->od_fb << 19);

	lcd_hiu_write(HHI_GP0_PLL_CNTL, pll_ctrl);
	lcd_hiu_write(HHI_GP0_PLL_CNTL1, pll_ctrl1);
	lcd_hiu_write(HHI_GP0_PLL_CNTL2, pll_ctrl2);
	lcd_hiu_write(HHI_GP0_PLL_CNTL3, 0x0a59a288);
	lcd_hiu_write(HHI_GP0_PLL_CNTL4, 0xc000004d);
	if (cConf->pll_fvco >= 1632000)
		lcd_hiu_write(HHI_GP0_PLL_CNTL5, 0x00058000);
	else
		lcd_hiu_write(HHI_GP0_PLL_CNTL5, 0x00078000);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL, 1, LCD_PLL_RST_AXG, 1);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL, 0, LCD_PLL_RST_AXG, 1);

	ret = lcd_pll_wait_lock(HHI_GP0_PLL_CNTL, LCD_PLL_LOCK_AXG);
	if (ret)
		LCDERR("gp0_pll lock failed\n");

}

static void lcd_update_pll_frac_axg(struct lcd_clk_config_s *cConf)
{
	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	lcd_hiu_setb(HHI_GP0_PLL_CNTL1, cConf->pll_frac, 0, 12);
}

/*3.3	GP0 DPLL
For example:
Set DCO = 5592M
N=1 M=233 24*233=5592M OD<2:0>=100, GP0_CLK_OUT=5592/16= 349.5MHz

GP0_PLL_CNTL= 0x380404e9        ********Enable PLL
GP0_PLL_CNTL1 =0x0  ********Set PLL parameter
GP0_PLL_CNTL2 =0x0?    ********Set PLL parameter
GP0_PLL_CNTL3 =0x08691c00    ********Set PLL parameter
GP0_PLL_CNTL4=0x33771291       ********Set PLL parameter
GP0_PLL_CNTL5 =0x39270000    ********Set PLL parameter
GP0_PLL_CNTL6= 0x50540000;
GP0_PLL_CNTL = 0x180404e9     ********Enable PLL and Reset(M value is depended on application)*/
static void lcd_set_gp0_pll_g12a(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl1, pll_ctrl3, pll_ctrl4, pll_ctrl6;
	int ret;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	pll_ctrl = ((1 << LCD_PLL_EN_GP0_G12A) |
		(cConf->pll_n << LCD_PLL_N_GP0_G12A) |
		(cConf->pll_m << LCD_PLL_M_GP0_G12A) |
		(cConf->pll_od1_sel << LCD_PLL_OD_GP0_G12A));
	pll_ctrl1 = (cConf->pll_frac << 0);
	if (cConf->pll_frac) {
		pll_ctrl |= (1 << 27);
		pll_ctrl3 = 0x6a285c00;
		pll_ctrl4 = 0x65771290;
		pll_ctrl6 = 0x56540000;
	} else {
		pll_ctrl3 = 0x48681c00;
		pll_ctrl4 = 0x33771290;
		pll_ctrl6 = 0x56540000;
	}

	lcd_hiu_write(HHI_GP0_PLL_CNTL0, pll_ctrl);
	lcd_hiu_write(HHI_GP0_PLL_CNTL1, pll_ctrl1);
	lcd_hiu_write(HHI_GP0_PLL_CNTL2, 0x00);
	lcd_hiu_write(HHI_GP0_PLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_GP0_PLL_CNTL4, pll_ctrl4);
	lcd_hiu_write(HHI_GP0_PLL_CNTL5, 0x39272000);
	lcd_hiu_write(HHI_GP0_PLL_CNTL6, pll_ctrl6);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL0, 1, LCD_PLL_RST_GP0_G12A, 1);
	udelay(100);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL0, 0, LCD_PLL_RST_GP0_G12A, 1);

	ret = lcd_pll_wait_lock_g12a(1);
	if (ret)
		LCDERR("gp0_pll lock failed\n");
}

static void lcd_update_gp0_pll_frac_g12a(struct lcd_clk_config_s *cConf)
{
	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	lcd_hiu_setb(HHI_GP0_PLL_CNTL1, cConf->pll_frac, 0, 19);
}

static void lcd_set_hpll_g12a(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl1, pll_ctrl3, pll_ctrl4, pll_ctrl6;
	int ret;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	pll_ctrl = ((1 << LCD_PLL_EN_HPLL_G12A) |
		(1 << 25) | /* clk out gate */
		(cConf->pll_n << LCD_PLL_N_HPLL_G12A) |
		(cConf->pll_m << LCD_PLL_M_HPLL_G12A) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_HPLL_G12A) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_HPLL_G12A) |
		(cConf->pll_od3_sel << LCD_PLL_OD3_HPLL_G12A));
	pll_ctrl1 = (cConf->pll_frac << 0);
	if (cConf->pll_frac) {
		pll_ctrl |= (1 << 27);
		pll_ctrl3 = 0x6a285c00;
		pll_ctrl4 = 0x65771290;
		pll_ctrl6 = 0x56540000;
	} else {
		pll_ctrl3 = 0x48681c00;
		pll_ctrl4 = 0x33771290;
		pll_ctrl6 = 0x56540000;
	}

	lcd_hiu_write(HHI_HDMI_PLL_CNTL0, pll_ctrl);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL1, pll_ctrl1);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL2, 0x00);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL4, pll_ctrl4);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL5, 0x39272000);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL6, pll_ctrl6);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL0, 1, LCD_PLL_RST_HPLL_G12A, 1);
	udelay(100);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL0, 0, LCD_PLL_RST_HPLL_G12A, 1);

	ret = lcd_pll_wait_lock_g12a(0);
	if (ret)
		LCDERR("hpll lock failed\n");
}

static void lcd_update_hpll_frac_g12a(struct lcd_clk_config_s *cConf)
{
	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	lcd_hiu_setb(HHI_HDMI_PLL_CNTL1, cConf->pll_frac, 0, 19);
}

static void lcd_set_gp0_pll_g12b(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl1, pll_ctrl3, pll_ctrl4, pll_ctrl6;
	int ret;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	pll_ctrl = ((1 << LCD_PLL_EN_GP0_G12A) |
		(cConf->pll_n << LCD_PLL_N_GP0_G12A) |
		(cConf->pll_m << LCD_PLL_M_GP0_G12A) |
		(cConf->pll_od1_sel << LCD_PLL_OD_GP0_G12A));
	pll_ctrl1 = (cConf->pll_frac << 0);
	if (cConf->pll_frac) {
		pll_ctrl |= (1 << 27);
		pll_ctrl3 = 0x6a285c00;
		pll_ctrl4 = 0x65771290;
		pll_ctrl6 = 0x56540000;
	} else {
		pll_ctrl3 = 0x48681c00;
		pll_ctrl4 = 0x33771290;
		pll_ctrl6 = 0x56540000;
	}

	lcd_hiu_write(HHI_GP0_PLL_CNTL0, pll_ctrl);
	lcd_hiu_write(HHI_GP0_PLL_CNTL1, pll_ctrl1);
	lcd_hiu_write(HHI_GP0_PLL_CNTL2, 0x00);
	lcd_hiu_write(HHI_GP0_PLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_GP0_PLL_CNTL4, pll_ctrl4);
	lcd_hiu_write(HHI_GP0_PLL_CNTL5, 0x39272000);
	lcd_hiu_write(HHI_GP0_PLL_CNTL6, pll_ctrl6);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL0, 1, LCD_PLL_RST_GP0_G12A, 1);
	udelay(100);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL0, 0, LCD_PLL_RST_GP0_G12A, 1);

	ret = lcd_pll_wait_lock(HHI_GP0_PLL_CNTL0, LCD_PLL_LOCK_GP0_G12A);
	if (ret)
		LCDERR("gp0_pll lock failed\n");
}

static void lcd_set_hpll_g12b(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl1, pll_ctrl3, pll_ctrl4, pll_ctrl6;
	int ret;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	pll_ctrl = ((1 << LCD_PLL_EN_HPLL_G12A) |
		(1 << 25) | /* clk out gate */
		(cConf->pll_n << LCD_PLL_N_HPLL_G12A) |
		(cConf->pll_m << LCD_PLL_M_HPLL_G12A) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_HPLL_G12A) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_HPLL_G12A) |
		(cConf->pll_od3_sel << LCD_PLL_OD3_HPLL_G12A));
	pll_ctrl1 = (cConf->pll_frac << 0);
	if (cConf->pll_frac) {
		pll_ctrl |= (1 << 27);
		pll_ctrl3 = 0x6a285c00;
		pll_ctrl4 = 0x65771290;
		pll_ctrl6 = 0x56540000;
	} else {
		pll_ctrl3 = 0x48681c00;
		pll_ctrl4 = 0x33771290;
		pll_ctrl6 = 0x56540000;
	}

	lcd_hiu_write(HHI_HDMI_PLL_CNTL0, pll_ctrl);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL1, pll_ctrl1);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL2, 0x00);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL4, pll_ctrl4);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL5, 0x39272000);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL6, pll_ctrl6);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL0, 1, LCD_PLL_RST_HPLL_G12A, 1);
	udelay(100);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL0, 0, LCD_PLL_RST_HPLL_G12A, 1);

	ret = lcd_pll_wait_lock(HHI_HDMI_PLL_CNTL0, LCD_PLL_LOCK_HPLL_G12A);
	if (ret)
		LCDERR("hpll lock failed\n");
}

static unsigned int lcd_clk_div_g9_gxtvbb[][3] = {
	/* divider,        shift_val,  shift_sel */
	{CLK_DIV_SEL_1,    0xffff,     0,},
	{CLK_DIV_SEL_2,    0x0aaa,     0,},
	{CLK_DIV_SEL_3,    0x0db6,     0,},
	{CLK_DIV_SEL_3p5,  0x36cc,     1,},
	{CLK_DIV_SEL_3p75, 0x6666,     2,},
	{CLK_DIV_SEL_4,    0x0ccc,     0,},
	{CLK_DIV_SEL_5,    0x739c,     2,},
	{CLK_DIV_SEL_6,    0x0e38,     0,},
	{CLK_DIV_SEL_6p25, 0x0000,     3,},
	{CLK_DIV_SEL_7,    0x3c78,     1,},
	{CLK_DIV_SEL_7p5,  0x78f0,     2,},
	{CLK_DIV_SEL_12,   0x0fc0,     0,},
	{CLK_DIV_SEL_14,   0x3f80,     1,},
	{CLK_DIV_SEL_15,   0x7f80,     2,},
	{CLK_DIV_SEL_2p5,  0x5294,     2,},
	{CLK_DIV_SEL_MAX,  0xffff,     0,},
};

static void lcd_set_vid_pll_div(struct lcd_clk_config_s *cConf)
{
	unsigned int shift_val, shift_sel;
	int i;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 0, VCLK2_EN, 1);
	udelay(5);

	/* Disable the div output clock */
	lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 19, 1);
	lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 15, 1);

	i = 0;
	while (lcd_clk_div_g9_gxtvbb[i][0] != CLK_DIV_SEL_MAX) {
		if (cConf->div_sel == lcd_clk_div_g9_gxtvbb[i][0])
			break;
		i++;
	}
	if (lcd_clk_div_g9_gxtvbb[i][0] == CLK_DIV_SEL_MAX)
		LCDERR("invalid clk divider\n");
	shift_val = lcd_clk_div_g9_gxtvbb[i][1];
	shift_sel = lcd_clk_div_g9_gxtvbb[i][2];

	if (shift_val == 0xffff) { /* if divide by 1 */
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 1, 18, 1);
	} else {
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 18, 1);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 16, 2);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 15, 1);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 0, 14);

		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, shift_sel, 16, 2);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 1, 15, 1);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, shift_val, 0, 14);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 15, 1);
	}
	/* Enable the final output clock */
	lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 1, 19, 1);
}

static void lcd_set_vclk_crt(int lcd_type, struct lcd_clk_config_s *cConf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	/* setup the XD divider value */
	lcd_hiu_setb(HHI_VIID_CLK_DIV, (cConf->xd-1), VCLK2_XD, 8);
	udelay(5);

	/* select vid_pll_clk */
	switch (lcd_drv->chip_type) {
	case LCD_CHIP_G12A:
	case LCD_CHIP_G12B:
	case LCD_CHIP_SM1:
		if (lcd_drv->lcd_config->lcd_clk_path)
			lcd_hiu_setb(HHI_VIID_CLK_CNTL, 1, VCLK2_CLK_IN_SEL, 3);
		else
			lcd_hiu_setb(HHI_VIID_CLK_CNTL, 0, VCLK2_CLK_IN_SEL, 3);
		break;
	default:
		lcd_hiu_setb(HHI_VIID_CLK_CNTL, 0, VCLK2_CLK_IN_SEL, 3);
		break;
	}
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 1, VCLK2_EN, 1);
	udelay(2);

	/* [15:12] encl_clk_sel, select vclk2_div1 */
	lcd_hiu_setb(HHI_VIID_CLK_DIV, 8, ENCL_CLK_SEL, 4);
	/* release vclk2_div_reset and enable vclk2_div */
	lcd_hiu_setb(HHI_VIID_CLK_DIV, 1, VCLK2_XD_EN, 2);
	udelay(5);

	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 1, VCLK2_DIV1_EN, 1);
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 1, VCLK2_SOFT_RST, 1);
	udelay(10);
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 0, VCLK2_SOFT_RST, 1);
	udelay(5);

	/* enable CTS_ENCL clk gate */
	lcd_hiu_setb(HHI_VID_CLK_CNTL2, 1, ENCL_GATE_VCLK, 1);
}

static void lcd_set_dsi_meas_clk(void)
{
	lcd_hiu_setb(HHI_VDIN_MEAS_CLK_CNTL, 0, 21, 3);
	lcd_hiu_setb(HHI_VDIN_MEAS_CLK_CNTL, 0, 12, 7);
	lcd_hiu_setb(HHI_VDIN_MEAS_CLK_CNTL, 1, 20, 1);
}

static void lcd_set_dsi_phy_clk(int sel)
{
	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	lcd_hiu_setb(HHI_MIPIDSI_PHY_CLK_CNTL, sel, 12, 3);
	lcd_hiu_setb(HHI_MIPIDSI_PHY_CLK_CNTL, 1, 8, 1);
	lcd_hiu_setb(HHI_MIPIDSI_PHY_CLK_CNTL, 0, 0, 7);
}

static void lcd_set_tcon_clk(struct lcd_config_s *pconf)
{
	unsigned int val;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_LVDS:
		lcd_hiu_write(HHI_DIF_TCON_CNTL0, 0x0);
		lcd_hiu_write(HHI_DIF_TCON_CNTL0, 0x80000000);
		lcd_hiu_write(HHI_DIF_TCON_CNTL1, 0x0);
		lcd_hiu_write(HHI_DIF_TCON_CNTL2, 0x0);
		break;
	case LCD_MLVDS:
		val = pconf->lcd_control.mlvds_config->pi_clk_sel;
		/*val = (~val) & 0x3ff;*/
		lcd_hiu_write(HHI_DIF_TCON_CNTL0, (val << 12));
		lcd_hiu_write(HHI_DIF_TCON_CNTL0, ((1 << 31) | (val << 12)));

		val = pconf->lcd_control.mlvds_config->clk_phase & 0xfff;
		lcd_hiu_write(HHI_DIF_TCON_CNTL1, val);
		lcd_hiu_write(HHI_DIF_TCON_CNTL2, 0x0);

		/* tcon_clk 50M */
		lcd_hiu_write(HHI_TCON_CLK_CNTL, (1 << 7) | (1 << 6) | (7 << 0));
		break;
	default:
		break;
	}
}

static unsigned int clk_vid_pll_div_calc(unsigned int clk,
		unsigned int div_sel, int dir)
{
	unsigned int clk_ret;

	switch (div_sel) {
	case CLK_DIV_SEL_1:
		clk_ret = clk;
		break;
	case CLK_DIV_SEL_2:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 2;
		else
			clk_ret = clk * 2;
		break;
	case CLK_DIV_SEL_3:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 3;
		else
			clk_ret = clk * 3;
		break;
	case CLK_DIV_SEL_3p5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 2 / 7;
		else
			clk_ret = clk * 7 / 2;
		break;
	case CLK_DIV_SEL_3p75:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 4 / 15;
		else
			clk_ret = clk * 15 / 4;
		break;
	case CLK_DIV_SEL_4:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 4;
		else
			clk_ret = clk * 4;
		break;
	case CLK_DIV_SEL_5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 5;
		else
			clk_ret = clk * 5;
		break;
	case CLK_DIV_SEL_6:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 6;
		else
			clk_ret = clk * 6;
		break;
	case CLK_DIV_SEL_6p25:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 4 / 25;
		else
			clk_ret = clk * 25 / 4;
		break;
	case CLK_DIV_SEL_7:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 7;
		else
			clk_ret = clk * 7;
		break;
	case CLK_DIV_SEL_7p5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 2 / 15;
		else
			clk_ret = clk * 15 / 2;
		break;
	case CLK_DIV_SEL_12:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 12;
		else
			clk_ret = clk * 12;
		break;
	case CLK_DIV_SEL_14:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 14;
		else
			clk_ret = clk * 14;
		break;
	case CLK_DIV_SEL_15:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 15;
		else
			clk_ret = clk * 15;
		break;
	case CLK_DIV_SEL_2p5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 2 / 5;
		else
			clk_ret = clk * 5 / 2;
		break;
	default:
		clk_ret = clk;
		LCDERR("clk_div_sel: Invalid parameter\n");
		break;
	}

	return clk_ret;
}

static unsigned int clk_div_get_gxtvbb(unsigned int clk_div)
{
	unsigned int div_sel;

	/* div * 100 */
	switch (clk_div) {
	case 375:
		div_sel = CLK_DIV_SEL_3p75;
		break;
	case 750:
		div_sel = CLK_DIV_SEL_7p5;
		break;
	case 1500:
		div_sel = CLK_DIV_SEL_15;
		break;
	case 500:
		div_sel = CLK_DIV_SEL_5;
		break;
	default:
		div_sel = CLK_DIV_SEL_MAX;
		break;
	}
	return div_sel;
}

static int check_pll_gxtvbb(struct lcd_clk_config_s *cConf,
		unsigned int pll_fout)
{
	unsigned int m, n;
	unsigned int od1_sel, od2_sel, od3_sel, od1, od2, od3;
	unsigned int pll_fod2_in, pll_fod3_in, pll_fvco;
	unsigned int od_fb = 0, pll_frac;
	int done;

	done = 0;
	if ((pll_fout > cConf->pll_out_fmax) ||
		(pll_fout < cConf->pll_out_fmin)) {
		return done;
	}
	for (od3_sel = cConf->pll_od_sel_max; od3_sel > 0; od3_sel--) {
		od3 = od_table[od3_sel - 1];
		pll_fod3_in = pll_fout * od3;
		for (od2_sel = od3_sel; od2_sel > 0; od2_sel--) {
			od2 = od_table[od2_sel - 1];
			pll_fod2_in = pll_fod3_in * od2;
			for (od1_sel = od2_sel; od1_sel > 0; od1_sel--) {
				od1 = od_table[od1_sel - 1];
				pll_fvco = pll_fod2_in * od1;
				if ((pll_fvco < cConf->pll_vco_fmin) ||
					(pll_fvco > cConf->pll_vco_fmax)) {
					continue;
				}
				cConf->pll_od1_sel = od1_sel - 1;
				cConf->pll_od2_sel = od2_sel - 1;
				cConf->pll_od3_sel = od3_sel - 1;
				cConf->pll_fout = pll_fout;
				if (lcd_debug_print_flag == 2) {
					LCDPR("od1=%d, od2=%d, od3=%d\n",
						(od1_sel - 1), (od2_sel - 1),
						(od3_sel - 1));
					LCDPR("pll_fvco=%d\n", pll_fvco);
				}
				cConf->pll_fvco = pll_fvco;
				n = 1;
				od_fb = cConf->od_fb; /* pll default */
				pll_fvco = pll_fvco / od_fb_table[od_fb + 1];
				m = pll_fvco / cConf->fin;
				pll_frac = (pll_fvco % cConf->fin) *
					cConf->pll_frac_range / cConf->fin;
				cConf->pll_m = m;
				cConf->pll_n = n;
				cConf->pll_frac = pll_frac;
				if (lcd_debug_print_flag == 2) {
					LCDPR("m=%d, n=%d, frac=0x%x\n",
						m, n, pll_frac);
				}
				done = 1;
				break;
			}
		}
	}
	return done;
}

static void lcd_clk_generate_gxtvbb(struct lcd_config_s *pconf)
{
	unsigned int pll_fout;
	unsigned int clk_div_in, clk_div_out;
	unsigned int clk_div_sel, xd;
	struct lcd_clk_config_s *cConf;
	int done;

	done = 0;
	cConf = get_lcd_clk_config();
	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	cConf->err_fmin = MAX_ERROR;

	if (cConf->fout > cConf->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
		goto generate_clk_done_gxtvbb;
	}

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_TTL:
		clk_div_sel = CLK_DIV_SEL_1;
		cConf->xd_max = CRT_VID_DIV_MAX;
		for (xd = 1; xd <= cConf->xd_max; xd++) {
			clk_div_out = cConf->fout * xd;
			if (clk_div_out > cConf->div_out_fmax)
				continue;
			if (lcd_debug_print_flag == 2) {
				LCDPR("fout=%d, xd=%d, clk_div_out=%d\n",
					cConf->fout, xd, clk_div_out);
			}
			clk_div_in = clk_vid_pll_div_calc(clk_div_out,
					clk_div_sel, CLK_DIV_O2I);
			if (clk_div_in > cConf->div_in_fmax)
				continue;
			cConf->xd = xd;
			cConf->div_sel = clk_div_sel;
			pll_fout = clk_div_in;
			if (lcd_debug_print_flag == 2) {
				LCDPR("clk_div_sel=%s(index %d), pll_fout=%d\n",
					lcd_clk_div_sel_table[clk_div_sel],
					clk_div_sel, pll_fout);
			}
			done = check_pll_gxtvbb(cConf, pll_fout);
			if (done)
				goto generate_clk_done_gxtvbb;
		}
		break;
	case LCD_LVDS:
		clk_div_sel = CLK_DIV_SEL_7;
		xd = 1;
		clk_div_out = cConf->fout * xd;
		if (clk_div_out > cConf->div_out_fmax)
			goto generate_clk_done_gxtvbb;
		if (lcd_debug_print_flag == 2) {
			LCDPR("fout=%d, xd=%d, clk_div_out=%d\n",
				cConf->fout, xd, clk_div_out);
		}
		clk_div_in = clk_vid_pll_div_calc(clk_div_out,
				clk_div_sel, CLK_DIV_O2I);
		if (clk_div_in > cConf->div_in_fmax)
			goto generate_clk_done_gxtvbb;
		cConf->xd = xd;
		cConf->div_sel = clk_div_sel;
		pll_fout = clk_div_in;
		if (lcd_debug_print_flag == 2) {
			LCDPR("clk_div_sel=%s(index %d), pll_fout=%d\n",
				lcd_clk_div_sel_table[clk_div_sel],
				clk_div_sel, pll_fout);
		}
		done = check_pll_gxtvbb(cConf, pll_fout);
		if (done)
			goto generate_clk_done_gxtvbb;
		break;
	case LCD_VBYONE:
		cConf->div_sel_max = CLK_DIV_SEL_MAX;
		cConf->xd_max = CRT_VID_DIV_MAX;
		pll_fout = pconf->lcd_control.vbyone_config->bit_rate / 1000;
		clk_div_in = pll_fout;
		if (clk_div_in > cConf->div_in_fmax)
			goto generate_clk_done_gxtvbb;
		if (lcd_debug_print_flag == 2)
			LCDPR("pll_fout=%d\n", pll_fout);
		if ((clk_div_in / cConf->fout) > 15)
			cConf->xd = 4;
		else
			cConf->xd = 1;
		clk_div_out = cConf->fout * cConf->xd;
		if (lcd_debug_print_flag == 2) {
			LCDPR("clk_div_in=%d, fout=%d, xd=%d, clk_div_out=%d\n",
				clk_div_in, cConf->fout,
				clk_div_out, cConf->xd);
		}
		if (clk_div_out > cConf->div_out_fmax)
			goto generate_clk_done_gxtvbb;
		clk_div_sel = clk_div_get_gxtvbb(
				clk_div_in * 100 / clk_div_out);
		cConf->div_sel = clk_div_sel;
		if (lcd_debug_print_flag == 2) {
			LCDPR("clk_div_sel=%s(index %d)\n",
				lcd_clk_div_sel_table[clk_div_sel],
				cConf->div_sel);
		}
		done = check_pll_gxtvbb(cConf, pll_fout);
		break;
	default:
		break;
	}

generate_clk_done_gxtvbb:
	if (done) {
		pconf->lcd_timing.pll_ctrl =
			(cConf->pll_od1_sel << PLL_CTRL_OD1) |
			(cConf->pll_od2_sel << PLL_CTRL_OD2) |
			(cConf->pll_od3_sel << PLL_CTRL_OD3) |
			(cConf->pll_n << PLL_CTRL_N)         |
			(cConf->pll_m << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(cConf->div_sel << DIV_CTRL_DIV_SEL) |
			(cConf->xd << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl =
			(cConf->pll_frac << CLK_CTRL_FRAC);
	} else {
		pconf->lcd_timing.pll_ctrl =
			(1 << PLL_CTRL_OD1) |
			(1 << PLL_CTRL_OD2) |
			(1 << PLL_CTRL_OD3) |
			(1 << PLL_CTRL_N)   |
			(50 << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_DIV_SEL) |
			(7 << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (0 << CLK_CTRL_FRAC);
		LCDERR("Out of clock range, reset to default setting\n");
	}
}

static void lcd_pll_frac_generate_gxtvbb(struct lcd_config_s *pconf)
{
	unsigned int pll_fout;
	unsigned int clk_div_in, clk_div_out, clk_div_sel;
	unsigned int od1, od2, od3, pll_fvco;
	unsigned int m, n, od_fb, frac, offset, temp;
	struct lcd_clk_config_s *cConf;

	cConf = get_lcd_clk_config();
	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	clk_div_sel = cConf->div_sel;
	od1 = od_table[cConf->pll_od1_sel];
	od2 = od_table[cConf->pll_od2_sel];
	od3 = od_table[cConf->pll_od3_sel];
	m = cConf->pll_m;
	n = cConf->pll_n;

	if (lcd_debug_print_flag == 2) {
		LCDPR("m=%d, n=%d, od1=%d, od2=%d, od3=%d\n",
			m, n, cConf->pll_od1_sel, cConf->pll_od2_sel,
			cConf->pll_od3_sel);
		LCDPR("clk_div_sel=%s(index %d), xd=%d\n",
			lcd_clk_div_sel_table[clk_div_sel],
			clk_div_sel, cConf->xd);
	}
	if (cConf->fout > cConf->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pclk=%d\n", __func__, cConf->fout);

	clk_div_out = cConf->fout * cConf->xd;
	if (clk_div_out > cConf->div_out_fmax) {
		LCDERR("%s: wrong clk_div_out value %dkHz\n",
			__func__, clk_div_out);
		return;
	}

	clk_div_in =
		clk_vid_pll_div_calc(clk_div_out, clk_div_sel, CLK_DIV_O2I);
	if (clk_div_in > cConf->div_in_fmax) {
		LCDERR("%s: wrong clk_div_in value %dkHz\n",
			__func__, clk_div_in);
		return;
	}

	pll_fout = clk_div_in;
	if ((pll_fout > cConf->pll_out_fmax) ||
		(pll_fout < cConf->pll_out_fmin)) {
		LCDERR("%s: wrong pll_fout value %dkHz\n", __func__, pll_fout);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pll_fout=%d\n", __func__, pll_fout);

	pll_fvco = pll_fout * od1 * od2 * od3;
	if ((pll_fvco < cConf->pll_vco_fmin) ||
		(pll_fvco > cConf->pll_vco_fmax)) {
		LCDERR("%s: wrong pll_fvco value %dkHz\n", __func__, pll_fvco);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pll_fvco=%d\n", __func__, pll_fvco);

	cConf->pll_fvco = pll_fvco;
	od_fb = cConf->od_fb; /* pll default */
	pll_fvco = pll_fvco / od_fb_table[od_fb + 1];
	temp = cConf->fin * m / n;
	if (pll_fvco >= temp) {
		temp = pll_fvco - temp;
		offset = 0;
	} else {
		temp = temp - pll_fvco;
		offset = 1;
	}
	if (temp >= (2 * cConf->fin)) {
		LCDERR("%s: pll changing %dkHz is too much\n",
			__func__, temp);
		return;
	}
	frac = temp * cConf->pll_frac_range * n / cConf->fin;
	cConf->pll_frac = frac | (offset << 11);
	if (lcd_debug_print_flag)
		LCDPR("lcd_pll_frac_generate frac=0x%x\n", frac);
}

static int check_pll_txl(struct lcd_clk_config_s *cConf,
		unsigned int pll_fout)
{
	unsigned int m, n;
	unsigned int od1_sel, od2_sel, od3_sel, od1, od2, od3;
	unsigned int pll_fod2_in, pll_fod3_in, pll_fvco;
	unsigned int od_fb = 0, pll_frac;
	int done;

	done = 0;
	if ((pll_fout > cConf->pll_out_fmax) ||
		(pll_fout < cConf->pll_out_fmin)) {
		return done;
	}
	for (od3_sel = cConf->pll_od_sel_max; od3_sel > 0; od3_sel--) {
		od3 = od_table[od3_sel - 1];
		pll_fod3_in = pll_fout * od3;
		for (od2_sel = od3_sel; od2_sel > 0; od2_sel--) {
			od2 = od_table[od2_sel - 1];
			pll_fod2_in = pll_fod3_in * od2;
			for (od1_sel = od2_sel; od1_sel > 0; od1_sel--) {
				od1 = od_table[od1_sel - 1];
				pll_fvco = pll_fod2_in * od1;
				if ((pll_fvco < cConf->pll_vco_fmin) ||
					(pll_fvco > cConf->pll_vco_fmax)) {
					continue;
				}
				cConf->pll_od1_sel = od1_sel - 1;
				cConf->pll_od2_sel = od2_sel - 1;
				cConf->pll_od3_sel = od3_sel - 1;
				cConf->pll_fout = pll_fout;
				if (lcd_debug_print_flag == 2) {
					LCDPR("od1=%d, od2=%d, od3=%d\n",
						(od1_sel - 1), (od2_sel - 1),
						(od3_sel - 1));
					LCDPR("pll_fvco=%d\n", pll_fvco);
				}
				cConf->pll_fvco = pll_fvco;
				n = 1;
				od_fb = cConf->od_fb;
				pll_fvco = pll_fvco / od_fb_table[od_fb];
				m = pll_fvco / cConf->fin;
				pll_frac = (pll_fvco % cConf->fin) *
					cConf->pll_frac_range / cConf->fin;
				cConf->pll_m = m;
				cConf->pll_n = n;
				cConf->pll_frac = pll_frac;
				if (lcd_debug_print_flag == 2) {
					LCDPR("m=%d, n=%d, frac=0x%x\n",
						m, n, pll_frac);
				}
				done = 1;
				break;
			}
		}
	}
	return done;
}

static int check_pll_txhd_mlvds(struct lcd_clk_config_s *cConf,
		unsigned int pll_fvco)
{
	unsigned int m, n;
	unsigned int od_fb = 0, pll_frac;
	int done = 0;

	if ((pll_fvco < cConf->pll_vco_fmin) || (pll_fvco > cConf->pll_vco_fmax)) {
		if (lcd_debug_print_flag == 2)
			LCDPR("pll_fvco %d is out of range\n", pll_fvco);
		return done;
	}

	cConf->pll_fvco = pll_fvco;
	n = 1;
	od_fb = cConf->od_fb;
	pll_fvco = pll_fvco / od_fb_table[od_fb];
	m = pll_fvco / cConf->fin;
	pll_frac = (pll_fvco % cConf->fin) *
		cConf->pll_frac_range / cConf->fin;
	cConf->pll_m = m;
	cConf->pll_n = n;
	cConf->pll_frac = pll_frac;
	if (lcd_debug_print_flag == 2) {
		LCDPR("m=%d, n=%d, frac=0x%x, pll_fvco=%d\n",
			m, n, pll_frac, pll_fvco);
	}
	done = 1;

	return done;
}

static int check_abs(int a, int b)
{
	if (a >= b)
		return (a - b);
	else
		return (b - a);
}

#define PLL_FVCO_ERR_MAX    2 /* kHz */
static int check_pll_od_txhd_mlvds(struct lcd_clk_config_s *cConf,
		unsigned int pll_fout)
{
	unsigned int od1_sel, od2_sel, od3_sel, od1, od2, od3;
	unsigned int pll_fod2_in, pll_fod3_in, pll_fvco;
	int done = 0;

	if ((pll_fout > cConf->pll_out_fmax) ||
		(pll_fout < cConf->pll_out_fmin)) {
		return done;
	}
	for (od3_sel = cConf->pll_od_sel_max; od3_sel > 0; od3_sel--) {
		od3 = od_table[od3_sel - 1];
		pll_fod3_in = pll_fout * od3;
		for (od2_sel = od3_sel; od2_sel > 0; od2_sel--) {
			od2 = od_table[od2_sel - 1];
			pll_fod2_in = pll_fod3_in * od2;
			for (od1_sel = od2_sel; od1_sel > 0; od1_sel--) {
				od1 = od_table[od1_sel - 1];
				pll_fvco = pll_fod2_in * od1;
				if ((pll_fvco < cConf->pll_vco_fmin) ||
					(pll_fvco > cConf->pll_vco_fmax)) {
					continue;
				}
				if (check_abs(pll_fvco, cConf->pll_fvco) < PLL_FVCO_ERR_MAX) {
					cConf->pll_od1_sel = od1_sel - 1;
					cConf->pll_od2_sel = od2_sel - 1;
					cConf->pll_od3_sel = od3_sel - 1;
					cConf->pll_fout = pll_fout;

					if (lcd_debug_print_flag == 2) {
						LCDPR("od1=%d, od2=%d, od3=%d\n",
							(od1_sel - 1), (od2_sel - 1),
							(od3_sel - 1));
					}
					done = 1;
					break;
				}
			}
		}
	}
	return done;
}

static void lcd_clk_generate_txl(struct lcd_config_s *pconf)
{
	unsigned int pll_fout, pll_fvco, bit_rate;
	unsigned int clk_div_in, clk_div_out;
	unsigned int clk_div_sel, xd, pi_div_sel;
	struct lcd_clk_config_s *cConf;
	int done;

	done = 0;
	cConf = get_lcd_clk_config();
	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	cConf->err_fmin = MAX_ERROR;

	if (cConf->fout > cConf->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
		goto generate_clk_done_txl;
	}

	if (pconf->lcd_timing.clk_auto == 2)
		cConf->pll_mode = 1;
	else
		cConf->pll_mode = 0;

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_TTL:
		clk_div_sel = CLK_DIV_SEL_1;
		cConf->xd_max = CRT_VID_DIV_MAX;
		for (xd = 1; xd <= cConf->xd_max; xd++) {
			clk_div_out = cConf->fout * xd;
			if (clk_div_out > cConf->div_out_fmax)
				continue;
			if (lcd_debug_print_flag == 2) {
				LCDPR("fout=%d, xd=%d, clk_div_out=%d\n",
					cConf->fout, xd, clk_div_out);
			}
			clk_div_in = clk_vid_pll_div_calc(clk_div_out,
					clk_div_sel, CLK_DIV_O2I);
			if (clk_div_in > cConf->div_in_fmax)
				continue;
			cConf->xd = xd;
			cConf->div_sel = clk_div_sel;
			pll_fout = clk_div_in;
			if (lcd_debug_print_flag == 2) {
				LCDPR("clk_div_sel=%s(index %d), pll_fout=%d\n",
					lcd_clk_div_sel_table[clk_div_sel],
					clk_div_sel, pll_fout);
			}
			done = check_pll_txl(cConf, pll_fout);
			if (done)
				goto generate_clk_done_txl;
		}
		break;
	case LCD_LVDS:
		clk_div_sel = CLK_DIV_SEL_7;
		xd = 1;
		clk_div_out = cConf->fout * xd;
		if (clk_div_out > cConf->div_out_fmax)
			goto generate_clk_done_txl;
		if (lcd_debug_print_flag == 2) {
			LCDPR("fout=%d, xd=%d, clk_div_out=%d\n",
				cConf->fout, xd, clk_div_out);
		}
		clk_div_in = clk_vid_pll_div_calc(clk_div_out,
				clk_div_sel, CLK_DIV_O2I);
		if (clk_div_in > cConf->div_in_fmax)
			goto generate_clk_done_txl;
		cConf->xd = xd;
		cConf->div_sel = clk_div_sel;
		pll_fout = clk_div_in;
		if (lcd_debug_print_flag == 2) {
			LCDPR("clk_div_sel=%s(index %d), pll_fout=%d\n",
				lcd_clk_div_sel_table[clk_div_sel],
				clk_div_sel, pll_fout);
		}
		done = check_pll_txl(cConf, pll_fout);
		if (done)
			goto generate_clk_done_txl;
		break;
	case LCD_VBYONE:
		cConf->div_sel_max = CLK_DIV_SEL_MAX;
		cConf->xd_max = CRT_VID_DIV_MAX;
		pll_fout = pconf->lcd_control.vbyone_config->bit_rate / 1000;
		clk_div_in = pll_fout;
		if (clk_div_in > cConf->div_in_fmax)
			goto generate_clk_done_txl;
		if (lcd_debug_print_flag == 2)
			LCDPR("pll_fout=%d\n", pll_fout);
		if ((clk_div_in / cConf->fout) > 15)
			cConf->xd = 4;
		else
			cConf->xd = 1;
		clk_div_out = cConf->fout * cConf->xd;
		if (lcd_debug_print_flag == 2) {
			LCDPR("clk_div_in=%d, fout=%d, xd=%d, clk_div_out=%d\n",
				clk_div_in, cConf->fout,
				clk_div_out, cConf->xd);
		}
		if (clk_div_out > cConf->div_out_fmax)
			goto generate_clk_done_txl;
		clk_div_sel = clk_div_get_gxtvbb(
				clk_div_in * 100 / clk_div_out);
		cConf->div_sel = clk_div_sel;
		if (lcd_debug_print_flag == 2) {
			LCDPR("clk_div_sel=%s(index %d)\n",
				lcd_clk_div_sel_table[clk_div_sel],
				cConf->div_sel);
		}
		done = check_pll_txl(cConf, pll_fout);
		break;
	case LCD_MLVDS:
		bit_rate = pconf->lcd_control.mlvds_config->bit_rate / 1000;
		for (pi_div_sel = 0; pi_div_sel < 2; pi_div_sel++) {
			pll_fvco = bit_rate * pi_div_table[pi_div_sel] * 4;
			done = check_pll_txhd_mlvds(cConf, pll_fvco);
			if (done) {
				clk_div_sel = CLK_DIV_SEL_1;
				cConf->xd_max = CRT_VID_DIV_MAX;
				for (xd = 1; xd <= cConf->xd_max; xd++) {
					clk_div_out = cConf->fout * xd;
					if (clk_div_out > cConf->div_out_fmax)
						continue;
					if (lcd_debug_print_flag == 2) {
						LCDPR("fout=%d, xd=%d, clk_div_out=%d\n",
							cConf->fout, xd, clk_div_out);
					}
					clk_div_in = clk_vid_pll_div_calc(clk_div_out,
							clk_div_sel, CLK_DIV_O2I);
					if (clk_div_in > cConf->div_in_fmax)
						continue;
					cConf->xd = xd;
					cConf->div_sel = clk_div_sel;
					cConf->pll_pi_div_sel = pi_div_sel;
					pll_fout = clk_div_in;
					if (lcd_debug_print_flag == 2) {
						LCDPR("clk_div_sel=%s(index %d), pll_fout=%d\n",
							lcd_clk_div_sel_table[clk_div_sel],
							clk_div_sel, pll_fout);
						LCDPR("pi_clk_sel=%d\n", pi_div_sel);
					}
					done = check_pll_od_txhd_mlvds(cConf, pll_fout);
					if (done)
						goto generate_clk_done_txl;
				}
			}
		}
		break;
	default:
		break;
	}

generate_clk_done_txl:
	if (done) {
		pconf->lcd_timing.pll_ctrl =
			(cConf->pll_od1_sel << PLL_CTRL_OD1) |
			(cConf->pll_od2_sel << PLL_CTRL_OD2) |
			(cConf->pll_od3_sel << PLL_CTRL_OD3) |
			(cConf->pll_n << PLL_CTRL_N)         |
			(cConf->pll_m << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(cConf->div_sel << DIV_CTRL_DIV_SEL) |
			(cConf->xd << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl =
			(cConf->pll_frac << CLK_CTRL_FRAC);
	} else {
		pconf->lcd_timing.pll_ctrl =
			(1 << PLL_CTRL_OD1) |
			(1 << PLL_CTRL_OD2) |
			(1 << PLL_CTRL_OD3) |
			(1 << PLL_CTRL_N)   |
			(50 << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_DIV_SEL) |
			(7 << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (0 << CLK_CTRL_FRAC);
		LCDERR("Out of clock range, reset to default setting\n");
	}
}

static void lcd_pll_frac_generate_txl(struct lcd_config_s *pconf)
{
	unsigned int pll_fout;
	unsigned int clk_div_in, clk_div_out, clk_div_sel;
	unsigned int od1, od2, od3, pll_fvco;
	unsigned int m, n, od_fb, frac, offset, temp;
	struct lcd_clk_config_s *cConf;

	cConf = get_lcd_clk_config();
	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	clk_div_sel = cConf->div_sel;
	od1 = od_table[cConf->pll_od1_sel];
	od2 = od_table[cConf->pll_od2_sel];
	od3 = od_table[cConf->pll_od3_sel];
	m = cConf->pll_m;
	n = cConf->pll_n;

	if (lcd_debug_print_flag == 2) {
		LCDPR("m=%d, n=%d, od1=%d, od2=%d, od3=%d\n",
			m, n, cConf->pll_od1_sel, cConf->pll_od2_sel,
			cConf->pll_od3_sel);
		LCDPR("clk_div_sel=%s(index %d), xd=%d\n",
			lcd_clk_div_sel_table[clk_div_sel],
			clk_div_sel, cConf->xd);
	}
	if (cConf->fout > cConf->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pclk=%d\n", __func__, cConf->fout);

	clk_div_out = cConf->fout * cConf->xd;
	if (clk_div_out > cConf->div_out_fmax) {
		LCDERR("%s: wrong clk_div_out value %dkHz\n",
			__func__, clk_div_out);
		return;
	}

	clk_div_in =
		clk_vid_pll_div_calc(clk_div_out, clk_div_sel, CLK_DIV_O2I);
	if (clk_div_in > cConf->div_in_fmax) {
		LCDERR("%s: wrong clk_div_in value %dkHz\n",
			__func__, clk_div_in);
		return;
	}

	pll_fout = clk_div_in;
	if ((pll_fout > cConf->pll_out_fmax) ||
		(pll_fout < cConf->pll_out_fmin)) {
		LCDERR("%s: wrong pll_fout value %dkHz\n", __func__, pll_fout);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pll_fout=%d\n", __func__, pll_fout);

	pll_fvco = pll_fout * od1 * od2 * od3;
	if ((pll_fvco < cConf->pll_vco_fmin) ||
		(pll_fvco > cConf->pll_vco_fmax)) {
		LCDERR("%s: wrong pll_fvco value %dkHz\n", __func__, pll_fvco);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pll_fvco=%d\n", __func__, pll_fvco);

	cConf->pll_fvco = pll_fvco;
	od_fb = cConf->od_fb;
	pll_fvco = pll_fvco / od_fb_table[od_fb];
	temp = cConf->fin * m / n;
	if (pll_fvco >= temp) {
		temp = pll_fvco - temp;
		offset = 0;
	} else {
		temp = temp - pll_fvco;
		offset = 1;
	}
	if (temp >= (2 * cConf->fin)) {
		LCDERR("%s: pll changing %dkHz is too much\n",
			__func__, temp);
		return;
	}
	frac = temp * cConf->pll_frac_range * n / cConf->fin;
	cConf->pll_frac = frac | (offset << 11);
	if (lcd_debug_print_flag)
		LCDPR("lcd_pll_frac_generate: frac=0x%x\n", frac);
}

static int check_pll_axg(struct lcd_clk_config_s *cConf,
		unsigned int pll_fout)
{
	unsigned int m, n, od_sel, od;
	unsigned int pll_fvco;
	unsigned int od_fb = 0, pll_frac;
	int done = 0;

	if ((pll_fout > cConf->pll_out_fmax) ||
		(pll_fout < cConf->pll_out_fmin)) {
		return done;
	}
	for (od_sel = cConf->pll_od_sel_max; od_sel > 0; od_sel--) {
		od = od_table[od_sel - 1];
		pll_fvco = pll_fout * od;
		if ((pll_fvco < cConf->pll_vco_fmin) ||
			(pll_fvco > cConf->pll_vco_fmax)) {
			continue;
		}
		cConf->pll_od1_sel = od_sel - 1;
		cConf->pll_fout = pll_fout;
		if (lcd_debug_print_flag == 2)
			LCDPR("od_sel=%d, pll_fvco=%d\n", (od_sel - 1), pll_fvco);

		cConf->pll_fvco = pll_fvco;
		n = 1;
		od_fb = cConf->od_fb;
		pll_fvco = pll_fvco / od_fb_table[od_fb];
		m = pll_fvco / cConf->fin;
		pll_frac = (pll_fvco % cConf->fin) *
				cConf->pll_frac_range / cConf->fin;
		cConf->pll_m = m;
		cConf->pll_n = n;
		cConf->pll_frac = pll_frac;
		if (lcd_debug_print_flag == 2)
			LCDPR("pll_m=%d, pll_n=%d, pll_frac=0x%x\n", m, n, pll_frac);
		done = 1;
		break;
	}
	return done;
}

static void lcd_clk_generate_axg(struct lcd_config_s *pconf)
{
	unsigned int pll_fout;
	unsigned int xd;
	unsigned int dsi_bit_rate_max = 0, dsi_bit_rate_min = 0;
	unsigned int tmp;
	struct lcd_clk_config_s *cConf;
	int done;

	done = 0;
	cConf = get_lcd_clk_config();
	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	cConf->err_fmin = MAX_ERROR;

	if (cConf->fout > cConf->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
		goto generate_clk_done_axg;
	}

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_MIPI:
		cConf->xd_max = CRT_VID_DIV_MAX;
		tmp = pconf->lcd_control.mipi_config->bit_rate_max;
		dsi_bit_rate_max = tmp * 1000; /* change to kHz */
		dsi_bit_rate_min = dsi_bit_rate_max - cConf->fout;

		for (xd = 1; xd <= cConf->xd_max; xd++) {
			pll_fout = cConf->fout * xd;
			if ((pll_fout > dsi_bit_rate_max) ||
				(pll_fout < dsi_bit_rate_min)) {
				continue;
			}
			if (lcd_debug_print_flag == 2)
				LCDPR("fout=%d, xd=%d\n", cConf->fout, xd);

			pconf->lcd_control.mipi_config->bit_rate = pll_fout * 1000;
			pconf->lcd_control.mipi_config->clk_factor = xd;
			cConf->xd = xd;
			done = check_pll_axg(cConf, pll_fout);
			if (done)
				goto generate_clk_done_axg;
		}
		break;
	default:
		break;
	}

generate_clk_done_axg:
	if (done) {
		pconf->lcd_timing.pll_ctrl =
			(cConf->pll_od1_sel << PLL_CTRL_OD1) |
			(cConf->pll_n << PLL_CTRL_N) |
			(cConf->pll_m << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_DIV_SEL) |
			(cConf->xd << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (cConf->pll_frac << CLK_CTRL_FRAC);
	} else {
		pconf->lcd_timing.pll_ctrl =
			(1 << PLL_CTRL_OD1) |
			(1 << PLL_CTRL_N)   |
			(50 << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_DIV_SEL) |
			(7 << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (0 << CLK_CTRL_FRAC);
		LCDERR("Out of clock range, reset to default setting!\n");
	}
}

static void lcd_pll_frac_generate_axg(struct lcd_config_s *pconf)
{
	unsigned int pll_fout;
	unsigned int od, pll_fvco;
	unsigned int m, n, od_fb, frac, offset, temp;
	struct lcd_clk_config_s *cConf;

	cConf = get_lcd_clk_config();
	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	od = od_table[cConf->pll_od1_sel];
	m = cConf->pll_m;
	n = cConf->pll_n;

	if (lcd_debug_print_flag == 2) {
		LCDPR("m=%d, n=%d, od=%d, xd=%d\n",
			m, n, cConf->pll_od1_sel, cConf->xd);
	}
	if (cConf->fout > cConf->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pclk=%d\n", __func__, cConf->fout);

	pll_fout = cConf->fout * cConf->xd;
	if ((pll_fout > cConf->pll_out_fmax) ||
		(pll_fout < cConf->pll_out_fmin)) {
		LCDERR("%s: wrong pll_fout value %dkHz\n", __func__, pll_fout);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pll_fout=%d\n", __func__, pll_fout);

	pll_fvco = pll_fout * od;
	if ((pll_fvco < cConf->pll_vco_fmin) ||
		(pll_fvco > cConf->pll_vco_fmax)) {
		LCDERR("%s: wrong pll_fvco value %dkHz\n", __func__, pll_fvco);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pll_fvco=%d\n", __func__, pll_fvco);

	cConf->pll_fvco = pll_fvco;
	od_fb = cConf->od_fb; /* pll default */
	pll_fvco = pll_fvco / od_fb_table[od_fb];
	temp = cConf->fin * m / n;
	if (pll_fvco >= temp) {
		temp = pll_fvco - temp;
		offset = 0;
	} else {
		temp = temp - pll_fvco;
		offset = 1;
	}
	if (temp >= (2 * cConf->fin)) {
		LCDERR("%s: pll changing %dkHz is too much\n",
			__func__, temp);
		return;
	}
	frac = temp * cConf->pll_frac_range * n / cConf->fin;
	cConf->pll_frac = frac | (offset << 11);
	if (lcd_debug_print_flag)
		LCDPR("lcd_pll_frac_generate: frac=0x%x\n", frac);
}

static void lcd_clk_generate_hpll_g12a(struct lcd_config_s *pconf)
{
	unsigned int pll_fout;
	unsigned int clk_div_sel, xd;
	unsigned int dsi_bit_rate_max = 0, dsi_bit_rate_min = 0;
	unsigned int tmp;
	struct lcd_clk_config_s *cConf = get_lcd_clk_config();
	int done;

	done = 0;
	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	cConf->err_fmin = MAX_ERROR;

	if (cConf->fout > cConf->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
	}

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_MIPI:
		cConf->xd_max = CRT_VID_DIV_MAX;
		tmp = pconf->lcd_control.mipi_config->bit_rate_max;
		dsi_bit_rate_max = tmp * 1000; /* change to kHz */
		dsi_bit_rate_min = dsi_bit_rate_max - cConf->fout;

		clk_div_sel = CLK_DIV_SEL_1;
		for (xd = 1; xd <= cConf->xd_max; xd++) {
			pll_fout = cConf->fout * xd;
			if ((pll_fout > dsi_bit_rate_max) ||
				(pll_fout < dsi_bit_rate_min)) {
				continue;
			}
			if (lcd_debug_print_flag == 2)
				LCDPR("fout=%d, xd=%d\n", cConf->fout, xd);

			pconf->lcd_control.mipi_config->bit_rate =
				pll_fout * 1000;
			pconf->lcd_control.mipi_config->clk_factor = xd;
			cConf->xd = xd;
			cConf->div_sel = clk_div_sel;
			done = check_pll_txl(cConf, pll_fout);
			if (done)
				goto generate_clk_done_g12a;
		}
		break;
	default:
		break;
	}

generate_clk_done_g12a:
	if (done) {
		pconf->lcd_timing.pll_ctrl =
			(cConf->pll_od1_sel << PLL_CTRL_OD1) |
			(cConf->pll_od2_sel << PLL_CTRL_OD2) |
			(cConf->pll_od3_sel << PLL_CTRL_OD3) |
			(cConf->pll_n << PLL_CTRL_N)         |
			(cConf->pll_m << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(cConf->div_sel << DIV_CTRL_DIV_SEL) |
			(cConf->xd << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (cConf->pll_frac << CLK_CTRL_FRAC);
	} else {
		pconf->lcd_timing.pll_ctrl =
			(1 << PLL_CTRL_OD1) |
			(1 << PLL_CTRL_OD2) |
			(1 << PLL_CTRL_OD3) |
			(1 << PLL_CTRL_N)   |
			(50 << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_DIV_SEL) |
			(7 << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (0 << CLK_CTRL_FRAC);
		LCDERR("Out of clock range, reset to default setting\n");
	}
}

void lcd_clk_generate_parameter(struct lcd_config_s *pconf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	switch (lcd_drv->chip_type) {
	case LCD_CHIP_GXTVBB:
		lcd_clk_generate_gxtvbb(pconf);
		break;
	case LCD_CHIP_GXL:
	case LCD_CHIP_GXM:
	case LCD_CHIP_TXL:
	case LCD_CHIP_TXLX:
	case LCD_CHIP_TXHD:
		lcd_clk_generate_txl(pconf);
		break;
	case LCD_CHIP_AXG:
		lcd_clk_generate_axg(pconf);
		break;
	case LCD_CHIP_G12A:
	case LCD_CHIP_G12B:
	case LCD_CHIP_SM1:
		if (lcd_drv->lcd_config->lcd_clk_path)
			lcd_clk_generate_axg(pconf);
		else
			lcd_clk_generate_hpll_g12a(pconf);
		break;
	default:
		break;
	}
}

static char lcd_ss_str[20] = {
	'u', 'n', 'k', 'n', 'o', 'w', '\0',
};
char *lcd_get_spread_spectrum(void)
{
	int ss_level;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	ss_level = lcd_drv->lcd_config->lcd_timing.ss_level;
	switch (lcd_drv->chip_type) {
	case LCD_CHIP_GXTVBB:
		ss_level = (ss_level >= clk_conf.ss_level_max) ? 0 : ss_level;
		sprintf(lcd_ss_str, "%s", lcd_pll_ss_table_gxtvbb[ss_level]);
		break;
	case LCD_CHIP_TXL:
		ss_level = (ss_level >= clk_conf.ss_level_max) ? 0 : ss_level;
		sprintf(lcd_ss_str, "%s", lcd_pll_ss_table_txl[ss_level]);
		break;
	case LCD_CHIP_TXLX:
		if (ss_level >= 1000) {
			if (ss_level > 60000)
				sprintf(lcd_ss_str, "invalid");
			else
				sprintf(lcd_ss_str, "%dppm", ss_level);
		} else {
			ss_level = (ss_level >= clk_conf.ss_level_max) ? 0 : ss_level;
			sprintf(lcd_ss_str, "%s", lcd_pll_ss_table_txlx[ss_level]);
		}
		break;
	case LCD_CHIP_TXHD:
		ss_level = (ss_level >= clk_conf.ss_level_max) ? 0 : ss_level;
		sprintf(lcd_ss_str, "%s", lcd_pll_ss_table_txhd[ss_level]);
		break;
	default:
		sprintf(lcd_ss_str, "unknown");
		break;
	}

	return lcd_ss_str;
}

void lcd_set_spread_spectrum(void)
{
	int ss_level;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	ss_level = lcd_drv->lcd_config->lcd_timing.ss_level;
	switch (lcd_drv->chip_type) {
	case LCD_CHIP_GXTVBB:
		clk_conf.ss_level = (ss_level >= clk_conf.ss_level_max) ? 0 : ss_level;
		lcd_set_pll_ss_gxtvbb(&clk_conf);
		break;
	case LCD_CHIP_TXL:
		clk_conf.ss_level = (ss_level >= clk_conf.ss_level_max) ? 0 : ss_level;
		lcd_set_pll_ss_txl(&clk_conf);
		break;
	case LCD_CHIP_TXLX:
		clk_conf.ss_level = ss_level;
		lcd_set_pll_ss_txlx(&clk_conf);
		break;
	case LCD_CHIP_TXHD:
		clk_conf.ss_level = (ss_level >= clk_conf.ss_level_max) ? 0 : ss_level;
		lcd_set_pll_ss_txhd(&clk_conf);
		break;
	default:
		break;
	}
}

/* for frame rate change */
void lcd_clk_update(struct lcd_config_s *pconf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	LCDPR("%s\n", __func__);

	switch (lcd_drv->chip_type) {
	case LCD_CHIP_GXTVBB:
		lcd_pll_frac_generate_gxtvbb(pconf);
		lcd_update_pll_frac_gxtvbb(&clk_conf);
		break;
	case LCD_CHIP_GXL:
	case LCD_CHIP_GXM:
	case LCD_CHIP_TXL:
	case LCD_CHIP_TXLX:
	case LCD_CHIP_TXHD:
		lcd_pll_frac_generate_txl(pconf);
		lcd_update_pll_frac_txl(&clk_conf);
		break;
	case LCD_CHIP_AXG:
		lcd_pll_frac_generate_axg(pconf);
		lcd_update_pll_frac_axg(&clk_conf);
		break;
	case LCD_CHIP_G12A:
	case LCD_CHIP_G12B:
	case LCD_CHIP_SM1:
		if (lcd_drv->lcd_config->lcd_clk_path) {
			lcd_pll_frac_generate_axg(pconf);
			lcd_update_gp0_pll_frac_g12a(&clk_conf);
		} else {
			lcd_pll_frac_generate_txl(pconf);
			lcd_update_hpll_frac_g12a(&clk_conf);
		}
		break;
	default:
		break;
	}
}

/* for timing change */
void lcd_clk_set(struct lcd_config_s *pconf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	switch (lcd_drv->chip_type) {
	case LCD_CHIP_GXTVBB:
		lcd_set_pll_gxtvbb(&clk_conf);
		lcd_set_vid_pll_div(&clk_conf);
		break;
	case LCD_CHIP_GXL:
	case LCD_CHIP_GXM:
	case LCD_CHIP_TXL:
		lcd_set_pll_txl(&clk_conf);
		lcd_set_vid_pll_div(&clk_conf);
		break;
	case LCD_CHIP_TXLX:
		lcd_set_pll_txlx(&clk_conf);
		lcd_set_vid_pll_div(&clk_conf);
		break;
	case LCD_CHIP_TXHD:
		lcd_set_tcon_clk(pconf);
		lcd_set_pll_txhd(&clk_conf);
		lcd_set_vid_pll_div(&clk_conf);
		break;
	case LCD_CHIP_AXG:
		lcd_set_pll_axg(&clk_conf);
		lcd_set_dsi_meas_clk();
		break;
	case LCD_CHIP_G12A:
	case LCD_CHIP_SM1:
		if (lcd_drv->lcd_config->lcd_clk_path) {
			lcd_set_gp0_pll_g12a(&clk_conf);
			lcd_set_dsi_meas_clk();
			lcd_set_dsi_phy_clk(1);
		} else {
			lcd_set_hpll_g12a(&clk_conf);
			lcd_set_vid_pll_div(&clk_conf);
			lcd_set_dsi_meas_clk();
			lcd_set_dsi_phy_clk(0);
		}
		break;
	case LCD_CHIP_G12B:
		if (lcd_drv->lcd_config->lcd_clk_path) {
			lcd_set_gp0_pll_g12b(&clk_conf);
			lcd_set_dsi_meas_clk();
			lcd_set_dsi_phy_clk(1);
		} else {
			lcd_set_hpll_g12b(&clk_conf);
			lcd_set_vid_pll_div(&clk_conf);
			lcd_set_dsi_meas_clk();
			lcd_set_dsi_phy_clk(0);
		}
		break;
	default:
		break;
	}
	lcd_set_vclk_crt(pconf->lcd_basic.lcd_type, &clk_conf);
	mdelay(10);
}

void lcd_clk_disable(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	lcd_hiu_setb(HHI_VID_CLK_CNTL2, 0, ENCL_GATE_VCLK, 1);

	/* close vclk2_div gate: 0x104b[4:0] */
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 0, 0, 5);
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 0, VCLK2_EN, 1);

	/* disable pll */
	switch (lcd_drv->chip_type) {
	case LCD_CHIP_GXTVBB:
		/* disable hdmi_pll: 0x10c8[30] */
		lcd_hiu_setb(HHI_HPLL_CNTL, 0, LCD_PLL_EN_GXTVBB, 1);
		lcd_hiu_setb(HHI_HPLL_CNTL5, 0, 30, 1); /* bandgap */
		break;
	case LCD_CHIP_GXL:
	case LCD_CHIP_GXM:
	case LCD_CHIP_TXL:
	case LCD_CHIP_TXLX:
	case LCD_CHIP_TXHD:
		/* disable hdmi_pll: 0x10c8[30] */
		lcd_hiu_setb(HHI_HPLL_CNTL, 0, LCD_PLL_EN_TXL, 1);
		break;
	case LCD_CHIP_AXG:
		/* disable hdmi_pll: 0x10c8[30] */
		lcd_hiu_setb(HHI_GP0_PLL_CNTL, 0, LCD_PLL_EN_AXG, 1);
		break;
	case LCD_CHIP_G12A:
	case LCD_CHIP_G12B:
	case LCD_CHIP_SM1:
		if (lcd_drv->lcd_config->lcd_clk_path) {
			lcd_hiu_setb(HHI_GP0_PLL_CNTL0, 0, LCD_PLL_EN_GP0_G12A, 1);
		} else {
			/* disable hdmi_pll: 0x10c8[28] */
			lcd_hiu_setb(HHI_HDMI_PLL_CNTL0, 0, LCD_PLL_EN_HPLL_G12A, 1);
		}
		break;

	default:
		break;
	}
}

void lcd_clk_config_probe(void)
{
	lcd_clk_config_chip_init();
}
