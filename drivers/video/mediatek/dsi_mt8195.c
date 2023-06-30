// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek mt8195 dsi setting
 *
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Tommy Chen <tommyyl.chen@mediatek.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitops.h>

#include "dsi_mt8195.h"

/* DSI platform functions */
void mtk_dsi_phy_timing(int data_rate, struct mtk_phy_timing *phy_timing)
{
	u32 cycle_time, ui;

	memset(phy_timing, 0, sizeof(*phy_timing));

	ui = 1000 / data_rate + 0x01;
	cycle_time = 8000 / data_rate + 0x01;

	phy_timing->lpx = NS_TO_CYCLE(data_rate * 0x4B, 0x1F40) + 0x1;
	phy_timing->da_hs_prepare = NS_TO_CYCLE((0x40 + 0x5 * ui), cycle_time) + 0x1;
	phy_timing->da_hs_zero = NS_TO_CYCLE((0xC8 + 0x0A * ui), cycle_time);
	phy_timing->da_hs_zero = phy_timing->da_hs_zero > phy_timing->da_hs_prepare ?
	phy_timing->da_hs_zero - phy_timing->da_hs_prepare : phy_timing->da_hs_zero;
	phy_timing->da_hs_trail = NS_TO_CYCLE((0x4 * ui + 0x50) *
		data_rate, 0x1F40) + 0x1;

	phy_timing->ta_go = 4 * phy_timing->lpx;
	phy_timing->ta_sure = 3 * phy_timing->lpx / 2;
	phy_timing->ta_get = 5 * phy_timing->lpx;
	phy_timing->da_hs_exit = 2 * phy_timing->lpx;

	phy_timing->da_hs_sync = 0x1;
	phy_timing->cont_det = 0x3;

	phy_timing->clk_hs_prepare = NS_TO_CYCLE(0x50 * data_rate, 0x1F40);
	phy_timing->clk_hs_post = NS_TO_CYCLE(0x60 + 0x34 * ui, cycle_time);
	phy_timing->clk_hs_trail = NS_TO_CYCLE(0x64 * data_rate, 0x1F40) + 0x1;
	phy_timing->clk_hs_zero = NS_TO_CYCLE(0x190, cycle_time);
	phy_timing->clk_hs_exit = 2 * phy_timing->lpx;

	/* Allow board-specific tuning */
	mtk_dsi_override_phy_timing(phy_timing);

	u32 timcon0, timcon1, timcon2, timcon3;

	timcon0 = phy_timing->lpx | phy_timing->da_hs_prepare << 8 |
		  phy_timing->da_hs_zero << 16 | phy_timing->da_hs_trail << 24;
	timcon1 = phy_timing->ta_go | phy_timing->ta_sure << 8 |
		  phy_timing->ta_get << 16 | phy_timing->da_hs_exit << 24;
	timcon2 = phy_timing->cont_det | phy_timing->da_hs_sync << 8 |
		  phy_timing->clk_hs_zero << 16 | phy_timing->clk_hs_trail << 24;
	timcon3 = phy_timing->clk_hs_prepare | phy_timing->clk_hs_post << 8 |
		  phy_timing->clk_hs_exit << 16;

	DSI_REG_WRITE(&dsi0->dsi_phy_timecon0, timcon0);
	DSI_REG_WRITE(&dsi0->dsi_phy_timecon1, timcon1);
	DSI_REG_WRITE(&dsi0->dsi_phy_timecon2, timcon2);
	DSI_REG_WRITE(&dsi0->dsi_phy_timecon3, timcon3);
}

void mtk_dsi_clk_hs_mode_enable(void)
{
	DSI_REG_SET_BITS(&dsi0->dsi_phy_lccon, LC_HS_TX_EN);
}

void mtk_dsi_clk_hs_mode_disable(void)
{
	DSI_REG_CLR_BITS(&dsi0->dsi_phy_lccon, LC_HS_TX_EN);
}

void mtk_dsi_set_mode(u32 mode_flags)
{
	u32 tmp_reg1 = 0;

	if (mode_flags & MIPI_DSI_MODE_VIDEO) {
		tmp_reg1 = SYNC_EVENT_MODE;

		if (mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
			tmp_reg1 = BURST_MODE;

		if (mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			tmp_reg1 = SYNC_PULSE_MODE;
	}

	DSI_REG_WRITE(&dsi0->dsi_mode_ctrl, tmp_reg1);
}

void mtk_dsi_rxtx_control(u32 mode_flags, u32 lanes)
{
	u32 tmp_reg = 0;

	switch (lanes) {
	case 1:
		tmp_reg = 1 << 2;
		break;
	case 2:
		tmp_reg = 3 << 2;
		break;
	case 3:
		tmp_reg = 7 << 2;
		break;
	case 4:
	default:
		tmp_reg = 0xf << 2;
		break;
	}

	tmp_reg |= (mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) << 6;
	tmp_reg |= (mode_flags & MIPI_DSI_MODE_EOT_PACKET) >> 3;

	DSI_REG_WRITE(&dsi0->dsi_txrx_ctrl, tmp_reg);
}

void mtk_dsi_config_vdo_timing(u32 mode_flags, u32 format, u32 lanes, struct videomode *vm,
			       const struct mtk_phy_timing *phy_timing)
{
	u32 hsync_active_byte;
	u32 hbp_byte;
	u32 hfp_byte;
	u32 bytes_per_pixel;
	u32 packet_fmt;
	u32 hactive;
	u32 vactive;
	u32 data_phy_cycles;
	u32 reg_value;

	bytes_per_pixel = DIV_ROUND_UP(mtk_dsi_get_bits_per_pixel(format), 8);

	DSI_REG_WRITE(&dsi0->dsi_vsa_nl, vm->vsync_len);
	DSI_REG_WRITE(&dsi0->dsi_vbp_nl, vm->vback_porch);
	DSI_REG_WRITE(&dsi0->dsi_vfp_nl, vm->vfront_porch);
	DSI_REG_WRITE(&dsi0->dsi_vact_nl, vm->vactive);

	if (mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) {
		hsync_active_byte = ALIGN_TO((vm->hsync_len * bytes_per_pixel - 10), 4);
		hbp_byte = ALIGN_TO((vm->hback_porch * bytes_per_pixel - 10), 4);
	} else {
		hsync_active_byte = ALIGN_TO((vm->hsync_len * bytes_per_pixel - 4), 4);
		hbp_byte = ALIGN_TO(((vm->hback_porch + vm->hsync_len) * bytes_per_pixel -
				 10), 4);
	}

	hfp_byte = ALIGN_TO((vm->hfront_porch * bytes_per_pixel - 12), 4);

	DSI_REG_WRITE(&dsi0->dsi_hsa_wc, hsync_active_byte);
	DSI_REG_WRITE(&dsi0->dsi_hbp_wc, hbp_byte);
	DSI_REG_WRITE(&dsi0->dsi_hfp_wc, hfp_byte);

	switch (format) {
	case MIPI_DSI_FMT_RGB888:
		packet_fmt = PACKED_PS_24BIT_RGB888;
		break;
	case MIPI_DSI_FMT_RGB666:
		packet_fmt = LOOSELY_PS_18BIT_RGB666;
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		packet_fmt = PACKED_PS_18BIT_RGB666;
		break;
	case MIPI_DSI_FMT_RGB565:
		packet_fmt = PACKED_PS_16BIT_RGB565;
		break;
	default:
		packet_fmt = PACKED_PS_24BIT_RGB888;
		break;
	}

	hactive = vm->hactive;
	vactive = vm->vactive;
	packet_fmt |= (hactive * bytes_per_pixel) & DSI_PS_WC;

	reg_value = PIXEL_STREAM_CUSTOM_HEADER << DSI_PSCON_CUSTOM_HEADER_SHIFT | packet_fmt;
	DSI_REG_WRITE(&dsi0->dsi_psctrl, reg_value);

	/* Older systems like MT8173 do not support size_con. */
	if (MTK_DSI_HAVE_SIZE_CON) {
		reg_value = vactive << DSI_SIZE_CON_HEIGHT_SHIFT;
		reg_value |= hactive << DSI_SIZE_CON_WIDTH_SHIFT;
		DSI_REG_WRITE(&dsi0->dsi_size_con, reg_value);
	}
}

void mtk_dsi_start(void)
{
	DSI_REG_WRITE(&dsi0->dsi_start, 0);
	/* Only start master DSI */
	DSI_REG_WRITE(&dsi0->dsi_start, 1);
}

void mtk_dsi_cmdq(const u8 *data, u8 len, u32 type)
{
	const u8 *tx_buf = data;
	u32 config;
	int i = 0, j;

	while (DSI_REG_READ(&dsi0->dsi_intsta) & DSI_BUSY) {
		if (i > 10) {
			printf("%s: cannot get DSI ready for sending commands\n"
			       " after 20ms and the panel may not work properly.\n",
			       __func__);
			return;
		}
		i++;
		mdelay(2);
	}

	DSI_REG_WRITE(&dsi0->dsi_intsta, 0);

	if (mtk_dsi_is_read_command(type))
		config = BTA;
	else
		config = (len > 2) ? LONG_PACKET : SHORT_PACKET;

	if (len <= 2) {
		u32 val = (type << 8) | config;

		for (i = 0; i < len; i++)
			val |= tx_buf[i] << (i + 2) * 8;
		DSI_REG_WRITE(&dsi0->dsi_cmdq[0], val);
		DSI_REG_WRITE(&dsi0->dsi_cmdq_size, 1);
	} else {
		/* TODO(hungte) Replace by buffer_to_fifo32_prefix */
		DSI_REG_WRITE(&dsi0->dsi_cmdq[0], (len << 16) | (type << 8) | config);
		for (i = 0; i < len; i += 4) {
			u32 val = 0;

			for (j = 0; j < ((len - i) < 4 ? (len - i) : 4); j++)
				val |= tx_buf[i + j] << j * 8;
			DSI_REG_WRITE(&dsi0->dsi_cmdq[i / 4 + 1], val);
		}
		DSI_REG_WRITE(&dsi0->dsi_cmdq_size, 1 + DIV_ROUND_UP(len, 4));
	}

	mtk_dsi_start();

	mdelay(2);

	if ((DSI_REG_READ(&dsi0->dsi_intsta) & CMD_DONE_INT_FLAG) == 0)
		printf("%s: failed sending DSI command,\n"
		       "panel may not work.\n", __func__);
}

void mtk_dsi_reset(void)
{
	DSI_REG_WRITE(&dsi0->dsi_force_commit, DSI_FORCE_COMMIT_USE_MMSYS |
				  DSI_FORCE_COMMIT_ALWAYS);
	DSI_REG_WRITE(&dsi0->dsi_con_ctrl, 1);
	DSI_REG_WRITE(&dsi0->dsi_con_ctrl, 0);
}

void mtk_dsi_reset_dphy(void)
{
	DSI_REG_SET_BITS(&dsi0->dsi_con_ctrl, DPHY_RESET);
	DSI_REG_CLR_BITS(&dsi0->dsi_con_ctrl, DPHY_RESET);
}

unsigned char panel_compare_id(void)
{
	unsigned char vendor_id = 0xFF;
	unsigned int loop_cnt = 0;
	s32 tmp;

	DSI_REG_WRITE(&dsi0->dsi_cmdq[0], 0x00013700);
	DSI_REG_WRITE(&dsi0->dsi_cmdq[1], 0x00da0604);
	DSI_REG_WRITE(&dsi0->dsi_cmdq_size, 0x2);
	DSI_REG_WRITE(&dsi0->dsi_start, 0x0);
	DSI_REG_WRITE(&dsi0->dsi_start, 0x1);

	while (loop_cnt < 100 * 1000) {
		tmp = readl(&dsi0->dsi_intsta);
		if (tmp & 0x2)
			break;

		loop_cnt++;
		udelay(1);
	}

	DSI_REG_CLRSET_BITS(&dsi0->dsi_intsta, 0x1 << 1, 0 << 1);
	tmp = DSI_REG_READ(&dsi0->dsi_rx_data03);
	vendor_id = (tmp & 0xff00) >> 8;

	printf("tmp=0x%x vendor_id=0x%x\n", tmp, vendor_id);
	DSI_REG_CLRSET_BITS(&dsi0->dsi_rx_rack, 0x1 << 1, 1 << 1);
	DSI_REG_CLRSET_BITS(&dsi0->dsi_intsta, 0x1 << 1, 0 << 1);

	loop_cnt = 0;
	while (loop_cnt < 100 * 1000) {
		tmp = readl(&dsi0->dsi_intsta);
		if (!(tmp & 0x80000000))
			return vendor_id;

		loop_cnt++;
		udelay(1);
	}

	return 0;
}

/* MIPI platform functions */
void mtk_dsi_configure_mipi_tx(int data_rate, u32 lanes)
{
	unsigned int txdiv0, txdiv1;
	u64 pcw;

	if (data_rate >= 2000) {
		txdiv0 = 0;
		txdiv1 = 0;
	} else if (data_rate >= 1000) {
		txdiv0 = 1;
		txdiv1 = 0;
	} else if (data_rate >= 500) {
		txdiv0 = 2;
		txdiv1 = 0;
	} else if (data_rate > 250) {
		/* (data_rate == 250MHz) is a special case that should go to the
		 * else-block below (txdiv0 = 4)
		 */
		txdiv0 = 3;
		txdiv1 = 0;
	} else {
		/* MIN = 125 */
		assert(data_rate >= MTK_DSI_DATA_RATE_MIN_MHZ);
		txdiv0 = 4;
		txdiv1 = 0;
	}

	DSI_REG_CLR_BITS(&mipi_tx->pll_con4, BIT(11) | BIT(10));
	DSI_REG_SET_BITS(&mipi_tx->pll_pwr, AD_DSI_PLL_SDM_PWR_ON);
	udelay(30);
	DSI_REG_CLR_BITS(&mipi_tx->pll_pwr, AD_DSI_PLL_SDM_ISO_EN);

	pcw = (u64)data_rate * (1 << txdiv0) * (1 << txdiv1);
	pcw <<= 24;
	pcw /= 26;

	DSI_REG_WRITE(&mipi_tx->pll_con0, pcw);
	DSI_REG_CLRSET_BITS(&mipi_tx->pll_con1, RG_DSI_PLL_POSDIV, txdiv0 << 8);
	udelay(30);
	DSI_REG_SET_BITS(&mipi_tx->pll_con1, RG_DSI_PLL_EN);

	/* BG_LPF_EN / BG_CORE_EN */
	DSI_REG_WRITE(&mipi_tx->lane_con, 0x3fff0180);
	udelay(40);
	DSI_REG_WRITE(&mipi_tx->lane_con, 0x3fff00c0);

	/* Switch OFF each Lane */
	DSI_REG_CLR_BITS(&mipi_tx->d0_sw_ctl_en, DSI_SW_CTL_EN);
	DSI_REG_CLR_BITS(&mipi_tx->d1_sw_ctl_en, DSI_SW_CTL_EN);
	DSI_REG_CLR_BITS(&mipi_tx->d2_sw_ctl_en, DSI_SW_CTL_EN);
	DSI_REG_CLR_BITS(&mipi_tx->d3_sw_ctl_en, DSI_SW_CTL_EN);
	DSI_REG_CLR_BITS(&mipi_tx->ck_sw_ctl_en, DSI_SW_CTL_EN);

	DSI_REG_SET_BITS(&mipi_tx->ck_ckmode_en, DSI_CK_CKMODE_EN);
}

int mtk_dsi_dump(void)
{
	int k;

	printf("- DSI REGS -\n");
	for (k = 0; k < 0x400; k += 16) {
		printf("0x%04x: 0x%08x 0x%08x 0x%08x 0x%08x\n", k,
		       DSI_REG_READ(DSI0_BASE + k),
		       DSI_REG_READ(DSI0_BASE + k + 0x4),
		       DSI_REG_READ(DSI0_BASE + k + 0x8),
		       DSI_REG_READ(DSI0_BASE + k + 0xc));
	}

	printf("- DSI CMD REGS -\n");
	for (k = 0; k < 32; k += 16) {
		printf("0x%04x: 0x%08x 0x%08x 0x%08x 0x%08x\n", k + 0xD00,
		       DSI_REG_READ(DSI0_BASE + 0xD00 + k),
		       DSI_REG_READ(DSI0_BASE + 0xD00 + k + 0x4),
		       DSI_REG_READ(DSI0_BASE + 0xD00 + k + 0x8),
		       DSI_REG_READ(DSI0_BASE + 0xD00 + k + 0xc));
	}
	printf("== MIPI REGS ==\n");
	for (k = 0; k < 0x6A0; k += 16) {
		printf("0x%04x: 0x%08x 0x%08x 0x%08x 0x%08x\n", k,
		       DSI_REG_READ(MIPITX_BASE + k),
		       DSI_REG_READ(MIPITX_BASE + k + 0x4),
		       DSI_REG_READ(MIPITX_BASE + k + 0x8),
		       DSI_REG_READ(MIPITX_BASE + k + 0xc));
	}

	return 0;
}

void mtk_dsi_override_phy_timing(struct mtk_phy_timing *timing)
{
	/* Allow board-specific tuning */
}
