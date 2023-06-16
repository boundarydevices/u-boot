/* SPDX-License-Identifier: GPL-2.0
 *
 * MediaTek mt8188 dsi header
 *
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Tommy Chen <tommyyl.chen@mediatek.com>
 */

#ifndef _DSI_REG_H_
#define _DSI_REG_H_

#include "mtk_dsi_common.h"

/* MIPITX_REG */
#define MIPITX_BASE 0x11c80000

/* MIPITX is SOC specific and cannot live in common. */

/* MIPITX_REG */
struct mipi_tx_regs {
	u32 reserved0[3];
	u32 lane_con;
	u32 reserved1[6];
	u32 pll_pwr;
	u32 pll_con0;
	u32 pll_con1;
	u32 pll_con2;
	u32 pll_con3;
	u32 pll_con4;
	u32 reserved2[65];
	u32 d2_sw_ctl_en;
	u32 reserved3[63];
	u32 d0_sw_ctl_en;
	u32 reserved4[56];
	u32 ck_ckmode_en;
	u32 reserved5[6];
	u32 ck_sw_ctl_en;
	u32 reserved6[63];
	u32 d1_sw_ctl_en;
	u32 reserved7[63];
	u32 d3_sw_ctl_en;
};

check_member(mipi_tx_regs, pll_con4, 0x3c);
check_member(mipi_tx_regs, d3_sw_ctl_en, 0x544);
static struct mipi_tx_regs *const mipi_tx = (void *)MIPITX_BASE;

/* Register values */
#define DSI_CK_CKMODE_EN	BIT(0)
#define DSI_SW_CTL_EN		BIT(0)
#define AD_DSI_PLL_SDM_PWR_ON	BIT(0)
#define AD_DSI_PLL_SDM_ISO_EN	BIT(1)

#define RG_DSI_PLL_EN		BIT(4)
#define RG_DSI_PLL_POSDIV	(0x7 << 8)

/* SOC specific functions */
void mtk_dsi_pin_drv_ctrl(void);

struct dsi_regs {
	u32 dsi_start;
	u8 reserved0[4];
	u32 dsi_inten;
	u32 dsi_intsta;
	u32 dsi_con_ctrl;
	u32 dsi_mode_ctrl;
	u32 dsi_txrx_ctrl;
	u32 dsi_psctrl;
	u32 dsi_vsa_nl;
	u32 dsi_vbp_nl;
	u32 dsi_vfp_nl;
	u32 dsi_vact_nl;
	u32 dsi_lfr_con;  /* Available since MT8183 */
	u32 dsi_lfr_sta;  /* Available since MT8183 */
	u32 dsi_size_con;  /* Available since MT8183 */
	u32 dsi_vfp_early_stop;  /* Available since MT8183 */
	u8 reserved1[16]; /* 0x40~0x4c */
	u32 dsi_hsa_wc; /* 0x50 */
	u32 dsi_hbp_wc;
	u32 dsi_hfp_wc;
	u32 dsi_bllp_wc;
	u32 dsi_cmdq_size;
	u32 dsi_hstx_cklp_wc;
	u8 reserved2[12];
	u32 dsi_rx_data03;
	u8 reserved3[12];
	u32 dsi_rx_rack;
	u8 reserved4[124];
	u32 dsi_phy_lccon;
	u32 dsi_phy_ld0con;
	u8 reserved5[4];
	u32 dsi_phy_timecon0;
	u32 dsi_phy_timecon1;
	u32 dsi_phy_timecon2;
	u32 dsi_phy_timecon3;
	u8 reserved6[68]; /* 0x120~0x160 */
	u32 dsi_state_dbg7; /* 0x164 */
	u8 reserved7[16]; /* 0x168~0x174 */
	u32 dsi_self_pat_con0; /* 0x178 */
	u32 dsi_self_pat_con1; /* 0x17C */
	u8 reserved8[128]; /* 0x180~0x1FC */
	u32 dsi_vm_cmd_con0; /* 0x200 */
	u32 dsi_vm_cmd_con1; /* 0x204 */
	u8 reserved9[2552]; /* 0x208~0xBFC */
	u32 dsi_force_commit;  /* 0xC00, Available since MT8183 */
	u8 reserved10[252]; /* 0xC04~0xCFC */
	u32 dsi_cmdq[128]; /* 0xD00~ */
};

static struct dsi_regs *const dsi0 = (void *)DSI0_BASE;

check_member(dsi_regs, dsi_phy_lccon, 0x104);
check_member(dsi_regs, dsi_phy_timecon3, 0x11c);
check_member(dsi_regs, dsi_vm_cmd_con0, 0x200);
check_member(dsi_regs, dsi_force_commit, 0xC00);
check_member(dsi_regs, dsi_cmdq, 0xD00);

#endif
