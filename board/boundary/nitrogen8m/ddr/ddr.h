/*
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

void ddr_init1(struct dram_timing_info *dram_timing);
extern struct dram_timing_info lpddr4_timing_;
void lpddr4_cfg_umctl2(struct dram_cfg_param *ddrc_cfg, int num);
void lpddr4_800M_cfg_phy(struct dram_timing_info *dram_timing);

static inline void reg32_writep(u32 *addr, u32 val)
{
	writel(val, addr);
}
