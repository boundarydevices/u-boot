/*
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

void ddr_init1(void);
void lpddr4_800M_cfg_phy(void);

static inline void reg32_writep(u32 *addr, u32 val)
{
	writel(val, addr);
}
