/*
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/ddr.h>
#include <asm/arch/imx8m_ddr.h>
#include "ddr.h"
#include "ddr_memory_map.h"

#ifdef CONFIG_ENABLE_DDR_TRAINING_DEBUG
#define ddr_printf(args...) printf(args)
#else
#define ddr_printf(args...)
#endif

#ifndef SRC_DDRC_RCR_ADDR
#define SRC_DDRC_RCR_ADDR SRC_IPS_BASE_ADDR +0x1000
#endif
#ifndef DDR_CSD1_BASE_ADDR
#define DDR_CSD1_BASE_ADDR 0x40000000
#endif
#define SILICON_TRAIN
#define DDR_BOOT_P1	/* default DDR boot frequency point */

volatile unsigned int tmp, tmp_t, i;

void ddr_init1(struct dram_timing_info *dram_timing)
{
	struct ccm_reg *ccm_reg = (struct ccm_reg *)CCM_BASE_ADDR;

	reg32_write(SRC_DDRC_RCR_ADDR + 0x04, 0x8F00000F);
	reg32_write(SRC_DDRC_RCR_ADDR, 0x8F00000F);
	mdelay(100);
	reg32_write(SRC_DDRC_RCR_ADDR + 0x04, 0x8F000000);

	/* change the clock source of dram_apb_clk_root */
	writel((0x7<<24)|(0x7<<16), &ccm_reg->ip_root[1].target_root_clr);
	writel((0x4<<24)|(0x3<<16), &ccm_reg->ip_root[1].target_root_set); /* to source 4 --800MHz/4 */

	/* disable iso */
	reg32_write(0x303A00EC, 0x0000ffff); /* PGC_CPU_MAPPING */
	reg32setbit(0x303A00F8, 5); /* PU_PGC_SW_PUP_REQ */

	dram_pll_init(MHZ(800));

	reg32_write(SRC_DDRC_RCR_ADDR, 0x8F000006);

	/* Configure uMCTL2's registers */
	lpddr4_cfg_umctl2(dram_timing->ddrc_cfg, dram_timing->ddrc_cfg_num);

#ifdef DDR_BOOT_P2
	reg32_write(DDRC_MSTR2(0), 0x2);
#else
#ifdef DDR_BOOT_P1
	reg32_write(DDRC_MSTR2(0), 0x1);
#endif
#endif
	/* release [1]ddr1_core_reset_n, [2]ddr1_phy_reset, [3]ddr1_phy_pwrokin_n */
	reg32_write(SRC_DDRC_RCR_ADDR, 0x8F000004);

	/* release [1]ddr1_core_reset_n, [2]ddr1_phy_reset, [3]ddr1_phy_pwrokin_n */
	reg32_write(SRC_DDRC_RCR_ADDR, 0x8F000000);

	reg32_write(DDRC_DBG1(0), 0x00000000);
	tmp = reg32_read(DDRC_PWRCTL(0));
	reg32_write(DDRC_PWRCTL(0), 0x000000a8);

	while ((reg32_read(DDRC_STAT(0)) & 0x33f) != 0x223)
		;

	reg32_write(DDRC_SWCTL(0), 0x00000000);

	/* LPDDR4 mode */
	reg32_write(DDRC_DDR_SS_GPR0, 0x01);

#ifdef DDR_BOOT_P1
	reg32_write(DDRC_DFIMISC(0), 0x00000110);
#else
	reg32_write(DDRC_DFIMISC(0), 0x00000010);
#endif
	/* LPDDR4 PHY config and training */
	ddr_cfg_phy(dram_timing);

	reg32_write(DDRC_RFSHCTL3(0), 0x00000000);

	reg32_write(DDRC_SWCTL(0), 0x0000);

	/* Set DFIMISC.dfi_init_start to 1 */
#ifdef DDR_BOOT_P2
	reg32_write(DDRC_DFIMISC(0), 0x00000230);
#else
#ifdef DDR_BOOT_P1
	reg32_write(DDRC_DFIMISC(0), 0x00000130);
#else
	reg32_write(DDRC_DFIMISC(0), 0x00000030);
#endif
#endif
	reg32_write(DDRC_SWCTL(0), 0x0001);

	/* wait DFISTAT.dfi_init_complete to 1 */
	while ((reg32_read(DDRC_DFISTAT(0)) & 0x1) == 0x0)
		;

	reg32_write(DDRC_SWCTL(0), 0x0000);

#ifdef DDR_BOOT_P2
	reg32_write(DDRC_DFIMISC(0), 0x00000210);
	/* set DFIMISC.dfi_init_complete_en again */
	reg32_write(DDRC_DFIMISC(0), 0x00000211);
#else
#ifdef DDR_BOOT_P1
	reg32_write(DDRC_DFIMISC(0), 0x00000110);
	/* set DFIMISC.dfi_init_complete_en again */
	reg32_write(DDRC_DFIMISC(0), 0x00000111);
#else
	/* clear DFIMISC.dfi_init_complete_en */
	reg32_write(DDRC_DFIMISC(0), 0x00000010);
	/* set DFIMISC.dfi_init_complete_en again */
	reg32_write(DDRC_DFIMISC(0), 0x00000011);
#endif
#endif

	reg32_write(DDRC_PWRCTL(0), 0x00000088);

	tmp = reg32_read(DDRC_CRCPARSTAT(0));

	/*
	 * set SWCTL.sw_done to enable quasi-dynamic register
	 * programming outside reset.
	 */
	reg32_write(DDRC_SWCTL(0), 0x00000001);

	/* wait SWSTAT.sw_done_ack to 1 */
	while ((reg32_read(DDRC_SWSTAT(0)) & 0x1) == 0x0)
		;

	/* wait STAT.operating_mode([1:0] for ddr3) to normal state */
	while ((reg32_read(DDRC_STAT(0)) & 0x3) != 0x1)
		;

	reg32_write(DDRC_PWRCTL(0), 0x00000088);

	tmp = reg32_read(DDRC_CRCPARSTAT(0));

	reg32_write(DDRC_PCTRL_0(0), 0x00000001);

	tmp = reg32_read(DDRC_CRCPARSTAT(0));
	reg32_write(DDRC_RFSHCTL3(0), 0x00000000);
}
