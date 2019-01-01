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

#include "wait_ddrphy_training_complete.c"
#ifndef SRC_DDRC_RCR_ADDR
#define SRC_DDRC_RCR_ADDR SRC_IPS_BASE_ADDR +0x1000
#endif
#ifndef DDR_CSD1_BASE_ADDR
#define DDR_CSD1_BASE_ADDR 0x40000000
#endif
#define SILICON_TRAIN
#define DDR_BOOT_P1	/* default DDR boot frequency point */
#define WR_POST_EXT_3200
#ifdef WR_POST_EXT_3200  // recommend to define
#define VAL_INIT4	0x00330008
#else
#define VAL_INIT4	0x00310008
#endif

#if CONFIG_DDR_MB == 2048
#define VAL_DDRC_RFSHTMG		0x00610090
#define VAL_DDRC_DRAMTMG14		0x00000096
#define VAL_DDRC_FREQ1_RFSHTMG		0x0014001F
#define VAL_DDRC_FREQ1_DRAMTMG14	0x00000020
#define VAL_DDRC_FREQ2_RFSHTMG		0x00030005
#define VAL_DDRC_FREQ2_DRAMTMG14	0x00000005
	/* Address map is from MSB 28: cs, r14, r13-r0, b2-b0, c9-c0 */
#define VAL_DDRC_ADDRMAP0		0x00000016
#define VAL_DDRC_ADDRMAP6		0x0f070707
#elif CONFIG_DDR_MB == 3072
#define VAL_DDRC_RFSHTMG		0x006100E0
#define VAL_DDRC_DRAMTMG14		0x000000E6
#define VAL_DDRC_FREQ1_RFSHTMG		0x0014002F
#define VAL_DDRC_FREQ1_DRAMTMG14	0x00000031
#define VAL_DDRC_FREQ2_RFSHTMG		0x00030007
#define VAL_DDRC_FREQ2_DRAMTMG14	0x00000008
	/* Address map is from MSB 29: r15, r14, cs, r13-r0, b2-b0, c9-c0 */
#define VAL_DDRC_ADDRMAP0		0x00000015
#define VAL_DDRC_ADDRMAP6		0x48080707
#elif CONFIG_DDR_MB == 4096
#define VAL_DDRC_RFSHTMG		0x006100E0
#define VAL_DDRC_DRAMTMG14		0x000000E6
#define VAL_DDRC_FREQ1_RFSHTMG		0x0014002F
#define VAL_DDRC_FREQ1_DRAMTMG14	0x00000031
#define VAL_DDRC_FREQ2_RFSHTMG		0x00030007
#define VAL_DDRC_FREQ2_DRAMTMG14	0x00000008
	/* Address map is from MSB 29: cs, r15, r14, r13-r0, b2-b0, c9-c0 */
#define VAL_DDRC_ADDRMAP0		0x00000017
#define VAL_DDRC_ADDRMAP6		0x07070707
#else
#error unsupported memory size
#endif

volatile unsigned int tmp, tmp_t, i;
void lpddr4_800MHz_cfg_umctl2(void)
{
	/* Start to config, default 3200mbps */
	/* dis_dq=1, indicates no reads or writes are issued to SDRAM */
	 reg32_write(DDRC_DBG1(0), 0x00000001);
	/* selfref_en=1, SDRAM enter self-refresh state */
	reg32_write(DDRC_PWRCTL(0), 0x00000001);
	reg32_write(DDRC_MSTR(0), 0xa3080020);
	reg32_write(DDRC_MSTR2(0), 0x00000000);
	reg32_write(DDRC_DERATEEN(0), 0x00000203);
	reg32_write(DDRC_DERATEINT(0), 0x0186A000);
	reg32_write(DDRC_RFSHTMG(0), VAL_DDRC_RFSHTMG);
	reg32_write(DDRC_INIT0(0), 0xC003061C);
	reg32_write(DDRC_INIT1(0), 0x009E0000);
	reg32_write(DDRC_INIT3(0), 0x00D4002D);
	reg32_write(DDRC_INIT4(0), VAL_INIT4);
	reg32_write(DDRC_INIT6(0), 0x0066004A);
	reg32_write(DDRC_INIT7(0), 0x0016004A);

	reg32_write(DDRC_DRAMTMG0(0), 0x1A201B22);
	reg32_write(DDRC_DRAMTMG1(0), 0x00060633);
	reg32_write(DDRC_DRAMTMG3(0), 0x00C0C000);
	reg32_write(DDRC_DRAMTMG4(0), 0x0F04080F);
	reg32_write(DDRC_DRAMTMG5(0), 0x02040C0C);
	reg32_write(DDRC_DRAMTMG6(0), 0x01010007);
	reg32_write(DDRC_DRAMTMG7(0), 0x00000401);
	reg32_write(DDRC_DRAMTMG12(0), 0x00020600);
	reg32_write(DDRC_DRAMTMG13(0), 0x0C100002);
	reg32_write(DDRC_DRAMTMG14(0), VAL_DDRC_DRAMTMG14);
	reg32_write(DDRC_DRAMTMG17(0), 0x00A00050);

	reg32_write(DDRC_ZQCTL0(0), 0xC3200018);
	reg32_write(DDRC_ZQCTL1(0), 0x028061A8);
	reg32_write(DDRC_ZQCTL2(0), 0x00000000);

	reg32_write(DDRC_DFITMG0(0), 0x0497820A);
	reg32_write(DDRC_DFITMG1(0), 0x00080303);
	reg32_write(DDRC_DFIUPD0(0), 0xE0400018);
	reg32_write(DDRC_DFIUPD1(0), 0x00DF00E4);
	reg32_write(DDRC_DFIUPD2(0), 0x80000000);
	reg32_write(DDRC_DFIMISC(0), 0x00000011);
	reg32_write(DDRC_DFITMG2(0), 0x0000170A);

	reg32_write(DDRC_DBICTL(0), 0x00000001);
	reg32_write(DDRC_DFIPHYMSTR(0), 0x00000001);

	/* need be refined by ddrphy trained value */
	reg32_write(DDRC_RANKCTL(0), 0x639);
	reg32_write(DDRC_DRAMTMG2(0), 0x070e1214);

	/* address mapping */
	reg32_write(DDRC_ADDRMAP0(0), VAL_DDRC_ADDRMAP0);
	reg32_write(DDRC_ADDRMAP3(0), 0x00000000);
	/* addrmap_col_b10 and addrmap_col_b11 set to de-activated (5-bit width) */
	reg32_write(DDRC_ADDRMAP4(0), 0x00001F1F);
	/* bank interleave */
	/* addrmap_bank_b2, addrmap_bank_b1, addrmap_bank_b0 */
	reg32_write(DDRC_ADDRMAP1(0), 0x00080808);
	/* addrmap_row_b11, addrmap_row_b10_b2, addrmap_row_b1, addrmap_row_b0 */
	reg32_write(DDRC_ADDRMAP5(0), 0x07070707);
	/* addrmap_row_b15, addrmap_row_b14, addrmap_row_b13, addrmap_row_b12 */
	reg32_write(DDRC_ADDRMAP6(0), VAL_DDRC_ADDRMAP6);
	reg32_write(DDRC_ADDRMAP7(0), 0x00000f0f);

	/* 667mts frequency setting */
	reg32_write(DDRC_FREQ1_DERATEEN(0), 0x0000001);
	reg32_write(DDRC_FREQ1_DERATEINT(0), 0x00518B00);
	reg32_write(DDRC_FREQ1_RFSHCTL0(0), 0x0020D040);
	reg32_write(DDRC_FREQ1_RFSHTMG(0), VAL_DDRC_FREQ1_RFSHTMG);
	reg32_write(DDRC_FREQ1_INIT3(0), 0x00940009);
	reg32_write(DDRC_FREQ1_INIT4(0), VAL_INIT4);
	reg32_write(DDRC_FREQ1_INIT6(0), 0x0066004A);
	reg32_write(DDRC_FREQ1_INIT7(0), 0x0016004A);
	reg32_write(DDRC_FREQ1_DRAMTMG0(0), 0x0B070508);
	reg32_write(DDRC_FREQ1_DRAMTMG1(0), 0x0003040B);
	reg32_write(DDRC_FREQ1_DRAMTMG2(0), 0x0305090C);
	reg32_write(DDRC_FREQ1_DRAMTMG3(0), 0x00505000);
	reg32_write(DDRC_FREQ1_DRAMTMG4(0), 0x04040204);
	reg32_write(DDRC_FREQ1_DRAMTMG5(0), 0x02030303);
	reg32_write(DDRC_FREQ1_DRAMTMG6(0), 0x01010004);
	reg32_write(DDRC_FREQ1_DRAMTMG7(0), 0x00000301);
	reg32_write(DDRC_FREQ1_DRAMTMG12(0), 0x00020300);
	reg32_write(DDRC_FREQ1_DRAMTMG13(0), 0x0A100002);
	reg32_write(DDRC_FREQ1_DRAMTMG14(0), VAL_DDRC_FREQ1_DRAMTMG14);
	reg32_write(DDRC_FREQ1_DRAMTMG17(0), 0x00220011);
	reg32_write(DDRC_FREQ1_ZQCTL0(0), 0xC0A70006);
	reg32_write(DDRC_FREQ1_DFITMG0(0), 0x03858202);
	reg32_write(DDRC_FREQ1_DFITMG1(0), 0x00080303);
	reg32_write(DDRC_FREQ1_DFITMG2(0), 0x00000502);

	/* 100mts frequency setting */
	reg32_write(DDRC_FREQ2_DERATEEN(0), 0x0000001);
	reg32_write(DDRC_FREQ2_DERATEINT(0), 0x000C3500);
	reg32_write(DDRC_FREQ2_RFSHCTL0(0), 0x0020D040);
	reg32_write(DDRC_FREQ2_RFSHTMG(0), VAL_DDRC_FREQ2_RFSHTMG);
	reg32_write(DDRC_FREQ2_INIT3(0), 0x00840000);
	reg32_write(DDRC_FREQ2_INIT4(0), VAL_INIT4);
	reg32_write(DDRC_FREQ2_INIT6(0), 0x0066004A);
	reg32_write(DDRC_FREQ2_INIT7(0), 0x0016004A);
	reg32_write(DDRC_FREQ2_DRAMTMG0(0), 0x0A010102);
	reg32_write(DDRC_FREQ2_DRAMTMG1(0), 0x00030404);
	reg32_write(DDRC_FREQ2_DRAMTMG2(0), 0x0203060B);
	reg32_write(DDRC_FREQ2_DRAMTMG3(0), 0x00505000);
	reg32_write(DDRC_FREQ2_DRAMTMG4(0), 0x02040202);
	reg32_write(DDRC_FREQ2_DRAMTMG5(0), 0x02030202);
	reg32_write(DDRC_FREQ2_DRAMTMG6(0), 0x01010004);
	reg32_write(DDRC_FREQ2_DRAMTMG7(0), 0x00000301);
	reg32_write(DDRC_FREQ2_DRAMTMG12(0), 0x00020300);
	reg32_write(DDRC_FREQ2_DRAMTMG13(0), 0x0A100002);
	reg32_write(DDRC_FREQ2_DRAMTMG14(0), VAL_DDRC_FREQ2_DRAMTMG14);
	reg32_write(DDRC_FREQ2_DRAMTMG17(0), 0x00050003);
	reg32_write(DDRC_FREQ2_ZQCTL0(0), 0xC0190004);
	reg32_write(DDRC_FREQ2_DFITMG0(0), 0x03818200);
	reg32_write(DDRC_FREQ2_DFITMG1(0), 0x00080303);
	reg32_write(DDRC_FREQ2_DFITMG2(0), 0x00000100);

	/* performance setting */
	reg32_write(DDRC_ODTCFG(0), 0x0b060908);
	reg32_write(DDRC_ODTMAP(0), 0x00000000);
	reg32_write(DDRC_SCHED(0), 0x29001505);
	reg32_write(DDRC_SCHED1(0), 0x0000002c);
	reg32_write(DDRC_PERFHPR1(0), 0x5900575b);
	/* 150T starve and 0x90 max tran len */
	reg32_write(DDRC_PERFLPR1(0), 0x90000096);
	/* 300T starve and 0x10 max tran len */
	reg32_write(DDRC_PERFWR1(0), 0x1000012c);

	reg32_write(DDRC_DBG0(0), 0x00000016);
	reg32_write(DDRC_DBG1(0), 0x00000000);
	reg32_write(DDRC_DBGCMD(0), 0x00000000);
	reg32_write(DDRC_SWCTL(0), 0x00000001);
	reg32_write(DDRC_POISONCFG(0), 0x00000011);
	reg32_write(DDRC_PCCFG(0), 0x00000111);
	reg32_write(DDRC_PCFGR_0(0), 0x000010f3);
	reg32_write(DDRC_PCFGW_0(0), 0x000072ff);
	reg32_write(DDRC_PCTRL_0(0), 0x00000001);
	/* disable Read Qos*/
	reg32_write(DDRC_PCFGQOS0_0(0), 0x00000e00);
	reg32_write(DDRC_PCFGQOS1_0(0), 0x0062ffff);
	/* disable Write Qos*/
	reg32_write(DDRC_PCFGWQOS0_0(0), 0x00000e00);
	reg32_write(DDRC_PCFGWQOS1_0(0), 0x0000ffff);
}

void ddr_init1(void)
{
	struct ccm_reg *ccm_reg = (struct ccm_reg *)CCM_BASE_ADDR;

	reg32_write(SRC_DDRC_RCR_ADDR + 0x04, 0x8F00000F);
	reg32_write(SRC_DDRC_RCR_ADDR, 0x8F00000F);
	mdelay(100);
	reg32_write(SRC_DDRC_RCR_ADDR + 0x04, 0x8F000000);

	/* change the clock source of dram_apb_clk_root */
	reg32_writep(&ccm_reg->ip_root[1].target_root_clr, (0x7<<24)|(0x7<<16));
	reg32_writep(&ccm_reg->ip_root[1].target_root_set, (0x4<<24)|(0x3<<16));

	/* disable iso */
	reg32_write(0x303A00EC, 0x0000ffff); /* PGC_CPU_MAPPING */
	reg32setbit(0x303A00F8, 5); /* PU_PGC_SW_PUP_REQ */

	dram_pll_init(MHZ(800));

	reg32_write(SRC_DDRC_RCR_ADDR, 0x8F000006);

	/* Configure uMCTL2's registers */
	lpddr4_800MHz_cfg_umctl2();

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
	lpddr4_800M_cfg_phy();

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
