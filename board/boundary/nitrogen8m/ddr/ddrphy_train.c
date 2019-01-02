/*
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/ddr.h>
#include <asm/arch/imx8m_ddr.h>
#include "ddr_memory_map.h"
#include "ddr.h"
#include "lpddr4_dvfs.h"

extern void wait_ddrphy_training_complete(void);

void sscgpll_bypass_enable(unsigned int reg_addr)
{
	unsigned int read_data;
	read_data = reg32_read(reg_addr);
	reg32_write(reg_addr, read_data | 0x00000010);
	read_data = reg32_read(reg_addr);
	reg32_write(reg_addr, read_data | 0x00000020);
}

void sscgpll_bypass_disable(unsigned int reg_addr)
{
	unsigned int read_data;
	read_data = reg32_read(reg_addr);
	reg32_write(reg_addr, read_data & 0xffffffdf);
	read_data = reg32_read(reg_addr);
	reg32_write(reg_addr, read_data & 0xffffffef);
}

unsigned int wait_pll_lock(unsigned int reg_addr)
{
	unsigned int pll_lock;
	pll_lock = reg32_read(reg_addr) >> 31;
	return pll_lock;
}

void ddr_pll_config_freq(unsigned int freq)
{
	unsigned int ddr_pll_lock = 0x0;
	sscgpll_bypass_enable(HW_DRAM_PLL_CFG0_ADDR);
	switch (freq) {
	case 800:
		reg32_write(HW_DRAM_PLL_CFG2_ADDR, 0x00ece580);
		break;
	case 700:
		reg32_write(HW_DRAM_PLL_CFG2_ADDR, 0x00ec4580);
		break;
	case 667:
		reg32_write(HW_DRAM_PLL_CFG2_ADDR, 0x00ece480);
		break;
	case 400:
		reg32_write(HW_DRAM_PLL_CFG2_ADDR, 0x00ec6984);
		break;
	case 167:
		reg32_write(HW_DRAM_PLL_CFG2_ADDR, 0x00f5a406);
		break;
	case 100:
		reg32_write(HW_DRAM_PLL_CFG2_ADDR, 0x015dea96);
		break;
	default:
		printf("Input freq=%d error.\n",freq);
	}

	sscgpll_bypass_disable(HW_DRAM_PLL_CFG0_ADDR);
	while (ddr_pll_lock != 0x1) {
		ddr_pll_lock = wait_pll_lock(HW_DRAM_PLL_CFG0_ADDR);
	}
}

void dwc_ddrphy_phyinit_userCustom_E_setDfiClk(int pstate)
{
	struct ccm_reg *ccm_reg = (struct ccm_reg *)CCM_BASE_ADDR;

	if (pstate == 0x1) {
		reg32_writep(&ccm_reg->ip_root[1].target_root_clr, (0x7<<24)|(0x7<<16));
		reg32_writep(&ccm_reg->ip_root[1].target_root_set, (0x4<<24)|(0x4<<16)); /* to source 4 --800MHz/5 */
		ddr_pll_config_freq(167);
	} else {
		ddr_pll_config_freq(800);
		reg32_writep(&ccm_reg->ip_root[1].target_root_clr, (0x7<<24)|(0x7<<16));
		reg32_writep(&ccm_reg->ip_root[1].target_root_set, (0x4<<24)|(0x3<<16)); /* to source 4 --800MHz/4 */
	}
}

static void dwc_ddrphy_apb_wr_list(struct dram_cfg_param *dram_cfg, int cnt)
{
	int i;

	for (i = 0; i < cnt; i++) {
		dwc_ddrphy_apb_wr(dram_cfg->reg, dram_cfg->val);
		dram_cfg++;
	}
}

void lpddr4_800M_cfg_phy(struct dram_timing_info *dram_timing)
{
	struct dram_fsp_msg *fsp_msg;
	int i = 0;
	int drate = 0;

	printf("start to config phy: p0=3200mts, p1=667mts with 1D2D training\n");
	dwc_ddrphy_apb_wr_list(dram_timing->ddrphy_cfg,
			dram_timing->ddrphy_cfg_num);

	/* load the frequency setpoint message block config */
	fsp_msg = dram_timing->fsp_msg;
	for (i = 0; i < dram_timing->fsp_msg_num; i++) {
		/* Set the PHY input clocks for pstate */
		if (drate != fsp_msg->drate) {
			drate = fsp_msg->drate;
			dwc_ddrphy_phyinit_userCustom_E_setDfiClk((fsp_msg->drate >= 1600) ? 0 : 1);
			dwc_ddrphy_apb_wr(0xd0000, 0x0);
		}
		dwc_ddrphy_apb_wr(0xd0000, 0x0);
		ddr_load_train_firmware(fsp_msg->fw_type);
		dwc_ddrphy_apb_wr(0xd0000, 0x1);

		printf("config to do %u %ud training.\n", fsp_msg->drate,
				fsp_msg->fw_type == FW_1D_IMAGE ? 1 : 2);

		/* load the frequency set point message block parameter */
		dwc_ddrphy_apb_wr_list(fsp_msg->fsp_cfg, fsp_msg->fsp_cfg_num);

		/*
		 * -------------------- excute the firmware --------------------
		 * Running the firmware is a simply process to taking the
		 * PMU out of reset and stall, then the firwmare will be run
		 * 1. reset the PMU;
		 * 2. begin the excution;
		 * 3. wait for the training done;
		 * 4. read the message block result.
		 * -------------------------------------------------------------
		 */
		dwc_ddrphy_apb_wr(0xd0000, 0x1);
		dwc_ddrphy_apb_wr(0xd0099, 0x9);
		dwc_ddrphy_apb_wr(0xd0099, 0x1);
		dwc_ddrphy_apb_wr(0xd0099, 0x0);

		/* Wait for the training firmware to complete */
		wait_ddrphy_training_complete();

		/* Halt the microcontroller. */
		dwc_ddrphy_apb_wr(0xd0099, 0x1);

		dwc_ddrphy_apb_wr(0xd0000, 0x0);
		dwc_ddrphy_apb_wr(0xd0000, 0x1);

		fsp_msg++;
	}

	/* (I) Load PHY Init Engine Image */
	printf("Load 201711 PIE\n");
	dwc_ddrphy_apb_wr_list(dram_timing->ddrphy_pie, dram_timing->ddrphy_pie_num);
}
