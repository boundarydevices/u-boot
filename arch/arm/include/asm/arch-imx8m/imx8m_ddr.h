/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier: GPL-2.0+
 * Common file for ddr code
 */

#ifndef __IMX8M_DDR_H__
#define __IMX8M_DDR_H__

#include <asm/io.h>
#include <asm/types.h>
#include <asm/arch/ddr_memory_map.h>

extern struct dram_timing_info lpddr4_timing;
extern struct dram_cfg_param ddrphy_trained_csr[];
extern uint32_t ddrphy_trained_csr_num;

void ddr_load_train_firmware(enum fw_type type);
void ddr_init(struct dram_timing_info *timing_info);
void ddr_cfg_phy(struct dram_timing_info *timing_info);
void load_lpddr4_phy_pie(void);
void ddrphy_trained_csr_save(struct dram_cfg_param *, unsigned int);
void dram_config_save(struct dram_timing_info *, unsigned long);

/* utils function for ddr phy training */
void wait_ddrphy_training_complete(void);
void ddrphy_init_set_dfi_clk(unsigned int drate);
void ddrphy_init_read_msg_block(enum fw_type type);


#define dwc_ddrphy_apb_wr(addr, data)	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * (addr), data)
#define dwc_ddrphy_apb_rd(addr)		reg32_read(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * (addr))

#endif /* __IMX8M_DDR_H__ */
