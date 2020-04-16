/*
 * Copyright 2020 Boundary Devices
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/ddr.h>
#include "bd_common.h"

struct dram_cfg_param_change {
	unsigned int reg;
	unsigned int val;
	unsigned int new_val;
};

/* Address map is from MSB 28: cs, r14, r13-r0, b2-b0, c9-c0 */
#define LPDDR4_CS	0x3	/* 1 rank bit, 2 chip selects */
#define VAL_DDRC_ADDRMAP0		0x00000016
#define VAL_DDRC_ADDRMAP6		0x0F070707

/* Address map is from MSB 28: r15, r14, r13-r0, b2-b0, c9-c0 */
#define LPDDR4_CS_NEW	0x1	/* 0 rank bits, 1 chip select */
#define VAL_DDRC_ADDRMAP0_NEW		0x0000001F
#define VAL_DDRC_ADDRMAP6_NEW		0x07070707

#if CONFIG_DDR_RANK_BITS != 0
struct dram_cfg_param_change ddrc_cfg_tbl[] = {
	{ DDRC_MSTR(0), 0xa0080020 | (LPDDR4_CS << 24), 0xa0080020 | (LPDDR4_CS_NEW << 24) },
	{ DDRC_ADDRMAP0(0), VAL_DDRC_ADDRMAP0, VAL_DDRC_ADDRMAP0_NEW },
	{ DDRC_ADDRMAP6(0), VAL_DDRC_ADDRMAP6, VAL_DDRC_ADDRMAP6_NEW },
};

struct dram_cfg_param_change ddrc_fsp_cfg_tbl[] = {
	{ 0x54012, (LPDDR4_CS << 8) | (0x0110 & 0xff), (LPDDR4_CS_NEW << 8) | (0x0110 & 0xff) },
	{ 0x5402c, LPDDR4_CS, LPDDR4_CS_NEW },
};

int fix_tbl(struct dram_cfg_param *cfg, int cfg_num, struct dram_cfg_param_change *fix, int fix_num)
{
	while (1) {
		if (cfg->reg == fix->reg) {
			if (cfg->val != fix->val) {
				printf("%s: expected %x, found %x\n", __func__, fix->val, cfg->val);
				break;
			} else {
				debug("%s: fix reg=%x, val=%x new_val=%x\n", __func__, fix->reg, fix->val, fix->new_val);
				cfg->val = fix->new_val;
			}
			fix++;
			if (!--fix_num)
				return 0;
		}
		cfg++;
		if (!--cfg_num) {
			printf("%s: did not use fix entry(%x,%x,%x)\n",
				__func__, fix->reg, fix->val, fix->new_val);
			break;
		}
	}
	return -EINVAL;
}
#endif

void spl_dram_init(void)
{
	struct dram_timing_info *dt = &dram_timing;
	/* ddr train */
	int ret = ddr_init(dt);

#if CONFIG_DDR_RANK_BITS != 0
	if (ret == -ENODEV) {
		struct dram_fsp_msg *msg = dt->fsp_msg;
		int cnt = dt->fsp_msg_num;

		printf("trying rank 0\n");
		if (fix_tbl(dt->ddrc_cfg, dt->ddrc_cfg_num, ddrc_cfg_tbl, ARRAY_SIZE(ddrc_cfg_tbl))) {
			printf("%s:error fixing ddrc_cfg\n", __func__);
			return;
		}
		while (cnt) {
			if (fix_tbl(msg->fsp_cfg, msg->fsp_cfg_num, ddrc_fsp_cfg_tbl, ARRAY_SIZE(ddrc_fsp_cfg_tbl))) {
				printf("%s:error fixing fsp_cfg\n", __func__);
				return;
			}
			msg++;
			cnt--;
		}
		ret = ddr_init(dt);
	}
#endif
	if (ret)
		printf("%s:error\n", __func__);
}
