/*
 * Copyright 2020 Boundary Devices
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <hang.h>
#include "bd_common.h"
#include "lpddr4_timing.h"

struct dram_cfg_param_change {
	unsigned int reg;
	unsigned int val;
	unsigned int new_val;
};

#ifdef LPDDR4_CS_NEW
struct dram_cfg_param_change ddrc_cfg_tbl[] = {
	{ DDRC_MSTR(0), 0xa0080020 | (LPDDR4_CS << 24), 0xa0080020 | (LPDDR4_CS_NEW << 24) },
	{ DDRC_ADDRMAP0(0), VAL_DDRC_ADDRMAP0, VAL_DDRC_ADDRMAP0_NEW },
	{ DDRC_ADDRMAP6(0), VAL_DDRC_ADDRMAP6, VAL_DDRC_ADDRMAP6_NEW },
};

struct dram_cfg_param_change ddrc_fsp_cfg_tbl[] = {
	{ 0x54012, (LPDDR4_CS << 8) | (0x0110 & 0xff), (LPDDR4_CS_NEW << 8) | (0x0110 & 0xff) },
#ifndef CONFIG_IMX8MN
	{ 0x5402c, LPDDR4_CS, LPDDR4_CS_NEW },
#endif
};

int fix_tbl(struct dram_cfg_param *cfg, int cfg_num, struct dram_cfg_param_change *fix, int fix_num)
{
	while (1) {
		if (cfg->reg == fix->reg) {
			if (cfg->val != fix->val) {
				printf("%s: expected %x, found %x\n", __func__, fix->val, cfg->val);
				break;
			} else {
				pr_debug("%s: fix reg=%x, val=%x new_val=%x\n", __func__, fix->reg, fix->val, fix->new_val);
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

#define SDRAM_SIZE	((1ULL * CONFIG_DDR_MB) << 20)
#define CNT 8
int test_ram(int show)
{
	unsigned base = CONFIG_SYS_SDRAM_BASE + (1 << 20);
	unsigned a1 = base;
	unsigned a2 = base + (SDRAM_SIZE/2);
	unsigned r1, r2;
	int i;
	unsigned ret = 0;

	for (i = 0 ; i < CNT; i++) {
		unsigned b1 = a1 + (i*4);
		unsigned b2 = a2 + (i*4);

		writel(b1, (unsigned *)(long)b1);
		writel(b2, (unsigned *)(long)b2);
	}

	/*
	 * This looks bizarre but somehow fixes memory writes
	 * when a rank1 board is first initialized as rank0
	 * before trying rank1.
	 */
#if 1
	for (i = 0 ; i < CNT; i++) {
		unsigned b1 = a1 + (i*4);
		unsigned b2 = a2 + (i*4);

		writel(b1, (unsigned *)(long)b1);
		writel(b2, (unsigned *)(long)b2);

		writel(b1, (unsigned *)(long)b1);
		writel(b2, (unsigned *)(long)b2);
	}
#endif

	for (i = 0 ; i < CNT; i++) {
		unsigned b1 = a1 + (i*4);
		unsigned b2 = a2 + (i*4);

		r1 = readl((unsigned *)(long)b1);
		r2 = readl((unsigned *)(long)b2);
		if ((r1 != b1) || (r2 != b2)) {
			if (show)
				printf("%s: %x != %x, or %x != %x\n",
					__func__, b1, r1, b2, r2);
			ret = -ENODEV;
		}
	}
	return ret;
}
#endif

void spl_dram_init(void)
{
	struct dram_timing_info *dt = &dram_timing;
	/* ddr train */
	int ret = ddr_init(dt);

#ifdef LPDDR4_CS_NEW
	if (!ret) {
		ret = test_ram(0);
	}
	if (ret == -ENODEV) {
		struct dram_fsp_msg *msg = dt->fsp_msg;
		int cnt = dt->fsp_msg_num;

		printf("trying rank %d\n", (LPDDR4_CS_NEW == 3) ? 1 : 0);
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
		if (!ret)
			ret = test_ram(1);
	}
#endif
	if (ret) {
		printf("%s:error\n", __func__);
		hang();
	}
}
