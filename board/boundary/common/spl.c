/*
 * Copyright 2020 Boundary Devices
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <hang.h>
#include "bd_common.h"
#if (CONFIG_DDR_CHANNEL_CNT == 1) || defined(CONFIG_DDR_CHANNEL_CNT_1_OR_2)
#include "lpddr4_timing_ch1.h"
#endif
#if (CONFIG_DDR_CHANNEL_CNT == 2) || defined(CONFIG_DDR_CHANNEL_CNT_1_OR_2)
#include "lpddr4_timing_ch2.h"
#endif

struct dram_cfg_param_change {
	unsigned int reg;
	unsigned int val;
	unsigned int new_val;
};

struct change_rank {
	int channels;
	int rank;
	struct dram_cfg_param_change *cfg_tbl;
	int cfg_tbl_cnt;
	struct dram_cfg_param_change *fsp_cfg_tbl;
	int fsp_cfg_tbl_cnt;
};

#ifdef CH1_LPDDR4_CS_NEW
#if (CONFIG_DDR_CHANNEL_CNT == 1) || defined(CONFIG_DDR_CHANNEL_CNT_1_OR_2)
static struct dram_cfg_param_change ddrc_cfg_tbl_ch1[] = {
#if !defined(CONFIG_IMX8MN)
	{ DDRC_MSTR(0), 0xa0081020 | (CH1_LPDDR4_CS << 24), 0xa0081020 | (CH1_LPDDR4_CS_NEW << 24) },
#else
	{ DDRC_MSTR(0), 0xa0080020 | (CH1_LPDDR4_CS << 24), 0xa0080020 | (CH1_LPDDR4_CS_NEW << 24) },
#endif
	{ DDRC_ADDRMAP0(0), CH1_VAL_DDRC_ADDRMAP0, CH1_VAL_DDRC_ADDRMAP0_NEW },
	{ DDRC_ADDRMAP6(0), CH1_VAL_DDRC_ADDRMAP6, CH1_VAL_DDRC_ADDRMAP6_NEW },
};

static struct dram_cfg_param_change ddrc_fsp_cfg_tbl_ch1[] = {
	{ 0x54012, (CH1_LPDDR4_CS << 8) | (0x0110 & 0xff), (CH1_LPDDR4_CS_NEW << 8) | (0x0110 & 0xff) },
};
static struct change_rank change_rank_ch1 = {
	1, (CH1_LPDDR4_CS_NEW == 3) ? 1 : 0,
	ddrc_cfg_tbl_ch1, ARRAY_SIZE(ddrc_cfg_tbl_ch1),
	ddrc_fsp_cfg_tbl_ch1, ARRAY_SIZE(ddrc_fsp_cfg_tbl_ch1)
};
#endif
#endif

#ifdef CH2_LPDDR4_CS_NEW
#if (CONFIG_DDR_CHANNEL_CNT == 2) || defined(CONFIG_DDR_CHANNEL_CNT_1_OR_2)
static struct dram_cfg_param_change ddrc_cfg_tbl_ch2[] = {
	{ DDRC_MSTR(0), 0xa0080020 | (CH2_LPDDR4_CS << 24), 0xa0080020 | (CH2_LPDDR4_CS_NEW << 24) },
	{ DDRC_ADDRMAP0(0), CH2_VAL_DDRC_ADDRMAP0, CH2_VAL_DDRC_ADDRMAP0_NEW },
	{ DDRC_ADDRMAP6(0), CH2_VAL_DDRC_ADDRMAP6, CH2_VAL_DDRC_ADDRMAP6_NEW },
};

static struct dram_cfg_param_change ddrc_fsp_cfg_tbl_ch2[] = {
	{ 0x54012, (CH2_LPDDR4_CS << 8) | (0x0110 & 0xff), (CH2_LPDDR4_CS_NEW << 8) | (0x0110 & 0xff) },
	{ 0x5402c, CH2_LPDDR4_CS, CH2_LPDDR4_CS_NEW },
};
static struct change_rank change_rank_ch2 = {
	2, (CH2_LPDDR4_CS_NEW == 3) ? 1 : 0,
	ddrc_cfg_tbl_ch2, ARRAY_SIZE(ddrc_cfg_tbl_ch2),
	ddrc_fsp_cfg_tbl_ch2, ARRAY_SIZE(ddrc_fsp_cfg_tbl_ch2)
};
#endif
#endif

#if defined(CH1_LPDDR4_CS_NEW) || defined(CH2_LPDDR4_CS_NEW)
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
#endif

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

int ddr_init_test(struct dram_timing_info *dt, int show)
{
	int ret = ddr_init(dt);

	if (!ret) {
		ret = test_ram(show);
	}
	return ret;
}

#if defined(CH1_LPDDR4_CS_NEW) || defined(CH2_LPDDR4_CS_NEW)
int other_rank_ddr_init(struct dram_timing_info *dt, struct change_rank *cr, int show)
{
	struct dram_fsp_msg *msg = dt->fsp_msg;
	int cnt = dt->fsp_msg_num;
	int ret;

	printf("trying channels %d, rank %d\n", cr->channels, cr->rank);
	ret = fix_tbl(dt->ddrc_cfg, dt->ddrc_cfg_num,
			cr->cfg_tbl, cr->cfg_tbl_cnt);
	if (ret) {
		printf("%s:error fixing ddrc_cfg\n", __func__);
		return -ENODEV;
	}
	while (cnt) {
		ret = fix_tbl(msg->fsp_cfg, msg->fsp_cfg_num,
			cr->fsp_cfg_tbl, cr->fsp_cfg_tbl_cnt);
		if (ret) {
			printf("%s:error fixing fsp_cfg\n", __func__);
			return -ENODEV;
		}
		msg++;
		cnt--;
	}
	return ddr_init_test(dt, show);
}
#endif

void spl_dram_init(void)
{
#if (CONFIG_DDR_CHANNEL_CNT == 1)
	struct dram_timing_info *dt = &dram_timing_ch1;
#ifdef CH1_LPDDR4_CS_NEW
	struct change_rank *cr = &change_rank_ch1;
#define CR
#endif
#else
	struct dram_timing_info *dt = &dram_timing_ch2;
#ifdef CH2_LPDDR4_CS_NEW
	struct change_rank *cr = &change_rank_ch2;
#define CR
#endif
#endif

#if defined(CONFIG_DDR_CHANNEL_CNT_1_OR_2)
#if (CONFIG_DDR_CHANNEL_CNT == 1)
	struct dram_timing_info *dt2 = &dram_timing_ch2;
#ifdef CH2_LPDDR4_CS_NEW
	struct change_rank *cr2 = &change_rank_ch2;
#define CR2
#endif
#else
	struct dram_timing_info *dt2 = &dram_timing_ch1;
#ifdef CH1_LPDDR4_CS_NEW
	struct change_rank *cr2 = &change_rank_ch1;
#define CR2
#endif
#endif
#endif
	/* ddr train */
	int ret = ddr_init_test(dt, 0);

#if defined(CONFIG_DDR_CHANNEL_CNT_1_OR_2)
	if (ret == -ENODEV) {
#if (CONFIG_DDR_CHANNEL_CNT == 1)
		printf("trying channels 2, rank %d\n", (CH2_LPDDR4_CS == 3) ? 1 : 0);
#else
		printf("trying channels 1, rank %d\n", (CH1_LPDDR4_CS == 3) ? 1 : 0);
#endif
		ret = ddr_init_test(dt2, 0);
		if (!ret)
			dt = dt2;
	}
#endif
#ifdef CR
	if (ret == -ENODEV) {
		ret = other_rank_ddr_init(dt, cr, 1);
	}
#endif
#ifdef CR2
	if (ret == -ENODEV) {
		ret = other_rank_ddr_init(dt2, cr2, 1);
		if (!ret)
			dt = dt2;
	}
#endif
	if (ret) {
		printf("%s:error\n", __func__);
		hang();
	}
#if CONFIG_SAVED_DRAM_TIMING_BASE >= CONFIG_SYS_SDRAM_BASE
	/*
	 * save the dram timing config into memory again,
	 * in case it had trouble the 1st time with changing ranks
	 */
	dram_config_save(dt, CONFIG_SAVED_DRAM_TIMING_BASE);
#endif
}
