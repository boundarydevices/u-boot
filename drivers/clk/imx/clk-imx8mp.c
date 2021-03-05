// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP
 * Peng Fan <peng.fan@nxp.com>
 */

#include <common.h>
#include <clk.h>
#include <clk-uclass.h>
#include <dm.h>
#include <log.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <dt-bindings/clock/imx8mp-clock.h>

#include "clk.h"

static u32 share_count_media;

#define PLL_1416X_RATE(_rate, _m, _p, _s)		\
	{						\
		.rate	=	(_rate),		\
		.mdiv	=	(_m),			\
		.pdiv	=	(_p),			\
		.sdiv	=	(_s),			\
	}

#define PLL_1443X_RATE(_rate, _m, _p, _s, _k)		\
	{						\
		.rate	=	(_rate),		\
		.mdiv	=	(_m),			\
		.pdiv	=	(_p),			\
		.sdiv	=	(_s),			\
		.kdiv	=	(_k),			\
	}

static const struct imx_pll14xx_rate_table imx8mp_pll1416x_tbl[] = {
	PLL_1416X_RATE(1800000000U, 225, 3, 0),
	PLL_1416X_RATE(1600000000U, 200, 3, 0),
	PLL_1416X_RATE(1200000000U, 300, 3, 1),
	PLL_1416X_RATE(1000000000U, 250, 3, 1),
	PLL_1416X_RATE(800000000U,  200, 3, 1),
	PLL_1416X_RATE(750000000U,  250, 2, 2),
	PLL_1416X_RATE(700000000U,  350, 3, 2),
	PLL_1416X_RATE(600000000U,  300, 3, 2),
};

static const struct imx_pll14xx_rate_table imx8mp_videopll_tbl[] = {
	PLL_1443X_RATE(1039500000U, 173, 2, 1, 16384),
	PLL_1443X_RATE(650000000U, 325, 3, 2, 0),
	PLL_1443X_RATE(594000000U, 198, 2, 2, 0),
	PLL_1443X_RATE(519750000U, 173, 2, 2, 16384),
};

static const struct imx_pll14xx_rate_table imx8mp_drampll_tbl[] = {
	PLL_1443X_RATE(650000000U, 325, 3, 2, 0),
};

static struct imx_pll14xx_clk imx8mp_video_pll = {
		.type = PLL_1443X,
		.rate_table = imx8mp_videopll_tbl,
		.rate_count = ARRAY_SIZE(imx8mp_videopll_tbl),
};

static struct imx_pll14xx_clk imx8mp_dram_pll __initdata = {
		.type = PLL_1443X,
		.rate_table = imx8mp_drampll_tbl,
		.rate_count = ARRAY_SIZE(imx8mp_drampll_tbl),
};

static struct imx_pll14xx_clk imx8mp_arm_pll __initdata = {
		.type = PLL_1416X,
		.rate_table = imx8mp_pll1416x_tbl,
		.rate_count = ARRAY_SIZE(imx8mp_pll1416x_tbl),
};

static struct imx_pll14xx_clk imx8mp_sys_pll __initdata = {
		.type = PLL_1416X,
		.rate_table = imx8mp_pll1416x_tbl,
		.rate_count = ARRAY_SIZE(imx8mp_pll1416x_tbl),
};

static const char *pll_ref_sels[] = { "clock-osc-24m", "dummy", "dummy", "dummy", };
static const char *video_pll1_bypass_sels[] = {"video_pll1", "video_pll1_ref_sel", };
static const char *dram_pll_bypass_sels[] = {"dram_pll", "dram_pll_ref_sel", };
static const char *arm_pll_bypass_sels[] = {"arm_pll", "arm_pll_ref_sel", };
static const char *sys_pll1_bypass_sels[] = {"sys_pll1", "sys_pll1_ref_sel", };
static const char *sys_pll2_bypass_sels[] = {"sys_pll2", "sys_pll2_ref_sel", };
static const char *sys_pll3_bypass_sels[] = {"sys_pll3", "sys_pll3_ref_sel", };

static const char *imx8mp_a53_sels[] = {"clock-osc-24m", "arm_pll_out", "sys_pll2_500m",
					"sys_pll2_1000m", "sys_pll1_800m", "sys_pll1_400m",
					"audio_pll1_out", "sys_pll3_out", };

static const char *imx8mp_main_axi_sels[] = {"clock-osc-24m", "sys_pll2_333m", "sys_pll1_800m",
					     "sys_pll2_250m", "sys_pll2_1000m", "audio_pll1_out",
					     "video_pll1_out", "sys_pll1_100m",};

static const char *imx8mp_enet_axi_sels[] = {"clock-osc-24m", "sys_pll1_266m", "sys_pll1_800m",
					     "sys_pll2_250m", "sys_pll2_200m", "audio_pll1_out",
					     "video_pll1_out", "sys_pll3_out", };

static const char *imx8mp_nand_usdhc_sels[] = {"clock-osc-24m", "sys_pll1_266m", "sys_pll1_800m",
					       "sys_pll2_200m", "sys_pll1_133m", "sys_pll3_out",
					       "sys_pll2_250m", "audio_pll1_out", };

static const char *imx8mp_media_axi_sels[] = {"clock-osc-24m", "sys_pll2_1000m", "sys_pll1_800m",
					      "sys_pll3_out", "sys_pll1_40m", "audio_pll2_out",
					      "clk_ext1", "sys_pll2_500m", };

static const char *imx8mp_media_apb_sels[] = {"clock-osc-24m", "sys_pll2_125m", "sys_pll1_800m",
					      "sys_pll3_out", "sys_pll1_40m", "audio_pll2_out",
					      "clk_ext1", "sys_pll1_133m", };

static const char *imx8mp_noc_sels[] = {"clock-osc-24m", "sys_pll1_800m", "sys_pll3_out",
					"sys_pll2_1000m", "sys_pll2_500m", "audio_pll1_out",
					"video_pll1_out", "audio_pll2_out", };

static const char *imx8mp_noc_io_sels[] = {"clock-osc-24m", "sys_pll1_800m", "sys_pll3_out",
					   "sys_pll2_1000m", "sys_pll2_500m", "audio_pll1_out",
					   "video_pll1_out", "audio_pll2_out", };

static const char *imx8mp_ahb_sels[] = {"clock-osc-24m", "sys_pll1_133m", "sys_pll1_800m",
					"sys_pll1_400m", "sys_pll2_125m", "sys_pll3_out",
					"audio_pll1_out", "video_pll1_out", };

static const char *imx8mp_media_disp2_pix_sels[] = {"clock-osc-24m", "video_pll1_out", "audio_pll2_out",
						    "audio_pll1_out", "sys_pll1_800m", "sys_pll2_1000m",
						    "sys_pll3_out", "clk_ext4", };

static const char *imx8mp_dram_alt_sels[] = {"clock-osc-24m", "sys_pll1_800m", "sys_pll1_100m",
					     "sys_pll2_500m", "sys_pll2_1000m", "sys_pll3_out",
					     "audio_pll1_out", "sys_pll1_266m", };

static const char *imx8mp_dram_apb_sels[] = {"clock-osc-24m", "sys_pll2_200m", "sys_pll1_40m",
					     "sys_pll1_160m", "sys_pll1_800m", "sys_pll3_out",
					     "sys_pll2_250m", "audio_pll2_out", };

static const char *imx8mp_i2c5_sels[] = {"clock-osc-24m", "sys_pll1_160m", "sys_pll2_50m",
					 "sys_pll3_out", "audio_pll1_out", "video_pll1_out",
					 "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mp_i2c6_sels[] = {"clock-osc-24m", "sys_pll1_160m", "sys_pll2_50m",
					 "sys_pll3_out", "audio_pll1_out", "video_pll1_out",
					 "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mp_usdhc1_sels[] = {"clock-osc-24m", "sys_pll1_400m", "sys_pll1_800m",
					   "sys_pll2_500m", "sys_pll3_out", "sys_pll1_266m",
					   "audio_pll2_out", "sys_pll1_100m", };

static const char *imx8mp_usdhc2_sels[] = {"clock-osc-24m", "sys_pll1_400m", "sys_pll1_800m",
					   "sys_pll2_500m", "sys_pll3_out", "sys_pll1_266m",
					   "audio_pll2_out", "sys_pll1_100m", };

static const char *imx8mp_i2c1_sels[] = {"clock-osc-24m", "sys_pll1_160m", "sys_pll2_50m",
					 "sys_pll3_out", "audio_pll1_out", "video_pll1_out",
					 "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mp_i2c2_sels[] = {"clock-osc-24m", "sys_pll1_160m", "sys_pll2_50m",
					 "sys_pll3_out", "audio_pll1_out", "video_pll1_out",
					 "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mp_i2c3_sels[] = {"clock-osc-24m", "sys_pll1_160m", "sys_pll2_50m",
					 "sys_pll3_out", "audio_pll1_out", "video_pll1_out",
					 "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mp_i2c4_sels[] = {"clock-osc-24m", "sys_pll1_160m", "sys_pll2_50m",
					 "sys_pll3_out", "audio_pll1_out", "video_pll1_out",
					 "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mp_uart1_sels[] = {"clock-osc-24m", "sys_pll1_80m", "sys_pll2_200m",
					  "sys_pll2_100m", "sys_pll3_out", "clk_ext2",
					  "clk_ext4", "audio_pll2_out", };

static const char *imx8mp_uart2_sels[] = {"clock-osc-24m", "sys_pll1_80m", "sys_pll2_200m",
					  "sys_pll2_100m", "sys_pll3_out", "clk_ext2",
					  "clk_ext3", "audio_pll2_out", };

static const char *imx8mp_uart3_sels[] = {"clock-osc-24m", "sys_pll1_80m", "sys_pll2_200m",
					  "sys_pll2_100m", "sys_pll3_out", "clk_ext2",
					  "clk_ext4", "audio_pll2_out", };

static const char *imx8mp_uart4_sels[] = {"clock-osc-24m", "sys_pll1_80m", "sys_pll2_200m",
					  "sys_pll2_100m", "sys_pll3_out", "clk_ext2",
					  "clk_ext3", "audio_pll2_out", };

static const char *imx8mp_gic_sels[] = {"clock-osc-24m", "sys_pll2_200m", "sys_pll1_40m",
					"sys_pll2_100m", "sys_pll1_800m",
					"sys_pll2_500m", "clk_ext4", "audio_pll2_out" };

static const char *imx8mp_pwm1_sels[] = {"clock-osc-24m", "sys_pll2_100m", "sys_pll1_160m",
					 "sys_pll1_40m", "sys_pll3_out", "clk_ext1",
					 "sys_pll1_80m", "video_pll1_out", };

static const char *imx8mp_pwm2_sels[] = {"clock-osc-24m", "sys_pll2_100m", "sys_pll1_160m",
					 "sys_pll1_40m", "sys_pll3_out", "clk_ext1",
					 "sys_pll1_80m", "video_pll1_out", };

static const char *imx8mp_pwm3_sels[] = {"clock-osc-24m", "sys_pll2_100m", "sys_pll1_160m",
					 "sys_pll1_40m", "sys_pll3_out", "clk_ext2",
					 "sys_pll1_80m", "video_pll1_out", };

static const char *imx8mp_pwm4_sels[] = {"clock-osc-24m", "sys_pll2_100m", "sys_pll1_160m",
					 "sys_pll1_40m", "sys_pll3_out", "clk_ext2",
					 "sys_pll1_80m", "video_pll1_out", };

static const char *imx8mp_wdog_sels[] = {"clock-osc-24m", "sys_pll1_133m", "sys_pll1_160m",
					 "vpu_pll_out", "sys_pll2_125m", "sys_pll3_out",
					 "sys_pll1_80m", "sys_pll2_166m" };

static const char *imx8mp_qspi_sels[] = {"clock-osc-24m", "sys_pll1_400m", "sys_pll2_333m",
					 "sys_pll2_500m", "audio_pll2_out", "sys_pll1_266m",
					 "sys_pll3_out", "sys_pll1_100m", };

static const char *imx8mp_hdmi_fdcc_tst_sels[] = {"clock-osc-24m", "sys_pll1_266m", "sys_pll2_250m",
						  "sys_pll1_800m", "sys_pll2_1000m", "sys_pll3_out",
						  "audio_pll2_out", "video_pll1_out", };

static const char *imx8mp_hdmi_24m_sels[] = {"clock-osc-24m", "sys_pll1_160m", "sys_pll2_50m",
					     "sys_pll3_out", "audio_pll1_out", "video_pll1_out",
					     "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mp_hdmi_ref_266m_sels[] = {"clock-osc-24m", "sys_pll1_400m", "sys_pll3_out",
						  "sys_pll2_333m", "sys_pll1_266m", "sys_pll2_200m",
						  "audio_pll1_out", "video_pll1_out", };

static const char *imx8mp_usdhc3_sels[] = {"clock-osc-24m", "sys_pll1_400m", "sys_pll1_800m",
					   "sys_pll2_500m", "sys_pll3_out", "sys_pll1_266m",
					   "audio_pll2_out", "sys_pll1_100m", };

static const char *imx8mp_media_mipi_phy1_ref_sels[] = {"clock-osc-24m", "sys_pll2_333m", "sys_pll2_100m",
							"sys_pll1_800m", "sys_pll2_1000m", "clk_ext2",
							"audio_pll2_out", "video_pll1_out", };

static const char *imx8mp_media_disp1_pix_sels[] = {"clock-osc-24m", "video_pll1_out", "audio_pll2_out",
						    "audio_pll1_out", "sys_pll1_800m", "sys_pll2_1000m",
						    "sys_pll3_out", "clk_ext4", };

static const char *imx8mp_media_ldb_sels[] = {"clock-osc-24m", "sys_pll2_333m", "sys_pll2_100m",
							"sys_pll1_800m", "sys_pll2_1000m", "clk_ext2",
							"audio_pll2_out", "video_pll1_out", };

static const char *imx8mp_enet_ref_sels[] = {"clock-osc-24m", "sys_pll2_125m", "sys_pll2_50m",
					     "sys_pll2_100m", "sys_pll1_160m", "audio_pll1_out",
					     "video_pll1_out", "clk_ext4", };

static const char *imx8mp_enet_timer_sels[] = {"clock-osc-24m", "sys_pll2_100m", "audio_pll1_out",
					       "clk_ext1", "clk_ext2", "clk_ext3",
					       "clk_ext4", "video_pll1_out", };

static const char *imx8mp_enet_phy_ref_sels[] = {"clock-osc-24m", "sys_pll2_50m", "sys_pll2_125m",
						 "sys_pll2_200m", "sys_pll2_500m", "audio_pll1_out",
						 "video_pll1_out", "audio_pll2_out", };

static const char *imx8mp_dram_core_sels[] = {"dram_pll_out", "dram_alt_root", };


static ulong imx8mp_clk_get_rate(struct clk *clk)
{
	struct clk *c;
	int ret;

	debug("%s(#%lu)\n", __func__, clk->id);

	ret = clk_get_by_id(clk->id, &c);
	if (ret)
		return ret;

	return clk_get_rate(c);
}

static ulong imx8mp_clk_set_rate(struct clk *clk, unsigned long rate)
{
	struct clk *c;
	int ret;

	debug("%s(#%lu), rate: %lu\n", __func__, clk->id, rate);

	ret = clk_get_by_id(clk->id, &c);
	if (ret)
		return ret;

	return clk_set_rate(c, rate);
}

static int __imx8mp_clk_enable(struct clk *clk, bool enable)
{
	struct clk *c;
	int ret;

	debug("%s(#%lu) en: %d\n", __func__, clk->id, enable);

	ret = clk_get_by_id(clk->id, &c);
	if (ret)
		return ret;

	if (enable)
		ret = clk_enable(c);
	else
		ret = clk_disable(c);

	return ret;
}

static int imx8mp_clk_disable(struct clk *clk)
{
	return __imx8mp_clk_enable(clk, 0);
}

static int imx8mp_clk_enable(struct clk *clk)
{
	return __imx8mp_clk_enable(clk, 1);
}

static int imx8mp_clk_set_parent(struct clk *clk, struct clk *parent)
{
	struct clk *c, *cp;
	int ret;

	debug("%s(#%lu), parent: %lu\n", __func__, clk->id, parent->id);

	ret = clk_get_by_id(clk->id, &c);
	if (ret)
		return ret;

	ret = clk_get_by_id(parent->id, &cp);
	if (ret)
		return ret;

	ret = clk_set_parent(c, cp);
	if (!ret) {
		list_del(&c->dev->sibling_node);
		list_add_tail(&c->dev->sibling_node, &cp->dev->child_head);
		c->dev->parent = cp->dev;
	} else {
		printf("%s: %s %s: %d failed\n", __func__,
				c->dev->name, cp->dev->name, ret);
	}
	debug("%s(#%lu)%s, parent: (%lu)%s\n", __func__, clk->id, c->dev->name,
			parent->id, cp->dev->name);

	return ret;
}

static struct clk_ops imx8mp_clk_ops = {
	.set_rate = imx8mp_clk_set_rate,
	.get_rate = imx8mp_clk_get_rate,
	.enable = imx8mp_clk_enable,
	.disable = imx8mp_clk_disable,
	.set_parent = imx8mp_clk_set_parent,
};

static inline void clk_dm2(ulong id, ofnode np, const char *name)
{
	struct clk clk_tmp;
	struct clk *pclk;
	int ret = clk_get_by_name_nodev(np, name, &clk_tmp);

	if (!ret) {
		pclk = dev_get_clk_ptr(clk_tmp.dev);
		pclk->id = id;
	} else {
		printf("%s: %s not found\n", __func__, name);
	}
}

static int imx8mp_clk_probe(struct udevice *dev)
{
	ofnode np = dev_ofnode(dev);
	void __iomem *base;

	base = (void *)ANATOP_BASE_ADDR;

	clk_dm2(IMX8MP_CLK_32K, np, "osc_32k");
	clk_dm2(IMX8MP_CLK_24M, np, "osc_24m");
	clk_dm2(IMX8MP_CLK_EXT1, np, "clk_ext1");
	clk_dm2(IMX8MP_CLK_EXT2, np, "clk_ext2");
	clk_dm2(IMX8MP_CLK_EXT3, np, "clk_ext3");
	clk_dm2(IMX8MP_CLK_EXT4, np, "clk_ext4");

	clk_dm(IMX8MP_VIDEO_PLL1_REF_SEL, imx_clk_mux("video_pll1_ref_sel", base + 0x28, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels)));
	clk_dm(IMX8MP_DRAM_PLL_REF_SEL, imx_clk_mux("dram_pll_ref_sel", base + 0x50, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels)));
	clk_dm(IMX8MP_ARM_PLL_REF_SEL, imx_clk_mux("arm_pll_ref_sel", base + 0x84, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels)));
	clk_dm(IMX8MP_SYS_PLL1_REF_SEL, imx_clk_mux("sys_pll1_ref_sel", base + 0x94, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels)));
	clk_dm(IMX8MP_SYS_PLL2_REF_SEL, imx_clk_mux("sys_pll2_ref_sel", base + 0x104, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels)));
	clk_dm(IMX8MP_SYS_PLL3_REF_SEL, imx_clk_mux("sys_pll3_ref_sel", base + 0x114, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels)));

	clk_dm(IMX8MP_VIDEO_PLL1, imx_clk_pll14xx("video_pll1", "video_pll1_ref_sel", base + 0x28, &imx8mp_video_pll));
	clk_dm(IMX8MP_DRAM_PLL, imx_clk_pll14xx("dram_pll", "dram_pll_ref_sel", base + 0x50, &imx8mp_dram_pll));
	clk_dm(IMX8MP_ARM_PLL, imx_clk_pll14xx("arm_pll", "arm_pll_ref_sel", base + 0x84, &imx8mp_arm_pll));
	clk_dm(IMX8MP_SYS_PLL1, imx_clk_pll14xx("sys_pll1", "sys_pll1_ref_sel", base + 0x94, &imx8mp_sys_pll));
	clk_dm(IMX8MP_SYS_PLL2, imx_clk_pll14xx("sys_pll2", "sys_pll2_ref_sel", base + 0x104, &imx8mp_sys_pll));
	clk_dm(IMX8MP_SYS_PLL3, imx_clk_pll14xx("sys_pll3", "sys_pll3_ref_sel", base + 0x114, &imx8mp_sys_pll));

	clk_dm(IMX8MP_VIDEO_PLL1_BYPASS, imx_clk_mux_flags("video_pll1_bypass", base + 0x28, 4, 1, video_pll1_bypass_sels, ARRAY_SIZE(video_pll1_bypass_sels), CLK_SET_RATE_PARENT));
	clk_dm(IMX8MP_DRAM_PLL_BYPASS, imx_clk_mux_flags("dram_pll_bypass", base + 0x50, 4, 1, dram_pll_bypass_sels, ARRAY_SIZE(dram_pll_bypass_sels), CLK_SET_RATE_PARENT));
	clk_dm(IMX8MP_ARM_PLL_BYPASS, imx_clk_mux_flags("arm_pll_bypass", base + 0x84, 4, 1, arm_pll_bypass_sels, ARRAY_SIZE(arm_pll_bypass_sels), CLK_SET_RATE_PARENT));
	clk_dm(IMX8MP_SYS_PLL1_BYPASS, imx_clk_mux_flags("sys_pll1_bypass", base + 0x94, 4, 1, sys_pll1_bypass_sels, ARRAY_SIZE(sys_pll1_bypass_sels), CLK_SET_RATE_PARENT));
	clk_dm(IMX8MP_SYS_PLL2_BYPASS, imx_clk_mux_flags("sys_pll2_bypass", base + 0x104, 4, 1, sys_pll2_bypass_sels, ARRAY_SIZE(sys_pll2_bypass_sels), CLK_SET_RATE_PARENT));
	clk_dm(IMX8MP_SYS_PLL3_BYPASS, imx_clk_mux_flags("sys_pll3_bypass", base + 0x114, 4, 1, sys_pll3_bypass_sels, ARRAY_SIZE(sys_pll3_bypass_sels), CLK_SET_RATE_PARENT));

	clk_dm(IMX8MP_VIDEO_PLL1_OUT, imx_clk_gate("video_pll1_out", "video_pll1_bypass", base + 0x28, 13));
	clk_dm(IMX8MP_DRAM_PLL_OUT, imx_clk_gate("dram_pll_out", "dram_pll_bypass", base + 0x50, 13));
	clk_dm(IMX8MP_ARM_PLL_OUT, imx_clk_gate("arm_pll_out", "arm_pll_bypass", base + 0x84, 11));
	clk_dm(IMX8MP_SYS_PLL1_OUT, imx_clk_gate("sys_pll1_out", "sys_pll1_bypass", base + 0x94, 11));
	clk_dm(IMX8MP_SYS_PLL2_OUT, imx_clk_gate("sys_pll2_out", "sys_pll2_bypass", base + 0x104, 11));
	clk_dm(IMX8MP_SYS_PLL3_OUT, imx_clk_gate("sys_pll3_out", "sys_pll3_bypass", base + 0x114, 11));

	clk_dm(IMX8MP_SYS_PLL1_40M, imx_clk_fixed_factor("sys_pll1_40m", "sys_pll1_out", 1, 20));
	clk_dm(IMX8MP_SYS_PLL1_80M, imx_clk_fixed_factor("sys_pll1_80m", "sys_pll1_out", 1, 10));
	clk_dm(IMX8MP_SYS_PLL1_100M, imx_clk_fixed_factor("sys_pll1_100m", "sys_pll1_out", 1, 8));
	clk_dm(IMX8MP_SYS_PLL1_133M, imx_clk_fixed_factor("sys_pll1_133m", "sys_pll1_out", 1, 6));
	clk_dm(IMX8MP_SYS_PLL1_160M, imx_clk_fixed_factor("sys_pll1_160m", "sys_pll1_out", 1, 5));
	clk_dm(IMX8MP_SYS_PLL1_200M, imx_clk_fixed_factor("sys_pll1_200m", "sys_pll1_out", 1, 4));
	clk_dm(IMX8MP_SYS_PLL1_266M, imx_clk_fixed_factor("sys_pll1_266m", "sys_pll1_out", 1, 3));
	clk_dm(IMX8MP_SYS_PLL1_400M, imx_clk_fixed_factor("sys_pll1_400m", "sys_pll1_out", 1, 2));
	clk_dm(IMX8MP_SYS_PLL1_800M, imx_clk_fixed_factor("sys_pll1_800m", "sys_pll1_out", 1, 1));

	clk_dm(IMX8MP_SYS_PLL2_50M, imx_clk_fixed_factor("sys_pll2_50m", "sys_pll2_out", 1, 20));
	clk_dm(IMX8MP_SYS_PLL2_100M, imx_clk_fixed_factor("sys_pll2_100m", "sys_pll2_out", 1, 10));
	clk_dm(IMX8MP_SYS_PLL2_125M, imx_clk_fixed_factor("sys_pll2_125m", "sys_pll2_out", 1, 8));
	clk_dm(IMX8MP_SYS_PLL2_166M, imx_clk_fixed_factor("sys_pll2_166m", "sys_pll2_out", 1, 6));
	clk_dm(IMX8MP_SYS_PLL2_200M, imx_clk_fixed_factor("sys_pll2_200m", "sys_pll2_out", 1, 5));
	clk_dm(IMX8MP_SYS_PLL2_250M, imx_clk_fixed_factor("sys_pll2_250m", "sys_pll2_out", 1, 4));
	clk_dm(IMX8MP_SYS_PLL2_333M, imx_clk_fixed_factor("sys_pll2_333m", "sys_pll2_out", 1, 3));
	clk_dm(IMX8MP_SYS_PLL2_500M, imx_clk_fixed_factor("sys_pll2_500m", "sys_pll2_out", 1, 2));
	clk_dm(IMX8MP_SYS_PLL2_1000M, imx_clk_fixed_factor("sys_pll2_1000m", "sys_pll2_out", 1, 1));

	base = dev_read_addr_ptr(dev);
	if (!base)
		return -EINVAL;

	clk_dm(IMX8MP_CLK_A53_SRC, imx_clk_mux2("arm_a53_src", base + 0x8000, 24, 3, imx8mp_a53_sels, ARRAY_SIZE(imx8mp_a53_sels)));
	clk_dm(IMX8MP_CLK_A53_CG, imx_clk_gate3("arm_a53_cg", "arm_a53_src", base + 0x8000, 28));
	clk_dm(IMX8MP_CLK_A53_DIV, imx_clk_divider2("arm_a53_div", "arm_a53_cg", base + 0x8000, 0, 3));

	clk_dm(IMX8MP_CLK_MAIN_AXI, imx8m_clk_composite_critical("main_axi", imx8mp_main_axi_sels, base + 0x8800));
	clk_dm(IMX8MP_CLK_ENET_AXI, imx8m_clk_composite_critical("enet_axi", imx8mp_enet_axi_sels, base + 0x8880));
	clk_dm(IMX8MP_CLK_NAND_USDHC_BUS, imx8m_clk_composite_critical("nand_usdhc_bus", imx8mp_nand_usdhc_sels, base + 0x8900));
	clk_dm(IMX8MP_CLK_MEDIA_AXI, imx8m_clk_composite("media_axi", imx8mp_media_axi_sels, base + 0x8a00));
	clk_dm(IMX8MP_CLK_MEDIA_APB, imx8m_clk_composite("media_apb", imx8mp_media_apb_sels, base + 0x8a80));
	clk_dm(IMX8MP_CLK_HDMI_APB, imx8m_clk_composite("hdmi_apb", imx8mp_media_apb_sels, base + 0x8b00));
	clk_dm(IMX8MP_CLK_HDMI_AXI, imx8m_clk_composite("hdmi_axi", imx8mp_media_axi_sels, base + 0x8b80));
	clk_dm(IMX8MP_CLK_NOC, imx8m_clk_composite_critical("noc", imx8mp_noc_sels, base + 0x8d00));
	clk_dm(IMX8MP_CLK_NOC_IO, imx8m_clk_composite_critical("noc_io", imx8mp_noc_io_sels, base + 0x8d80));

	clk_dm(IMX8MP_CLK_AHB, imx8m_clk_composite_critical("ahb_root", imx8mp_ahb_sels, base + 0x9000));

	clk_dm(IMX8MP_CLK_IPG_ROOT, imx_clk_divider2("ipg_root", "ahb_root", base + 0x9080, 0, 1));
	clk_dm(IMX8MP_CLK_MEDIA_DISP2_PIX, imx8m_clk_composite("media_disp2_pix", imx8mp_media_disp2_pix_sels, base + 0x9300));

	clk_dm(IMX8MP_CLK_DRAM_ALT, imx8m_clk_composite("dram_alt", imx8mp_dram_alt_sels, base + 0xa000));
	clk_dm(IMX8MP_CLK_DRAM_APB, imx8m_clk_composite_critical("dram_apb", imx8mp_dram_apb_sels, base + 0xa080));
	clk_dm(IMX8MP_CLK_I2C5, imx8m_clk_composite("i2c5", imx8mp_i2c5_sels, base + 0xa480));
	clk_dm(IMX8MP_CLK_I2C6, imx8m_clk_composite("i2c6", imx8mp_i2c6_sels, base + 0xa500));
	clk_dm(IMX8MP_CLK_ENET_REF, imx8m_clk_composite("enet_ref", imx8mp_enet_ref_sels, base + 0xa980));
	clk_dm(IMX8MP_CLK_ENET_TIMER, imx8m_clk_composite("enet_timer", imx8mp_enet_timer_sels, base + 0xaa00));
	clk_dm(IMX8MP_CLK_ENET_PHY_REF, imx8m_clk_composite("enet_phy_ref", imx8mp_enet_phy_ref_sels, base + 0xaa80));
	clk_dm(IMX8MP_CLK_QSPI, imx8m_clk_composite("qspi", imx8mp_qspi_sels, base + 0xab80));
	clk_dm(IMX8MP_CLK_USDHC1, imx8m_clk_composite("usdhc1", imx8mp_usdhc1_sels, base + 0xac00));
	clk_dm(IMX8MP_CLK_USDHC2, imx8m_clk_composite("usdhc2", imx8mp_usdhc2_sels, base + 0xac80));
	clk_dm(IMX8MP_CLK_I2C1, imx8m_clk_composite("i2c1", imx8mp_i2c1_sels, base + 0xad00));
	clk_dm(IMX8MP_CLK_I2C2, imx8m_clk_composite("i2c2", imx8mp_i2c2_sels, base + 0xad80));
	clk_dm(IMX8MP_CLK_I2C3, imx8m_clk_composite("i2c3", imx8mp_i2c3_sels, base + 0xae00));
	clk_dm(IMX8MP_CLK_I2C4, imx8m_clk_composite("i2c4", imx8mp_i2c4_sels, base + 0xae80));

	clk_dm(IMX8MP_CLK_UART1, imx8m_clk_composite("uart1", imx8mp_uart1_sels, base + 0xaf00));
	clk_dm(IMX8MP_CLK_UART2, imx8m_clk_composite("uart2", imx8mp_uart2_sels, base + 0xaf80));
	clk_dm(IMX8MP_CLK_UART3, imx8m_clk_composite("uart3", imx8mp_uart3_sels, base + 0xb000));
	clk_dm(IMX8MP_CLK_UART4, imx8m_clk_composite("uart4", imx8mp_uart4_sels, base + 0xb080));
	clk_dm(IMX8MP_CLK_GIC, imx8m_clk_composite_critical("gic", imx8mp_gic_sels, base + 0xb200));
	clk_dm(IMX8MP_CLK_PWM1, imx8m_clk_composite("pwm1", imx8mp_pwm1_sels, base + 0xb380));
	clk_dm(IMX8MP_CLK_PWM2, imx8m_clk_composite("pwm2", imx8mp_pwm2_sels, base + 0xb400));
	clk_dm(IMX8MP_CLK_PWM3, imx8m_clk_composite("pwm3", imx8mp_pwm3_sels, base + 0xb480));
	clk_dm(IMX8MP_CLK_PWM4, imx8m_clk_composite("pwm4", imx8mp_pwm4_sels, base + 0xb500));

	clk_dm(IMX8MP_CLK_WDOG, imx8m_clk_composite("wdog", imx8mp_wdog_sels, base + 0xb900));
	clk_dm(IMX8MP_CLK_USDHC3, imx8m_clk_composite("usdhc3", imx8mp_usdhc3_sels, base + 0xbc80));
	clk_dm(IMX8MP_CLK_HDMI_FDCC_TST, imx8m_clk_composite("hdmi_fdcc_tst", imx8mp_hdmi_fdcc_tst_sels, base + 0xbb00));
	clk_dm(IMX8MP_CLK_HDMI_24M, imx8m_clk_composite("hdmi_24m", imx8mp_hdmi_24m_sels, base + 0xbb80));
	clk_dm(IMX8MP_CLK_HDMI_REF_266M, imx8m_clk_composite("hdmi_ref_266m", imx8mp_hdmi_ref_266m_sels, base + 0xbc00));
	clk_dm(IMX8MP_CLK_MEDIA_MIPI_PHY1_REF, imx8m_clk_composite("media_mipi_phy1_ref", imx8mp_media_mipi_phy1_ref_sels, base + 0xbd80));
	clk_dm(IMX8MP_CLK_MEDIA_DISP1_PIX, imx8m_clk_composite("media_disp1_pix", imx8mp_media_disp1_pix_sels, base + 0xbe00));
	clk_dm(IMX8MP_CLK_MEDIA_LDB, imx8m_clk_composite("media_ldb", imx8mp_media_ldb_sels, base + 0xbf00));

	clk_dm(IMX8MP_CLK_DRAM_ALT_ROOT, imx_clk_fixed_factor("dram_alt_root", "dram_alt", 1, 4));
	clk_dm(IMX8MP_CLK_DRAM_CORE, imx_clk_mux2_flags("dram_core_clk", base + 0x9800, 24, 1, imx8mp_dram_core_sels, ARRAY_SIZE(imx8mp_dram_core_sels), CLK_IS_CRITICAL));

	clk_dm(IMX8MP_CLK_DRAM1_ROOT, imx_clk_gate4_flags("dram1_root_clk", "dram_core_clk", base + 0x4050, 0, CLK_IS_CRITICAL));

	clk_dm(IMX8MP_CLK_ENET1_ROOT, imx_clk_gate4("enet1_root_clk", "enet_axi", base + 0x40a0, 0));
	clk_dm(IMX8MP_CLK_GPIO1_ROOT, imx_clk_gate4("gpio1_root_clk", "ipg_root", base + 0x40b0, 0));
	clk_dm(IMX8MP_CLK_GPIO2_ROOT, imx_clk_gate4("gpio2_root_clk", "ipg_root", base + 0x40c0, 0));
	clk_dm(IMX8MP_CLK_GPIO3_ROOT, imx_clk_gate4("gpio3_root_clk", "ipg_root", base + 0x40d0, 0));
	clk_dm(IMX8MP_CLK_GPIO4_ROOT, imx_clk_gate4("gpio4_root_clk", "ipg_root", base + 0x40e0, 0));
	clk_dm(IMX8MP_CLK_GPIO5_ROOT, imx_clk_gate4("gpio5_root_clk", "ipg_root", base + 0x40f0, 0));
	clk_dm(IMX8MP_CLK_I2C1_ROOT, imx_clk_gate4("i2c1_root_clk", "i2c1", base + 0x4170, 0));
	clk_dm(IMX8MP_CLK_I2C2_ROOT, imx_clk_gate4("i2c2_root_clk", "i2c2", base + 0x4180, 0));
	clk_dm(IMX8MP_CLK_I2C3_ROOT, imx_clk_gate4("i2c3_root_clk", "i2c3", base + 0x4190, 0));
	clk_dm(IMX8MP_CLK_I2C4_ROOT, imx_clk_gate4("i2c4_root_clk", "i2c4", base + 0x41a0, 0));
	clk_dm(IMX8MP_CLK_PWM1_ROOT, imx_clk_gate4("pwm1_root_clk", "pwm1", base + 0x4280, 0));
	clk_dm(IMX8MP_CLK_PWM2_ROOT, imx_clk_gate4("pwm2_root_clk", "pwm2", base + 0x4290, 0));
	clk_dm(IMX8MP_CLK_PWM3_ROOT, imx_clk_gate4("pwm3_root_clk", "pwm3", base + 0x42a0, 0));
	clk_dm(IMX8MP_CLK_PWM4_ROOT, imx_clk_gate4("pwm4_root_clk", "pwm4", base + 0x42b0, 0));
	clk_dm(IMX8MP_CLK_QSPI_ROOT, imx_clk_gate4("qspi_root_clk", "qspi", base + 0x42f0, 0));
	clk_dm(IMX8MP_CLK_I2C5_ROOT, imx_clk_gate2("i2c5_root_clk", "i2c5", base + 0x4330, 0));
	clk_dm(IMX8MP_CLK_I2C6_ROOT, imx_clk_gate2("i2c6_root_clk", "i2c6", base + 0x4340, 0));
	clk_dm(IMX8MP_CLK_SIM_ENET_ROOT, imx_clk_gate4("sim_enet_root_clk", "enet_axi", base + 0x4400, 0));
	clk_dm(IMX8MP_CLK_UART1_ROOT, imx_clk_gate4("uart1_root_clk", "uart1", base + 0x4490, 0));
	clk_dm(IMX8MP_CLK_UART2_ROOT, imx_clk_gate4("uart2_root_clk", "uart2", base + 0x44a0, 0));
	clk_dm(IMX8MP_CLK_UART3_ROOT, imx_clk_gate4("uart3_root_clk", "uart3", base + 0x44b0, 0));
	clk_dm(IMX8MP_CLK_UART4_ROOT, imx_clk_gate4("uart4_root_clk", "uart4", base + 0x44c0, 0));
	clk_dm(IMX8MP_CLK_USDHC1_ROOT, imx_clk_gate4("usdhc1_root_clk", "usdhc1", base + 0x4510, 0));
	clk_dm(IMX8MP_CLK_USDHC2_ROOT, imx_clk_gate4("usdhc2_root_clk", "usdhc2", base + 0x4520, 0));
	clk_dm(IMX8MP_CLK_WDOG1_ROOT, imx_clk_gate4("wdog1_root_clk", "wdog", base + 0x4530, 0));
	clk_dm(IMX8MP_CLK_WDOG2_ROOT, imx_clk_gate4("wdog2_root_clk", "wdog", base + 0x4540, 0));
	clk_dm(IMX8MP_CLK_WDOG3_ROOT, imx_clk_gate4("wdog3_root_clk", "wdog", base + 0x4550, 0));
	clk_dm(IMX8MP_CLK_MEDIA_APB_ROOT, imx_clk_gate2_shared2("media_apb_root_clk", "media_apb", base + 0x45d0, 0, &share_count_media));
	clk_dm(IMX8MP_CLK_MEDIA_AXI_ROOT, imx_clk_gate2_shared2("media_axi_root_clk", "media_axi", base + 0x45d0, 0, &share_count_media));
	clk_dm(IMX8MP_CLK_MEDIA_DISP1_PIX_ROOT, imx_clk_gate2_shared2("media_disp1_pix_root_clk", "media_disp1_pix", base + 0x45d0, 0, &share_count_media));
	clk_dm(IMX8MP_CLK_MEDIA_DISP2_PIX_ROOT, imx_clk_gate2_shared2("media_disp2_pix_root_clk", "media_disp2_pix", base + 0x45d0, 0, &share_count_media));
	clk_dm(IMX8MP_CLK_MEDIA_LDB_ROOT, imx_clk_gate2_shared2("media_ldb_root_clk", "media_ldb", base + 0x45d0, 0, &share_count_media));

	clk_dm(IMX8MP_CLK_USDHC3_ROOT, imx_clk_gate4("usdhc3_root_clk", "usdhc3", base + 0x45e0, 0));
	clk_dm(IMX8MP_CLK_HDMI_ROOT, imx_clk_gate4("hdmi_root_clk", "hdmi_axi", base + 0x45f0, 0));

	return 0;
}

static const struct udevice_id imx8mp_clk_ids[] = {
	{ .compatible = "fsl,imx8mp-ccm" },
	{ },
};

U_BOOT_DRIVER(imx8mp_clk) = {
	.name = "clk_imx8mp",
	.id = UCLASS_CLK,
	.of_match = imx8mp_clk_ids,
	.ops = &imx8mp_clk_ops,
	.probe = imx8mp_clk_probe,
	.flags = DM_FLAG_PRE_RELOC,
};
