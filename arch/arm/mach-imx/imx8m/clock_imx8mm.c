// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018-2019 NXP
 *
 * Peng Fan <peng.fan@nxp.com>
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <div64.h>
#include <errno.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

DECLARE_GLOBAL_DATA_PTR;

static struct anamix_pll *ana_pll = (struct anamix_pll *)ANATOP_BASE_ADDR;

static u32 get_root_clk(enum clk_root_index clock_id);

#ifdef CONFIG_IMX_HAB
void hab_caam_clock_enable(unsigned char enable)
{
	/* The CAAM clock is always on for iMX8M */
}
#endif

void enable_ocotp_clk(unsigned char enable)
{
	clock_enable(CCGR_OCOTP, !!enable);
}

int enable_i2c_clk(unsigned char enable, unsigned i2c_num)
{
	/* 0 - 3 is valid i2c num */
	if (i2c_num > 3)
		return -EINVAL;

	clock_enable(CCGR_I2C1 + i2c_num, !!enable);

	return 0;
}

static struct imx_int_pll_rate_table imx8mm_fracpll_tbl[] = {
	PLL_1443X_RATE(1000000000U, 250, 3, 1, 0),
	PLL_1443X_RATE(800000000U, 300, 9, 0, 0),
	PLL_1443X_RATE(750000000U, 250, 8, 0, 0),
	PLL_1443X_RATE(650000000U, 325, 3, 2, 0),
	PLL_1443X_RATE(600000000U, 300, 3, 2, 0),
	PLL_1443X_RATE(594000000U, 99, 1, 2, 0),
	PLL_1443X_RATE(400000000U, 300, 9, 1, 0),
	PLL_1443X_RATE(266000000U, 400, 9, 2, 0),
	PLL_1443X_RATE(167000000U, 334, 3, 4, 0),
	PLL_1443X_RATE(100000000U, 300, 9, 3, 0),
};

static int fracpll_configure(enum pll_clocks clock, u32 freq)
{
	int i;
	u32 tmp, div_val;
	struct ana_grp* pll;
	struct imx_int_pll_rate_table *rate;
	u32 val;
	int ret;

	for (i = 0; i < ARRAY_SIZE(imx8mm_fracpll_tbl); i++) {
		if (freq == imx8mm_fracpll_tbl[i].rate)
			break;
	}

	if (i == ARRAY_SIZE(imx8mm_fracpll_tbl)) {
		printf("%s: No matched freq table %u\n", __func__, freq);
		return -EINVAL;
	}

	rate = &imx8mm_fracpll_tbl[i];

	switch (clock) {
	case ANATOP_DRAM_PLL:
		setbits_le32(GPC_BASE_ADDR + 0xEC, 1 << 7);
		setbits_le32(GPC_BASE_ADDR + 0xF8, 1 << 5);
		writel(SRC_DDR1_ENABLE_MASK, SRC_BASE_ADDR + 0x1004);

		pll = &ana_pll->dram_pll;
		break;
	case ANATOP_VIDEO_PLL:
		pll = &ana_pll->video_pll1;
		break;
	default:
		return -EINVAL;
	}
	/* Bypass clock and set lock to pll output lock */
	tmp = readl(&pll->gnrl_ctl);
	tmp |= BYPASS_MASK;
	writel(tmp, &pll->gnrl_ctl);

	/* Enable RST */
	tmp &= ~RST_MASK;
	writel(tmp, &pll->gnrl_ctl);

	div_val = (rate->mdiv << MDIV_SHIFT) | (rate->pdiv << PDIV_SHIFT) |
		(rate->sdiv << SDIV_SHIFT);
	writel(div_val, &pll->fdiv_ctl0);
	writel(rate->kdiv << KDIV_SHIFT, &pll->fdiv_ctl1);

	__udelay(100);

	/* Disable RST */
	tmp |= RST_MASK;
	writel(tmp, &pll->gnrl_ctl);

	/* Wait Lock*/
	ret = readl_poll_timeout(&pll->gnrl_ctl, val, val & LOCK_STATUS, 600);
	if (ret)
		printf("%s timeout\n", __func__);

	/* Bypass */
	tmp &= ~BYPASS_MASK;
	writel(tmp, &pll->gnrl_ctl);

	return ret;
}

u32 imx_get_fecclk(void)
{
	return get_root_clk(ENET_AXI_CLK_ROOT);
}

#ifdef CONFIG_SPL_BUILD
void dram_pll_init(ulong pll_val)
{
	fracpll_configure(ANATOP_DRAM_PLL, pll_val);
}

static struct dram_bypass_clk_setting imx8mm_dram_bypass_tbl[] = {
	DRAM_BYPASS_ROOT_CONFIG(MHZ(100), 2, CLK_ROOT_PRE_DIV1, 2,
				CLK_ROOT_PRE_DIV2),
	DRAM_BYPASS_ROOT_CONFIG(MHZ(250), 3, CLK_ROOT_PRE_DIV2, 2,
				CLK_ROOT_PRE_DIV2),
	DRAM_BYPASS_ROOT_CONFIG(MHZ(400), 1, CLK_ROOT_PRE_DIV2, 3,
				CLK_ROOT_PRE_DIV2),
};

void dram_enable_bypass(ulong clk_val)
{
	int i;
	struct dram_bypass_clk_setting *config;

	for (i = 0; i < ARRAY_SIZE(imx8mm_dram_bypass_tbl); i++) {
		if (clk_val == imx8mm_dram_bypass_tbl[i].clk)
			break;
	}

	if (i == ARRAY_SIZE(imx8mm_dram_bypass_tbl)) {
		printf("%s: No matched freq table %lu\n", __func__, clk_val);
		return;
	}

	config = &imx8mm_dram_bypass_tbl[i];

	clock_set_target_val(DRAM_ALT_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(config->alt_root_sel) |
			     CLK_ROOT_PRE_DIV(config->alt_pre_div));
	clock_set_target_val(DRAM_APB_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(config->apb_root_sel) |
			     CLK_ROOT_PRE_DIV(config->apb_pre_div));
	clock_set_target_val(DRAM_SEL_CFG, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(1));
}

void dram_disable_bypass(void)
{
	clock_set_target_val(DRAM_SEL_CFG, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(0));
	clock_set_target_val(DRAM_APB_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(4) |
			     CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV5));
}
#endif

int intpll_configure(enum pll_clocks clock, ulong freq)
{
	struct ana_grp2 *ag;
	u32 pll_div_ctl_val, pll_clke_masks;
	u32 val;
	int ret;

	switch (clock) {
	case ANATOP_SYSTEM_PLL1:
		ag = &ana_pll->sys_pll1;
		pll_clke_masks = INTPLL_DIV20_CLKE_MASK |
			INTPLL_DIV10_CLKE_MASK | INTPLL_DIV8_CLKE_MASK |
			INTPLL_DIV6_CLKE_MASK | INTPLL_DIV5_CLKE_MASK |
			INTPLL_DIV4_CLKE_MASK | INTPLL_DIV3_CLKE_MASK |
			INTPLL_DIV2_CLKE_MASK | INTPLL_CLKE_MASK;
		break;
	case ANATOP_SYSTEM_PLL2:
		ag = &ana_pll->sys_pll2;
		pll_clke_masks = INTPLL_DIV20_CLKE_MASK |
			INTPLL_DIV10_CLKE_MASK | INTPLL_DIV8_CLKE_MASK |
			INTPLL_DIV6_CLKE_MASK | INTPLL_DIV5_CLKE_MASK |
			INTPLL_DIV4_CLKE_MASK | INTPLL_DIV3_CLKE_MASK |
			INTPLL_DIV2_CLKE_MASK | INTPLL_CLKE_MASK;
		break;
	case ANATOP_SYSTEM_PLL3:
		ag = &ana_pll->sys_pll3;
		pll_clke_masks = INTPLL_CLKE_MASK;
		break;
	case ANATOP_ARM_PLL:
		ag = &ana_pll->arm_pll;
		pll_clke_masks = INTPLL_CLKE_MASK;
		break;
	case ANATOP_GPU_PLL:
		ag = &ana_pll->gpu_pll;
		pll_clke_masks = INTPLL_CLKE_MASK;
		break;
	case ANATOP_VPU_PLL:
		ag = &ana_pll->vpu_pll;
		pll_clke_masks = INTPLL_CLKE_MASK;
		break;
	default:
		return -EINVAL;
	}

	switch (freq) {
	case MHZ(600):
		/* 24 * 0x12c / 3 / 2 ^ 2 */
		pll_div_ctl_val = INTPLL_MAIN_DIV_VAL(0x12c) |
			INTPLL_PRE_DIV_VAL(3) | INTPLL_POST_DIV_VAL(2);
		break;
	case MHZ(750):
		/* 24 * 0xfa / 2 / 2 ^ 2 */
		pll_div_ctl_val = INTPLL_MAIN_DIV_VAL(0xfa) |
			INTPLL_PRE_DIV_VAL(2) | INTPLL_POST_DIV_VAL(2);
		break;
	case MHZ(800):
		/* 24 * 0x190 / 3 / 2 ^ 2 */
		pll_div_ctl_val = INTPLL_MAIN_DIV_VAL(0x190) |
			INTPLL_PRE_DIV_VAL(3) | INTPLL_POST_DIV_VAL(2);
		break;
	case MHZ(1000):
		/* 24 * 0xfa / 3 / 2 ^ 1 */
		pll_div_ctl_val = INTPLL_MAIN_DIV_VAL(0xfa) |
			INTPLL_PRE_DIV_VAL(3) | INTPLL_POST_DIV_VAL(1);
		break;
	case MHZ(1200):
		/* 24 * 0x12c / 3 / 2 ^ 1 */
		pll_div_ctl_val = INTPLL_MAIN_DIV_VAL(0x12c) |
			INTPLL_PRE_DIV_VAL(3) | INTPLL_POST_DIV_VAL(1);
		break;
	case MHZ(1400):
		/* 24 * 0x15e / 3 / 2 ^ 1 */
		pll_div_ctl_val = INTPLL_MAIN_DIV_VAL(0x15e) |
			INTPLL_PRE_DIV_VAL(3) | INTPLL_POST_DIV_VAL(1);
		break;
	case MHZ(1500):
		/* 24 * 0x177 / 3 / 2 ^ 1 */
		pll_div_ctl_val = INTPLL_MAIN_DIV_VAL(0x177) |
			INTPLL_PRE_DIV_VAL(3) | INTPLL_POST_DIV_VAL(1);
		break;
	case MHZ(1600):
		/* 24 * 0xc8 / 3 / 2 ^ 0 */
		pll_div_ctl_val = INTPLL_MAIN_DIV_VAL(0xc8) |
			INTPLL_PRE_DIV_VAL(3) | INTPLL_POST_DIV_VAL(0);
		break;
	case MHZ(1800):
		/* 24 * 0xe1 / 3 / 2 ^ 0 */
		pll_div_ctl_val = INTPLL_MAIN_DIV_VAL(0xe1) |
			INTPLL_PRE_DIV_VAL(3) | INTPLL_POST_DIV_VAL(0);
		break;
	case MHZ(2000):
		/* 24 * 0xfa / 3 / 2 ^ 0 */
		pll_div_ctl_val = INTPLL_MAIN_DIV_VAL(0xfa) |
			INTPLL_PRE_DIV_VAL(3) | INTPLL_POST_DIV_VAL(0);
		break;
	default:
		return -EINVAL;
	}
	/* Bypass clock and set lock to pll output lock */
	setbits_le32(&ag->gnrl_ctl, INTPLL_BYPASS_MASK | INTPLL_LOCK_SEL_MASK);
	/* Enable reset */
	clrbits_le32(&ag->gnrl_ctl, INTPLL_RST_MASK);
	/* Configure */
	writel(pll_div_ctl_val, &ag->div_ctl);

	__udelay(100);

	/* Disable reset */
	setbits_le32(&ag->gnrl_ctl, INTPLL_RST_MASK);
	/* Wait Lock */
	ret = readl_poll_timeout(&ag->gnrl_ctl, val, val & INTPLL_LOCK_MASK, 100);
	if (ret)
		printf("%s timeout\n", __func__);

	/* Clear bypass */
	clrbits_le32(&ag->gnrl_ctl, INTPLL_BYPASS_MASK);
	setbits_le32(&ag->gnrl_ctl, pll_clke_masks);

	return ret;
}

#define VIDEO_PLL_RATE 594000000U

#ifdef CONFIG_IMX8MP
void enable_display_clk(unsigned char enable)
{
	if (enable) {
		clock_enable(CCGR_DISPMIX, false);

		/* Set Video PLL to 594Mhz, p = 1, m = 99,  k = 0, s = 2 */
		fracpll_configure(ANATOP_VIDEO_PLL, VIDEO_PLL_RATE);

		/* 500Mhz */
		clock_set_target_val(MEDIA_AXI_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(1) | CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV2));

		/* 200Mhz */
		clock_set_target_val(MEDIA_APB_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(2) |CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV4));

		/* 27Mhz MIPI DPHY PLL ref from video PLL */
		clock_set_target_val(MEDIA_MIPI_PHY1_REF_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(7) |CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV22));
		clock_enable(CCGR_DISPMIX, true);
	} else {
		clock_enable(CCGR_DISPMIX, false);
	}
}
#else
void enable_display_clk(unsigned char enable)
{
	if (enable) {
		clock_enable(CCGR_DISPMIX, false);

		/* Set Video PLL to 594Mhz, p = 1, m = 99,  k = 0, s = 2 */
		fracpll_configure(ANATOP_VIDEO_PLL, VIDEO_PLL_RATE);

		/* 500Mhz */
		clock_set_target_val(DISPLAY_AXI_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(1) | CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV2));

		/* 200Mhz */
		clock_set_target_val(DISPLAY_APB_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(2) |CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV4));

		clock_set_target_val(MIPI_DSI_CORE_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(1));

		/* 27Mhz MIPI DPHY PLL ref from video PLL */
#ifdef CONFIG_IMX8MN
		clock_set_target_val(DISPLAY_DSI_PHY_REF_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(7) |CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV22));
#else
		clock_set_target_val(MIPI_DSI_PHY_REF_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(7) |CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV22));
#endif
		clock_enable(CCGR_DISPMIX, true);
	} else {
		clock_enable(CCGR_DISPMIX, false);
	}
}
#endif

void init_uart_clk(u32 index)
{
	/*
	 * set uart clock root
	 * 24M OSC
	 */
	switch (index) {
	case 0:
		clock_enable(CCGR_UART1, 0);
		clock_set_target_val(UART1_CLK_ROOT, CLK_ROOT_ON |
				     CLK_ROOT_SOURCE_SEL(0));
		clock_enable(CCGR_UART1, 1);
		return;
	case 1:
		clock_enable(CCGR_UART2, 0);
		clock_set_target_val(UART2_CLK_ROOT, CLK_ROOT_ON |
				     CLK_ROOT_SOURCE_SEL(0));
		clock_enable(CCGR_UART2, 1);
		return;
	case 2:
		clock_enable(CCGR_UART3, 0);
		clock_set_target_val(UART3_CLK_ROOT, CLK_ROOT_ON |
				     CLK_ROOT_SOURCE_SEL(0));
		clock_enable(CCGR_UART3, 1);
		return;
	case 3:
		clock_enable(CCGR_UART4, 0);
		clock_set_target_val(UART4_CLK_ROOT, CLK_ROOT_ON |
				     CLK_ROOT_SOURCE_SEL(0));
		clock_enable(CCGR_UART4, 1);
		return;
	default:
		printf("Invalid uart index\n");
		return;
	}
}

void init_wdog_clk(void)
{
	clock_enable(CCGR_WDOG1, 0);
	clock_enable(CCGR_WDOG2, 0);
	clock_enable(CCGR_WDOG3, 0);
	clock_set_target_val(WDOG_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(0));
	clock_enable(CCGR_WDOG1, 1);
	clock_enable(CCGR_WDOG2, 1);
	clock_enable(CCGR_WDOG3, 1);
}

static enum clk_ccgr_index ccgr_usdhc[] = {CCGR_USDHC1, CCGR_USDHC2, CCGR_USDHC3,};
static enum clk_root_index root_usdhc[] = {USDHC1_CLK_ROOT, USDHC2_CLK_ROOT, USDHC3_CLK_ROOT,};
void init_clk_usdhc(u32 index)
{
	if (index > ARRAY_SIZE(ccgr_usdhc)) {
		printf("Invalid usdhc index\n");
		return;
	}
	/*
	 * set usdhc clock root
	 * sys pll1 400M
	 */
	clock_enable(ccgr_usdhc[index], 0);
	clock_set_target_val(root_usdhc[index], CLK_ROOT_ON |
				     CLK_ROOT_SOURCE_SEL(1));
	clock_enable(ccgr_usdhc[index], 1);
}

void init_clk_ecspi(u32 index)
{
	switch (index) {
	case 0:
		clock_enable(CCGR_ECSPI1, 0);
		clock_set_target_val(ECSPI1_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(0));
		clock_enable(CCGR_ECSPI1, 1);
		return;
	case 1:
		clock_enable(CCGR_ECSPI2, 0);
		clock_set_target_val(ECSPI2_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(0));
		clock_enable(CCGR_ECSPI2, 1);
		return;
	case 2:
		clock_enable(CCGR_ECSPI3, 0);
		clock_set_target_val(ECSPI3_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(0));
		clock_enable(CCGR_ECSPI3, 1);
		return;
	default:
		printf("Invalid ecspi index\n");
		return;
	}
}

void init_nand_clk(void)
{
	/*
	 * set rawnand root
	 * sys pll1 400M
	 */
	clock_enable(CCGR_RAWNAND, 0);
	clock_set_target_val(NAND_CLK_ROOT, CLK_ROOT_ON |
		CLK_ROOT_SOURCE_SEL(3) | CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV4)); /* 100M */
	clock_enable(CCGR_RAWNAND, 1);
}

int clock_init(void)
{
	u32 val_cfg0;

	/*
	 * The gate is not exported to clk tree, so configure them here.
	 * According to ANAMIX SPEC
	 * sys pll1 fixed at 800MHz
	 * sys pll2 fixed at 1GHz
	 * Here we only enable the outputs.
	 */
	val_cfg0 = readl(&ana_pll->sys_pll1.gnrl_ctl);
	val_cfg0 |= INTPLL_CLKE_MASK | INTPLL_DIV2_CLKE_MASK |
		INTPLL_DIV3_CLKE_MASK | INTPLL_DIV4_CLKE_MASK |
		INTPLL_DIV5_CLKE_MASK | INTPLL_DIV6_CLKE_MASK |
		INTPLL_DIV8_CLKE_MASK | INTPLL_DIV10_CLKE_MASK |
		INTPLL_DIV20_CLKE_MASK;
	writel(val_cfg0, &ana_pll->sys_pll1.gnrl_ctl);

	val_cfg0 = readl(&ana_pll->sys_pll2.gnrl_ctl);
	val_cfg0 |= INTPLL_CLKE_MASK | INTPLL_DIV2_CLKE_MASK |
		INTPLL_DIV3_CLKE_MASK | INTPLL_DIV4_CLKE_MASK |
		INTPLL_DIV5_CLKE_MASK | INTPLL_DIV6_CLKE_MASK |
		INTPLL_DIV8_CLKE_MASK | INTPLL_DIV10_CLKE_MASK |
		INTPLL_DIV20_CLKE_MASK;
	writel(val_cfg0, &ana_pll->sys_pll2.gnrl_ctl);

	/* Configure ARM at 1.2GHz */
	clock_set_target_val(ARM_A53_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(2));

	intpll_configure(ANATOP_ARM_PLL, MHZ(1200));

	clock_set_target_val(ARM_A53_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(1) |
			     CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV1));

	if (is_imx8mn() || is_imx8mp())
		intpll_configure(ANATOP_SYSTEM_PLL3, MHZ(600));
	else
		intpll_configure(ANATOP_SYSTEM_PLL3, MHZ(750));

#ifdef CONFIG_IMX8MP
	/* 8MP ROM already set NOC to 800Mhz, only need to configure NOC_IO clk to 600Mhz */
	/* 8MP ROM already set GIC to 400Mhz, system_pll1_800m with div = 2 */
	clock_set_target_val(NOC_IO_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(2));
#else
	clock_set_target_val(NOC_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(2));

	/* config GIC to sys_pll2_100m */
	clock_enable(CCGR_GIC, 0);
	clock_set_target_val(GIC_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(3));
	clock_enable(CCGR_GIC, 1);
#endif

	clock_set_target_val(NAND_USDHC_BUS_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(1));

	clock_enable(CCGR_DDR1, 0);
	clock_set_target_val(DRAM_ALT_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(1));
	clock_set_target_val(DRAM_APB_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(1));
	clock_enable(CCGR_DDR1, 1);

	init_wdog_clk();

	clock_enable(CCGR_TEMP_SENSOR, 1);

	clock_enable(CCGR_SEC_DEBUG, 1);

	enable_display_clk(1);
	return 0;
};

void mxs_set_lcdclk(uint32_t base_addr, uint32_t freq)
{
	uint32_t div, pre, post;

	div = VIDEO_PLL_RATE / 1000;
	div = (div + freq - 1) / freq;

	if (div < 1)
		div = 1;

	for (pre = 1; pre <= 8; pre++) {
		for (post = 1; post <= 64; post++) {
			if (pre * post == div) {
				goto find;
			}
		}
	}

	printf("Fail to set rate to %dkhz", freq);
	return;

find:
	/* Select to video PLL */
	pr_debug("mxs_set_lcdclk, pre = %d, post = %d\n", pre, post);
#ifdef CONFIG_IMX8MM
	clock_set_target_val(LCDIF_PIXEL_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(1) | CLK_ROOT_PRE_DIV(pre - 1) | CLK_ROOT_POST_DIV(post - 1));
#endif

}

u32 imx_get_uartclk(void)
{
	return 24000000U;
}

static u32 decode_intpll(enum clk_root_src intpll)
{
	struct ana_grp2 *ag;
	u32 pll_gnrl_ctl, pll_div_ctl, pll_clke_mask;
	u32 main_div, pre_div, post_div, div;
	u64 freq;

	switch (intpll) {
	case ARM_PLL_CLK:
		ag = &ana_pll->arm_pll;
		break;
	case GPU_PLL_CLK:
		ag = &ana_pll->gpu_pll;
		break;
	case VPU_PLL_CLK:
		ag = &ana_pll->vpu_pll;
		break;
	case SYSTEM_PLL1_800M_CLK:
	case SYSTEM_PLL1_400M_CLK:
	case SYSTEM_PLL1_266M_CLK:
	case SYSTEM_PLL1_200M_CLK:
	case SYSTEM_PLL1_160M_CLK:
	case SYSTEM_PLL1_133M_CLK:
	case SYSTEM_PLL1_100M_CLK:
	case SYSTEM_PLL1_80M_CLK:
	case SYSTEM_PLL1_40M_CLK:
		ag = &ana_pll->sys_pll1;
		break;
	case SYSTEM_PLL2_1000M_CLK:
	case SYSTEM_PLL2_500M_CLK:
	case SYSTEM_PLL2_333M_CLK:
	case SYSTEM_PLL2_250M_CLK:
	case SYSTEM_PLL2_200M_CLK:
	case SYSTEM_PLL2_166M_CLK:
	case SYSTEM_PLL2_125M_CLK:
	case SYSTEM_PLL2_100M_CLK:
	case SYSTEM_PLL2_50M_CLK:
		ag = &ana_pll->sys_pll2;
		break;
	case SYSTEM_PLL3_CLK:
		ag = &ana_pll->sys_pll3;
		break;
	default:
		return -EINVAL;
	}
	pll_gnrl_ctl = readl(&ag->gnrl_ctl);
	pll_div_ctl = readl(&ag->div_ctl);

	/* Only support SYS_XTAL 24M, PAD_CLK not take into consideration */
	if ((pll_gnrl_ctl & INTPLL_REF_CLK_SEL_MASK) != 0)
		return 0;

	if ((pll_gnrl_ctl & INTPLL_RST_MASK) == 0)
		return 0;

	/*
	 * When BYPASS is equal to 1, PLL enters the bypass mode
	 * regardless of the values of RESETB
	 */
	if (pll_gnrl_ctl & INTPLL_BYPASS_MASK)
		return 24000000u;

	if (!(pll_gnrl_ctl & INTPLL_LOCK_MASK)) {
		puts("pll not locked\n");
		return 0;
	}

	switch (intpll) {
	case ARM_PLL_CLK:
	case GPU_PLL_CLK:
	case VPU_PLL_CLK:
	case SYSTEM_PLL3_CLK:
	case SYSTEM_PLL1_800M_CLK:
	case SYSTEM_PLL2_1000M_CLK:
		pll_clke_mask = INTPLL_CLKE_MASK;
		div = 1;
		break;

	case SYSTEM_PLL1_400M_CLK:
	case SYSTEM_PLL2_500M_CLK:
		pll_clke_mask = INTPLL_DIV2_CLKE_MASK;
		div = 2;
		break;

	case SYSTEM_PLL1_266M_CLK:
	case SYSTEM_PLL2_333M_CLK:
		pll_clke_mask = INTPLL_DIV3_CLKE_MASK;
		div = 3;
		break;

	case SYSTEM_PLL1_200M_CLK:
	case SYSTEM_PLL2_250M_CLK:
		pll_clke_mask = INTPLL_DIV4_CLKE_MASK;
		div = 4;
		break;

	case SYSTEM_PLL1_160M_CLK:
	case SYSTEM_PLL2_200M_CLK:
		pll_clke_mask = INTPLL_DIV5_CLKE_MASK;
		div = 5;
		break;

	case SYSTEM_PLL1_133M_CLK:
	case SYSTEM_PLL2_166M_CLK:
		pll_clke_mask = INTPLL_DIV6_CLKE_MASK;
		div = 6;
		break;

	case SYSTEM_PLL1_100M_CLK:
	case SYSTEM_PLL2_125M_CLK:
		pll_clke_mask = INTPLL_DIV8_CLKE_MASK;
		div = 8;
		break;

	case SYSTEM_PLL1_80M_CLK:
	case SYSTEM_PLL2_100M_CLK:
		pll_clke_mask = INTPLL_DIV10_CLKE_MASK;
		div = 10;
		break;

	case SYSTEM_PLL1_40M_CLK:
	case SYSTEM_PLL2_50M_CLK:
		pll_clke_mask = INTPLL_DIV20_CLKE_MASK;
		div = 20;
		break;
	default:
		return -EINVAL;
	}

	if ((pll_gnrl_ctl & pll_clke_mask) == 0)
		return 0;

	main_div = (pll_div_ctl & INTPLL_MAIN_DIV_MASK) >>
		INTPLL_MAIN_DIV_SHIFT;
	pre_div = (pll_div_ctl & INTPLL_PRE_DIV_MASK) >>
		INTPLL_PRE_DIV_SHIFT;
	post_div = (pll_div_ctl & INTPLL_POST_DIV_MASK) >>
		INTPLL_POST_DIV_SHIFT;

	/* FFVCO = (m * FFIN) / p, FFOUT = (m * FFIN) / (p * 2^s) */
	freq = 24000000ULL * main_div;
	return lldiv(freq, pre_div * (1 << post_div) * div);
}

static u32 decode_fracpll(enum clk_root_src frac_pll)
{
	struct ana_grp *ag;
	u32 pll_gnrl_ctl, pll_fdiv_ctl0, pll_fdiv_ctl1;
	u32 main_div, pre_div, post_div, k;

	switch (frac_pll) {
	case DRAM_PLL1_CLK:
		ag = &ana_pll->dram_pll;
		break;
	case AUDIO_PLL1_CLK:
		ag = &ana_pll->audio_pll1;
		break;
	case AUDIO_PLL2_CLK:
		ag = &ana_pll->audio_pll2;
		break;
	case VIDEO_PLL_CLK:
		ag = &ana_pll->video_pll1;
		break;
	default:
		printf("Unsupported clk_root_src %d\n", frac_pll);
		return 0;
	}
	pll_gnrl_ctl = readl(&ag->gnrl_ctl);
	pll_fdiv_ctl0 = readl(&ag->fdiv_ctl0);
	pll_fdiv_ctl1 = readl(&ag->fdiv_ctl1);

	/* Only support SYS_XTAL 24M, PAD_CLK not take into consideration */
	if ((pll_gnrl_ctl & GENMASK(1, 0)) != 0)
		return 0;

	if ((pll_gnrl_ctl & RST_MASK) == 0)
		return 0;
	/*
	 * When BYPASS is equal to 1, PLL enters the bypass mode
	 * regardless of the values of RESETB
	 */
	if (pll_gnrl_ctl & BYPASS_MASK)
		return 24000000u;

	if (!(pll_gnrl_ctl & LOCK_STATUS)) {
		puts("pll not locked\n");
		return 0;
	}

	if (!(pll_gnrl_ctl & CLKE_MASK))
		return 0;

	main_div = (pll_fdiv_ctl0 & MDIV_MASK) >>
		MDIV_SHIFT;
	pre_div = (pll_fdiv_ctl0 & PDIV_MASK) >>
		PDIV_SHIFT;
	post_div = (pll_fdiv_ctl0 & SDIV_MASK) >>
		SDIV_SHIFT;

	k = pll_fdiv_ctl1 & KDIV_MASK;

	return lldiv((main_div * 65536 + k) * 24000000ULL,
		     65536 * pre_div * (1 << post_div));
}

static u32 get_root_src_clk(enum clk_root_src root_src)
{
	switch (root_src) {
	case OSC_24M_CLK:
		return 24000000u;
	case OSC_HDMI_CLK:
		return 26000000u;
	case OSC_32K_CLK:
		return 32000u;
	case ARM_PLL_CLK:
	case GPU_PLL_CLK:
	case VPU_PLL_CLK:
	case SYSTEM_PLL1_800M_CLK:
	case SYSTEM_PLL1_400M_CLK:
	case SYSTEM_PLL1_266M_CLK:
	case SYSTEM_PLL1_200M_CLK:
	case SYSTEM_PLL1_160M_CLK:
	case SYSTEM_PLL1_133M_CLK:
	case SYSTEM_PLL1_100M_CLK:
	case SYSTEM_PLL1_80M_CLK:
	case SYSTEM_PLL1_40M_CLK:
	case SYSTEM_PLL2_1000M_CLK:
	case SYSTEM_PLL2_500M_CLK:
	case SYSTEM_PLL2_333M_CLK:
	case SYSTEM_PLL2_250M_CLK:
	case SYSTEM_PLL2_200M_CLK:
	case SYSTEM_PLL2_166M_CLK:
	case SYSTEM_PLL2_125M_CLK:
	case SYSTEM_PLL2_100M_CLK:
	case SYSTEM_PLL2_50M_CLK:
	case SYSTEM_PLL3_CLK:
		return decode_intpll(root_src);
	case DRAM_PLL1_CLK:
	case AUDIO_PLL1_CLK:
	case AUDIO_PLL2_CLK:
	case VIDEO_PLL_CLK:
		return decode_fracpll(root_src);
	case ARM_A53_ALT_CLK:
		return get_root_clk(ARM_A53_CLK_ROOT);
	default:
		return 0;
	}

	return 0;
}

static u32 get_root_clk(enum clk_root_index clock_id)
{
	enum clk_root_src root_src;
	u32 post_podf, pre_podf, root_src_clk;

	if (clock_root_enabled(clock_id) <= 0)
		return 0;

	if (clock_get_prediv(clock_id, &pre_podf) < 0)
		return 0;

	if (clock_get_postdiv(clock_id, &post_podf) < 0)
		return 0;

	if (clock_get_src(clock_id, &root_src) < 0)
		return 0;

	root_src_clk = get_root_src_clk(root_src);

	return root_src_clk / (post_podf + 1) / (pre_podf + 1);
}

u32 get_arm_core_clk(void)
{
	enum clk_root_src root_src;
	u32 root_src_clk;

	if (clock_get_src(CORE_SEL_CFG, &root_src) < 0)
		return 0;

	root_src_clk = get_root_src_clk(root_src);

	return root_src_clk;
}

u32 mxc_get_clock(enum clk_root_index clk)
{
	u32 val;

	switch (clk) {
	case MXC_ARM_CLK:
		return get_arm_core_clk();
	case MXC_IPG_CLK:
		clock_get_target_val(IPG_CLK_ROOT, &val);
		val = val & 0x3;
		return get_root_clk(AHB_CLK_ROOT) / 2 / (val + 1);
	default:
		break;
	}
	return get_root_clk(clk);
}

#ifdef CONFIG_DWC_ETH_QOS
int set_clk_eqos(enum enet_freq type)
{
	u32 target;
	u32 enet1_ref;

	switch (type) {
	case ENET_125MHZ:
		enet1_ref = ENET1_REF_CLK_ROOT_FROM_PLL_ENET_MAIN_125M_CLK;
		break;
	case ENET_50MHZ:
		enet1_ref = ENET1_REF_CLK_ROOT_FROM_PLL_ENET_MAIN_50M_CLK;
		break;
	case ENET_25MHZ:
		enet1_ref = ENET1_REF_CLK_ROOT_FROM_PLL_ENET_MAIN_25M_CLK;
		break;
	default:
		return -EINVAL;
	}

	/* disable the clock first */
	clock_enable(CCGR_QOS_ETHENET, 0);
	clock_enable(CCGR_SDMA2, 0);

	/* set enet axi clock 266Mhz */
	target = CLK_ROOT_ON | ENET_AXI_CLK_ROOT_FROM_SYS1_PLL_266M |
		 CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV1) |
		 CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV1);
	clock_set_target_val(ENET_AXI_CLK_ROOT, target);

	target = CLK_ROOT_ON | enet1_ref |
		 CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV1) |
		 CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV1);
	clock_set_target_val(ENET_QOS_CLK_ROOT, target);

	target = CLK_ROOT_ON |
		ENET1_TIME_CLK_ROOT_FROM_PLL_ENET_MAIN_100M_CLK |
		CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV1) |
		CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV4);
	clock_set_target_val(ENET_QOS_TIMER_CLK_ROOT, target);

	/* enable clock */
	clock_enable(CCGR_QOS_ETHENET, 1);
	clock_enable(CCGR_SDMA2, 1);

	return 0;
}

int imx_eqos_txclk_set_rate(ulong rate)
{
	u32 val;
	u32 eqos_post_div;

	/* disable the clock first */
	clock_enable(CCGR_QOS_ETHENET, 0);
	clock_enable(CCGR_SDMA2, 0);

	switch (rate) {
	case 125000000:
		eqos_post_div = 1;
		break;
	case 25000000:
		eqos_post_div = 125000000 / 25000000;
		break;
	case 2500000:
		eqos_post_div = 125000000 / 2500000;
		break;
	default:
		return -EINVAL;
	}

	clock_get_target_val(ENET_QOS_CLK_ROOT, &val);
	val &= ~(CLK_ROOT_PRE_DIV_MASK | CLK_ROOT_POST_DIV_MASK);
	val |= CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV1) |
	       CLK_ROOT_POST_DIV(eqos_post_div - 1);
	clock_set_target_val(ENET_QOS_CLK_ROOT, val);

	/* enable clock */
	clock_enable(CCGR_QOS_ETHENET, 1);
	clock_enable(CCGR_SDMA2, 1);

	return 0;
}

u32 imx_get_eqos_csr_clk(void)
{
	return get_root_clk(ENET_AXI_CLK_ROOT);
}
#endif

#if defined(CONFIG_IMX8MP)
void init_usb_clk(int usbno)
{
	clock_enable(CCGR_USB_MSCALE_PL301, 0);
	clock_enable(CCGR_USB_PHY_8MP, 0);

	/* HSIOMIX AXI BUS root already been set by ROM */

	/* 100MHz */
	clock_set_target_val(USB_CORE_REF_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(1));
	/* 100MHz */
	clock_set_target_val(USB_PHY_REF_CLK_ROOT, CLK_ROOT_ON |
			     CLK_ROOT_SOURCE_SEL(1));

	clock_enable(CCGR_USB_MSCALE_PL301, 1);
	clock_enable(CCGR_USB_PHY_8MP, 1);
}
#else
void enable_usboh3_clk(unsigned char enable)
{
	if (enable) {
		clock_enable(CCGR_USB_MSCALE_PL301, 0);
		/* 500M */
		clock_set_target_val(USB_BUS_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(1));
		/* 100M */
		clock_set_target_val(USB_CORE_REF_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(1));
		/* 100M */
		clock_set_target_val(USB_PHY_REF_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(1));
		clock_enable(CCGR_USB_MSCALE_PL301, 1);
	} else {
		clock_enable(CCGR_USB_MSCALE_PL301, 0);
	}
}
#endif

#ifdef CONFIG_FEC_MXC
int set_clk_enet(enum enet_freq type)
{
	u32 target;
	u32 enet1_ref;

	switch (type) {
	case ENET_125MHZ:
		enet1_ref = ENET1_REF_CLK_ROOT_FROM_PLL_ENET_MAIN_125M_CLK;
		break;
	case ENET_50MHZ:
		enet1_ref = ENET1_REF_CLK_ROOT_FROM_PLL_ENET_MAIN_50M_CLK;
		break;
	case ENET_25MHZ:
		enet1_ref = ENET1_REF_CLK_ROOT_FROM_PLL_ENET_MAIN_25M_CLK;
		break;
	default:
		return -EINVAL;
	}

	/* disable the clock first */
	clock_enable(CCGR_ENET1, 0);
	clock_enable(CCGR_SIM_ENET, 0);

	/* set enet axi clock 266Mhz */
	target = CLK_ROOT_ON | ENET_AXI_CLK_ROOT_FROM_SYS1_PLL_266M |
		 CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV1) |
		 CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV1);
	clock_set_target_val(ENET_AXI_CLK_ROOT, target);

	target = CLK_ROOT_ON | enet1_ref |
		 CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV1) |
		 CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV1);
	clock_set_target_val(ENET_REF_CLK_ROOT, target);

	target = CLK_ROOT_ON |
		ENET1_TIME_CLK_ROOT_FROM_PLL_ENET_MAIN_100M_CLK |
		CLK_ROOT_PRE_DIV(CLK_ROOT_PRE_DIV1) |
		CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV4);
	clock_set_target_val(ENET_TIMER_CLK_ROOT, target);

	/* enable clock */
	clock_enable(CCGR_SIM_ENET, 1);
	clock_enable(CCGR_ENET1, 1);

	return 0;
}
#endif
