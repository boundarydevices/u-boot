/*
 * Copyright (C) 2011-2015 Masahiro Yamada <yamada.masahiro@socionext.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <linux/io.h>
#include <mach/sc-regs.h>
#include <mach/sg-regs.h>

#undef DPLL_SSC_RATE_1PER

static void dpll_init(void)
{
	u32 tmp;

	/*
	 * Set Frequency
	 * Set 0xc(1600MHz)/0xd(1333MHz)/0xe(1066MHz)
	 * to FOUT ( DPLLCTRL.bit[29:20] )
	 */
	tmp = readl(SC_DPLLCTRL);
	tmp &= ~(0x000f0000);
#if CONFIG_DDR_FREQ == 1600
	tmp |= 0x000c0000;
#elif CONFIG_DDR_FREQ == 1333
	tmp |= 0x000d0000;
#else
# error "Unsupported frequency"
#endif

	/*
	 * Set Moduration rate
	 * Set 0x0(1%)/0x1(2%) to SSC_RATE(DPLLCTRL.bit[15])
	 */
#if defined(DPLL_SSC_RATE_1PER)
	tmp &= ~0x00008000;
#else
	tmp |= 0x00008000;
#endif
	writel(tmp, SC_DPLLCTRL);

	tmp = readl(SC_DPLLCTRL2);
	tmp |= SC_DPLLCTRL2_NRSTDS;
	writel(tmp, SC_DPLLCTRL2);
}

static void vpll_init(void)
{
	u32 tmp, clk_mode_axosel;

	/* Set VPLL27A &  VPLL27B */
	tmp = readl(SG_PINMON0);
	clk_mode_axosel = tmp & SG_PINMON0_CLK_MODE_AXOSEL_MASK;

#if defined(CONFIG_MACH_PH1_PRO4)
	/* 25MHz or 6.25MHz is default for Pro4R, no need to set VPLLA/B */
	if (clk_mode_axosel == SG_PINMON0_CLK_MODE_AXOSEL_25000KHZ ||
	    clk_mode_axosel == SG_PINMON0_CLK_MODE_AXOSEL_6250KHZ)
		return;
#endif

	/* Disable write protect of VPLL27ACTRL[2-7]*, VPLL27BCTRL[2-8] */
	tmp = readl(SC_VPLL27ACTRL);
	tmp |= 0x00000001;
	writel(tmp, SC_VPLL27ACTRL);
	tmp = readl(SC_VPLL27BCTRL);
	tmp |= 0x00000001;
	writel(tmp, SC_VPLL27BCTRL);

	/* Unset VPLA_K_LD and VPLB_K_LD bit */
	tmp = readl(SC_VPLL27ACTRL3);
	tmp &= ~0x10000000;
	writel(tmp, SC_VPLL27ACTRL3);
	tmp = readl(SC_VPLL27BCTRL3);
	tmp &= ~0x10000000;
	writel(tmp, SC_VPLL27BCTRL3);

	/* Set VPLA_M and VPLB_M to 0x20 */
	tmp = readl(SC_VPLL27ACTRL2);
	tmp &= ~0x0000007f;
	tmp |= 0x00000020;
	writel(tmp, SC_VPLL27ACTRL2);
	tmp = readl(SC_VPLL27BCTRL2);
	tmp &= ~0x0000007f;
	tmp |= 0x00000020;
	writel(tmp, SC_VPLL27BCTRL2);

	if (clk_mode_axosel == SG_PINMON0_CLK_MODE_AXOSEL_25000KHZ ||
	    clk_mode_axosel == SG_PINMON0_CLK_MODE_AXOSEL_6250KHZ) {
		/* Set VPLA_K and VPLB_K for AXO: 25MHz */
		tmp = readl(SC_VPLL27ACTRL3);
		tmp &= ~0x000fffff;
		tmp |= 0x00066666;
		writel(tmp, SC_VPLL27ACTRL3);
		tmp = readl(SC_VPLL27BCTRL3);
		tmp &= ~0x000fffff;
		tmp |= 0x00066666;
		writel(tmp, SC_VPLL27BCTRL3);
	} else {
		/* Set VPLA_K and VPLB_K for AXO: 24.576 MHz */
		tmp = readl(SC_VPLL27ACTRL3);
		tmp &= ~0x000fffff;
		tmp |= 0x000f5800;
		writel(tmp, SC_VPLL27ACTRL3);
		tmp = readl(SC_VPLL27BCTRL3);
		tmp &= ~0x000fffff;
		tmp |= 0x000f5800;
		writel(tmp, SC_VPLL27BCTRL3);
	}

	/* wait 1 usec */
	udelay(1);

	/* Set VPLA_K_LD and VPLB_K_LD to load K parameters */
	tmp = readl(SC_VPLL27ACTRL3);
	tmp |= 0x10000000;
	writel(tmp, SC_VPLL27ACTRL3);
	tmp = readl(SC_VPLL27BCTRL3);
	tmp |= 0x10000000;
	writel(tmp, SC_VPLL27BCTRL3);

	/* Unset VPLA_SNRST and VPLB_SNRST bit */
	tmp = readl(SC_VPLL27ACTRL2);
	tmp |= 0x10000000;
	writel(tmp, SC_VPLL27ACTRL2);
	tmp = readl(SC_VPLL27BCTRL2);
	tmp |= 0x10000000;
	writel(tmp, SC_VPLL27BCTRL2);

	/* Enable write protect of VPLL27ACTRL[2-7]*, VPLL27BCTRL[2-8] */
	tmp = readl(SC_VPLL27ACTRL);
	tmp &= ~0x00000001;
	writel(tmp, SC_VPLL27ACTRL);
	tmp = readl(SC_VPLL27BCTRL);
	tmp &= ~0x00000001;
	writel(tmp, SC_VPLL27BCTRL);
}

void pll_init(void)
{
	dpll_init();
	vpll_init();

	/*
	 * Wait 500 usec until dpll get stable
	 * We wait 1 usec in vpll_init() so 1 usec can be saved here.
	 */
	udelay(499);
}
