// SPDX-License-Identifier: GPL-2.0-only

#include <common.h>
#include <asm/io.h>
#include <clk.h>
#include <div64.h>
#include <dm/device.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include "clk-mtk-pll.h"

#define UBOOT_DM_CLK_CCF_MTK_PLL "ccf_clk_mtk_pll"

#define MHZ			(1000 * 1000)
#define REG_CON0		0
#define REG_CON1		4

#define CON0_BASE_EN		BIT(0)
#define CON0_PWR_ON		BIT(0)
#define CON0_ISO_EN		BIT(1)
#define PCW_CHG_MASK		BIT(31)

#define AUDPLL_TUNER_EN		BIT(31)

#define POSTDIV_MASK		0x7

/* default 7 bits integer, can be overridden with pcwibits. */
#define INTEGER_BITS		7

struct mtk_pll_div_table {
        u32 div;
        unsigned long freq;
};
/*
 * MediaTek PLLs are configured through their pcw value. The pcw value describes
 * a divider in the PLL feedback loop which consists of 7 bits for the integer
 * part and the remaining bits (if present) for the fractional part. Also they
 * have a 3 bit power-of-two post divider.
 */

struct mtk_clk_pll {
	struct clk	clk;
	void __iomem	*base_addr;
	void __iomem	*pd_addr;
	void __iomem	*pwr_addr;
	void __iomem	*tuner_addr;
	void __iomem	*tuner_en_addr;
	void __iomem	*pcw_addr;
	void __iomem	*pcw_chg_addr;
	void __iomem	*en_addr;
	int id;
	const char *name;
	u32 en_mask;
	u8 tuner_en_bit;
	int pd_shift;
	unsigned int flags;
	const struct clk_ops *ops;
	u32 rst_bar_mask;
	unsigned long fmin;
	unsigned long fmax;
	int pcwbits;
	int pcwibits;
	int pcw_shift;
	const struct mtk_pll_div_table *div_table;
	const char *parent_name;
	u8 pll_en_bit; /* Assume 0, indicates BIT(0) by default */
#define MTKF_RST_BAR	BIT(0)
#define MTKF_PWR_REG1	BIT(1)
#define MTKF_PLL_AO	BIT(2)
	u8 mtk_flags;
};

static inline struct mtk_clk_pll *to_mtk_clk_pll(struct clk *_clk)
{
	return container_of(_clk, struct mtk_clk_pll, clk);
}

static unsigned long __mtk_pll_recalc_rate(struct mtk_clk_pll *pll, ulong fin,
		u32 pcw, int postdiv)
{
	int pcwbits = pll->pcwbits;
	int pcwfbits = 0;
	int ibits;
	u64 vco;
	u8 c = 0;

	/* The fractional part of the PLL divider. */
	ibits = pll->pcwibits ? pll->pcwibits : INTEGER_BITS;
	if (pcwbits > ibits)
		pcwfbits = pcwbits - ibits;

	vco = (u64)fin * pcw;

	if (pcwfbits && (vco & GENMASK(pcwfbits - 1, 0)))
		c = 1;

	vco >>= pcwfbits;

	if (c)
		vco++;

	debug("%s: (%ld * %d >> %d) / %d = %ld\n", __func__, fin, pcw, pcwfbits, postdiv, ((unsigned long)vco + postdiv - 1) / postdiv);
	return ((unsigned long)vco + postdiv - 1) / postdiv;
}

static void __mtk_pll_tuner_enable(struct mtk_clk_pll *pll)
{
	u32 r;

	if (pll->tuner_en_addr) {
		r = readl(pll->tuner_en_addr) | BIT(pll->tuner_en_bit);
		writel(r, pll->tuner_en_addr);
	} else if (pll->tuner_addr) {
		r = readl(pll->tuner_addr) | AUDPLL_TUNER_EN;
		writel(r, pll->tuner_addr);
	}
}

static void __mtk_pll_tuner_disable(struct mtk_clk_pll *pll)
{
	u32 r;

	if (pll->tuner_en_addr) {
		r = readl(pll->tuner_en_addr) & ~BIT(pll->tuner_en_bit);
		writel(r, pll->tuner_en_addr);
	} else if (pll->tuner_addr) {
		r = readl(pll->tuner_addr) & ~AUDPLL_TUNER_EN;
		writel(r, pll->tuner_addr);
	}
}

static void mtk_pll_set_rate_regs(struct mtk_clk_pll *pll, u32 pcw,
		int postdiv)
{
	u32 chg, val;

	/* disable tuner */
	__mtk_pll_tuner_disable(pll);

	/* set postdiv */
	val = readl(pll->pd_addr);
	val &= ~(POSTDIV_MASK << pll->pd_shift);
	val |= (ffs(postdiv) - 1) << pll->pd_shift;

	/* postdiv and pcw need to set at the same time if on same register */
	if (pll->pd_addr != pll->pcw_addr) {
		writel(val, pll->pd_addr);
		val = readl(pll->pcw_addr);
	}

	/* set pcw */
	val &= ~GENMASK(pll->pcw_shift + pll->pcwbits - 1,
			pll->pcw_shift);
	val |= pcw << pll->pcw_shift;
	writel(val, pll->pcw_addr);
	chg = readl(pll->pcw_chg_addr) | PCW_CHG_MASK;
	writel(chg, pll->pcw_chg_addr);
	if (pll->tuner_addr)
		writel(val + 1, pll->tuner_addr);

	/* restore tuner_en */
	__mtk_pll_tuner_enable(pll);

	udelay(20);
}

/*
 * mtk_pll_calc_values - calculate good values for a given input frequency.
 * @pll:	The pll
 * @pcw:	The pcw value (output)
 * @postdiv:	The post divider (output)
 * @freq:	The desired target frequency
 * @fin:	The input frequency
 *
 */
static void mtk_pll_calc_values(struct mtk_clk_pll *pll, u32 *pcw, u32 *postdiv,
		u32 freq, u32 fin)
{
	unsigned long fmin = pll->fmin ? pll->fmin : (1000 * MHZ);
	const struct mtk_pll_div_table *div_table = pll->div_table;
	u64 _pcw;
	int ibits;
	u32 val;

	if (freq > pll->fmax)
		freq = pll->fmax;

	if (div_table) {
		if (freq > div_table[0].freq)
			freq = div_table[0].freq;

		for (val = 0; div_table[val + 1].freq != 0; val++) {
			if (freq > div_table[val + 1].freq)
				break;
		}
		*postdiv = 1 << val;
	} else {
		for (val = 0; val < 5; val++) {
			*postdiv = 1 << val;
			if ((u64)freq * *postdiv >= fmin)
				break;
		}
	}

	/* _pcw = freq * postdiv / fin * 2^pcwfbits */
	ibits = pll->pcwibits ? pll->pcwibits : INTEGER_BITS;
	_pcw = ((u64)freq << val) << (pll->pcwbits - ibits);
	do_div(_pcw, fin);

	*pcw = (u32)_pcw;
}

static ulong mtk_pll_set_rate(struct clk *clk, unsigned long rate)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(clk);
	unsigned long parent_rate = clk_get_parent_rate(clk);
	u32 pcw = 0;
	u32 postdiv;

	mtk_pll_calc_values(pll, &pcw, &postdiv, rate, parent_rate);
	mtk_pll_set_rate_regs(pll, pcw, postdiv);

	return __mtk_pll_recalc_rate(pll, parent_rate, pcw, postdiv);
}

static unsigned long mtk_pll_recalc_rate(struct clk *clk)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(clk);
	unsigned long parent_rate = clk_get_parent_rate(clk);
	u32 postdiv;
	u32 pcw;

	postdiv = (readl(pll->pd_addr) >> pll->pd_shift) & POSTDIV_MASK;
	postdiv = 1 << postdiv;

	pcw = readl(pll->pcw_addr) >> pll->pcw_shift;
	pcw &= GENMASK(pll->pcwbits - 1, 0);

	return __mtk_pll_recalc_rate(pll, parent_rate, pcw, postdiv);
}

static int mtk_pll_prepare(struct clk *clk)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(clk);
	u32 r;
	u32 div_en_mask;

	r = readl(pll->pwr_addr) | CON0_PWR_ON;
	writel(r, pll->pwr_addr);
	udelay(1);

	r = readl(pll->pwr_addr) & ~CON0_ISO_EN;
	writel(r, pll->pwr_addr);
	udelay(1);

	r = readl(pll->en_addr) | BIT(pll->pll_en_bit);
	writel(r, pll->en_addr);

	div_en_mask = pll->en_mask & ~CON0_BASE_EN;
	if (div_en_mask) {
		r = readl(pll->base_addr + REG_CON0) | div_en_mask;
		writel(r, pll->base_addr + REG_CON0);
	}

	__mtk_pll_tuner_enable(pll);

	udelay(20);

	if (pll->mtk_flags & MTKF_RST_BAR) {
		r = readl(pll->base_addr + REG_CON0);
		r |= pll->rst_bar_mask;
		writel(r, pll->base_addr + REG_CON0);
	}

	return 0;
}

static int mtk_pll_unprepare(struct clk *clk)
{
	struct mtk_clk_pll *pll = to_mtk_clk_pll(clk);
	u32 r;
	u32 div_en_mask;

	if (pll->mtk_flags & MTKF_RST_BAR) {
		r = readl(pll->base_addr + REG_CON0);
		r &= ~pll->rst_bar_mask;
		writel(r, pll->base_addr + REG_CON0);
	}

	__mtk_pll_tuner_disable(pll);

	div_en_mask = pll->en_mask & ~CON0_BASE_EN;
	if (div_en_mask) {
		r = readl(pll->base_addr + REG_CON0) & ~div_en_mask;
		writel(r, pll->base_addr + REG_CON0);
	}

	r = readl(pll->en_addr) & ~BIT(pll->pll_en_bit);
	writel(r, pll->en_addr);

	r = readl(pll->pwr_addr) | CON0_ISO_EN;
	writel(r, pll->pwr_addr);

	r = readl(pll->pwr_addr) & ~CON0_PWR_ON;
	writel(r, pll->pwr_addr);
	return 0;
}

struct clk *mtk_clk_register_pll(const char *name,
	const char *parent, unsigned long flags,
	void __iomem *base, unsigned reg_ofs,
	unsigned tuner_ofs, unsigned tuner_en_ofs,
	u8 tuner_en_bit, u8 pll_en_bit, u8 mtk_flags)
{
	void __iomem *base_addr = base + reg_ofs;
	struct mtk_clk_pll *pll;
	struct clk *clk;
	int ret;

	if (mtk_flags & MTKF_PLL_AO)
		flags |= CLK_IS_CRITICAL;
	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->base_addr = base_addr;
	pll->pwr_addr = base_addr + ((mtk_flags & MTKF_PWR_REG1) ? 0x10 : 0x0c);
	pll->pd_addr = base_addr + 0x04;
	pll->pcw_addr = base_addr + ((mtk_flags & MTKF_PWR_REG1) ? 8 : 4);
	pll->pcw_chg_addr = pll->base_addr + REG_CON1;
	clk = &pll->clk;
	clk->flags = flags;

	if (tuner_ofs)
		pll->tuner_addr = base + tuner_ofs;
	if (tuner_en_ofs || tuner_en_bit)
		pll->tuner_en_addr = base + tuner_en_ofs;
	pll->en_addr = base_addr + REG_CON0;
	pll->tuner_en_bit = tuner_en_bit;
	pll->pll_en_bit = pll_en_bit;
	pll->mtk_flags = mtk_flags;
	pll->en_mask = ((mtk_flags & MTKF_RST_BAR) ? 0xff000000 : 0);
	pll->rst_bar_mask = BIT(23);
	pll->pcwbits = ((mtk_flags & MTKF_PWR_REG1) ? 32 : 22);
	pll->pd_shift = 24;
	pll->fmax = (3800UL * MHZ);
	pll->fmin = (1500UL * MHZ);
	pll->pcwibits = 8;
	pll->pcw_shift = 0;

	ret = clk_register(clk, UBOOT_DM_CLK_CCF_MTK_PLL,
		name, parent);
	if (ret) {
		kfree(pll);
		return ERR_PTR(ret);
	}
	return clk;
}

static const struct clk_ops mtk_clk_pll_ops = {
	.enable		= mtk_pll_prepare,
	.disable	= mtk_pll_unprepare,
	.get_rate	= mtk_pll_recalc_rate,
	.set_rate	= mtk_pll_set_rate,
};

U_BOOT_DRIVER(mtk_clk_pll) = {
	.name	= UBOOT_DM_CLK_CCF_MTK_PLL,
	.id	= UCLASS_CLK,
	.ops	= &mtk_clk_pll_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
