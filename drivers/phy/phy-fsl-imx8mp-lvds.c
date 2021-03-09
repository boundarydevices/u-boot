// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2020 NXP
 */

#include <common.h>
#include <clk.h>
#include <div64.h>
#include <dm.h>
#include <dm/devres.h>
#include <dm/ofnode.h>
#include <generic-phy.h>
#include <syscon.h>
#include <regmap.h>

#define LVDS_CTRL		0x128
#define SPARE_IN(n)		(((n) & 0x7) << 25)
#define SPARE_IN_MASK		0xe000000
#define TEST_RANDOM_NUM_EN	BIT(24)
#define TEST_MUX_SRC(n)		(((n) & 0x3) << 22)
#define TEST_MUX_SRC_MASK	0xc00000
#define TEST_EN			BIT(21)
#define TEST_DIV4_EN		BIT(20)
#define VBG_ADJ(n)		(((n) & 0x7) << 17)
#define VBG_ADJ_MASK		0xe0000
#define SLEW_ADJ(n)		(((n) & 0x7) << 14)
#define SLEW_ADJ_MASK		0x1c000
#define CC_ADJ(n)		(((n) & 0x7) << 11)
#define CC_ADJ_MASK		0x3800
#define CM_ADJ(n)		(((n) & 0x7) << 8)
#define CM_ADJ_MASK		0x700
#define PRE_EMPH_ADJ(n)		(((n) & 0x7) << 5)
#define PRE_EMPH_ADJ_MASK	0xe0
#define PRE_EMPH_EN		BIT(4)
#define HS_EN			BIT(3)
#define BG_EN			BIT(2)
#define CH_EN(id)		BIT(id)


struct imx8mp_lvds_phy_priv {
	struct udevice *dev;
	struct regmap *regmap;
	struct clk *apb_clk;
};

static inline unsigned int phy_read(struct phy *phy, unsigned int reg)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_priv(phy->dev);
	unsigned int val = 0;

	regmap_read(priv->regmap, reg, &val);

	debug("%s: (0x%x) read 0x%x\n", __func__, reg, val);
	return val;
}

static inline void phy_write(struct phy *phy, unsigned int reg,
		unsigned int value)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_priv(phy->dev);

	regmap_write(priv->regmap, reg, value);
	debug("%s: (0x%x) = 0x%x\n", __func__, reg, value);
}

static int imx8mp_lvds_phy_init(struct phy *phy)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_priv(phy->dev);

	clk_prepare_enable(priv->apb_clk);

	phy_write(phy, LVDS_CTRL,
			CC_ADJ(0x2) | PRE_EMPH_EN | PRE_EMPH_ADJ(0x3));

	clk_disable_unprepare(priv->apb_clk);

	return 0;
}

static int imx8mp_lvds_phy_power_on(struct phy *phy)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_priv(phy->dev);
	unsigned int id = phy->id;
	unsigned int val;
	bool bg_en;

	debug("%s:\n", __func__);
	clk_prepare_enable(priv->apb_clk);

	val = phy_read(phy, LVDS_CTRL);
	bg_en = !!(val & BG_EN);
	val |= BG_EN;
	phy_write(phy, LVDS_CTRL, val);

	/* Wait 15us to make sure the bandgap to be stable. */
	if (!bg_en)
		udelay(15);

	val = phy_read(phy, LVDS_CTRL);
	val |= CH_EN(id);
	phy_write(phy, LVDS_CTRL, val);

	clk_disable_unprepare(priv->apb_clk);

	/* Wait 5us to ensure the phy be settling. */
	udelay(5);

	return 0;
}

static int imx8mp_lvds_phy_power_off(struct phy *phy)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_priv(phy->dev);
	unsigned int id = phy->id;
	unsigned int val;

	clk_prepare_enable(priv->apb_clk);

	val = phy_read(phy, LVDS_CTRL);
	val &= ~BG_EN;
	phy_write(phy, LVDS_CTRL, val);

	val = phy_read(phy, LVDS_CTRL);
	val &= ~CH_EN(id);
	phy_write(phy, LVDS_CTRL, val);

	clk_disable_unprepare(priv->apb_clk);
	return 0;
}

static int imx8mp_lvds_phy_xlate(struct phy *phy,
		struct ofnode_phandle_args *args)
{
	if (args->args_count)
		phy->id = args->args[0];
	else
		phy->id = 0;

	debug("%s: phy_id = %ld\n", __func__, phy->id);
	if (phy->id > 1)
		return -EINVAL;
	return 0;
}

static const struct phy_ops imx8mp_lvds_phy_ops = {
	.of_xlate = imx8mp_lvds_phy_xlate,
	.init = imx8mp_lvds_phy_init,
	.power_on = imx8mp_lvds_phy_power_on,
	.power_off = imx8mp_lvds_phy_power_off,
};

static int imx8mp_lvds_phy_probe(struct udevice *dev)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_priv(dev);

	priv->regmap = syscon_regmap_lookup_by_phandle(dev, "gpr");
	if (IS_ERR(priv->regmap)) {
		debug("failed to get regmap\n");
		return PTR_ERR(priv->regmap);
	}

	priv->dev = dev;

	priv->apb_clk = devm_clk_get(dev, "apb");
	if (IS_ERR(priv->apb_clk)) {
		debug("cannot get apb clock\n");
		return PTR_ERR(priv->apb_clk);
	}
	return 0;
}

static const struct udevice_id imx8mp_lvds_phy_of_match[] = {
	{ .compatible = "fsl,imx8mp-lvds-phy" },
	{}
};


U_BOOT_DRIVER(mixel_dphy) = {
	.name	= "imx8mp-lvds-phy",
	.id	= UCLASS_PHY,
	.of_match = imx8mp_lvds_phy_of_match,
	.ops = &imx8mp_lvds_phy_ops,
	.probe = imx8mp_lvds_phy_probe,
	.priv_auto_alloc_size = sizeof(struct imx8mp_lvds_phy_priv),
};
