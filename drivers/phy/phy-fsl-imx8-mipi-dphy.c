// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2017,2018 NXP
 * Copyright 2019 Purism SPC
 */

#include <common.h>
#include <clk.h>
#include <div64.h>
#include <dm.h>
#include <dm/devres.h>
#include <generic-phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <regmap.h>

/* DPHY registers */
#define DPHY_PD_DPHY			0x00
#define DPHY_M_PRG_HS_PREPARE		0x04
#define DPHY_MC_PRG_HS_PREPARE		0x08
#define DPHY_M_PRG_HS_ZERO		0x0c
#define DPHY_MC_PRG_HS_ZERO		0x10
#define DPHY_M_PRG_HS_TRAIL		0x14
#define DPHY_MC_PRG_HS_TRAIL		0x18
#define DPHY_PD_PLL			0x1c
#define DPHY_TST			0x20
#define DPHY_CN				0x24
#define DPHY_CM				0x28
#define DPHY_CO				0x2c
#define DPHY_LOCK			0x30
#define DPHY_LOCK_BYP			0x34
#define DPHY_REG_BYPASS_PLL		0x4C

#define MBPS(x) ((x) * 1000000)

#define DATA_RATE_MAX_SPEED MBPS(1500)
#define DATA_RATE_MIN_SPEED MBPS(80)

#define PLL_LOCK_SLEEP 10
#define PLL_LOCK_TIMEOUT 1000

#define CN_BUF	0xcb7a89c0
#define CO_BUF	0x63
#define CM(x)	(				  \
		((x) <	32) ? 0xe0 | ((x) - 16) : \
		((x) <	64) ? 0xc0 | ((x) - 32) : \
		((x) < 128) ? 0x80 | ((x) - 64) : \
		((x) - 128))
#define CN(x)	(((x) == 1) ? 0x1f : (((CN_BUF) >> ((x) - 1)) & 0x1f))
#define CO(x)	((CO_BUF) >> (8 - (x)) & 0x03)

/* PHY power on is active low */
#define PWR_ON	0
#define PWR_OFF	1

enum mixel_dphy_devtype {
	MIXEL_IMX8MQ,
	MIXEL_IMX8QM,
	MIXEL_IMX8QX,
};

struct mixel_dphy_devdata {
	u8 reg_tx_rcal;
	u8 reg_auto_pd_en;
	u8 reg_rxlprp;
	u8 reg_rxcdrp;
	u8 reg_rxhs_settle;
};

struct mixel_dphy_cfg {
	/* DPHY PLL parameters */
	u32 cm;
	u32 cn;
	u32 co;
	/* DPHY register values */
	u8 mc_prg_hs_prepare;
	u8 m_prg_hs_prepare;
	u8 mc_prg_hs_zero;
	u8 m_prg_hs_zero;
	u8 mc_prg_hs_trail;
	u8 m_prg_hs_trail;
	u8 rxhs_settle;
};

struct mixel_dphy_priv {
	struct mixel_dphy_cfg cfg;
	struct regmap *regmap;
	struct clk *phy_ref_clk;
	struct clk	*dsi_clk;
	unsigned long	frequency;
	const struct mixel_dphy_devdata *devdata;
};

static int phy_write(struct phy *phy, u32 value, unsigned int reg)
{
	struct mixel_dphy_priv *priv = dev_get_priv(phy->dev);
	int ret;

	if (0) debug("%s: %p %x %x\n", __func__, priv->regmap, reg, value);
	ret = regmap_write(priv->regmap, reg, value);
	if (ret < 0)
		debug("Failed to write DPHY reg %d: %d\n", reg,
			ret);
	return ret;
}

/*
 * Find a ratio close to the desired one using continued fraction
 * approximation ending either at exact match or maximum allowed
 * nominator, denominator.
 * continued fraction
 *      2  1  2  1  2
 * 0 1  2  3  8 11 30
 * 1 0  1  1  3  4 11
 */
static void get_best_ratio(unsigned long *pnum, unsigned long *pdenom, u32 max_n, u32 max_d)
{
	unsigned long a = *pnum;
	unsigned long b = *pdenom;
	unsigned long c;
	u32 n[] = {0, 1};
	u32 d[] = {1, 0};
	u32 whole;
	unsigned int i = 1;

	while (b) {
		i ^= 1;
		whole = a / b;
		n[i] += (n[i ^ 1] * whole);
		d[i] += (d[i ^ 1] * whole);
//		printf("cf=%i n=%i d=%i\n", whole, n[i], d[i]);
		if ((n[i] > max_n) || (d[i] > max_d)) {
			i ^= 1;
			break;
		}
		c = a - (b * whole);
		a = b;
		b = c;
	}
	*pnum = n[i];
	*pdenom = d[i];
}

static int mixel_dphy_config_from_opts(struct phy *phy,
	       struct phy_configure_opts_mipi_dphy *dphy_opts,
	       struct mixel_dphy_cfg *cfg)
{
	struct mixel_dphy_priv *priv = dev_get_priv(phy->dev);
	unsigned long ref_clk = clk_get_rate(priv->phy_ref_clk);
	u32 lp_t;
	unsigned long numerator, denominator;
	unsigned max_d = 256;
	unsigned long bit_clk;
	unsigned long long tmp;
	u32 n;
	int i = 0;

	bit_clk = dphy_opts->hs_clk_rate;
	if (bit_clk > DATA_RATE_MAX_SPEED ||
	    bit_clk < DATA_RATE_MIN_SPEED)
		return -EINVAL;

	/* CM ranges between 16 and 255 */
	/* CN ranges between 1 and 32 */
	/* CO is power of 2: 1, 2, 4, 8 */
	do {
		numerator = bit_clk << i;
		denominator = ref_clk;
		get_best_ratio(&numerator, &denominator, 255, max_d >> i);
		denominator <<= i;
		i++;
	} while ((denominator >> __ffs(denominator)) > 32);

	if (!numerator || !denominator) {
		debug("Invalid %ld/%ld for %ld/%ld\n",
			numerator, denominator,
			bit_clk, ref_clk);
		return -EINVAL;
	}

	while ((numerator < 16) && (denominator <= 128)) {
		numerator <<= 1;
		denominator <<= 1;
	}
	/*
	 * CM ranges between 16 and 255
	 * CN ranges between 1 and 32
	 * CO is power of 2: 1, 2, 4, 8
	 */
	i = __ffs(denominator);
	if (i > 3)
		i = 3;
	cfg->cn = denominator >> i;
	cfg->co = 1 << i;
	cfg->cm = numerator;

	if (cfg->cm < 16 || cfg->cm > 255 ||
	    cfg->cn < 1 || cfg->cn > 32 ||
	    cfg->co < 1 || cfg->co > 8) {
		debug("Invalid CM/CN/CO values: %u/%u/%u\n",
			cfg->cm, cfg->cn, cfg->co);
		debug("for hs_clk/ref_clk=%ld/%ld ~ %ld/%ld\n",
			bit_clk, ref_clk,
			numerator, denominator);
		return -EINVAL;
	}

	dphy_opts->hs_clk_rate = ref_clk * cfg->cm / (cfg->co * cfg->cn);
	debug("hs_clk(%ld)/ref_clk=%ld/%ld ~ %ld/%ld\n",
			dphy_opts->hs_clk_rate, bit_clk, ref_clk,
			numerator, denominator);

	/* LP clock period */
	tmp = 1000000000000LL;
	do_div(tmp, dphy_opts->lp_clk_rate); /* ps */
	if (tmp > ULONG_MAX)
		return -EINVAL;

	lp_t = tmp;
	debug("LP clock %lu, period: %u ps\n",
		dphy_opts->lp_clk_rate, lp_t);

	/* hs_prepare: in lp clock periods */
	if (2 * dphy_opts->hs_prepare > 5 * lp_t) {
		dev_err(&phy->dev,
			"hs_prepare (%u) > 2.5 * lp clock period (%u)\n",
			dphy_opts->hs_prepare, lp_t);
		return -EINVAL;
	}
	/* 00: lp_t, 01: 1.5 * lp_t, 10: 2 * lp_t, 11: 2.5 * lp_t */
	tmp = 2 * (dphy_opts->hs_prepare) + lp_t - 1;
	do_div(tmp, lp_t);
	if (tmp > 2) {
		n = tmp - 2;
		if (n > 3)
			n = 3;
	} else {
		n = 0;
	}
	cfg->m_prg_hs_prepare = n;

	/* clk_prepare: in lp clock periods */
	if (2 * dphy_opts->clk_prepare > 3 * lp_t) {
		dev_err(&phy->dev,
			"clk_prepare (%u) > 1.5 * lp clock period (%u)\n",
			dphy_opts->clk_prepare, lp_t);
		return -EINVAL;
	}
	/* 00: lp_t, 01: 1.5 * lp_t */
	cfg->mc_prg_hs_prepare = dphy_opts->clk_prepare > lp_t ? 1 : 0;

	/* hs_zero: formula from NXP BSP */
	n = (144 * (bit_clk / 1000000) - 47500) / 10000;
	cfg->m_prg_hs_zero = n < 1 ? 1 : n;

	/* clk_zero: formula from NXP BSP */
	n = (34 * (bit_clk / 1000000) - 2500) / 1000;
	cfg->mc_prg_hs_zero = n < 1 ? 1 : n;

	/* clk_trail, hs_trail: formula from NXP BSP */
	n = (103 * (bit_clk / 1000000) + 10000) / 10000;
	if (n > 15)
		n = 15;
	if (n < 1)
		n = 1;
	cfg->m_prg_hs_trail = n;
	cfg->mc_prg_hs_trail = n;

	/* rxhs_settle: formula from NXP BSP */
	if (bit_clk < MBPS(80))
		cfg->rxhs_settle = 0x0d;
	else if (bit_clk < MBPS(90))
		cfg->rxhs_settle = 0x0c;
	else if (bit_clk < MBPS(125))
		cfg->rxhs_settle = 0x0b;
	else if (bit_clk < MBPS(150))
		cfg->rxhs_settle = 0x0a;
	else if (bit_clk < MBPS(225))
		cfg->rxhs_settle = 0x09;
	else if (bit_clk < MBPS(500))
		cfg->rxhs_settle = 0x08;
	else
		cfg->rxhs_settle = 0x07;

	debug("phy_config: %u %u %u %u %u %u %u\n",
		cfg->m_prg_hs_prepare, cfg->mc_prg_hs_prepare,
		cfg->m_prg_hs_zero, cfg->mc_prg_hs_zero,
		cfg->m_prg_hs_trail, cfg->mc_prg_hs_trail,
		cfg->rxhs_settle);

	return 0;
}

static void mixel_phy_set_hs_timings(struct phy *phy)
{
	struct mixel_dphy_priv *priv = dev_get_priv(phy->dev);

	phy_write(phy, priv->cfg.m_prg_hs_prepare, DPHY_M_PRG_HS_PREPARE);
	phy_write(phy, priv->cfg.mc_prg_hs_prepare, DPHY_MC_PRG_HS_PREPARE);
	phy_write(phy, priv->cfg.m_prg_hs_zero, DPHY_M_PRG_HS_ZERO);
	phy_write(phy, priv->cfg.mc_prg_hs_zero, DPHY_MC_PRG_HS_ZERO);
	phy_write(phy, priv->cfg.m_prg_hs_trail, DPHY_M_PRG_HS_TRAIL);
	phy_write(phy, priv->cfg.mc_prg_hs_trail, DPHY_MC_PRG_HS_TRAIL);
	phy_write(phy, priv->cfg.rxhs_settle, priv->devdata->reg_rxhs_settle);
}

static int mixel_dphy_set_pll_params(struct phy *phy)
{
	struct mixel_dphy_priv *priv = dev_get_priv(phy->dev);

	if (priv->cfg.cm < 16 || priv->cfg.cm > 255 ||
	    priv->cfg.cn < 1 || priv->cfg.cn > 32 ||
	    priv->cfg.co < 1 || priv->cfg.co > 8) {
		debug("Invalid CM/CN/CO values! (%u/%u/%u)\n",
			priv->cfg.cm, priv->cfg.cn, priv->cfg.co);
		return -EINVAL;
	}
	debug("Using CM:%u CN:%u CO:%u\n",
		priv->cfg.cm, priv->cfg.cn, priv->cfg.co);
	phy_write(phy, CM(priv->cfg.cm), DPHY_CM);
	phy_write(phy, CN(priv->cfg.cn), DPHY_CN);
	phy_write(phy, CO(priv->cfg.co), DPHY_CO);
	return 0;
}

static int mixel_dphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct mixel_dphy_priv *priv = dev_get_priv(phy->dev);
	struct mixel_dphy_cfg cfg = { 0 };
	int ret;
	unsigned long ref_clk;

	debug("%s:\n", __func__);
	ret = mixel_dphy_config_from_opts(phy, &opts->mipi_dphy, &cfg);
	if (ret)
		return ret;

	/* Update the configuration */
	memcpy(&priv->cfg, &cfg, sizeof(struct mixel_dphy_cfg));

	ref_clk = clk_get_rate(priv->phy_ref_clk);
	/* Divided by 2 because mipi output clock is DDR */
	priv->frequency = opts->mipi_dphy.hs_clk_rate >> 1;
	if (priv->dsi_clk) {
		clk_set_rate(priv->dsi_clk, priv->frequency);
	}
	debug("%s:%ld ref_clk=%ld, cm=%d, co=%d cn=%d\n",
		__func__, priv->frequency, ref_clk, cfg.cm, cfg.co, cfg.cn);

	phy_write(phy, 0x00, DPHY_LOCK_BYP);
	phy_write(phy, 0x01, priv->devdata->reg_tx_rcal);
	phy_write(phy, 0x00, priv->devdata->reg_auto_pd_en);
	phy_write(phy, 0x02, priv->devdata->reg_rxlprp);
	phy_write(phy, 0x02, priv->devdata->reg_rxcdrp);
	phy_write(phy, 0x25, DPHY_TST);

	mixel_phy_set_hs_timings(phy);
	ret = mixel_dphy_set_pll_params(phy);
	if (ret < 0)
		return ret;

	return 0;
}

static int mixel_dphy_validate(struct phy *phy, enum phy_mode mode, int submode,
			       union phy_configure_opts *opts)
{
	struct mixel_dphy_cfg cfg = { 0 };

	debug("%s:\n", __func__);
	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	return mixel_dphy_config_from_opts(phy, &opts->mipi_dphy, &cfg);
}

static int mixel_dphy_init(struct phy *phy)
{
	debug("%s:\n", __func__);
	phy_write(phy, PWR_OFF, DPHY_PD_PLL);
	phy_write(phy, PWR_OFF, DPHY_PD_DPHY);

	return 0;
}

static int mixel_dphy_exit(struct phy *phy)
{
	phy_write(phy, 0, DPHY_CM);
	phy_write(phy, 0, DPHY_CN);
	phy_write(phy, 0, DPHY_CO);

	return 0;
}

static int mixel_dphy_power_on(struct phy *phy)
{
	struct mixel_dphy_priv *priv = dev_get_priv(phy->dev);
	u32 locked;
	int ret;

	debug("%s:\n", __func__);
	ret = clk_prepare_enable(priv->phy_ref_clk);
	if (ret < 0)
		return ret;

	phy_write(phy, PWR_ON, DPHY_PD_PLL);
	ret = regmap_read_poll_timeout(priv->regmap, DPHY_LOCK, locked,
				       locked, PLL_LOCK_SLEEP,
				       PLL_LOCK_TIMEOUT);
	if (ret < 0) {
		debug("Could not get DPHY lock (%d)!\n", ret);
		goto clock_disable;
	}
	phy_write(phy, PWR_ON, DPHY_PD_DPHY);

	return 0;
clock_disable:
	clk_disable_unprepare(priv->phy_ref_clk);
	return ret;
}

static int mixel_dphy_power_off(struct phy *phy)
{
	struct mixel_dphy_priv *priv = dev_get_priv(phy->dev);

	debug("%s:\n", __func__);
	phy_write(phy, PWR_OFF, DPHY_PD_PLL);
	phy_write(phy, PWR_OFF, DPHY_PD_DPHY);

	clk_disable_unprepare(priv->phy_ref_clk);

	return 0;
}

static int mixel_dphy_xlate(struct phy *phy,
		struct ofnode_phandle_args *args)
{
	if (args->args_count)
		phy->id = args->args[0];
	else
		phy->id = 0;

	debug("%s: phy_id = %ld\n", __func__, phy->id);
	return 0;
}


static const struct phy_ops mixel_dphy_ops = {
	.of_xlate = mixel_dphy_xlate,
	.init = mixel_dphy_init,
	.power_on = mixel_dphy_power_on,
	.power_off = mixel_dphy_power_off,
	.exit = mixel_dphy_exit,
	.configure = mixel_dphy_configure,
	.validate = mixel_dphy_validate,
};

static const struct mixel_dphy_devdata mixel_dphy_devdata[] = {
	[MIXEL_IMX8MQ] = {
		.reg_tx_rcal = 0x38,
		.reg_auto_pd_en = 0x3c,
		.reg_rxlprp = 0x40,
		.reg_rxcdrp = 0x44,
		.reg_rxhs_settle = 0x48,
	},
	[MIXEL_IMX8QM] = {
		.reg_tx_rcal = 0x00,
		.reg_auto_pd_en = 0x38,
		.reg_rxlprp = 0x3c,
		.reg_rxcdrp = 0x40,
		.reg_rxhs_settle = 0x44,
	},
	[MIXEL_IMX8QX] = {
		.reg_tx_rcal = 0x00,
		.reg_auto_pd_en = 0x38,
		.reg_rxlprp = 0x3c,
		.reg_rxcdrp = 0x40,
		.reg_rxhs_settle = 0x44,
	},
};

static int mixel_dphy_probe(struct udevice *dev)
{
	struct mixel_dphy_priv *priv = dev_get_priv(dev);
	ofnode np = dev_ofnode(dev);
	int ret;

	priv->devdata = &mixel_dphy_devdata[dev_get_driver_data(dev)];
	if (!priv->devdata)
		return -EINVAL;

	ret = regmap_init_mem(np, &priv->regmap);
	if (ret) {
		debug("Couldn't create the DPHY regmap\n");
		return ret;
	}

	priv->phy_ref_clk = devm_clk_get(dev, "phy_ref");
	if (IS_ERR(priv->phy_ref_clk)) {
		debug("No phy_ref clock found\n");
		return PTR_ERR(priv->phy_ref_clk);
	}
	debug("phy_ref clock rate: %lu\n",
		clk_get_rate(priv->phy_ref_clk));

	priv->dsi_clk = devm_clk_get(dev, "result");
	if (IS_ERR(priv->dsi_clk)) {
		debug("No dsi_clk clock found\n");
		return PTR_ERR(priv->dsi_clk);
	}
	return 0;
}

static const struct udevice_id mixel_dphy_ids[] = {
	{ .compatible = "fsl,imx8mq-mipi-dphy", .data = MIXEL_IMX8MQ },
	{ .compatible = "fsl,imx8qm-mipi-dphy", .data = MIXEL_IMX8QM },
	{ .compatible = "fsl,imx8qx-mipi-dphy", .data = MIXEL_IMX8QX },
	{ /* sentinel */ },
};

U_BOOT_DRIVER(mixel_dphy) = {
	.name	= "mixel-dphy",
	.id	= UCLASS_PHY,
	.of_match = mixel_dphy_ids,
	.ops = &mixel_dphy_ops,
	.probe = mixel_dphy_probe,
	.priv_auto_alloc_size = sizeof(struct mixel_dphy_priv),
};
