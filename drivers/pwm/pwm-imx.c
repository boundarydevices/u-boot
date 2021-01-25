// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2014
 * Heiko Schocher, DENX Software Engineering, hs@denx.de.
 *
 * Basic support for the pwm module on imx6.
 */

#include <common.h>
#include <clk.h>
#include <div64.h>
#include <dm.h>
#include <log.h>
#include <pwm.h>
#include <asm/arch/imx-regs.h>
#include <asm/io.h>
#include "pwm-imx-util.h"

int pwm_init(int pwm_id, int div, int invert)
{
	struct pwm_regs *pwm = (struct pwm_regs *)pwm_id_to_reg(pwm_id);

	if (!pwm)
		return -1;

	writel(0, &pwm->ir);
	return 0;
}

int pwm_config_internal(struct pwm_regs *pwm, unsigned long period_cycles,
			unsigned long duty_cycles, unsigned long prescale)
{
	u32 cr;

	writel(0, &pwm->ir);
	cr = PWMCR_PRESCALER(prescale) |
		PWMCR_DOZEEN | PWMCR_WAITEN |
		PWMCR_DBGEN | PWMCR_CLKSRC_IPG_HIGH;

	writel(cr, &pwm->cr);
	/* set duty cycles */
	writel(duty_cycles, &pwm->sar);
	/* set period cycles */
	writel(period_cycles, &pwm->pr);
	return 0;
}

int pwm_config(int pwm_id, int duty_ns, int period_ns)
{
	struct pwm_regs *pwm = (struct pwm_regs *)pwm_id_to_reg(pwm_id);
	unsigned long period_cycles, duty_cycles, prescale;

	if (!pwm)
		return -1;

	pwm_imx_get_parms(CONFIG_IMX6_PWM_PER_CLK, period_ns, duty_ns,
			&period_cycles, &duty_cycles, &prescale);

	return pwm_config_internal(pwm, period_cycles, duty_cycles, prescale);
}

int pwm_enable(int pwm_id)
{
	struct pwm_regs *pwm = (struct pwm_regs *)pwm_id_to_reg(pwm_id);

	if (!pwm)
		return -1;

	setbits_le32(&pwm->cr, PWMCR_EN);
	return 0;
}

void pwm_disable(int pwm_id)
{
	struct pwm_regs *pwm = (struct pwm_regs *)pwm_id_to_reg(pwm_id);

	if (!pwm)
		return;

	clrbits_le32(&pwm->cr, PWMCR_EN);
}

#if defined(CONFIG_DM_PWM)
struct imx_pwm_priv {
	struct pwm_regs *regs;
	bool invert;
	struct clk *ipg_clk;
	struct clk *per_clk;
	unsigned long per_rate;
	int enabled;
};

static int imx_pwm_set_invert(struct udevice *dev, uint channel,
			      bool polarity)
{
	struct imx_pwm_priv *priv = dev_get_priv(dev);

	debug("%s: polarity=%u\n", __func__, polarity);
	priv->invert = polarity;

	return 0;
}

static int imx_pwm_set_config(struct udevice *dev, uint channel,
			      uint period_ns, uint duty_ns)
{
	struct imx_pwm_priv *priv = dev_get_priv(dev);
	struct pwm_regs *regs = priv->regs;
	unsigned long period_cycles, duty_cycles, prescale;

	debug("%s: Config '%s' channel: %d\n", __func__, dev->name, channel);
	if (!priv->enabled) {
		priv->enabled = 1;
		if (priv->ipg_clk)
			clk_prepare_enable(priv->ipg_clk);
		if (priv->per_clk)
			clk_prepare_enable(priv->per_clk);
	}

	pwm_imx_get_parms(priv->per_rate, period_ns, duty_ns, &period_cycles, &duty_cycles,
			  &prescale);

	return pwm_config_internal(regs, period_cycles, duty_cycles, prescale);
};

static int imx_pwm_set_enable(struct udevice *dev, uint channel, bool enable)
{
	struct imx_pwm_priv *priv = dev_get_priv(dev);
	struct pwm_regs *regs = priv->regs;

	debug("%s: Enable '%s' state: %d\n", __func__, dev->name, enable);

	if (enable)
		setbits_le32(&regs->cr, PWMCR_EN);
	else
		clrbits_le32(&regs->cr, PWMCR_EN);

	return 0;
};

static int imx_pwm_ofdata_to_platdata(struct udevice *dev)
{
	struct imx_pwm_priv *priv = dev_get_priv(dev);

	priv->regs = dev_read_addr_ptr(dev);
#if CONFIG_IS_ENABLED(CLK)
	priv->ipg_clk = devm_clk_get_optional(dev, "ipg");
	if (IS_ERR(priv->ipg_clk)) {
		dev_err(dev, "Unable to get ipg clock: %ld\n",
				PTR_ERR(priv->ipg_clk));
		return PTR_ERR(priv->ipg_clk);
	}
	priv->per_clk = devm_clk_get_optional(dev, "per");
	if (IS_ERR(priv->per_clk)) {
		dev_err(dev, "Unable to get per clock: %ld\n",
				PTR_ERR(priv->per_clk));
		return PTR_ERR(priv->per_clk);
	}
#endif
	priv->per_rate = clk_get_rate(priv->per_clk);

	return 0;
}

static int imx_pwm_probe(struct udevice *dev)
{
	return 0;
}

static const struct pwm_ops imx_pwm_ops = {
	.set_invert	= imx_pwm_set_invert,
	.set_config	= imx_pwm_set_config,
	.set_enable	= imx_pwm_set_enable,
};

static const struct udevice_id imx_pwm_ids[] = {
	{ .compatible = "fsl,imx27-pwm" },
	{ }
};

U_BOOT_DRIVER(imx_pwm) = {
	.name	= "imx_pwm",
	.id	= UCLASS_PWM,
	.of_match = imx_pwm_ids,
	.ops	= &imx_pwm_ops,
	.ofdata_to_platdata	= imx_pwm_ofdata_to_platdata,
	.probe		= imx_pwm_probe,
	.priv_auto_alloc_size	= sizeof(struct imx_pwm_priv),
};
#endif
