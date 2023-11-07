// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 BayLibre, SAS
 * Author: Vitor Sato Eschholz <vitor.satoes@baylibre.com>
 *
 */

#include <adc.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <errno.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <power/regulator.h>

#define MT6577_AUXADC_CON1                  0x04
#define MT6577_AUXADC_CON2                  0x10
#define MT6577_AUXADC_STA                   BIT(0)
#define MT6577_AUXADC_DAT0                  0x14
#define MT6577_AUXADC_RDY0                  BIT(12)
#define MT6577_AUXADC_MISC                  0x94
#define MT6577_AUXADC_PDN_EN                BIT(14)

#define MT6577_AUXADC_DAT_MASK              0xfff
#define MT6577_AUXADC_TIMEOUT_US            10000
#define MT6577_AUXADC_POWER_READY_MS        1
#define MT6577_AUXADC_SAMPLE_READY_US       25
#define MT6577_AUXADC_NUM_CHANNELS          15

struct mtk_auxadc_compatible {
	bool sample_data_cali;
	bool check_global_idle;
};

struct mt6577_auxadc_priv {
	void __iomem *regs;
	struct clk adc_clk;
	const struct mtk_auxadc_compatible *dev_comp;
	int active_channel;
};

static inline void mt6577_auxadc_mod_reg(void __iomem *reg,
					 u32 or_mask, u32 and_mask)
{
	u32 val;

	val = readl(reg);
	val |= or_mask;
	val &= ~and_mask;
	writel(val, reg);
}

static int mt6577_auxadc_channel_data(struct udevice *dev, int channel,
				      unsigned int *data)
{
	struct mt6577_auxadc_priv *priv = dev_get_priv(dev);
	void __iomem *reg_channel;
	int ret;
	u32 val;

	if (channel != priv->active_channel) {
		dev_err(dev, "Requested channel is not active!");
		return -EINVAL;
	}

	reg_channel = priv->regs + MT6577_AUXADC_DAT0 + (channel * 0x04);

	ret = readl_poll_timeout(reg_channel, val,
				 ((val & MT6577_AUXADC_RDY0) != 0),
				 MT6577_AUXADC_TIMEOUT_US);
	if (ret == -ETIMEDOUT) {
		dev_err(dev, "ADC channel[%d] conversion timed out\n", channel);
		return ret;
	}

	*data = readl(reg_channel);

	return 0;
}

static int mt6577_auxadc_start_channel(struct udevice *dev, int channel)
{
	struct mt6577_auxadc_priv *priv = dev_get_priv(dev);
	void __iomem *reg_channel;
	u32 val;
	int ret;

	if (channel < 0 || channel >= MT6577_AUXADC_NUM_CHANNELS) {
		dev_err(dev, "Requested channel is invalid!");
		return -EINVAL;
	}

	reg_channel = priv->regs + MT6577_AUXADC_DAT0 + (channel * 0x04);

	/* Set Immediate Mode to channel */
	mt6577_auxadc_mod_reg(priv->regs + MT6577_AUXADC_CON1,
			      0, 1 << channel);

	/* Make sure ready bit == 0 */
	ret = readl_poll_timeout(reg_channel, val,
				 ((val & MT6577_AUXADC_RDY0) == 0),
				 MT6577_AUXADC_TIMEOUT_US);
	if (ret == -ETIMEDOUT) {
		dev_err(dev, "Waiting for channel[%d] ready bit clear timed out\n",
		       channel);
		return ret;
	}

	/* Trigger sampling */
	mt6577_auxadc_mod_reg(priv->regs + MT6577_AUXADC_CON1,
			      1 << channel, 0);

	udelay(MT6577_AUXADC_SAMPLE_READY_US);

	if (priv->dev_comp->check_global_idle) {
		ret = readl_poll_timeout(priv->regs + MT6577_AUXADC_CON2,
					 val, ((val & MT6577_AUXADC_STA) == 0),
					 MT6577_AUXADC_TIMEOUT_US);
		if (ret == -ETIMEDOUT) {
			dev_err(dev, "AUXADC is busy\n");
			return ret;
		}
	}

	priv->active_channel = channel;

	return 0;
}

static int mt6577_auxadc_stop(struct udevice *dev)
{
	struct mt6577_auxadc_priv *priv = dev_get_priv(dev);

	priv->active_channel = -1;

	return 0;
}

static int mt6577_auxadc_probe(struct udevice *dev)
{
	struct adc_uclass_plat *uc_pdata = dev_get_uclass_plat(dev);
	struct mt6577_auxadc_priv *priv = dev_get_priv(dev);
	struct udevice *vref;
	int vref_uv;
	int ret;

	priv->dev_comp = (struct mtk_auxadc_compatible *)dev_get_driver_data(dev);

	/* Enable auxadc clock */
	mt6577_auxadc_mod_reg(priv->regs + MT6577_AUXADC_MISC,
			      MT6577_AUXADC_PDN_EN, 0);

	mdelay(MT6577_AUXADC_POWER_READY_MS);

	priv->active_channel = -1;

	ret = device_get_supply_regulator(dev, "vref-supply", &vref);
	if (ret) {
		dev_warn(dev, "can't get vref-supply: %d\n", ret);
		dev_warn(dev, "will only read raw values\n");
	} else {
		vref_uv = regulator_get_value(vref);
		if (vref_uv < 0) {
			dev_err(dev, "can't get vref-supply value: %d\n", vref_uv);
			return vref_uv;
		}
		/* VDD supplied by common vref pin */
		uc_pdata->vdd_supply = vref;
		uc_pdata->vdd_microvolts = vref_uv;
		uc_pdata->vss_microvolts = 0;
	}

	return 0;
}

static int mt6577_auxadc_of_to_plat(struct udevice *dev)
{
	struct adc_uclass_plat *uc_pdata = dev_get_uclass_plat(dev);
	struct mt6577_auxadc_priv *priv = dev_get_priv(dev);
	int ret;

	priv->regs = dev_read_addr_ptr(dev);
	if (priv->regs == (struct mt6577_auxadc *)FDT_ADDR_T_NONE) {
		pr_err("Dev: %s - failed to get base address!", dev->name);
		return -ENODATA;
	}

	ret = clk_get_by_name(dev, "main", &priv->adc_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to get main adc clk: %d\n", ret);
		return ret;
	}
	ret = clk_enable(&priv->adc_clk);
	if (ret) {
		dev_err(dev, "Failed to enable main adc clk: %d\n", ret);
		return ret;
	}

	uc_pdata->data_mask = MT6577_AUXADC_DAT_MASK;
	uc_pdata->data_format = ADC_DATA_FORMAT_BIN;
	uc_pdata->data_timeout_us = MT6577_AUXADC_TIMEOUT_US;
	uc_pdata->channel_mask = GENMASK(MT6577_AUXADC_NUM_CHANNELS - 1, 0);

	return 0;

}

static const struct adc_ops mt6577_auxadc_ops = {
	.start_channel = mt6577_auxadc_start_channel,
	.channel_data = mt6577_auxadc_channel_data,
	.stop = mt6577_auxadc_stop,
};

static const struct mtk_auxadc_compatible mt6765_compat = {
	.sample_data_cali = true,
	.check_global_idle = false,
};

static const struct mtk_auxadc_compatible mt8173_compat = {
	.sample_data_cali = false,
	.check_global_idle = true,
};

static const struct mtk_auxadc_compatible mt8186_compat = {
	.sample_data_cali = false,
	.check_global_idle = false,
};

static const struct udevice_id mt6577_auxadc_ids[] = {
	{ .compatible = "mediatek,mt2701-auxadc",
	  .data = (ulong)&mt8173_compat },
	{ .compatible = "mediatek,mt2712-auxadc",
	  .data = (ulong)&mt8173_compat },
	{ .compatible = "mediatek,mt6765-auxadc",
	  .data = (ulong)&mt6765_compat },
	{ .compatible = "mediatek,mt7622-auxadc",
	  .data = (ulong)&mt8173_compat },
	{ .compatible = "mediatek,mt8173-auxadc",
	  .data = (ulong)&mt8173_compat },
	{ .compatible = "mediatek,mt8188-auxadc",
	  .data = (ulong)&mt8173_compat },
	{ .compatible = "mediatek,mt8186-auxadc",
	  .data = (ulong)&mt8186_compat },
	{ }
};

U_BOOT_DRIVER(mt6577_auxadc) = {
	.name       = "mt6577-auxadc",
	.id         = UCLASS_ADC,
	.of_match   = mt6577_auxadc_ids,
	.ops        = &mt6577_auxadc_ops,
	.probe      = mt6577_auxadc_probe,
	.of_to_plat = mt6577_auxadc_of_to_plat,
	.priv_auto  = sizeof(struct mt6577_auxadc_priv),
};
