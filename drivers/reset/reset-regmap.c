// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2021 NXP
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <imx_sip.h>
#include <regmap.h>
#include <reset.h>
#include <reset-uclass.h>
#include <regmap.h>
#ifdef CONFIG_IMX8ULP
#include <dt-bindings/reset/imx8ulp-sim-reset.h>
#include <dt-bindings/reset/imx8ulp-pcc-reset.h>
#endif

struct reset_init {
	int	num_resets;
	const struct reset_desc *resets;
};

struct reset_desc {
	unsigned char offset;
	unsigned char bit;
};

struct reset_regmap_priv {
	struct regmap	*regmap;
	int		num_resets;
	const struct reset_desc *resets;
};

static int reset_regmap_assert(struct reset_ctl *rst)
{
	struct reset_regmap_priv *priv = dev_get_priv(rst->dev);
	const struct reset_desc *rd;
	u32 mask;

	if (rst->id >= priv->num_resets) {
		debug("%s:error %ld >= %d\n", __func__, rst->id, priv->num_resets);
		return -EINVAL;
	}

	rd = &priv->resets[rst->id];
	mask = 1 << rd->bit;
	return regmap_update_bits(priv->regmap, rd->offset, mask, 0);
}

static int reset_regmap_deassert(struct reset_ctl *rst)
{
	struct reset_regmap_priv *priv = dev_get_priv(rst->dev);
	const struct reset_desc *rd;
	u32 mask;

	if (rst->id >= priv->num_resets) {
		debug("%s:error %ld >= %d\n", __func__, rst->id, priv->num_resets);
		return -EINVAL;
	}

	rd = &priv->resets[rst->id];
	mask = 1 << rd->bit;
	return regmap_update_bits(priv->regmap, rd->offset, mask, mask);
}

static int reset_regmap_request(struct reset_ctl *rst)
{
	struct reset_regmap_priv *priv = dev_get_priv(rst->dev);

	if (rst->id >= priv->num_resets) {
		debug("%s:error %ld >= %d\n", __func__, rst->id, priv->num_resets);
		return -EINVAL;
	}
	return 0;
}

static int reset_regmap_free(struct reset_ctl *rst)
{
	return 0;
}

static const struct reset_ops reset_regmap_ops = {
	.request	= reset_regmap_request,
	.rfree		= reset_regmap_free,
	.rst_assert	= reset_regmap_assert,
	.rst_deassert	= reset_regmap_deassert,
};

#ifdef CONFIG_IMX8ULP
#define PCC_SW_RST_BIT	28

static const struct reset_desc imx8ulp_pcc5_resets[] = {
[PCC5_TPM8_SWRST]	= { 0xa0, PCC_SW_RST_BIT},
[PCC5_SAI6_SWRST]	= { 0xa4, PCC_SW_RST_BIT},
[PCC5_SAI7_SWRST]	= { 0xa8, PCC_SW_RST_BIT},
[PCC5_SPDIF_SWRST]	= { 0xac, PCC_SW_RST_BIT},
[PCC5_ISI_SWRST]	= { 0xb0, PCC_SW_RST_BIT},
[PCC5_CSI_REGS_SWRST]	= { 0xb4, PCC_SW_RST_BIT},
[PCC5_CSI_SWRST]	= { 0xbc, PCC_SW_RST_BIT},
[PCC5_DSI_SWRST]	= { 0xc0, PCC_SW_RST_BIT},
[PCC5_WDOG5_SWRST]	= { 0xc8, PCC_SW_RST_BIT},
[PCC5_EPDC_SWRST]	= { 0xcc, PCC_SW_RST_BIT},
[PCC5_PXP_SWRST]	= { 0xd0, PCC_SW_RST_BIT},
[PCC5_GPU2D_SWRST]	= { 0xf0, PCC_SW_RST_BIT},
[PCC5_GPU3D_SWRST]	= { 0xf4, PCC_SW_RST_BIT},
[PCC5_DC_NANO_SWRST]	= { 0xf8, PCC_SW_RST_BIT},
};

const struct reset_init imx8ulp_pcc5_reset = {
	.num_resets = ARRAY_SIZE(imx8ulp_pcc5_resets),
	.resets = imx8ulp_pcc5_resets,
};

#define AVD_SIM_SYSCTRL0	0x8
#define AVDSIM_DSI_RST_DPI_BIT	3
#define AVDSIM_DSI_RST_ESC_BIT	4
#define AVDSIM_DSI_RST_BYTE_BIT	5

static const struct reset_desc imx8ulp_sim_resets[IMX8ULP_SIM_RESET_NUM] = {
[IMX8ULP_SIM_RESET_MIPI_DSI_RST_DPI_N] = { AVD_SIM_SYSCTRL0, AVDSIM_DSI_RST_DPI_BIT, },
[IMX8ULP_SIM_RESET_MIPI_DSI_RST_ESC_N] = { AVD_SIM_SYSCTRL0, AVDSIM_DSI_RST_ESC_BIT, },
[IMX8ULP_SIM_RESET_MIPI_DSI_RST_BYTE_N] = { AVD_SIM_SYSCTRL0, AVDSIM_DSI_RST_BYTE_BIT, },
};

const struct reset_init imx8ulp_sim_reset = {
	.num_resets = ARRAY_SIZE(imx8ulp_sim_resets),
	.resets = imx8ulp_sim_resets,
};
#endif

static const struct udevice_id reset_regmap_dt_ids[] = {
#ifdef CONFIG_IMX8ULP
	{ .compatible = "nxp,imx8ulp-avd-sim-reset", .data = (long)&imx8ulp_sim_reset },
	{ .compatible = "nxp,imx8ulp-pcc5-reset", .data = (long)&imx8ulp_pcc5_reset },
#endif
	{ /* sentinel */ },
};

static int reset_regmap_probe(struct udevice *dev)
{
	struct reset_regmap_priv *priv = dev_get_priv(dev);
	const struct reset_init *pdata;
	int ret;

	pdata = (const struct reset_init *)dev->driver_data;
	priv->num_resets = pdata->num_resets;
	priv->resets = pdata->resets;
	debug("%s: %d\n", __func__, priv->num_resets);
	ret = regmap_init_mem(dev_ofnode(dev), &priv->regmap);
	if (ret) {
		debug("%s: Could not initialize regmap (err = %d)\n", dev->name,
		      ret);
		return ret;
	} else {
		debug("%s: initialized regmap\n", dev->name);
	}
	return 0;
}

U_BOOT_DRIVER(reset_regmap) = {
	.name = "reset_regmap",
	.id = UCLASS_RESET,
	.of_match = reset_regmap_dt_ids,
	.ops = &reset_regmap_ops,
	.probe = reset_regmap_probe,
	.priv_auto = sizeof(struct reset_regmap_priv),
};
