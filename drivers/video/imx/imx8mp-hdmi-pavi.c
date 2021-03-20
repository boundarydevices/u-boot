// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2020 NXP
 *
 * Programe Video/Audio Interface between LCDIF and HDMI Ctrl in HDMIMIX
 *
 */

#include <common.h>
#include <clk.h>
#include <dm/device.h>
#include <dm/read.h>
#include <fdtdec.h>
#include <generic-phy.h>
#include <linux/io.h>
#include <reset.h>
#include "imx8mp-hdmi-pavi.h"

#define HTX_PVI_CTRL         0x0
#define HTX_PVI_IRQ_MASK     0x04
#define HTX_TMG_GEN_DISP_LRC 0x10
#define HTX_TMG_GEN_DE_ULC   0x14
#define HTX_TMG_GEN_DE_LRC   0x18
#define HTX_TMG_GEN_HSYNC    0x1c
#define HTX_TMG_GEN_VSYNC    0x20
#define HTX_TMG_GEN_IRQ0     0x24
#define HTX_TMG_GEN_IRQ1     0x28
#define HTX_TMG_GEN_IRQ2     0x2c
#define HTX_TMG_GEN_IRQ3     0x30
#define HTX_TMG_GEN_CFG      0x40

#define HTX_PAI_CTRL        0x800
#define HTX_PAI_CTRL_EXT    0x804
#define HTX_PAI_FIELD_CTRL  0x808

#define HTX_PAI_CTRL_ENABLE 1

/* PAI APIs  */
static int imx8mp_hdmi_pai_enable(struct imx8mp_hdmi_pavi *pavi)
{
	/* PAI set */
	writel((0x3030000 | ((pavi->channel - 1) << 8)),
			pavi->base + HTX_PAI_CTRL_EXT);

	/* hbr */
	if (pavi->non_pcm && pavi->width == 32 && pavi->channel == 8 && pavi->rate == 192000)
		writel(0x004e77df, pavi->base + HTX_PAI_FIELD_CTRL);
	else if (pavi->width == 32)
		writel(0x1c8c675b, pavi->base + HTX_PAI_FIELD_CTRL);
	else
		writel(0x1c0c675b, pavi->base + HTX_PAI_FIELD_CTRL);

	/* PAI start running */
	writel(HTX_PAI_CTRL_ENABLE, pavi->base + HTX_PAI_CTRL);
	return 0;
}

static int imx8mp_hdmi_pai_disable(struct imx8mp_hdmi_pavi *pavi)
{
	/* stop PAI */
	writel(0, pavi->base + HTX_PAI_CTRL);
	return 0;
}

/* PVI APIs  */
static int imx8mp_hdmi_pvi_enable(struct imx8mp_hdmi_pavi *pavi)
{
	writel(0x00000003, pavi->base + HTX_PVI_IRQ_MASK);
	writel(0x08970464, pavi->base + HTX_TMG_GEN_DISP_LRC);
	writel(0x00bf0029, pavi->base + HTX_TMG_GEN_DE_ULC);
	writel(0x083f0460, pavi->base + HTX_TMG_GEN_DE_LRC);
	writel(0x0897002b, pavi->base + HTX_TMG_GEN_HSYNC);
	writel(0x04640004, pavi->base + HTX_TMG_GEN_VSYNC);
	writel(0x000100ff, pavi->base + HTX_TMG_GEN_IRQ0);
	writel(0x000100f0, pavi->base + HTX_TMG_GEN_IRQ1);
	writel(0x00010315, pavi->base + HTX_TMG_GEN_IRQ2);
	writel(0x00010207, pavi->base + HTX_TMG_GEN_IRQ3);
	writel(0x84640000, pavi->base + HTX_TMG_GEN_CFG);

	/* DE/VSYN/HSYNC pol */
	if ((pavi->flags & DISPLAY_FLAGS_VSYNC_HIGH) &&
			(pavi->flags & DISPLAY_FLAGS_HSYNC_HIGH)) {
		writel(0x00377004, pavi->base + HTX_PVI_CTRL);
		writel(0x00377005, pavi->base + HTX_PVI_CTRL);
	} else {
		writel(0x00311004, pavi->base + HTX_PVI_CTRL);
		writel(0x00311005, pavi->base + HTX_PVI_CTRL);
	}
	return 0;
}

static int imx8mp_hdmi_pvi_disable(struct imx8mp_hdmi_pavi *pavi)
{
	/* Stop PVI */
	writel(0x0, pavi->base + HTX_PVI_CTRL);
	return 0;
}

static int imx8mp_hdmi_pavi_phy_init(struct phy *phy)
{
	return 0;
}

static int imx8mp_hdmi_pavi_phy_power_on(struct phy *phy)
{
	struct imx8mp_hdmi_pavi *pavi = dev_get_priv(phy->dev);

	debug("%s: phy_id = %ld\n", __func__, phy->id);
	if (phy->id == 1)
		return imx8mp_hdmi_pvi_enable(pavi);
	else if (phy->id == 2)
		return imx8mp_hdmi_pai_enable(pavi);

	clk_prepare_enable(pavi->clk_pvi);
	clk_prepare_enable(pavi->clk_pai);

	/* deassert pai reset */
	if (!pavi->reset_pai)
		reset_deassert(pavi->reset_pai);

	/* deassert pvi reset */
	if (!pavi->reset_pvi)
		reset_deassert(pavi->reset_pvi);
	return 0;
}

static int imx8mp_hdmi_pavi_phy_power_off(struct phy *phy)
{
	struct imx8mp_hdmi_pavi *pavi = dev_get_priv(phy->dev);

	debug("%s: phy_id = %ld\n", __func__, phy->id);
	if (phy->id == 1)
		return imx8mp_hdmi_pvi_disable(pavi);
	else if (phy->id == 2)
		return imx8mp_hdmi_pai_disable(pavi);

	/* set pvi reset */
	if (!pavi->reset_pvi)
		reset_assert(pavi->reset_pvi);

	/* set pai reset */
	if (!pavi->reset_pai)
		reset_assert(pavi->reset_pai);

	clk_disable_unprepare(pavi->clk_pai);
	clk_disable_unprepare(pavi->clk_pvi);
	return 0;
}

static int imx8mp_hdmi_pavi_phy_xlate(struct phy *phy,
		struct ofnode_phandle_args *args)
{
	if (args->args_count)
		phy->id = args->args[0];
	else
		phy->id = 0;

	if (phy->id > 2)
		return -EINVAL;
	return 0;
}

static int imx8mp_hdmi_pavi_probe(struct udevice *dev)
{
	struct imx8mp_hdmi_pavi *pavi = dev_get_priv(dev);
	int ret;

	debug("%s: probe begin\n", __func__);

	pavi->dev = dev;
	pavi->base = (void *)dev_read_addr(dev);
	if (IS_ERR(pavi->base))
		return PTR_ERR(pavi->base);

	pavi->clk_pvi = devm_clk_get(dev, "pvi_clk");
	if (IS_ERR(pavi->clk_pvi)) {
		ret = PTR_ERR(pavi->clk_pvi);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "pvi clock failed %d\n", ret);
		return ret;
	}

	pavi->clk_pai = devm_clk_get(dev, "pai_clk");
	if (IS_ERR(pavi->clk_pai)) {
		ret = PTR_ERR(pavi->clk_pai);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "pai clock failed %d\n", ret);
		return ret;
	}

	pavi->reset_pai = devm_reset_control_get(dev, "pai_rst");
	if (IS_ERR(pavi->reset_pai)) {
		ret = PTR_ERR(pavi->reset_pai);
		debug("PAI reset failed %d\n", ret);
		return ret;
	}

	pavi->reset_pvi = devm_reset_control_get(dev, "pvi_rst");
	if (IS_ERR(pavi->reset_pvi)) {
		ret = PTR_ERR(pavi->reset_pvi);
		debug("PVI reset failed %d\n", ret);
		return ret;
	}

	debug("%s: probe success\n", __func__);
	return 0;
}

static int imx8mp_hdmi_pavi_remove(struct udevice *dev)
{
	return 0;
}

static const struct udevice_id imx8mp_hdmi_pavi_dt_ids[] = {
	{ .compatible = "fsl,imx8mp-hdmi-pavi", },
	{ /* sentinel */ }
};

static const struct phy_ops imx8mp_hdmi_pavi_phy_ops = {
	.of_xlate = imx8mp_hdmi_pavi_phy_xlate,
	.init = imx8mp_hdmi_pavi_phy_init,
	.power_on = imx8mp_hdmi_pavi_phy_power_on,
	.power_off = imx8mp_hdmi_pavi_phy_power_off,
};

U_BOOT_DRIVER(imx8mp_hdmi_pavi) = {
	.name				= "imx8mp-hdmi-pavi",
	.id				= UCLASS_PHY,
	.of_match			= imx8mp_hdmi_pavi_dt_ids,
	.ops = &imx8mp_hdmi_pavi_phy_ops,
	.bind				= dm_scan_fdt_dev,
	.probe				= imx8mp_hdmi_pavi_probe,
	.remove				= imx8mp_hdmi_pavi_remove,
	.priv_auto_alloc_size		= sizeof(struct imx8mp_hdmi_pavi),
};
