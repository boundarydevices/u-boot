// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DesignWare HDMI hpd driver
 *
 * Copyright (C) 2021 Boundary Devices.
 */

#include <common.h>
#include <clk.h>
#include <display_detect.h>
#include <dm/device.h>
#include <dm/ofnode.h>
#include <dm/read.h>
#include <regmap.h>

#define HDMI_PHY_STAT0	0x0004
#define HDMI_PHY_HPD	0x02

struct dw_hdmi_detect {
	struct regmap *regm;
};

static int dw_hdmi_detect_probe(struct udevice *dev)
{
	struct dw_hdmi_detect *dhd = dev_get_priv(dev);
	ofnode np = dev_ofnode(dev);
	struct regmap *regm;
	int ret;

	ret = regmap_init_mem(np, &regm);
	if (ret) {
		debug("%s: Couldn't create regmap %d\n", __func__, ret);
		return ret;
	}
	dhd->regm = regm;

	return ret;
}

static int dw_hdmi_detect_remove(struct udevice *dev)
{
	return 0;
}

static bool dw_hdmi_detect_hpd(struct udevice *dev)
{
	struct dw_hdmi_detect *dhd = dev_get_priv(dev);
	unsigned int stat = 0;

	regmap_raw_read(dhd->regm, HDMI_PHY_STAT0, &stat, REGMAP_SIZE_8);

	return stat & HDMI_PHY_HPD ? 1 : 0;
}

struct dm_display_detect_ops dw_hdmi_detect_ops = {
	.detect = dw_hdmi_detect_hpd,
};

static const struct udevice_id dw_hdmi_detect_dt_ids[] = {
	{ .compatible = "fsl,imx6q-hdmi-hpd",},
	{ .compatible = "fsl,imx8mp-hdmi-hpd",},
	{},
};

U_BOOT_DRIVER(dw_hdmi_hpd) = {
	.name				= "dw-hdmi-hdp",
	.id				= UCLASS_DISPLAY_DETECT,
	.of_match			= dw_hdmi_detect_dt_ids,
	.bind				= dm_scan_fdt_dev,
	.probe				= dw_hdmi_detect_probe,
	.remove				= dw_hdmi_detect_remove,
	.ops				= &dw_hdmi_detect_ops,
	.priv_auto_alloc_size		= sizeof(struct dw_hdmi_detect),
};
