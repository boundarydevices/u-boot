// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 NXP
 *
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
#include <reset.h>

struct samsung_hdmi_phy {
	struct udevice *dev;
	struct clk *apbclk;
	struct clk *phyclk;
};

static int samsung_hdmi_phy_init(struct phy *phy)
{
	return 0;
}

static int samsung_hdmi_phy_power_on(struct phy *phy)
{
	return 0;
}

static int samsung_hdmi_phy_power_off(struct phy *phy)
{
	return 0;
}

static int samsung_hdmi_phy_xlate(struct phy *phy,
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

static int samsung_hdmi_phy_probe(struct udevice *dev)
{
	struct samsung_hdmi_phy *samsung = dev_get_priv(dev);
	int ret;

	debug("%s\n", __func__);

	samsung->dev = dev;

	samsung->apbclk = devm_clk_get(samsung->dev, "apb");
	if (IS_ERR(samsung->apbclk)) {
		ret = PTR_ERR(samsung->apbclk);
		debug("failed to get phy apb clk: %d\n", ret);
		return ret;
	}

	samsung->phyclk = devm_clk_get(samsung->dev, "hdmi_phy");
	if (IS_ERR(samsung->phyclk)) {
		ret = PTR_ERR(samsung->phyclk);
		debug("failed to register clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(samsung->apbclk);
	if (ret) {
		debug("failed to enable apbclk\n");
		return ret;
	}

	ret = device_reset(dev);
	if (ret) {
		dev_warn(dev, "failed to reset hdmi phy %d\n", ret);
		goto phy_failed;
	}
	return 0;

phy_failed:
	clk_disable_unprepare(samsung->apbclk);
	return ret;
}

static const struct udevice_id samsung_hdmi_phy_of_match[] = {
	{
		.compatible = "fsl,samsung-hdmi-phy",
	}, { /* sentinel */ }
};

static const struct phy_ops samsung_hdmi_phy_ops = {
	.of_xlate = samsung_hdmi_phy_xlate,
	.init = samsung_hdmi_phy_init,
	.power_on = samsung_hdmi_phy_power_on,
	.power_off = samsung_hdmi_phy_power_off,
};

U_BOOT_DRIVER(samsung_hdmi_phy) = {
	.name	= "samsung-hdmi-phy",
	.id	= UCLASS_PHY,
	.of_match = samsung_hdmi_phy_of_match,
	.ops = &samsung_hdmi_phy_ops,
	.probe = samsung_hdmi_phy_probe,
	.priv_auto_alloc_size = sizeof(struct samsung_hdmi_phy),
};
