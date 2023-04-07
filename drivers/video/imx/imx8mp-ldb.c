// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */

#include <common.h>
#include <backlight.h>
#include <clk.h>
#include <dm/device.h>
#include <dm/read.h>
#include <dm/device-internal.h>
#include <generic-phy.h>
#include <panel.h>
#include <syscon.h>
#include <video_bridge.h>
#include <video_link.h>

#include "fsl_imx_ldb.h"

#define DRIVER_NAME "imx8mp-ldb"

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_CH1_MODE_EN_TO_DI0		(1 << 2)
#define LDB_CH1_MODE_EN_TO_DI1		(3 << 2)
#define LDB_CH1_MODE_EN_MASK		(3 << 2)
#define LDB_REG_CH0_FIFO_RESET		(1 << 11)
#define LDB_REG_CH1_FIFO_RESET		(1 << 12)
#define LDB_REG_ASYNC_FIFO_EN		(1 << 24)
#define LDB_FIFO_THRESHOLD		(4 << 25)

#define LDB_CH_NUM      2

struct imx8mp_ldb;

struct imx8mp_ldb_channel {
	struct ldb_channel base;
	struct imx8mp_ldb *imx8mp_ldb;

	struct udevice *backlight;
	struct phy phy;
	bool phy_is_on;

	struct i2c_adapter *ddc;
	void *edid;
	int edid_len;
	int mode_valid;
	u32 bus_flags;
};

struct imx8mp_ldb {
	struct ldb base;
	enum display_flags flags;
	struct imx8mp_ldb_channel channel[LDB_CH_NUM];
	struct clk *clk_root;
};

static int en_masks[] = {LDB_CH0_MODE_EN_MASK, LDB_CH1_MODE_EN_MASK};
static int di_masks[] = {LDB_CH0_MODE_EN_TO_DI0, LDB_CH1_MODE_EN_TO_DI0};

static void imx8mp_ldb_enable(struct imx8mp_ldb *imx8mp_ldb)
{
	struct ldb *ldb = &imx8mp_ldb->base;
	int i;

	clk_prepare_enable(imx8mp_ldb->clk_root);

	for (i = 0; i < LDB_CH_NUM; i++) {
		struct imx8mp_ldb_channel *imx8mp_ldb_ch =
				&imx8mp_ldb->channel[i];

		if (imx8mp_ldb_ch->base.is_valid || ldb->dual) {
			ldb->ldb_ctrl &= ~en_masks[i];
			ldb->ldb_ctrl |= di_masks[i];
			generic_phy_power_on(&imx8mp_ldb_ch->phy);
			imx8mp_ldb_ch->phy_is_on = true;
		}
	}
}

static void imx8mp_ldb_set_clock(struct imx8mp_ldb *imx8mp_ldb, u32 frequency)
{
	struct ldb *ldb = &imx8mp_ldb->base;
	unsigned long serial_clk;

	if (frequency > 160000000) {
		debug("%s: mode exceeds 160 MHz pixel clock\n", __func__);
	}
	if (frequency > 80000000 && !ldb->dual) {
		debug("%s: mode exceeds 80 MHz pixel clock\n", __func__);
	}

	serial_clk = frequency * 7;
	if (ldb->dual)
		serial_clk >>= 1;
	clk_set_rate(imx8mp_ldb->clk_root, serial_clk);
}

static void imx8mp_ldb_disable(struct imx8mp_ldb *imx8mp_ldb)
{
	struct ldb *ldb = &imx8mp_ldb->base;
	int i;

	for (i = 0; i < LDB_CH_NUM; i++) {
		struct imx8mp_ldb_channel *imx8mp_ldb_ch =
				&imx8mp_ldb->channel[i];

		if (imx8mp_ldb_ch->phy_is_on) {
			imx8mp_ldb_ch->phy_is_on = false;
			ldb->ldb_ctrl &= ~en_masks[i];
			generic_phy_power_off(&imx8mp_ldb_ch->phy);
		}
	}
	clk_disable_unprepare(imx8mp_ldb->clk_root);
}

static int imx8mp_ldb_ch_attach(struct imx8mp_ldb *imx8mp_ldb, int i)
{
	struct ldb *ldb = &imx8mp_ldb->base;
	struct ldb_channel *ldb_ch = ldb->channel[i];
	int ret = 0;

	if (ldb_ch->is_valid) {
		ldb_bridge_mode_set(ldb_ch, imx8mp_ldb->flags);
		if (ldb_ch->panel)
			ret = panel_init(ldb_ch->panel);
	}
	return ret;
}

static int imx8mp_ldb_ch_enable(struct imx8mp_ldb *imx8mp_ldb, int i)
{
	struct ldb *ldb = &imx8mp_ldb->base;
	struct ldb_channel *ldb_ch = ldb->channel[i];
	int ret = 0;

	if (ldb_ch->is_valid) {
		ldb_bridge_enable(ldb_ch);
		if (ldb_ch->panel) {
			ret = panel_enable_backlight(ldb_ch->panel);
			if (ret) {
				debug("panel %s enable backlight error %d\n",
					ldb_ch->panel->name, ret);
			}
		}
		if (imx8mp_ldb->channel[i].backlight) {
			debug("%s: backlight_enable\n", __func__);
			backlight_enable(imx8mp_ldb->channel[i].backlight);
		}
	}
	return ret;
}

static void imx8mp_ldb_ch_disable(struct imx8mp_ldb *imx8mp_ldb, int i)
{
	struct ldb *ldb = &imx8mp_ldb->base;
	struct ldb_channel *ldb_ch = ldb->channel[i];

	if (ldb_ch->is_valid) {
		ldb_bridge_disable(ldb_ch);
		if (imx8mp_ldb->channel[i].backlight) {
			debug("%s: backlight_set_brightness off\n", __func__);
			backlight_set_brightness(imx8mp_ldb->channel[i].backlight, 0);
		}
		if (ldb_ch->panel)
			device_remove(ldb_ch->panel, DM_REMOVE_NORMAL);
	}
}

static int imx8mp_ldb_attach(struct udevice *dev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_priv(dev);
	struct display_timing timings;
	int ret;

	debug("%s\n", __func__);
	ret = video_link_get_display_timings(&timings);
	if (ret) {
		debug("decode display timing error %d\n", ret);
		return ret;
	}
#if 0
	/*
	 * Due to limited video PLL frequency points on i.MX8mp,
	 * we do mode fixup here in case any mode is unsupported.
	 */
	if (imx8mp_ldb->base.dual)
		timings.pixelclock.typ = timings.pixelclock.typ > 100000000 ? 148500000 : 74250000;
	else
		timings.pixelclock.typ = 74250000;
#endif

	imx8mp_ldb->flags = timings.flags;
	imx8mp_ldb_set_clock(imx8mp_ldb, timings.pixelclock.typ);

	ret = imx8mp_ldb_ch_attach(imx8mp_ldb, 0);
	if (!ret)
		ret = imx8mp_ldb_ch_attach(imx8mp_ldb, 1);
	if (ret)
		debug("%s: ret=%d\n", __func__, ret);
	return ret;
}

static int imx8mp_ldb_set_backlight(struct udevice *dev, int percent)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_priv(dev);
	int ret;

	debug("%s\n", __func__);
	imx8mp_ldb_enable(imx8mp_ldb);
	ret = imx8mp_ldb_ch_enable(imx8mp_ldb, 0);
	if (!ret)
		ret = imx8mp_ldb_ch_enable(imx8mp_ldb, 1);
	if (ret)
		debug("%s: ret=%d\n", __func__, ret);
	return ret;
}

static int imx8mp_ldb_bind(struct imx8mp_ldb *imx8mp_ldb, struct udevice *dev)
{
	ofnode np = dev_ofnode(dev);
	ofnode child;
	struct ldb *ldb;
	int ret;
	int i;

	ldb = &imx8mp_ldb->base;
	ldb->dev = dev;
	ldb->ctrl_reg = 0x5c,
	ldb->output_port = 1;

	ldb->regmap = syscon_regmap_lookup_by_phandle(dev, "gpr");
	if (IS_ERR(ldb->regmap)) {
		debug("failed to get parent regmap\n");
		return PTR_ERR(ldb->regmap);
	}

	for (i = 0; i < LDB_CH_NUM; i++) {
		imx8mp_ldb->channel[i].imx8mp_ldb = imx8mp_ldb;
		ldb->channel[i] = &imx8mp_ldb->channel[i].base;
	}

	imx8mp_ldb->clk_root = devm_clk_get(dev, "ldb");
	if (IS_ERR(imx8mp_ldb->clk_root))
		return PTR_ERR(imx8mp_ldb->clk_root);

	ret = ldb_bind(ldb);
	if (ret)
		goto free_child;

	ofnode_for_each_subnode(child, np) {
		struct imx8mp_ldb_channel *imx8mp_ldb_ch;
		bool auxiliary_ch = false;
		u32 u = 0;

		ret = ofnode_read_u32(child, "reg", &u);
		i = u;
		if (ret || i < 0 || i > 1) {
			ret = -EINVAL;
			goto free_child;
		}

		if (ldb->dual && i > 0) {
			auxiliary_ch = true;
			imx8mp_ldb_ch = &imx8mp_ldb->channel[i];
			goto get_phy;
		}

		if (!ofnode_is_available(child))
			continue;

		imx8mp_ldb_ch = &imx8mp_ldb->channel[i];
get_phy:
		ret = ofnode_generic_phy_get_by_name(child, "ldb_phy", &imx8mp_ldb_ch->phy);
		if (ret) {
			debug("can't get channel%d phy: %d\n", i, ret);
			goto free_child;
		}

		ret = generic_phy_init(&imx8mp_ldb_ch->phy);
		if (ret < 0) {
			debug("failed to initialize channel%d phy: %d\n", i, ret);
			goto free_child;
		}

		ret = uclass_device_by_phandle(UCLASS_PANEL_BACKLIGHT, child,
						   "backlight", &imx8mp_ldb_ch->backlight);
		if (ret) {
			debug("%s: Cannot get backlight: ret=%d\n", __func__, ret);
			if (ret != -ENOENT)
				return ret;
		}
	}

	return 0;

free_child:
	return ret;
}

static int imx8mp_ldb_probe(struct udevice *dev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_priv(dev);

	return imx8mp_ldb_bind(imx8mp_ldb, dev);
}

static int imx8mp_ldb_remove(struct udevice *dev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_priv(dev);

	imx8mp_ldb_disable(imx8mp_ldb);
	imx8mp_ldb_ch_disable(imx8mp_ldb, 0);
	imx8mp_ldb_ch_disable(imx8mp_ldb, 1);

	return 0;
}

struct video_bridge_ops imx8mp_ldb_ops = {
	.attach = imx8mp_ldb_attach,
	.set_backlight = imx8mp_ldb_set_backlight,
};

static const struct udevice_id imx8mp_ldb_dt_ids[] = {
	{ .compatible = "fsl,imx8mp-ldb", },
	{ }
};

U_BOOT_DRIVER(imx8mp_ldb) = {
	.name				= "imx8mp_ldb",
	.id				= UCLASS_VIDEO_BRIDGE,
	.of_match			= imx8mp_ldb_dt_ids,
	.bind				= dm_scan_fdt_dev,
	.remove 			= imx8mp_ldb_remove,
	.probe				= imx8mp_ldb_probe,
	.ops				= &imx8mp_ldb_ops,
	.priv_auto			= sizeof(struct imx8mp_ldb),
};
