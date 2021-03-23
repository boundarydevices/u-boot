// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012 Sascha Hauer, Pengutronix
 * Copyright 2020 NXP
 */

#include <common.h>
#include <clk.h>
#include <dm/read.h>
#include <regmap.h>
#include <syscon.h>
#include <video_bridge.h>
#include <video_link.h>
#include <media_bus_format.h>
#include "fsl_imx_ldb.h"

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_CH1_MODE_EN_TO_DI0		(1 << 2)
#define LDB_CH1_MODE_EN_TO_DI1		(3 << 2)
#define LDB_CH1_MODE_EN_MASK		(3 << 2)
#define LDB_SPLIT_MODE_EN		(1 << 4)
#define LDB_DATA_WIDTH_CH0_24		(1 << 5)
#define LDB_BIT_MAP_CH0_JEIDA		(1 << 6)
#define LDB_DATA_WIDTH_CH1_24		(1 << 7)
#define LDB_BIT_MAP_CH1_JEIDA		(1 << 8)
#define LDB_DI0_VS_POL_ACT_LOW		(1 << 9)
#define LDB_DI1_VS_POL_ACT_LOW		(1 << 10)

struct ldb_bit_mapping {
	u32 bus_format;
	u32 datawidth;
	const char * const mapping;
};

static const struct ldb_bit_mapping ldb_bit_mappings[] = {
	{ MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,  18, "spwg" },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,  24, "spwg" },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA, 24, "jeida" },
};

static u32 ofnode_get_bus_format(ofnode np)
{
	const char *bm;
	u32 datawidth = 0;
	int i;

	bm = ofnode_read_string(np, "fsl,data-mapping");
	if (!bm)
		return -EINVAL;

	ofnode_read_u32(np, "fsl,data-width", &datawidth);

	for (i = 0; i < ARRAY_SIZE(ldb_bit_mappings); i++) {
		if (!strcasecmp(bm, ldb_bit_mappings[i].mapping) &&
		    datawidth == ldb_bit_mappings[i].datawidth)
			return ldb_bit_mappings[i].bus_format;
	}

	debug("invalid data mapping: %d-bit \"%s\"\n", datawidth, bm);

	return -ENOENT;
}

static void ldb_ch_set_bus_format(struct ldb_channel *ldb_ch, u32 bus_format)
{
	struct ldb *ldb = ldb_ch->ldb;

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		/* i.MX8MP needs Jeida bit set, imx6 needs it clear */
		if (device_is_compatible(ldb_ch->ldb->dev, "fsl,imx8mp-ldb")) {
			if (ldb_ch->chno == 0 || ldb->dual)
				ldb->ldb_ctrl |= LDB_BIT_MAP_CH0_JEIDA;
			if (ldb_ch->chno == 1 || ldb->dual)
				ldb->ldb_ctrl |= LDB_BIT_MAP_CH1_JEIDA;
		}
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH0_24;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH1_24;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH0_24 |
					 LDB_BIT_MAP_CH0_JEIDA;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH1_24 |
					 LDB_BIT_MAP_CH1_JEIDA;
		break;
	}
}

void ldb_bridge_mode_set(struct ldb_channel *ldb_ch, enum display_flags flags)
{
	struct ldb *ldb = ldb_ch->ldb;

	/* FIXME - assumes straight connections DI0 --> CH0, DI1 --> CH1 */
	if (ldb_ch == ldb->channel[0] || ldb->dual) {
		if (flags & DISPLAY_FLAGS_VSYNC_LOW)
			ldb->ldb_ctrl |= LDB_DI0_VS_POL_ACT_LOW;
		else if (flags & DISPLAY_FLAGS_VSYNC_HIGH)
			ldb->ldb_ctrl &= ~LDB_DI0_VS_POL_ACT_LOW;
	}
	if (ldb_ch == ldb->channel[1] || ldb->dual) {
		if (flags & DISPLAY_FLAGS_VSYNC_LOW)
			ldb->ldb_ctrl |= LDB_DI1_VS_POL_ACT_LOW;
		else if (flags & DISPLAY_FLAGS_VSYNC_HIGH)
			ldb->ldb_ctrl &= ~LDB_DI1_VS_POL_ACT_LOW;
	}

	ldb_ch_set_bus_format(ldb_ch, ldb_ch->bus_format);
}

void ldb_bridge_enable(struct ldb_channel *ldb_ch)
{
	struct ldb *ldb = ldb_ch->ldb;

	debug("%s: 0x%x = 0x%x\n", __func__, ldb->ctrl_reg, ldb->ldb_ctrl);
	regmap_write(ldb->regmap, ldb->ctrl_reg, ldb->ldb_ctrl);
}

void ldb_bridge_disable(struct ldb_channel *ldb_ch)
{
	struct ldb *ldb = ldb_ch->ldb;

	if (ldb_ch == ldb->channel[0] || ldb->dual)
		ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
	if (ldb_ch == ldb->channel[1] || ldb->dual)
		ldb->ldb_ctrl &= ~LDB_CH1_MODE_EN_MASK;

	debug("%s: 0x%x = 0x%x\n", __func__, ldb->ctrl_reg, ldb->ldb_ctrl);
	regmap_write(ldb->regmap, ldb->ctrl_reg, ldb->ldb_ctrl);
}

int ldb_bind(struct ldb *ldb)
{
	struct udevice *dev = ldb->dev;
	ofnode np = dev_ofnode(dev);
	ofnode child;
	int ret = 0;
	int i;

	ldb->regmap = syscon_regmap_lookup_by_phandle(dev, "gpr");
	if (IS_ERR(ldb->regmap)) {
		debug("failed to get parent regmap\n");
		return PTR_ERR(ldb->regmap);
	}

	/* disable LDB by resetting the control register to POR default */
	regmap_write(ldb->regmap, ldb->ctrl_reg, 0);

	ldb->dual = ofnode_read_bool(np, "fsl,dual-channel");
	if (ldb->dual)
		ldb->ldb_ctrl |= LDB_SPLIT_MODE_EN;

	ofnode_for_each_subnode(child, np) {
		struct ldb_channel *ldb_ch;
		int bus_format;
		u32 u = 0;

		ret = ofnode_read_u32(child, "reg", &u);
		i = u;
		if (ret || i < 0 || i > 1) {
			ret = -EINVAL;
			goto free_child;
		}

		if (!ofnode_is_available(child))
			continue;

		if (ldb->dual && i > 0) {
			debug("dual-channel mode, ignoring second output\n");
			continue;
		}

		ldb_ch = ldb->channel[i];
		ldb_ch->ldb = ldb;
		ldb_ch->chno = i;
		ldb_ch->is_valid = false;

		ldb_ch->panel = video_link_get_next_device(dev);
		if (!ldb_ch->panel ||
			device_get_uclass_id(ldb_ch->panel) != UCLASS_PANEL) {
			debug("get panel device error\n");
		}

		bus_format = ofnode_get_bus_format(child);
		if (bus_format == -EINVAL)
			bus_format = 0;
		ldb_ch->bus_format = bus_format;
		ldb_ch->child = child;
		ldb_ch->is_valid = true;
	}

	return 0;

free_child:
	return ret;
}
