// SPDX-License-Identifier: GPL-2.0
/*
 * MMIO register bitfield-controlled multiplexer driver
 *
 * Copyright (C) 2017 Pengutronix, Philipp Zabel <kernel@pengutronix.de>
 */

#define DEBUG
#include <common.h>
#include <dm/device.h>
#include <dm/ofnode.h>
#include <dm/read.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/mux/driver.h>
#include <regmap.h>
#include <syscon.h>

#define MAX_FIELDS	16
struct mmio_mux_priv {
	struct regmap_field *fields[MAX_FIELDS];
};

static int mux_mmio_set(struct mux_control *mux, int state)
{
	struct mmio_mux_priv *priv = dev_get_priv(mux->chip->dev);

	return regmap_field_write(priv->fields[mux_control_get_index(mux)], state);
}

static int mux_mmio_probe(struct udevice *dev)
{
	struct mux_chip *mux_chip;
	struct mmio_mux_priv *priv = dev_get_priv(dev);
	ofnode np = dev_ofnode(dev);
	struct regmap *regmap = NULL;
	int num_fields;
	int length;
	int ret;
	int i;

	if (ofnode_device_is_compatible(np, "mmio-mux"))
		regmap = syscon_node_to_regmap(dev_get_parent(dev)->node);

	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		debug("failed to get regmap: %d\n", ret);
		return ret;
	}

	length = ofnode_read_size(np, "mux-reg-masks") / sizeof(u32);
	if (!length || (length & 1))
		return -EINVAL;
	num_fields = length / 2;

	mux_chip = devm_mux_chip_alloc(dev, num_fields);
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	for (i = 0; i < num_fields; i++) {
		struct mux_control *mux = &mux_chip->mux[i];
		struct reg_field field;
		s32 idle_state = MUX_IDLE_AS_IS;
		u32 reg, mask;
		int bits;

		ret = ofnode_read_u32_index(np, "mux-reg-masks",
						2 * i, &reg);
		if (!ret)
			ret = ofnode_read_u32_index(np, "mux-reg-masks",
							 2 * i + 1, &mask);
		if (ret < 0) {
			debug("bitfield %d: failed to read mux-reg-masks property: %d\n",
				i, ret);
			return ret;
		}

		field.reg = reg;
		field.msb = fls(mask) - 1;
		field.lsb = ffs(mask) - 1;

		if (mask != GENMASK(field.msb, field.lsb)) {
			debug("bitfield %d: invalid mask 0x%x\n",
				i, mask);
			return -EINVAL;
		}

		priv->fields[i] = devm_regmap_field_alloc(dev, regmap, field);
		if (IS_ERR(priv->fields[i])) {
			ret = PTR_ERR(priv->fields[i]);
			debug("bitfield %d: failed allocate: %d\n", i, ret);
			return ret;
		}

		bits = 1 + field.msb - field.lsb;
		mux->states = 1 << bits;

		ofnode_read_u32_index(np, "idle-states", i,
					   (u32 *)&idle_state);
		if (idle_state != MUX_IDLE_AS_IS) {
			if (idle_state < 0 || idle_state >= mux->states) {
				debug("bitfield: %d: out of range idle state %d\n",
					i, idle_state);
				return -EINVAL;
			}

			mux->idle_state = idle_state;
		}
	}
	return 0;
}

static const struct mux_control_ops mux_mmio_ops = {
	.set = mux_mmio_set,
};

static const struct udevice_id mux_mmio_dt_ids[] = {
	{ .compatible = "mmio-mux", },
	{ .compatible = "reg-mux", },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(imx_sec_dsim) = {
	.name				= "mmio-mux",
	.id				= UCLASS_MUX,
	.of_match			= mux_mmio_dt_ids,
	.bind				= dm_scan_fdt_dev,
	.probe				= mux_mmio_probe,
	.ops				= &mux_mmio_ops,
	.priv_auto_alloc_size		= sizeof(struct mmio_mux_priv),
};
