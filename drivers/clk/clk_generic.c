/*
 * Copyright 2021 Boundary Devices
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <linux/clk-provider.h>
/* ------------------------------------------------------------------- */

struct gen_clk_priv {
	struct clk	clk;
	struct clk	parent_clk;
	unsigned long	frequency;
};
#define to_clk_gen_clk(_clk) container_of(_clk, struct gen_clk_priv, clk)


static unsigned long gen_clk_recalc_rate(struct clk *clk)
{
	struct gen_clk_priv *clk_priv = dev_get_priv(clk->dev);

	debug("%s: %p(%s) %p: %lu\n", __func__, clk->dev, clk->dev->name, clk_priv, clk_priv->frequency);
	return clk_priv->frequency;
}

static unsigned long gen_clk_set_rate(struct clk *clk, unsigned long rate)
{
	struct gen_clk_priv *clk_priv = dev_get_priv(clk->dev);

	clk_priv->frequency = rate;
	debug("%s: %p %lu\n", __func__, clk_priv, rate);
	return 0;
}

static const char strclk[] = "dsim-clk";
static const char strclk_parent[] = "dsi_phy_ref";

static int gen_clk_probe(struct udevice *dev)
{
	struct gen_clk_priv *clk_priv = dev_get_priv(dev);
	const char* clk_name = strclk;
	const char* clk_parent_name = strclk_parent;
	const char* name;
	struct clk *c;
	int ret;

	/* optional override of the clockname */
	name = dev_read_string(dev, "clock-output-names");
	if (name)
		clk_name = name;

	ret = clk_get_by_name(dev, "parent", &clk_priv->parent_clk);
	if (!ret) {
		ret = clk_get_by_id(clk_priv->parent_clk.id, &c);
		if (!ret)
			clk_parent_name = c->dev->name;
	}
	dev->uclass_priv = &clk_priv->clk;
	clk_priv->clk.dev = dev;
	if (!ret) {
		debug("%s: %s %s\n", __func__, clk_name, clk_parent_name);
		list_del(&dev->sibling_node);
		list_add_tail(&dev->sibling_node, &c->dev->child_head);
		dev->parent = c->dev;
	}
	return 0;
}

static struct clk_ops gen_clk_ops = {
	.set_rate = gen_clk_set_rate,
	.get_rate = gen_clk_recalc_rate,
};

static const struct udevice_id gen_clk_ids[] = {
	{ .compatible = "generic,clk" },
	{ }
};

U_BOOT_DRIVER(clk_generic) = {
	.name = "generic-clk",
	.id = UCLASS_CLK,
	.of_match = gen_clk_ids,
	.ops = &gen_clk_ops,
	.probe = gen_clk_probe,
	.priv_auto_alloc_size = sizeof(struct gen_clk_priv),
};
