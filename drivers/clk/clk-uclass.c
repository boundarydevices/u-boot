/*
 * Copyright (C) 2015 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 * Copyright (c) 2016, NVIDIA CORPORATION.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <clk.h>
#include <clk-uclass.h>
#include <dm.h>
#include <dt-structs.h>
#include <errno.h>

DECLARE_GLOBAL_DATA_PTR;

static inline struct clk_ops *clk_dev_ops(struct udevice *dev)
{
	return (struct clk_ops *)dev->driver->ops;
}

#if CONFIG_IS_ENABLED(OF_CONTROL)
# if CONFIG_IS_ENABLED(OF_PLATDATA)
int clk_get_by_index_platdata(struct udevice *dev, int index,
			      struct phandle_2_cell *cells, struct clk *clk)
{
	int ret;

	if (index != 0)
		return -ENOSYS;
	ret = uclass_get_device(UCLASS_CLK, 0, &clk->dev);
	if (ret)
		return ret;
	clk->id = cells[0].id;

	return 0;
}
# else
static int clk_of_xlate_default(struct clk *clk,
				struct ofnode_phandle_args *args)
{
	debug("%s(clk=%p)\n", __func__, clk);

	if (args->args_count > 1) {
		debug("Invaild args_count: %d\n", args->args_count);
		return -EINVAL;
	}

	if (args->args_count)
		clk->id = args->args[0];
	else
		clk->id = 0;

	return 0;
}

int clk_get_by_index(struct udevice *dev, int index, struct clk *clk)
{
	int ret;
	struct ofnode_phandle_args args;
	struct udevice *dev_clk;
	struct clk_ops *ops;

	debug("%s(dev=%p, index=%d, clk=%p)\n", __func__, dev, index, clk);

	assert(clk);
	ret = dev_read_phandle_with_args(dev, "clocks", "#clock-cells", 0,
					  index, &args);
	if (ret) {
		debug("%s: fdtdec_parse_phandle_with_args failed: err=%d\n",
		      __func__, ret);
		return ret;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_CLK, args.node, &dev_clk);
	if (ret) {
		debug("%s: uclass_get_device_by_of_offset failed: err=%d\n",
		      __func__, ret);
		return ret;
	}

	clk->dev = dev_clk;

	ops = clk_dev_ops(dev_clk);

	if (ops->of_xlate)
		ret = ops->of_xlate(clk, &args);
	else
		ret = clk_of_xlate_default(clk, &args);
	if (ret) {
		debug("of_xlate() failed: %d\n", ret);
		return ret;
	}

	return clk_request(dev_clk, clk);
}
# endif /* OF_PLATDATA */

int clk_get_by_name(struct udevice *dev, const char *name, struct clk *clk)
{
	int index;

	debug("%s(dev=%p, name=%s, clk=%p)\n", __func__, dev, name, clk);

	index = dev_read_stringlist_search(dev, "clock-names", name);
	if (index < 0) {
		debug("fdt_stringlist_search() failed: %d\n", index);
		return index;
	}

	return clk_get_by_index(dev, index, clk);
}
#endif /* OF_CONTROL */

int clk_request(struct udevice *dev, struct clk *clk)
{
	struct clk_ops *ops = clk_dev_ops(dev);

	debug("%s(dev=%p, clk=%p)\n", __func__, dev, clk);

	clk->dev = dev;

	if (!ops->request)
		return 0;

	return ops->request(clk);
}

int clk_free(struct clk *clk)
{
	struct clk_ops *ops = clk_dev_ops(clk->dev);

	debug("%s(clk=%p)\n", __func__, clk);

	if (!ops->free)
		return 0;

	return ops->free(clk);
}

ulong clk_get_rate(struct clk *clk)
{
	struct clk_ops *ops = clk_dev_ops(clk->dev);

	debug("%s(clk=%p)\n", __func__, clk);

	if (!ops->get_rate)
		return -ENOSYS;

	return ops->get_rate(clk);
}

ulong clk_set_rate(struct clk *clk, ulong rate)
{
	struct clk_ops *ops = clk_dev_ops(clk->dev);

	debug("%s(clk=%p, rate=%lu)\n", __func__, clk, rate);

	if (!ops->set_rate)
		return -ENOSYS;

	return ops->set_rate(clk, rate);
}

int clk_enable(struct clk *clk)
{
	struct clk_ops *ops = clk_dev_ops(clk->dev);

	debug("%s(clk=%p)\n", __func__, clk);

	if (!ops->enable)
		return -ENOSYS;

	return ops->enable(clk);
}

int clk_disable(struct clk *clk)
{
	struct clk_ops *ops = clk_dev_ops(clk->dev);

	debug("%s(clk=%p)\n", __func__, clk);

	if (!ops->disable)
		return -ENOSYS;

	return ops->disable(clk);
}

UCLASS_DRIVER(clk) = {
	.id		= UCLASS_CLK,
	.name		= "clk",
};
