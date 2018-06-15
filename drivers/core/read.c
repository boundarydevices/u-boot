/*
 * Copyright (c) 2017 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <dm/of_access.h>

int dev_read_u32_default(struct udevice *dev, const char *propname, int def)
{
	return ofnode_read_u32_default(dev_ofnode(dev), propname, def);
}

const char *dev_read_string(struct udevice *dev, const char *propname)
{
	return ofnode_read_string(dev_ofnode(dev), propname);
}

bool dev_read_bool(struct udevice *dev, const char *propname)
{
	return ofnode_read_bool(dev_ofnode(dev), propname);
}

ofnode dev_read_subnode(struct udevice *dev, const char *subnode_name)
{
	return ofnode_find_subnode(dev_ofnode(dev), subnode_name);
}

ofnode dev_read_first_subnode(struct udevice *dev)
{
	return ofnode_first_subnode(dev_ofnode(dev));
}

ofnode dev_read_next_subnode(ofnode node)
{
	return ofnode_next_subnode(node);
}

int dev_read_size(struct udevice *dev, const char *propname)
{
	return ofnode_read_size(dev_ofnode(dev), propname);
}

fdt_addr_t dev_read_addr_index(struct udevice *dev, int index)
{
	if (ofnode_is_np(dev_ofnode(dev)))
		return ofnode_get_addr_index(dev_ofnode(dev), index);
	else
		return devfdt_get_addr_index(dev, index);
}

fdt_addr_t dev_read_addr(struct udevice *dev)
{
	return dev_read_addr_index(dev, 0);
}

fdt_addr_t dev_read_addr_size(struct udevice *dev, const char *property,
				fdt_size_t *sizep)
{
	return ofnode_get_addr_size(dev_ofnode(dev), property, sizep);
}

const char *dev_read_name(struct udevice *dev)
{
	return ofnode_get_name(dev_ofnode(dev));
}

int dev_read_stringlist_search(struct udevice *dev, const char *property,
			  const char *string)
{
	return ofnode_stringlist_search(dev_ofnode(dev), property, string);
}

int dev_read_phandle_with_args(struct udevice *dev, const char *list_name,
				const char *cells_name, int cell_count,
				int index,
				struct ofnode_phandle_args *out_args)
{
	return ofnode_parse_phandle_with_args(dev_ofnode(dev), list_name,
					      cells_name, cell_count, index,
					      out_args);
}

int dev_read_addr_cells(struct udevice *dev)
{
	return ofnode_read_addr_cells(dev_ofnode(dev));
}

int dev_read_size_cells(struct udevice *dev)
{
	return ofnode_read_size_cells(dev_ofnode(dev));
}

int dev_read_phandle(struct udevice *dev)
{
	ofnode node = dev_ofnode(dev);

	if (ofnode_is_np(node))
		return ofnode_to_np(node)->phandle;
	else
		return fdt_get_phandle(gd->fdt_blob, ofnode_to_offset(node));
}

const u32 *dev_read_prop(struct udevice *dev, const char *propname, int *lenp)
{
	return ofnode_read_prop(dev_ofnode(dev), propname, lenp);
}

int dev_read_alias_seq(struct udevice *dev, int *devnump)
{
	ofnode node = dev_ofnode(dev);
	const char *uc_name = dev->uclass->uc_drv->name;
	int ret;

	if (ofnode_is_np(node)) {
		ret = of_alias_get_id(ofnode_to_np(node), uc_name);
		if (ret >= 0)
			*devnump = ret;
	} else {
		ret = fdtdec_get_alias_seq(gd->fdt_blob, uc_name,
					   ofnode_to_offset(node), devnump);
	}

	return ret;
}

int dev_read_u32_array(struct udevice *dev, const char *propname,
		       u32 *out_values, size_t sz)
{
	return ofnode_read_u32_array(dev_ofnode(dev), propname, out_values, sz);
}

const uint8_t *dev_read_u8_array_ptr(struct udevice *dev, const char *propname,
				     size_t sz)
{
	return ofnode_read_u8_array_ptr(dev_ofnode(dev), propname, sz);
}
