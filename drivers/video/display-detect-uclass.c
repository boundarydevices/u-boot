// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2021
 * Boundary Devices
 */

#include <common.h>
#include <display_detect.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <dm/uclass-internal.h>

int display_detect(struct udevice *dev)
{
	struct dm_display_detect_ops *ops = display_detect_get_ops(dev);

	if (!ops || !ops->detect)
		return -ENOSYS;
	return ops->detect(dev);
}

bool display_detect_by_node_name(const char* path)
{
	ofnode node = ofnode_path(path);
	struct udevice *dev;
	int ret = -EINVAL;

	if (!ofnode_valid(node)) {
		printf("missing node %s\n", path);
		return false;
	}
	ret = device_find_by_ofnode(node, &dev);
	if (ret) {
		printf("%s: %s: device_find_by_ofnode=%d\n", __func__,
			ofnode_get_name(node), ret);
		return false;
	}
	ret = device_probe(dev);
	if (ret) {
		printf("fail to probe panel device %s\n", dev->name);
		return ret;
	}
	return display_detect(dev);
}

UCLASS_DRIVER(video_osd) = {
	.id		= UCLASS_DISPLAY_DETECT,
	.name		= "display_detect",
	.flags		= DM_UC_FLAG_SEQ_ALIAS,
};
