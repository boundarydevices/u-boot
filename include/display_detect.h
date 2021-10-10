/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2014 Google Inc.
 */

#ifndef _DISPLAY_DETECT_H
#define _DISPLAY_DETECT_H

struct udevice;

int display_detect(struct udevice *dev);
bool display_detect_by_node_name(const char* path);

struct dm_display_detect_ops {
	bool (*detect)(struct udevice *dev);
};

#define display_detect_get_ops(dev)	((struct dm_display_detect_ops *)(dev)->driver->ops)

#endif
