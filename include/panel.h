/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2016 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 */

#ifndef _PANEL_H
#define _PANEL_H

struct panel_ops {
	int (*init)(struct udevice *dev);
	void (*uninit)(struct udevice *dev);
	int (*enable)(struct udevice *dev);
	/**
	 * enable_backlight() - Enable the panel backlight
	 *
	 * @dev:	Panel device containing the backlight to enable
	 * @return 0 if OK, -ve on error
	 */
	int (*enable_backlight)(struct udevice *dev);

	/**
	 * set_backlight - Set panel backlight brightness
	 *
	 * @dev:	Panel device containing the backlight to update
	 * @percent:	Brightness value (0 to 100, or BACKLIGHT_... value)
	 * @return 0 if OK, -ve on error
	 */
	int (*set_backlight)(struct udevice *dev, int percent);

	/**
	 * get_timings() - Get display timings from panel.
	 *
	 * @dev:	Panel device containing the display timings
	 * @tim:	Place to put timings
	 * @return 0 if OK, -ve on error
	 */
	int (*get_display_timing)(struct udevice *dev,
				  struct display_timing *timing);
};

#define panel_get_ops(dev)	((struct panel_ops *)(dev)->driver->ops)

int panel_init(struct udevice *dev);
void panel_uninit(struct udevice *dev);
int panel_enable(struct udevice *dev);

/**
 * panel_enable_backlight() - Enable/disable the panel backlight
 *
 * @dev:	Panel device containing the backlight to enable
 * @enable:	true to enable the backlight, false to dis
 * Return: 0 if OK, -ve on error
 */
int panel_enable_backlight(struct udevice *dev);

/**
 * panel_set_backlight - Set brightness for the panel backlight
 *
 * @dev:	Panel device containing the backlight to update
 * @percent:	Brightness value (0 to 100, or BACKLIGHT_... value)
 * Return: 0 if OK, -ve on error
 */
int panel_set_backlight(struct udevice *dev, int percent);

/**
 * panel_get_display_timing() - Get display timings from panel.
 *
 * @dev:	Panel device containing the display timings
 * Return: 0 if OK, -ve on error
 */
int panel_get_display_timing(struct udevice *dev,
			     struct display_timing *timing);

#endif
