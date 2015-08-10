/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __IMX_VIDEO_H_
#define __IMX_VIDEO_H_

#include <linux/fb.h>
#include <ipu_pixfmt.h>

struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);

#define FB_HDMI		0
#define FB_LCD		1
#define FB_LVDS		2
#define FB_LVDS2	3
#define FB_COUNT	4
	int	fbtype;

#define FBF_MODESTR	1
#define FBF_JEIDA	2
#define FBF_SPLITMODE	4
	int	fbflags;
	struct	fb_videomode mode;
};

#ifdef CONFIG_IMX_HDMI
extern int detect_hdmi(struct display_info_t const *dev);
#endif

#ifdef CONFIG_IMX_VIDEO_SKIP
extern struct display_info_t const displays[];
extern size_t display_count;
#endif

static void __board_enable_hdmi(struct display_info_t const *di)
{
}

static void __board_enable_lcd(struct display_info_t const *di)
{
}

static void __board_enable_lvds(struct display_info_t const *di)
{
}

static void __board_enable_lvds2(struct display_info_t const *di)
{
}

void board_enable_hdmi(struct display_info_t const *di)
	__attribute__((weak, alias("__board_enable_hdmi")));
void board_enable_lcd(struct display_info_t const *di)
	__attribute__((weak, alias("__board_enable_lcd")));
void board_enable_lvds(struct display_info_t const *di)
	__attribute__((weak, alias("__board_enable_lvds")));
void board_enable_lvds2(struct display_info_t const *di)
	__attribute__((weak, alias("__board_enable_lvds2")));
#endif
