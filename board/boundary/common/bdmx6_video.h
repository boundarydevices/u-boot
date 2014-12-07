/*
 * (C) Copyright 2014 Boundary Devices
 *
 * Eric Nelson <eric.nelson@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __BDMX6_VIDEO_H_
#define __BDMX6_VIDEO_H_

#include <asm/imx-common/video.h>

#define BDMX6_HDMI_SVGA { \
		.name           = "HDMI", \
		.refresh        = 60, \
		.xres           = 1024, \
		.yres           = 768, \
		.pixclock       = 15385, \
		.left_margin    = 220, \
		.right_margin   = 40, \
		.upper_margin   = 21, \
		.lower_margin   = 7, \
		.hsync_len      = 60, \
		.vsync_len      = 10, \
		.sync           = FB_SYNC_EXT, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_LVDS_WXGA { \
		.name           = "LDB-WXGA", \
		.refresh        = 60, \
		.xres           = 1280, \
		.yres           = 800, \
		.pixclock       = 14065, \
		.left_margin    = 40, \
		.right_margin   = 40, \
		.upper_margin   = 3, \
		.lower_margin   = 80, \
		.hsync_len      = 10, \
		.vsync_len      = 10, \
		.sync           = FB_SYNC_EXT, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_LVDS_WXGA_S { \
		.name           = "LDB-WXGA-S", \
		.refresh        = 60, \
		.xres           = 1280, \
		.yres           = 800, \
		.pixclock       = 14065, \
		.left_margin    = 40, \
		.right_margin   = 40, \
		.upper_margin   = 3, \
		.lower_margin   = 80, \
		.hsync_len      = 10, \
		.vsync_len      = 10, \
		.sync           = FB_SYNC_EXT, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_LVDS_HANNSTAR { \
		.name           = "Hannstar-XGA", \
		.refresh        = 60, \
		.xres           = 1024, \
		.yres           = 768, \
		.pixclock       = 15385, \
		.left_margin    = 220, \
		.right_margin   = 40, \
		.upper_margin   = 21, \
		.lower_margin   = 7, \
		.hsync_len      = 60, \
		.vsync_len      = 10, \
		.sync           = FB_SYNC_EXT, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_LVDS_LG_9_7 { \
		.name           = "LG-9.7", \
		.refresh        = 60, \
		.xres           = 1024, \
		.yres           = 768, \
		.pixclock       = 15385, /* ~65MHz */ \
		.left_margin    = 480, \
		.right_margin   = 260, \
		.upper_margin   = 16, \
		.lower_margin   = 6, \
		.hsync_len      = 250, \
		.vsync_len      = 10, \
		.sync           = FB_SYNC_EXT, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_LVDS_WSVGA { \
		.name           = "wsvga-lvds", \
		.refresh        = 60, \
		.xres           = 1024, \
		.yres           = 600, \
		.pixclock       = 15385, \
		.left_margin    = 220, \
		.right_margin   = 40, \
		.upper_margin   = 21, \
		.lower_margin   = 7, \
		.hsync_len      = 60, \
		.vsync_len      = 10, \
		.sync           = FB_SYNC_EXT, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_RGB_FUSION7 { \
		.name           = "fusion7", \
		.refresh        = 60, \
		.xres           = 800, \
		.yres           = 480, \
		.pixclock       = 33898, \
		.left_margin    = 96, \
		.right_margin   = 24, \
		.upper_margin   = 3, \
		.lower_margin   = 10, \
		.hsync_len      = 72, \
		.vsync_len      = 7, \
		.sync           = 0x40000002, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_RGB_SVGA { \
		.name           = "svga", \
		.refresh        = 60, \
		.xres           = 800, \
		.yres           = 600, \
		.pixclock       = 15385, \
		.left_margin    = 220, \
		.right_margin   = 40, \
		.upper_margin   = 21, \
		.lower_margin   = 7, \
		.hsync_len      = 60, \
		.vsync_len      = 10, \
		.sync           = 0, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_LVDS_AMP_1024X600 { \
		.name           = "amp1024x600", \
		.refresh        = 60, \
		.xres           = 1024, \
		.yres           = 600, \
		.pixclock       = 15385, \
		.left_margin    = 220, \
		.right_margin   = 40, \
		.upper_margin   = 21, \
		.lower_margin   = 7, \
		.hsync_len      = 60, \
		.vsync_len      = 10, \
		.sync           = FB_SYNC_EXT, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_LVDS_WVGA_SPWG { \
		.name           = "LDB-WVGA-SPWG", \
		.refresh        = 57, \
		.xres           = 800, \
		.yres           = 480, \
		.pixclock       = 15385, \
		.left_margin    = 220, \
		.right_margin   = 40, \
		.upper_margin   = 21, \
		.lower_margin   = 7, \
		.hsync_len      = 60, \
		.vsync_len      = 10, \
		.sync           = FB_SYNC_EXT, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_LVDS_WVGA_JEIDA { \
		.name           = "LDB-WVGA-JEIDA", \
		.refresh        = 57, \
		.xres           = 800, \
		.yres           = 480, \
		.pixclock       = 15385, \
		.left_margin    = 220, \
		.right_margin   = 40, \
		.upper_margin   = 21, \
		.lower_margin   = 7, \
		.hsync_len      = 60, \
		.vsync_len      = 10, \
		.sync           = FB_SYNC_EXT, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_RGB_WVGA { \
		.name           = "wvga-rgb", \
		.refresh        = 57, \
		.xres           = 800, \
		.yres           = 480, \
		.pixclock       = 37037, \
		.left_margin    = 40, \
		.right_margin   = 60, \
		.upper_margin   = 10, \
		.lower_margin   = 10, \
		.hsync_len      = 20, \
		.vsync_len      = 10, \
		.sync           = 0, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_RGB_HITACHI_HVGA { \
		.name		= "hitachi_hvga", \
		 /* \
		  * hitachi 640x240 \
		  * vsync = 60 \
		  * hsync = 260 * vsync = 15.6 Khz \
		  * pixclk = 800 * hsync = 12.48 MHz \
		  */ \
		.refresh	= 60, \
		.xres		= 640, \
		.yres		= 240, \
		.pixclock	= 1000000000 / 800 * 1000 / 260 / 60, \
		.left_margin	= 34, \
		.right_margin	= 1, \
		.upper_margin	= 8, \
		.lower_margin	= 3, \
		.hsync_len	= 125, \
		.vsync_len	= 9, \
		.sync           = FB_SYNC_CLK_LAT_FALL, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_RGB_WQVGA { \
		.name           = "wqvga-rgb", \
		.refresh        = 57, \
		.xres           = 480, \
		.yres           = 272, \
		.pixclock       = 97786, \
		.left_margin    = 2, \
		.right_margin   = 1, \
		.upper_margin   = 3, \
		.lower_margin   = 2, \
		.hsync_len      = 41, \
		.vsync_len      = 10, \
		.sync           = 0, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#define BDMX6_RGB_QVGA { \
		.name           = "qvga", \
		.refresh        = 60, \
		.xres           = 320, \
		.yres           = 240, \
		.pixclock       = 37037, \
		.left_margin    = 38, \
		.right_margin   = 37, \
		.upper_margin   = 16, \
		.lower_margin   = 15, \
		.hsync_len      = 30, \
		.vsync_len      = 3, \
		.sync           = 0, \
		.vmode          = FB_VMODE_NONINTERLACED \
}

#ifdef CONFIG_IMX_HDMI
void do_enable_hdmi(struct display_info_t const *dev);
#endif

#ifdef CONFIG_BDMX6_LVDS0
void enable_lvds_spwg(struct display_info_t const *dev);
void enable_lvds_jeida(struct display_info_t const *dev);
#endif

#ifdef CONFIG_BDMX6_RGB
void enable_rgb(struct display_info_t const *dev);
#endif

#endif
