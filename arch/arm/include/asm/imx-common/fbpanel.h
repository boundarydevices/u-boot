/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __IMX_VIDEO_H_
#define __IMX_VIDEO_H_

#include <linux/fb.h>
#include <ipu_pixfmt.h>

struct display_info_t {
	int	bus;	/* (bus >> 8) is gpio to enable bus if <>0 */
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev, int enable);
	void	(*pre_enable)(struct display_info_t const *dev);
	int	pwm_period;
#define FB_HDMI		0
#define FB_LCD		1
#define FB_LCD2		2
#define FB_LVDS		3
#define FB_LVDS2	4
#define FB_COUNT	5
	int	fbtype;

#define FBF_MODESTR	1
#define FBF_JEIDA	2
#define FBF_SPLITMODE	4
#define FBF_SPI		8
#define FBF_BKLIT_LOW_ACTIVE	0x10
#define FBF_BKLIT_DTB		0x20
	int	fbflags;
	struct	fb_videomode mode;
};
int ipu_set_ldb_clock(int rate);

void board_enable_hdmi(const struct display_info_t *di, int enable);
void board_enable_lcd(const struct display_info_t *di, int enable);
void board_enable_lvds(const struct display_info_t *di, int enable);
void board_enable_lvds2(const struct display_info_t *di, int enable);

void fbp_enable_fb(struct display_info_t const *di, int enable);
int fbp_detect_i2c(struct display_info_t const *di);
void fbp_setup_display(const struct display_info_t *displays, int cnt);

#define VD_1280_720M_60(_mode, _detect, _bus, _addr)	VDF_1280_720M_60(_mode, "1280x720M@60", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_1920_1080M_60(_mode, _detect, _bus, _addr)	VDF_1920_1080M_60(_mode, "1920x1080M@60", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_1024_768M_60(_mode, _detect, _bus, _addr)	VDF_1024_768M_60(_mode, "1024x768M@60", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_800_600MR_60(_mode, _detect, _bus, _addr)	VDF_800_600MR_60(_mode, "800x600MR@60", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_640_480M_60(_mode, _detect, _bus, _addr)	VDF_640_480M_60(_mode, "640x480M@60", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_720_480M_60(_mode, _detect, _bus, _addr)	VDF_720_480M_60(_mode, "720x480M@60", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_CLAA_WVGA(_mode, _detect, _bus, _addr)	VDF_CLAA_WVGA(_mode, "CLAA-WVGA", RGB666, FBF_MODESTR, _detect, _bus, _addr)
#define VD_SHARP_WVGA(_mode, _detect, _bus, _addr)	VDF_SHARP_WVGA(_mode, "sharp-wvga", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_TFC_A9700LTWV35TC_C1(_mode, _detect, _bus, _addr)	VDF_TFC_A9700LTWV35TC_C1(_mode, "tfc-a9700ltwv35tc-c1", RGB24, 0, _detect, _bus, _addr)
#define VD_800X300_565(_mode, _detect, _bus, _addr)	VDF_800X300(_mode, "800x300rgb565", RGB565, FBF_MODESTR, _detect, _bus, _addr)
#define VD_HITACHI_HVGA(_mode, _detect, _bus, _addr)	VDF_HITACHI_HVGA(_mode, "hitachi_hvga", RGB666, FBF_MODESTR, _detect, _bus, _addr)
#define VD_HITACHI_HVGA565(_mode, _detect, _bus, _addr)	VDF_HITACHI_HVGA(_mode, "hitachi_hvga565", RGB565, FBF_MODESTR, _detect, _bus, _addr)
#define VD_NEON_TOUCH640X240(_mode, _detect, _bus, _addr) VDF_NEON_TOUCH640X240(_mode, "NeonTouch640x240", RGB565, FBF_MODESTR, _detect, _bus, _addr)
#define VD_DC050WX(_mode, _detect, _bus, _addr)		VDF_DC050WX(_mode, "DC050WX", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_INNOLUX_WXGA_14IN_12V(_mode, _detect, _bus, _addr) VDF_INNOLUX_WXGA_14IN_12V(_mode, "INNOLUX-WXGA-IN14-12V", RGB666, 0, _detect, _bus, _addr)
#define VD_AUO_WXGA_11IN_12V(_mode, _detect, _bus, _addr) VDF_AUO_WXGA_11IN_12V(_mode, "AUO-WXGA-IN11-12V", RGB24, 0, _detect, _bus, _addr)
#define VD_OSD_WSVGA(_mode, _detect, _bus, _addr)	VDF_OSD_WSVGA(_mode, "OSD-WSVGA", RGB666, 0, _detect, _bus, _addr)
#define VD_INNOLUX_WVGA(_mode, _detect, _bus, _addr)	VDF_INNOLUX_WVGA(_mode, "INNOLUX-WVGA", RGB666, 0, _detect, _bus, _addr)
#define VD_INNOLUX_WVGA_12V(_mode, _detect, _bus, _addr) VDF_INNOLUX_WVGA(_mode, "INNOLUX-WVGA-12V", RGB666, 0, _detect, _bus, _addr)
#define VD_INNOLUX_WVGA_M(_mode, _detect, _bus, _addr)	VDF_INNOLUX_WVGA(_mode, "INNOLUX-WVGA", RGB666, FBF_MODESTR, _detect, _bus, _addr)
#define VD_OKAYA_480_272(_mode, _detect, _bus, _addr)	VDF_OKAYA_480_272(_mode, "okaya_480x272", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_QVGA(_mode, _detect, _bus, _addr)		VDF_QVGA(_mode, "qvga", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_AT035GT_07ET3(_mode, _detect, _bus, _addr)	VDF_AT035GT_07ET3(_mode, "AT035GT-07ET3", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_AMP1024_600(_mode, _detect, _bus, _addr)	VDF_AMP1024_600(_mode, "amp1024x600", RGB666, 0, _detect, _bus, _addr)
#define VD_ND1024_600(_mode, _detect, _bus, _addr)	VDF_ND1024_600(_mode, "ND-070PCAP-1024x600", RGB24, 0, _detect, _bus, _addr)
#define VD_TM070JDHG30(_mode, _detect, _bus, _addr)	VDF_TM070JDHG30(_mode, "tm070jdhg30", RGB24, 0, _detect, _bus, _addr)
#define VD_AUO_B101EW05(_mode, _detect, _bus, _addr)	VDF_AUO_B101EW05(_mode, "auo_b101ew05", RGB666, 0, _detect, _bus, _addr)
#define VD_HANNSTAR7(_mode, _detect, _bus, _addr)	VDF_HANNSTAR7(_mode, "hannstar7", RGB666, 0, _detect, _bus, _addr)
#define VD_LG1280_800(_mode, _detect, _bus, _addr)	VDF_HANNSTAR7(_mode, "lg1280x800", RGB666, 0, _detect, _bus, _addr)
#define VD_M101NWWB(_mode, _detect, _bus, _addr)	VDF_HANNSTAR7(_mode, "M101NWWB", RGB24, 0, _detect, _bus, _addr)
#define VD_LD101WX1(_mode, _detect, _bus, _addr)	VDF_HANNSTAR7(_mode, "ld101wx1", RGB24, 0, _detect, _bus, _addr)
#define VD_DT070BTFT(_mode, _detect, _bus, _addr)	VDF_DT070BTFT(_mode, "dt070btft", RGB24, FBF_JEIDA, _detect, _bus, _addr)
#define VD_WSVGA(_mode, _detect, _bus, _addr)		VDF_WSVGA(_mode, "wsvga", RGB666, 0, _detect, _bus, _addr)
#define VD_ASIT500MA6F5D(_mode, _detect, _bus, _addr)	VDF_ASIT500MA6F5D(_mode, "ASIT500MA6F5D", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_FUSION7(_mode, _detect, _bus, _addr)		VDF_FUSION7(_mode, "fusion7", RGB666, FBF_MODESTR, _detect, _bus, _addr)
#define VD_HANNSTAR(_mode, _detect, _bus, _addr)	VDF_HANNSTAR(_mode, "hannstar", RGB666, 0, _detect, _bus, _addr)
#define VD_1024_600(_mode, _detect, _bus, _addr)	VDF_1024_600(_mode, "1024x600", RGB666, 0, _detect, _bus, _addr)
#define VD_AFK1024600A02(_mode, _detect, _bus, _addr)	VDF_AFK1024600A02(_mode, "AFK1024600A02", RGB24, 0, _detect, _bus, _addr)
#define VD_LG9_7(_mode, _detect, _bus, _addr)		VDF_LG9_7(_mode, "lg9.7", RGB666, 0, _detect, _bus, _addr)
#define VD_1080P60(_mode, _detect, _bus, _addr)		VDF_1080P60(_mode, "1080P60", RGB24, FBF_SPLITMODE, _detect, _bus, _addr)
#define VD_1080P60_J(_mode, _detect, _bus, _addr)	VDF_1080P60(_mode, "1080P60_J", RGB24, FBF_SPLITMODE | FBF_JEIDA, _detect, _bus, _addr)
#define VD_DV210FBM(_mode, _detect, _bus, _addr)	VDF_DV210FBM(_mode, "dv210fbm", RGB24, FBF_SPLITMODE, _detect, _bus, _addr)
#define VD_SHARP_LQ101K1LY04(_mode, _detect, _bus, _addr) VDF_SHARP_LQ101K1LY04(_mode, "sharp-LQ101K1LY04", RGB24, FBF_JEIDA, _detect, _bus, _addr)
#define VD_WXGA(_mode, _detect, _bus, _addr)		VDF_WXGA(_mode, "wxga", RGB24, 0, _detect, _bus, _addr)
#define VD_WXGA_J(_mode, _detect, _bus, _addr)		VDF_WXGA(_mode, "wxga_j", RGB24, FBF_JEIDA, _detect, _bus, _addr)
#define VD_LD070WSVGA(_mode, _detect, _bus, _addr)	VDF_LD070WSVGA(_mode, "ld070wsvga", RGB24, 0, _detect, _bus, _addr)
#define VD_SVGA(_mode, _detect, _bus, _addr)		VDF_SVGA(_mode, "svga", RGB666, FBF_MODESTR, _detect, _bus, _addr)
#define VD_WVGA_TX23D200_24(_mode, _detect, _bus, _addr) VDF_WVGA_TX23D200(_mode, "tx23d200_24", RGB24, 0, _detect, _bus, _addr)
#define VD_WVGA_TX23D200_24H(_mode, _detect, _bus, _addr) VDF_WVGA_TX23D200(_mode, "tx23d200_24h", RGB24, FBF_BKLIT_DTB, _detect, _bus, _addr)
#define VD_WVGA_TX23D200_24L(_mode, _detect, _bus, _addr) VDF_WVGA_TX23D200(_mode, "tx23d200_24l", RGB24, FBF_BKLIT_DTB | FBF_BKLIT_LOW_ACTIVE, _detect, _bus, _addr)
#define VD_WVGA_TX23D200_18(_mode, _detect, _bus, _addr) VDF_WVGA_TX23D200(_mode, "tx23d200_18", RGB666, 0, _detect, _bus, _addr)
#define VD_WVGA_TX23D200_18H(_mode, _detect, _bus, _addr) VDF_WVGA_TX23D200(_mode, "tx23d200_18h", RGB666, FBF_BKLIT_DTB, _detect, _bus, _addr)
#define VD_WVGA_TX23D200_18L(_mode, _detect, _bus, _addr) VDF_WVGA_TX23D200(_mode, "tx23d200_18l", RGB666, FBF_BKLIT_DTB | FBF_BKLIT_LOW_ACTIVE, _detect, _bus, _addr)
#define VD_WVGA(_mode, _detect, _bus, _addr)		VDF_WVGA(_mode, "wvga", RGB666, 0, _detect, _bus, _addr)
#define VD_WVGA_J(_mode, _detect, _bus, _addr)		VDF_WVGA(_mode, "wvga_j", RGB24, FBF_JEIDA, _detect, _bus, _addr)
#define VD_AA065VE11(_mode, _detect, _bus, _addr)	VDF_AA065VE11(_mode, "AA065VE11", RGB24, 0, _detect, _bus, _addr)
#define VD_VGA(_mode, _detect, _bus, _addr)		VDF_VGA(_mode, "vga", RGB24, 0, _detect, _bus, _addr)
#define VD_LSA40AT9001(_mode, _detect, _bus, _addr)	VDF_LSA40AT9001(_mode, "LSA40AT9001", RGB24, FBF_MODESTR, _detect, _bus, _addr)
#define VD_AUO_G050(_mode, _detect, _bus, _addr)	VDF_AUO_G050(_mode, "AUO_G050", RGB24, FBF_MODESTR | FBF_SPI, _detect, _bus, _addr)
#define VD_A030JN01_UPS051(_mode, _detect, _bus, _addr)	VDF_A030JN01_UPS051(_mode, "A030JN01_UPS051", UPS051, FBF_MODESTR | FBF_SPI, _detect, _bus, _addr)
#define VD_A030JN01_YUV720(_mode, _detect, _bus, _addr) VDF_A030JN01_YUV720(_mode, "A030JN01_YUV720", YUYV, FBF_MODESTR | FBF_SPI, _detect, _bus, _addr)
#define VD_KD024FM(_mode, _detect, _bus, _addr)		VDF_KD024FM(_mode, "KD024FM", RGB666, FBF_MODESTR, _detect, _bus, _addr)

#define VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr) \
	.bus	= _bus,\
	.addr	= _addr,\
	.pixfmt	= IPU_PIX_FMT_##_fmt,\
	.detect	= _detect,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = _flags

/* hdmi settings */
#define VDF_1280_720M_60(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 720,\
		.pixclock       = 1000000000000ULL/((1280+216+72+80)*(720+22+3+5)*60),\
		.left_margin    = 220,\
		.right_margin   = 110,\
		.upper_margin   = 20,\
		.lower_margin   = 5,\
		.hsync_len      = 40,\
		.vsync_len      = 5,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_1920_1080M_60(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1920,\
		.yres           = 1080,\
		.pixclock       = 1000000000000ULL/((1920+148+88+44)*(1080+36+4+5)*60),\
		.left_margin    = 148,\
		.right_margin   = 88,\
		.upper_margin   = 36,\
		.lower_margin   = 4,\
		.hsync_len      = 44,\
		.vsync_len      = 5,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}


#define VDF_1024_768M_60(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 768,\
		.pixclock       = 1000000000000ULL/((1024+220+40+60)*(768+21+7+10)*60),\
		.left_margin    = 220,\
		.right_margin   = 40,\
		.upper_margin   = 21,\
		.lower_margin   = 7,\
		.hsync_len      = 60,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_800_600MR_60(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 600,\
		.pixclock       = 1000000000000ULL/((800+88+40+128)*(600+23+2+3)*60),\
		.left_margin    = 88,\
		.right_margin   = 40,\
		.upper_margin   = 23,\
		.lower_margin   = 2,	/* must be >=2 */\
		.hsync_len      = 128,\
		.vsync_len      = 3,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_640_480M_60(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 640,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((640+48+16+96)*(480+33+10+2)*60),\
		.left_margin    = 48,\
		.right_margin   = 16,\
		.upper_margin   = 33,\
		.lower_margin   = 10,\
		.hsync_len      = 96,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_720_480M_60(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 720,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((720+60+16+62)*(480+30+9+6)*60),\
		.left_margin    = 60,\
		.right_margin   = 16,\
		.upper_margin   = 30,\
		.lower_margin   = 9,\
		.hsync_len      = 62,\
		.vsync_len      = 6,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* tsc2004 */
#define VDF_CLAA_WVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 57,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 37037,\
		.left_margin    = 40,\
		.right_margin   = 60,\
		.upper_margin   = 10,\
		.lower_margin   = 10,\
		.hsync_len      = 20,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* tsc2004 */
/* keep compatible with LQ050Y3DC01 */
#define VDF_SHARP_WVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+40+40+48)*(480+31+11+3)*60),\
		.left_margin    = 40,\
		.right_margin   = 40,\
		.upper_margin   = 31,\
		.lower_margin   = 11,\
		.hsync_len      = 48,\
		.vsync_len      = 3,\
		.sync           = FB_SYNC_EXT |FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_TFC_A9700LTWV35TC_C1(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+40+40+48)*(480+29+13+3)*60),\
		.left_margin    = 40,\
		.right_margin   = 40,\
		.upper_margin   = 29,\
		.lower_margin   = 13,\
		.hsync_len      = 48,\
		.vsync_len      = 3,\
		.sync           = FB_SYNC_EXT |FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}


/*
 * 800x300
 * vsync = 60
 * hsync = 260 * vsync = 15.6 Khz
 * pixclk = 800 * hsync = 12.48 MHz
 */
#define VDF_800X300(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 800,\
		.yres		= 300,\
		.pixclock	= 1000000000000ULL / (800+50+1+110) / (300+8+3+1) / 60,\
		.left_margin	= 50,\
		.right_margin	= 1,\
		.upper_margin	= 8,\
		.lower_margin	= 3,\
		.hsync_len	= 110,\
		.vsync_len	= 1,\
		.sync           = FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/*
 * hitachi 640x240
 * vsync = 60
 * hsync = 260 * vsync = 15.6 Khz
 * pixclk = 800 * hsync = 12.48 MHz
 */
#define VDF_HITACHI_HVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 640,\
		.yres		= 240,\
		.pixclock	= 1000000000000ULL / (640+34+1+125) / (240+8+3+9) / 60,\
		.left_margin	= 34,\
		.right_margin	= 1,\
		.upper_margin	= 8,\
		.lower_margin	= 3,\
		.hsync_len	= 125,\
		.vsync_len	= 9,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_NEON_TOUCH640X240(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 640,\
		.yres		= 240,\
		.pixclock	= 1000000000000ULL / (640+34+1+125) / (240+8+3+9) / 60,\
		.left_margin	= 34,\
		.right_margin	= 1,\
		.upper_margin	= 8,\
		.lower_margin	= 3,\
		.hsync_len	= 125,\
		.vsync_len	= 9,\
		.sync           = FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* tsc2004 */
#define VDF_DC050WX(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 33898,\
		.left_margin    = 96,\
		.right_margin   = 24,\
		.upper_margin   = 3,\
		.lower_margin   = 10,\
		.hsync_len      = 40,\
		.vsync_len      = 32,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}


/* INNOLUX model N140BGE, 18 bit LVDS */
#define VDF_INNOLUX_WXGA_14IN_12V(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 1366,\
		.yres		= 768,\
		.pixclock	= 1000000000000ULL / (1366+108+108+10) / (768+8+8+16) / 60,\
		.left_margin	= 108,\
		.right_margin	= 108,\
		.upper_margin	= 8,\
		.lower_margin	= 8,\
		.hsync_len	= 10,\
		.vsync_len	= 16,\
		.sync		= FB_SYNC_EXT,\
		.vmode		= FB_VMODE_NONINTERLACED,\
	}\
}

/* AUO model B116XAN03.0, 11.6", 1366x768, 24 bit lvds */
#define VDF_AUO_WXGA_11IN_12V(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 1366,\
		.yres		= 768,\
		.pixclock	= 1000000000000ULL / (1366+67+67+100) / (768+10+10+6) / 60,\
		.left_margin	= 67,\
		.right_margin	= 67,\
		.upper_margin	= 10,\
		.lower_margin	= 10,\
		.hsync_len	= 100,\
		.vsync_len	= 6,\
		.sync		= FB_SYNC_EXT,\
		.vmode		= FB_VMODE_NONINTERLACED,\
	}\
}

/* OSD model OSD101T1315-45, 18 bit LVDS*/
#define VDF_OSD_WSVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 1024,\
		.yres		= 600,\
		.pixclock	= 1000000000000ULL / (1024+45+210+1) / (600+22+132+1) / 60,\
		.left_margin	= 45,\
		.right_margin	= 210,\
		.upper_margin	= 22,\
		.lower_margin	= 132,\
		.hsync_len	= 1,\
		.vsync_len	= 1,\
		.sync		= FB_SYNC_EXT,\
		.vmode		= FB_VMODE_NONINTERLACED,\
	}\
}

/* INNOLUX model AT070TN83, 800x480  18 bit RGB with or without LVDS converter board */
#define VDF_INNOLUX_WVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 800,\
		.yres		= 480,\
		.pixclock	= 1000000000000ULL / (800+45+16+1) / (480+22+125+1) / 60,\
		.left_margin	= 45,\
		.right_margin	= 16,\
		.upper_margin	= 22,\
		.lower_margin	= 125,\
		.hsync_len	= 1,\
		.vsync_len	= 1,\
		.sync		= FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode		= FB_VMODE_NONINTERLACED,\
	}\
}

#define VDF_LB043(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 57,\
		.xres           = 480,\
		.yres           = 800,\
		.pixclock       = 37037,\
		.left_margin    = 40,\
		.right_margin   = 60,\
		.upper_margin   = 10,\
		.lower_margin   = 10,\
		.hsync_len      = 20,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	},\
}

#define VDF_OKAYA_480_272(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 57,\
		.xres		= 480,\
		.yres		= 272,\
		.pixclock	= 97786,\
		.left_margin	= 2,\
		.right_margin	= 1,\
		.upper_margin	= 3,\
		.lower_margin	= 2,\
		.hsync_len	= 41,\
		.vsync_len	= 10,\
		.sync		= FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode		= FB_VMODE_NONINTERLACED\
	}\
}

/* tsc2004 */
#define VDF_QVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 320,\
		.yres           = 240,\
		.pixclock       = 1000000000000ULL/((320+38+37+30)*(240+16+15+3)*60),\
		.left_margin    = 38,\
		.right_margin   = 37,\
		.upper_margin   = 16,\
		.lower_margin   = 15,\
		.hsync_len      = 30,\
		.vsync_len      = 3,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_SPI_QVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 320,\
		.yres           = 240,\
		.pixclock       = 1000000000000ULL/((320+16+20+52)*(240+16+4+2)*60),\
		.left_margin    = 16,\
		.right_margin   = 20,\
		.upper_margin   = 16,\
		.lower_margin   = 4,\
		.hsync_len      = 52,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* tsc2004 */
#define VDF_AT035GT_07ET3(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 320,\
		.yres           = 240,\
		.pixclock       = 1000000000000ULL/((320+40+18+30)*(240+10+9+3)*60),\
		.left_margin    = 40,\
		.right_margin   = 18,\
		.upper_margin   = 10,\
		.lower_margin   = 9,\
		.hsync_len      = 30,\
		.vsync_len      = 3,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ili210x touch screen */
#define VDF_AMP1024_600(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock       = 1000000000000ULL/((1024+220+40+60)*(600+21+7+10)*60),\
		.left_margin    = 220,\
		.right_margin   = 40,\
		.upper_margin   = 21,\
		.lower_margin   = 7,\
		.hsync_len      = 60,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ft5x06 touch screen */
#define VDF_ND1024_600(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock       = 1000000000000ULL/((1024+160+80+80)*(600+19+8+8)*60),\
		.left_margin    = 160,\
		.right_margin   = 80,\
		.upper_margin   = 19,\
		.lower_margin   = 8,\
		.hsync_len      = 80,\
		.vsync_len      = 8,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ft5x06 touch screen */
/* Tianma panel TM070JDHG30 is a 24 bit spwg panel */
#define VDF_TM070JDHG30(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock       = 1000000000000ULL/((1280+5+63+1)*(800+2+39+1)*60),\
		.left_margin    = 5,\
		.right_margin   = 63,\
		.upper_margin   = 2,\
		.lower_margin   = 39,\
		.hsync_len      = 1,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_AUO_B101EW05(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock       = 1000000000000ULL/((1280+48+48+32)*(800+8+2+6)*60),\
		.left_margin    = 48,\
		.right_margin   = 48,\
		.upper_margin   = 8,\
		.lower_margin   = 2,\
		.hsync_len      = 32,\
		.vsync_len      = 6,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ft5x06_ts */
/* lg1280x800(LP101WX1) == hannstar7 */
/* LG panel LD101WX1 is a 24 bit spwg panel */
#define VDF_HANNSTAR7(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock       = 1000000000000ULL/((1280+80+48+32)*(800+15+2+6)*60),\
		.left_margin    = 80,\
		.right_margin   = 48,\
		.upper_margin   = 15,\
		.lower_margin   = 2,\
		.hsync_len      = 32,\
		.vsync_len      = 6,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ft5x06_ts */
#define VDF_DT070BTFT(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock       = 1000000000000ULL/((1024+220+40+60)*(600+21+4+10)*60),\
		.left_margin    = 220,\
		.right_margin   = 40,\
		.upper_margin   = 21,\
		.lower_margin   = 4,\
		.hsync_len      = 60,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ft5x06_ts */
#define VDF_WSVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock       = 1000000000000ULL/((1024+220+40+60)*(600+21+7+10)*60),\
		.left_margin    = 220,\
		.right_margin   = 40,\
		.upper_margin   = 21,\
		.lower_margin   = 7,\
		.hsync_len      = 60,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* Also works for ER-TFT050-3 */
#define VDF_ASIT500MA6F5D(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.pwm_period = 100000, \
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL / (800+88+40+48) / (480+32+13+3) / 60,\
		.left_margin    = 88,\
		.right_margin   = 40,\
		.upper_margin   = 32,\
		.lower_margin   = 13,\
		.hsync_len      = 48,\
		.vsync_len      = 3,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* fusion7 */
#define VDF_FUSION7(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+96+24+136)*(480+31+11+3)*60),\
		.left_margin    = 96,\
		.right_margin   = 24,\
		.upper_margin   = 31,\
		.lower_margin   = 11,\
		.hsync_len      = 136,\
		.vsync_len      = 3,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* egalax_ts */
#define VDF_HANNSTAR(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 768,\
		.pixclock       = 1000000000000ULL/((1024+220+40+60)*(768+21+7+10)*60),\
		.left_margin    = 220,\
		.right_margin   = 40,\
		.upper_margin   = 21,\
		.lower_margin   = 7,\
		.hsync_len      = 60,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_1024_600(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock       = 20408,\
		.left_margin    = 144,\
		.right_margin   = 40,\
		.upper_margin   = 3,\
		.lower_margin   = 11,\
		.hsync_len      = 104,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_AFK1024600A02(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock       = 1000000000000ULL/((1024+160+80+80)*(600+19+8+8)*60),\
		.left_margin    = 160,\
		.right_margin   = 80,\
		.upper_margin   = 19,\
		.lower_margin   = 8,\
		.hsync_len      = 80,\
		.vsync_len      = 8,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* egalax_ts */
#define VDF_LG9_7(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 768,\
		.pixclock       = 1000000000000ULL/((1024+480+260+250)*(768+16+6+10)*60),\
		.left_margin    = 480,\
		.right_margin   = 260,\
		.upper_margin   = 16,\
		.lower_margin   = 6,\
		.hsync_len      = 250,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_1080P60(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1920,\
		.yres           = 1080,\
		.pixclock       = 1000000000000ULL/((1920+148+88+44)*(1080+36+4+5)*60),\
		.left_margin    = 148,\
		.right_margin   = 88,\
		.upper_margin   = 36,\
		.lower_margin   = 4,\
		.hsync_len      = 44,\
		.vsync_len      = 5,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_DV210FBM(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1920,\
		.yres           = 1080, /* really 132 */\
		.pixclock       = 1000000000000ULL/((1920+120+120+40)*(1080+22+22+1)*60),\
		.left_margin    = 120,\
		.right_margin   = 120,\
		.upper_margin   = 22,\
		.lower_margin   = 22,\
		.hsync_len      = 44,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_SHARP_LQ101K1LY04(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock       = 1000000000000ULL/((1280+20+20+10)*(800+4+4+4)*60),\
		.left_margin    = 20,\
		.right_margin   = 20,\
		.upper_margin   = 4,\
		.lower_margin   = 4,\
		.hsync_len      = 10,\
		.vsync_len      = 4,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_WXGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock       = 1000000000000ULL/((1280+40+40+10)*(800+3+80+10)*60),\
		.left_margin    = 40,\
		.right_margin   = 40,\
		.upper_margin   = 3,\
		.lower_margin   = 80,\
		.hsync_len      = 10,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_LD070WSVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 55,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock       = 1000000000000ULL/((1024+160+160+10)*(600+23+12+3)*55),\
		.left_margin    = 160,\
		.right_margin   = 160,\
		.upper_margin   = 23,\
		.lower_margin   = 12,\
		.hsync_len      = 10,\
		.vsync_len      = 3,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_SVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 600,\
		.pixclock       = 15385,\
		.left_margin    = 220,\
		.right_margin   = 40,\
		.upper_margin   = 21,\
		.lower_margin   = 7,\
		.hsync_len      = 60,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* 40.0 ns (25 Mhz) to 28.6 ns (34.965 MHz)
 * 35.35 ns(28.28 Mhz)
 * 1000000000/1056/525/40 =  45.094 frames/second
 * 1000000000/1056/525/35.35 = 51 frames/second
 * 1000000000/1056/525/28.6 =  63.068 frames/second
 */
#define VDF_WVGA_TX23D200(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+220+18+18)*(480+21+14+10)*52),\
		.left_margin    = 220,\
		.right_margin   = 18,\
		.upper_margin   = 21,\
		.lower_margin   = 14,\
		.hsync_len      = 18,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_WVGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+220+40+60)*(480+21+7+10)*60),\
		.left_margin    = 220,\
		.right_margin   = 40,\
		.upper_margin   = 21,\
		.lower_margin   = 7,\
		.hsync_len      = 60,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_AA065VE11(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 70,\
		.xres           = 640,\
		.yres           = 480,\
		.pixclock       = 22858,	/* 23000 works 23100 doesn't */\
		.left_margin    = 48 + 100,\
		.right_margin   = 16 + 100,\
		.upper_margin   = 33 + 50,\
		.lower_margin   = 10 + 50,\
		.hsync_len      = 96,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_VGA(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 640,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((640+48+16+96)*(480+33+10+2)*60),\
		.left_margin    = 48,\
		.right_margin   = 16,\
		.upper_margin   = 33,\
		.lower_margin   = 10,\
		.hsync_len      = 96,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_LSA40AT9001(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 600,\
		.pixclock       = 1000000000000ULL/((800+46+210+10)*(600+23+12+1)*60),\
		.left_margin    = 46,\
		.right_margin   = 210,\
		.upper_margin   = 23,\
		.lower_margin   = 12,\
		.hsync_len      = 10,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* spi panels */
#define VDF_AUO_G050(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 480,\
		.yres           = 800,\
		.pixclock       = 1000000000000ULL/((480+18+16+2)*(800+18+16+2)*60),\
		.left_margin    = 18,\
		.right_margin   = 16,\
		.upper_margin   = 18,\
		.lower_margin   = 16,\
		.hsync_len      = 2,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

//640 * 3/2 = 960, (1.5 clocks per pixel)
//33.7M
#define VDF_A030JN01_UPS051(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 640,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((960+40+48+20)*(480+27+18+1)*60),\
		.left_margin    = 40,\
		.right_margin   = 48,\
		.upper_margin   = 27,\
		.lower_margin   = 18,\
		.hsync_len      = 20,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* 27.11 MHz pixel clock */
#define VDF_A030JN01_YUV720(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.mode   = {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 720,\
		.yres           = 480,\
		.pixclock       =  1000000000000ULL/((720+40+98+1)*(480+27+18+1)*60),\
		.left_margin    = 40,\
		.right_margin   = 98,\
		.upper_margin   = 27,\
		.lower_margin   = 18,\
		.hsync_len      = 1,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_KD024FM(_mode, _name, _fmt, _flags, _detect, _bus, _addr) \
{\
	VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr),\
	.pre_enable = board_pre_enable, \
	.pwm_period = 100000, \
	.mode   = {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 240,\
		.yres           = 320,\
		.pixclock       =  1000000000000ULL/((240+10+38+10)*(320+4+8+4)*60),\
		.left_margin    = 10,\
		.right_margin   = 38,\
		.upper_margin   = 4,\
		.lower_margin   = 8,\
		.hsync_len      = 10,\
		.vsync_len      = 4,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}
#endif
