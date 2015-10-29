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

#define FB_HDMI		0
#define FB_LCD		1
#define FB_LVDS		2
#define FB_LVDS2	3
#define FB_COUNT	4
	int	fbtype;

#define FBF_MODESTR	1
#define FBF_JEIDA	2
#define FBF_SPLITMODE	4
#define FBF_SPI		8
	int	fbflags;
	struct	fb_videomode mode;
};

void board_enable_hdmi(const struct display_info_t *di, int enable);
void board_enable_lcd(const struct display_info_t *di, int enable);
void board_enable_lvds(const struct display_info_t *di, int enable);
void board_enable_lvds2(const struct display_info_t *di, int enable);

void fbp_enable_fb(struct display_info_t const *di, int enable);
int fbp_detect_i2c(struct display_info_t const *di);
void fbp_setup_display(const struct display_info_t *displays, int cnt);

/* hdmi settings */
#define IMX_VD50_1280_720M_60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x50,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "1280x720M@60",\
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

#define IMX_VD50_1920_1080M_60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x50,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "1920x1080M@60",\
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
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}


#define IMX_VD50_1024_768M_60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x50,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "1024x768M@60",\
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

#define IMX_VD50_800_600MR_60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "800x600MR@60",\
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
		.sync           = FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define IMX_VD50_640_480M_60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x50,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "640x480M@60",\
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

#define IMX_VD50_720_480M_60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x50,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "720x480M@60",\
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
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* tsc2004 */
#define IMX_VD48_CLAA_WVGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x48,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "CLAA-WVGA",\
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
		.sync           = FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* tsc2004 */
/* keep compatible with LQ050Y3DC01 */
#define IMX_VD48_SHARP_WVGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x48,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "sharp-wvga",\
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
#define IMX_VD48_HITACHI_HVGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x48,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name		= "hitachi_hvga",\
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
#define IMX_VD48_DC050WX(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x48,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "DC050WX",\
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
		.sync           = FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define IMX_VD48_INNOLUX_WVGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x48,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name = "INNOLUX-WVGA",\
		.refresh = 57,\
		.xres = 800,\
		.yres = 480,\
		.pixclock = 25000,\
		.left_margin = 45,\
		.right_margin = 1056 - 1 - 45 - 800,\
		.upper_margin = 22,\
		.lower_margin = 635 - 1 - 22 - 480,\
		.hsync_len = 1,\
		.vsync_len = 1,\
		.sync = 0,\
		.vmode = FB_VMODE_NONINTERLACED,\
	}\
}

#define IMX_VD48_OKAYA_480_272(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x48,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "okaya_480x272",\
		.refresh        = 57,\
		.xres           = 480,\
		.yres           = 272,\
		.pixclock       = 97786,\
		.left_margin    = 2,\
		.right_margin   = 1,\
		.upper_margin   = 3,\
		.lower_margin   = 2,\
		.hsync_len      = 41,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* tsc2004 */
#define IMX_VD48_QVGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x48,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "qvga",\
		.refresh        = 60,\
		.xres           = 320,\
		.yres           = 240,\
		.pixclock       = 37037,\
		.left_margin    = 38,\
		.right_margin   = 37,\
		.upper_margin   = 16,\
		.lower_margin   = 15,\
		.hsync_len      = 30,\
		.vsync_len      = 3,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* tsc2004 */
#define IMX_VD48_AT035GT_07ET3(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x48,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "AT035GT-07ET3",\
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
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ili210x touch screen */
#define IMX_VD41_AMP1024_600(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x41,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name           = "amp1024x600",\
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
/* LG panel LD101WX1 is a 24 bit spwg panel */
#define IMX_VD38_LD101WX1(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x38,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.mode	= {\
		.name           = "ld101wx1",\
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

/* lg1280x800(LP101WX1) == hannstar7 */
#define IMX_VD38_LG1280_800(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x38,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.mode	= {\
		.name           = "lg1280x800",\
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
/* lg1280x800(LP101WX1) == hannstar7 */
#define IMX_VD38_HANNSTAR7(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x38,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.mode	= {\
		.name           = "hannstar7",\
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
#define IMX_VD38_WSVGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x38,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name           = "wsvga",\
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
#define IMX_VD38_ASIT500MA6F5D(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x38,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "ASIT500MA6F5D",\
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
#define IMX_VD38_HITACHI_HVGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x38,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name		= "hitachi_hvga",\
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

/* fusion7 */
#define IMX_VD10_FUSION7(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x10,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "fusion7",\
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
		.sync           = FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* egalax_ts */
#define IMX_VD04_HANNSTAR(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x4,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name           = "hannstar",\
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

#define IMX_VD04_1024_600(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x4,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name           = "1024x600",\
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

/* egalax_ts */
#define IMX_VD04_LG9_7(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x4,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name           = "lg9.7",\
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

#define IMX_VD_1080P60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_SPLITMODE,\
	.mode	= {\
		.name           = "1080P60",\
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

#define IMX_VD_SHARP_LQ101K1LY04(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_JEIDA,\
	.mode	= {\
		.name           = "sharp-LQ101K1LY04",\
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

#define IMX_VD_WXGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name           = "wxga",\
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

#define IMX_VD_WXGA_J(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_JEIDA,\
	.mode	= {\
		.name           = "wxga_j",\
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

#define IMX_VD_SVGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "svga",\
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
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define IMX_VD_WVGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name           = "wvga",\
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

#define IMX_VD_WVGA_J(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_JEIDA,\
	.mode	= {\
		.name           = "wvga_j",\
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

#define IMX_VD_AA065VE11(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name           = "AA065VE11",\
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

#define IMX_VD_VGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name           = "vga",\
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

#define IMX_VD_LSA40AT9001(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "LSA40AT9001",\
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
		.sync           = FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* spi panels */
#define IMX_VD_AUO_G050(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR | FBF_SPI,\
	.mode	= {\
		.name           = "AUO_G050",\
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
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

//640 * 3/2 = 960, (1.5 clocks per pixel)
//33.7M
#define IMX_VD_A030JN01_UPS051(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 2,\
	.pixfmt	= IPU_PIX_FMT_UPS051,\
	.detect	= NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR | FBF_SPI,\
	.mode	= {\
		.name           = "A030JN01_UPS051",\
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
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* 27.11 MHz pixel clock */
#define IMX_VD_A030JN01_YUV720(_mode, _detect, _bus) \
{\
	.bus    = _bus,\
	.addr   = 1,\
	.pixfmt = IPU_PIX_FMT_YUYV,\
	.detect = NULL,\
	.enable = fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR | FBF_SPI,\
	.mode   = {\
		.name           = "A030JN01_YUV720",\
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
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#endif
