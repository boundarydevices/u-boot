/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __IMX_VIDEO_H_
#define __IMX_VIDEO_H_

#include <linux/fb.h>
#include <ipu_pixfmt.h>

#define fbp_bus_gp(bus_num, bus_gp, enable_gp, bus_delay) ((bus_num) | \
		((bus_gp) << 8) | ((enable_gp) << 16) | ((bus_delay) << 24))

#define fbp_bus_gp2(bus_num, bus_gp, enable_gp, bus_gp_delay_ms, \
		enable_high_duration_us, enable_low_duration_us, power_gp, \
		power_delay_ms) \
		((bus_num) | ((bus_gp) << 8) | \
		 ((u64)(enable_gp) << 16) | ((u64)(bus_gp_delay_ms) << 24) | \
		 ((u64)(enable_high_duration_us) << 32) | ((u64)(enable_low_duration_us) << 40) | \
		 (((u64)power_gp) << 48) | (((u64)power_delay_ms) << 56))

#define fbp_addr_gp(addr_num, backlight_en_gp, min_hs_clock_multiple, reset_gp) ((addr_num) | \
		((backlight_en_gp) << 8) | ((min_hs_clock_multiple) << 16) | ((reset_gp) << 24))

enum alias_names {
	FBTS_NONE = 0,
	FBTS_ATMEL_MT,
	FBTS_CRTOUCH,
	FBTS_CYTTSP5,
	FBTS_EGALAX,
	FBTS_EXC3000,
	FBTS_FT5X06,
	FBTS_FT5X06_2,
	FBTS_FT5X06_3,
	FBTS_FT7250,
	FBTS_FUSION7,
	FBTS_GOODIX,
	FBTS_GOODIX2,
	FBTS_GOODIX3,
	FBTS_GSL1680,
	FBTS_ILI210X,
	FBTS_ILI251X,
	FBTS_ST1633I,
	FBTS_TSC2004,
	FBTS_LVDS_ATMEL_MT,
	FBTS_LVDS_EGALAX,
	FBTS_LVDS_FT5X06,
	FBTS_LVDS_FT5X06_2,
	FBTS_LVDS_GOODIX,
	FBTS_LVDS_GOODIX2,
	FBTS_LVDS_ILI251X,
	FBP_MIPI_TO_LVDS,
	FBP_LT8912,
	FBP_LT8912_2,
	FBP_SPI_LCD,
	FBP_PCA9540,
	FBP_PCA9540_2,
	FBP_PCA9546,
	FBP_BACKLIGHT_MIPI_ALT,
	FBP_PWM_MIPI_ALT,
};

/* keep beginning of structure synced with fb_videomode, memcpy is used */
struct fb_videomode_f {
        const char *name;       /* optional */
        u32 refresh;            /* optional */
        u32 xres;
        u32 yres;
        u32 pixclock_f;
        u32 left_margin;
        u32 right_margin;
        u32 upper_margin;
        u32 lower_margin;
        u32 hsync_len;
        u32 vsync_len;
        u32 sync;
        u32 vmode;
        u32 flag;
};

struct display_info_t {
	union {
		u64	bus;
		struct {
			unsigned char bus_num;
			unsigned char bus_gp;
			unsigned char enable_gp;
			unsigned char bus_gp_delay_ms;
			unsigned char enable_high_duration_us;
			unsigned char enable_low_duration_us;
			unsigned char power_gp;
			unsigned char power_delay_ms;
		};
	};
	union {
		u32	addr;
		struct {
			unsigned char addr_num;
			unsigned char backlight_en_gp;
			unsigned char min_hs_clock_multiple;
			unsigned char reset_gp;
		};
	};
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev, int enable);
	void	(*pre_enable)(struct display_info_t const *dev);
	int	pwm_period;
#define FB_HDMI		0	/* 0-5 also in linux/fb.h */
#define FB_LCD		1
#define FB_LCD2		2
#define FB_LVDS		3
#define FB_LVDS2	4
#define FB_MIPI		5
#define FB_MIPI2	6
#define FB_MIPI_BRIDGE	7	/* Not included in FB_COUNT */
#define FB_COUNT	7
	int	fbtype;

#define FBF_MODESTR	1
#define FBF_JEIDA	2	/* JEIDA - BGR 0/1 on last differential pair */
#define FBF_SPLITMODE	4
#define FBF_SPI		8
#define FBF_BKLIT_LOW_ACTIVE	0x010	/* pwm active level */
#define FBF_BKLIT_DTB		0x020	/* brightness levels for pwm */
#define FBF_PINCTRL		0x040
#define FBF_ENABLE_GPIOS_ACTIVE_LOW 0x080
#define FBF_ENABLE_GPIOS_DTB	0x100
#define FBF_BKLIT_EN_LOW_ACTIVE	0x200
#define FBF_BKLIT_EN_DTB	0x400
#define FBF_MIPI_CMDS		0x800
#define FBF_MODE_SKIP_EOT	0x1000
#define FBF_MODE_VIDEO		0x2000
#define FBF_MODE_VIDEO_BURST	0x4000
	/* mipi byte clock a multiple of the pixel clock */
#define FBF_MODE_VIDEO_MBC	0x8000
#define FBF_MODE_VIDEO_SYNC_PULSE 0x10000
#define FBF_DSI_HBP		BIT(17)
#define FBF_DSI_HFP		BIT(18)
#define FBF_DSI_HSA		BIT(19)
#define FBF_USE_IPU_CLOCK	BIT(20)
#define FBF_MODESTR_IPU		(FBF_MODESTR | FBF_USE_IPU_CLOCK)
#define FBF_ENABLE_GPIOS_OPEN_DRAIN BIT(21)
#define FBF_ALT_PWM		BIT(22)
#define FBF_SN65_ALT		BIT(23)

#define FBF_DSI_LANE_SHIFT	29
#define FBF_DSI_LANES_1		(0x1 << FBF_DSI_LANE_SHIFT)
#define FBF_DSI_LANES_2		(0x2 << FBF_DSI_LANE_SHIFT)
#define FBF_DSI_LANES_3		(0x3 << FBF_DSI_LANE_SHIFT)
#define FBF_DSI_LANES_4		(0x4UL << FBF_DSI_LANE_SHIFT)

#define FBF_DMT050WVNXCMI	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4)
#define FBF_DMT055FHNMCMI	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_ENABLE_GPIOS_DTB)
#define FBF_ER_TFT050		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_ENABLE_GPIOS_DTB)
#define FBF_LCM_JM430		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_MODE_VIDEO_MBC | FBF_MIPI_CMDS | FBF_DSI_LANES_1 | FBF_BKLIT_EN_DTB | FBF_ENABLE_GPIOS_DTB)
#define FBF_LS050T1SX12		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_LTK0680YTMDB	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_LTK0680YTMDB_LCD	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_MODESTR)
#define FBF_LTK069WXBCT02	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_LTK069WXBCT02_LCD	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_MODESTR)
#define FBF_LTK069WXICT10	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_LTK069WXICT10_LCD	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_MODESTR)
#define FBF_LTK069WXICT11	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_LTK069WXICT11_LCD	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_MODESTR)
#define FBF_LXD_M8509A		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_2)
#define FBF_MIPI_LT8912		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_DSI_LANES_4 |FBF_BKLIT_DTB)
#define FBF_MIPI_LT8912_2	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_DSI_LANES_4)
#define FBF_MIPI_MQ_LT8912	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_DSI_LANES_4 | FBF_BKLIT_DTB)

#define FBF_G156HCE_L01		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_DSI_LANES_4 | FBF_SPLITMODE)
#define FBF_COM50H5N03ULC	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_LTK080A60A004T	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_CS005_0004_03	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB | FBF_BKLIT_EN_DTB)
#define FBF_B_M101NWWB		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4)
#define FBF_B_TM070JDHG30	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4)
#define FBF_BT_TM070JDHG30	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_SN65_ALT)
#define FBF_U_M101NWWB		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_DSI_HBP | FBF_DSI_HFP | FBF_DSI_HSA)
#define FBF_U_TM070JDHG30	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_DSI_HBP | FBF_DSI_HFP | FBF_DSI_HSA)
#define FBF_UT_TM070JDHG30	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_DSI_HBP | FBF_DSI_HFP | FBF_DSI_HSA | FBF_SN65_ALT)
#define FBF_DLC0350GEV06	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_COM35H3R04ULY	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_LCD133_070		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_TCXD070		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_BKLIT_EN_DTB | FBF_ENABLE_GPIOS_DTB)
#define FBF_ET055FH06		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_BKLIT_EN_DTB | FBF_ENABLE_GPIOS_DTB)
#define FBF_ZWT055AZH		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_BKLIT_EN_DTB | FBF_ENABLE_GPIOS_DTB)
#define FBF_MIPI_TO_HDMI	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_DSI_LANES_4)
#define FBF_VTFT101		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_DSI_LANES_4)
#define FBF_E_M101NWWB		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MIPI_CMDS | FBF_DSI_LANES_4)
#define FBF_E_TM070JDHG30	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MIPI_CMDS | FBF_DSI_LANES_4)
#define FBF_ET_TM070JDHG30	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_SN65_ALT)

#define FBF_MQ_VTFT101		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_DSI_LANES_4)
#define FBF_MQ_Q035_014		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_MIPI_CMDS | FBF_DSI_LANES_4 | FBF_PINCTRL | FBF_ENABLE_GPIOS_DTB)
#define FBF_MIPI_MQ_TO_HDMI	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_DSI_LANES_4)
#define FBF_MIPI_MQ2_TO_HDMI	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_DSI_LANES_2)
#define FBF_M101NWWB_NO_CMDS	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_DSI_LANES_4 | FBF_BKLIT_EN_DTB)
#define FBF_MTD0900DCP27KF	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_DSI_LANES_4 | FBF_DSI_HBP | FBF_DSI_HFP | FBF_DSI_HSA)
#define FBF_AM_TFT1280X800	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_DSI_LANES_4 | FBF_DSI_HBP | FBF_DSI_HFP | FBF_DSI_HSA)
#define FBF_AM_TFT1280X800W	(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_SYNC_PULSE | FBF_DSI_LANES_4)
#define FBF_OSD050T		(FBF_MODE_SKIP_EOT | FBF_MODE_VIDEO | FBF_MODE_VIDEO_BURST | FBF_MIPI_CMDS | FBF_DSI_LANES_2)

	unsigned fbflags;
	unsigned char enable_alias[4];
	struct	fb_videomode_f mode;
};
int ipu_set_ldb_clock(int rate);

void board_enable_hdmi(const struct display_info_t *di, int enable);
void board_enable_lcd(const struct display_info_t *di, int enable);
void board_enable_lvds(const struct display_info_t *di, int enable);
void board_enable_lvds2(const struct display_info_t *di, int enable);

void fbp_enable_fb(struct display_info_t const *di, int enable);
int fbp_detect_i2c(struct display_info_t const *di);
void fbp_setup_display(const struct display_info_t *displays, int cnt);
void fbp_setup_env_cmds(void);

#define VD_1920_1080M_60(_mode, args...)	VDF_1920_1080M_60(_mode, "1920x1080M@60", RGB24, FBF_MODESTR, args)
#define VD_1280_800M_60(_mode, args...)		VDF_1280_800M_60(_mode, "1280x800M@60", RGB24, FBF_MODESTR, args)
#define VD_1280_720M_60(_mode, args...)		VDF_1280_720M_60(_mode, "1280x720M@60", RGB24, FBF_MODESTR, args)
#define VD_1024_768M_60(_mode, args...)		VDF_1024_768M_60(_mode, "1024x768M@60", RGB24, FBF_MODESTR, args)
#define VD_800_600MR_60(_mode, args...)		VDF_800_600MR_60(_mode, "800x600MR@60", RGB24, FBF_MODESTR, args)
#define VD_640_480M_60(_mode, args...)		VDF_640_480M_60(_mode, "640x480M@60", RGB24, FBF_MODESTR, args)
#define VD_720_480M_60(_mode, args...)		VDF_720_480M_60(_mode, "720x480M@60", RGB24, FBF_MODESTR, args)
#define VD_MIPI_1280_800M_60(_mode, args...)	VDF_1280_800M_60(_mode, "dsi-1280x800M@60", RGB24, FBF_MIPI_TO_HDMI, args)
#define VD_TM070JDHG30_LT8912(_mode, args...)	VDF_MIPI_TM070JDHG30(_mode, "tm070jdhg30_lt8912", RGB24, FBF_M101NWWB_NO_CMDS, args)
#define VD_MIPI_TM070JDHG30_LT8912(_mode, args...) VDF_MIPI_TM070JDHG30(_mode, "tm070jdhg30_lt8912", RGB24, FBF_MIPI_LT8912_2, args)
#define VD_MIPI_TM070JDHG30_LT8912_2(_mode, args...) VDF_MIPI_TM070JDHG30(_mode, "tm070jdhg30_lt8912-2", RGB24, FBF_MIPI_LT8912_2, args)
#define VD_MIPI_MQ_TM070JDHG30_LT8912(_mode, args...) VDF_MIPI_TM070JDHG30(_mode, "tm070jdhg30_lt8912", RGB24, FBF_MIPI_MQ_LT8912, args)
#define VD_MIPI_MQ_TM070JDHG30_LT8912_2(_mode, args...) VDF_MIPI_TM070JDHG30(_mode, "tm070jdhg30_lt8912-2", RGB24, FBF_MIPI_MQ_LT8912, args)

#define VD_MIPI_1280_720M_60(_mode, args...)	VDF_1280_720M_60(_mode, "dsi-1280x720M@60", RGB24, FBF_MIPI_TO_HDMI, args)
#define VD_MIPI_1920_1080M_60(_mode, args...)	VDF_1920_1080M_60(_mode, "dsi-1920x1080M@60", RGB24, FBF_MIPI_TO_HDMI, args)
#define VD_MIPI_1024_768M_60(_mode, args...)	VDF_1024_768M_60(_mode, "dsi-1024x768M@60", RGB24, FBF_MIPI_TO_HDMI, args)
#define VD_MIPI_800_600MR_60(_mode, args...)	VDF_800_600MR_60(_mode, "dsi-800x600MR@60", RGB24, FBF_MIPI_TO_HDMI, args)
#define VD_MIPI_640_480M_60(_mode, args...)	VDF_640_480M_60(_mode, "dsi-640x480M@60", RGB24, FBF_MIPI_TO_HDMI, args)
#define VD_MIPI_720_480M_60(_mode, args...)	VDF_720_480M_60(_mode, "dsi-720x480M@60", RGB24, FBF_MIPI_TO_HDMI, args)
#define VD_MIPI_CS005_0004_03(_mode, args...)	VDF_MIPI_CS005_0004_03(_mode, "cs005_0004_03", RGB24, FBF_CS005_0004_03, args)
#define VD_VTFT101RPFT20(_mode, args...)	VDF_MIPI_VTFT101RPFT20(_mode, "vtft101rpft20", RGB24, 0, args)
#define VD_MIPI_VTFT101RPFT20(_mode, args...)	VDF_MIPI_VTFT101RPFT20(_mode, "vtft101rpft20", RGB24, FBF_VTFT101, args)
#define VD_MIPI_VTFT101RPFT20_2(_mode, args...)	VDF_MIPI_VTFT101RPFT20(_mode, "vtft101rpft20-2", RGB24, FBF_VTFT101, args)
#define VD_MIPI_VTFT101RPFT20_3(_mode, args...)	VDF_MIPI_VTFT101RPFT20(_mode, "vtft101rpft20-3", RGB24, FBF_VTFT101, args)
#define VD_MIPI_MQ_VTFT101RPFT20(_mode, args...) VDF_MIPI_VTFT101RPFT20(_mode, "vtft101rpft20", RGB24, FBF_MQ_VTFT101, args)
#define VD_MIPI_MQ_VTFT101RPFT20_2(_mode, args...) VDF_MIPI_VTFT101RPFT20(_mode, "vtft101rpft20-2", RGB24, FBF_MQ_VTFT101, args)
#define VD_MIPI_MQ_VTFT101RPFT20_3(_mode, args...) VDF_MIPI_VTFT101RPFT20(_mode, "vtft101rpft20-3", RGB24, FBF_MQ_VTFT101, args)
#define VD_MIPI_TCXD070IBLMAT77(_mode, args...) VDF_MIPI_TCXD070IBLMAT77(_mode, "tcxd070iblmat77", RGB24, FBF_TCXD070, args)
#define VD_MIPI_ZWT055AZH(_mode, args...)	VDF_MIPI_ZWT055AZH(_mode, "zwt055azh", RGB24, FBF_ZWT055AZH, args)
#define VD_MIPI_ET055FH06(_mode, args...)	VDF_MIPI_ET055FH06(_mode, "et055fh06", RGB24, FBF_ET055FH06, args)

#define VD_MIPI_MQ_1920_1080M_60(_mode, args...) VDF_1920_1080M_60(_mode, "dsi-mq1920x1080M@60", RGB24, FBF_MIPI_MQ_TO_HDMI, args)
#define VD_MIPI_MQ_1280_800M_60(_mode, args...) VDF_1280_800M_60(_mode, "dsi-mq1280x800M@60", RGB24, FBF_MIPI_MQ_TO_HDMI, args)
#define VD_MIPI_MQ_1280_720M_60(_mode, args...) VDF_1280_720M_60(_mode, "dsi-mq1280x720M@60", RGB24, FBF_MIPI_MQ_TO_HDMI, args)
#define VD_MIPI_MQ_1024_768M_60(_mode, args...) VDF_1024_768M_60(_mode, "dsi-mq1024x768M@60", RGB24, FBF_MIPI_MQ_TO_HDMI, args)
#define VD_MIPI_MQ_800_600MR_60(_mode, args...) VDF_800_600MR_60(_mode, "dsi-mq800x600MR@60", RGB24, FBF_MIPI_MQ_TO_HDMI, args)
#define VD_MIPI_MQ_720_480M_60(_mode, args...)	VDF_720_480M_60(_mode, "dsi-mq720x480M@60", RGB24, FBF_MIPI_MQ_TO_HDMI, args)
#define VD_MIPI_MQ_640_480M_60(_mode, args...)	VDF_640_480M_60(_mode, "dsi-mq640x480M@60", RGB24, FBF_MIPI_MQ2_TO_HDMI, args)
#define VD_CLAA_WVGA(_mode, args...)		VDF_CLAA_WVGA(_mode, "CLAA-WVGA", RGB666, FBF_MODESTR, args)
#define VD_SHARP_WVGA(_mode, args...)		VDF_SHARP_WVGA(_mode, "sharp-wvga", RGB24, FBF_MODESTR, args)
#define VD_TFC_A9700LTWV35TC_C1(_mode, args...)	VDF_TFC_A9700LTWV35TC_C1(_mode, "tfc-a9700ltwv35tc-c1", RGB24, 0, args)
#define VD_800X300_565(_mode, args...)		VDF_800X300(_mode, "800x300rgb565", RGB565, FBF_MODESTR, args)
#define VD_Q035_014(_mode, args...)		VDF_Q035_014(_mode, "q035_014", RGB24, FBF_MQ_Q035_014, args)
#define VD_HITACHI_HVGA(_mode, args...)		VDF_HITACHI_HVGA(_mode, "hitachi_hvga", RGB666, FBF_MODESTR, args)
#define VD_HITACHI_HVGA_5D(_mode, args...)	VDF_HITACHI_HVGA(_mode, "hitachi_hvga-5d", RGB666, FBF_MODESTR, args)
#define VD_HITACHI_HVGA565(_mode, args...)	VDF_HITACHI_HVGA(_mode, "hitachi_hvga565", RGB565, FBF_MODESTR, args)
#define VD_NEON_TOUCH640X240(_mode, args...)	VDF_NEON_TOUCH640X240(_mode, "NeonTouch640x240", RGB565, FBF_MODESTR, args)
#define VD_DC050WX(_mode, args...)		VDF_DC050WX(_mode, "DC050WX", RGB24, FBF_MODESTR, args)
#define VD_INNOLUX_WXGA_14IN_12V(_mode, args...) VDF_INNOLUX_WXGA_14IN_12V(_mode, "INNOLUX-WXGA-IN14-12V", RGB666, 0, args)
#define VD_AUO_WXGA_11IN_12V(_mode, args...)	VDF_AUO_WXGA_11IN_12V(_mode, "AUO-WXGA-IN11-12V", RGB24, 0, args)
#define VD_OSD_WSVGA(_mode, args...)		VDF_OSD_WSVGA(_mode, "OSD-WSVGA", RGB666, 0, args)
#define VD_INNOLUX_WVGA(_mode, args...)		VDF_INNOLUX_WVGA(_mode, "INNOLUX-WVGA", RGB666, 0, args)
#define VD_INNOLUX_WVGA_12V(_mode, args...)	VDF_INNOLUX_WVGA(_mode, "INNOLUX-WVGA-12V", RGB666, 0, args)
#define VD_INNOLUX_WVGA_M(_mode, args...)	VDF_INNOLUX_WVGA(_mode, "INNOLUX-WVGA", RGB666, FBF_MODESTR, args)
#define VD_OKAYA_480_272(_mode, args...)	VDF_OKAYA_480_272(_mode, "okaya_480x272", RGB24, FBF_MODESTR, args)
#define VD_OKAYA_480_272_IPU(_mode, args...)	VDF_OKAYA_480_272_IPU(_mode, "okaya_480x272ipu", RGB24, FBF_MODESTR_IPU, args)
#define VD_DMT050WVNXCMI(_mode, args...)	VDF_DMT050WVNXCMI(_mode, "dmt050wvnxcmi", RGB24, FBF_DMT050WVNXCMI, args)
#define VD_DMT055FHNMCMI(_mode, args...)	VDF_DMT055FHNMCMI(_mode, "dmt055fhnmcmi", RGB24, FBF_DMT055FHNMCMI, args)

#define VD_ER_TFT050_MINI(_mode, args...)	VDF_ER_TFT050_MINI(_mode, "er_tft050", RGB24, FBF_ER_TFT050, args)
#define VD_LCM_JM430(_mode, args...)		VDF_LCM_JM430(_mode, "lcm_jm430", RGB24, FBF_LCM_JM430, args)
#define VD_LCM_JM430_MINI(_mode, args...)	VDF_LCM_JM430_MINI(_mode, "lcm_jm430", RGB24, FBF_LCM_JM430, args)
#define VD_QVGA(_mode, args...)			VDF_QVGA(_mode, "qvga", RGB24, FBF_MODESTR, args)
#define VD_DT035BTFT(_mode, args...)		VDF_DT035BTFT(_mode, "DT035BTFT", BGR24, FBF_MODESTR, args)
#define VD_AT035GT_07ET3(_mode, args...)	VDF_AT035GT_07ET3(_mode, "AT035GT-07ET3", RGB24, FBF_MODESTR, args)
#define VD_PV03505YP54D(_mode, args...)		VDF_PV03505YP54D(_mode, "pv03505yp54d", RGB24, 0, args)
#define VD_AM320240UTMQW(_mode, args...)	VDF_AM320240UTMQW(_mode, "am320240utmqw", RGB24, 0, args)
#define VD_AMP1024_600(_mode, args...)		VDF_AMP1024_600(_mode, "amp1024x600", RGB666, 0, args)
#define VD_ND1024_600(_mode, args...)		VDF_ND1024_600(_mode, "ND-070PCAP-1024x600", RGB24, 0, args)

#define VD_AM_1280800P2TZQW(_mode, args...)	VDF_AM_1280800P2TZQW(_mode, "AM-1280800P2TZQW", RGB24, FBF_BKLIT_DTB, args)
#define VD_TM070JDHG30(_mode, args...)		VDF_TM070JDHG30(_mode, "tm070jdhg30", RGB24, 0, args)
#define VD_TM070JDHG30_14(_mode, args...)	VDF_TM070JDHG30(_mode, "tm070jdhg30-14", RGB24, 0, args)
#define VD_TM070JDHG30_5D(_mode, args...)	VDF_TM070JDHG30(_mode, "tm070jdhg30-5d", RGB24, 0, args)
#define VD_G101EVN01(_mode, args...)		VDF_G101EVN01(_mode, "g101evn01", RGB666, FBF_BKLIT_DTB, args)
#define VD_AUO_B101EW05(_mode, args...)		VDF_AUO_B101EW05(_mode, "auo_b101ew05", RGB666, 0, args)
#define VD_HANNSTAR7(_mode, args...)		VDF_HANNSTAR7(_mode, "hannstar7", RGB666, 0, args)
#define VD_LG1280_800(_mode, args...)		VDF_HANNSTAR7(_mode, "lg1280x800", RGB666, 0, args)
#define VD_HDA800XPT(_mode, args...)		VDF_HDA800XPT(_mode, "hda800xpt", RGB24, FBF_BKLIT_DTB | FBF_BKLIT_LOW_ACTIVE | FBF_BKLIT_EN_LOW_ACTIVE | FBF_BKLIT_EN_DTB, args)
#define VD_M101NWWB(_mode, args...)		VDF_M101NWWB(_mode, "M101NWWB", RGB24, 0, args)
#define VD_M101NWWB_14(_mode, args...)		VDF_M101NWWB(_mode, "M101NWWB-14", RGB24, 0, args)
#define VD_M101NWWB_5D(_mode, args...)		VDF_M101NWWB(_mode, "M101NWWB-5d", RGB24, 0, args)
#define VD_LD101WX1(_mode, args...)		VDF_HANNSTAR7(_mode, "ld101wx1", RGB24, 0, args)
#define VD_DT070BTFT(_mode, args...)		VDF_DT070BTFT(_mode, "dt070btft", RGB24, FBF_JEIDA, args)
#define VD_DT070BTFT_18(_mode, args...)		VDF_DT070BTFT(_mode, "dt070btft_18", RGB666, 0, args)
#define VD_DT070BTFT_18H(_mode, args...)		VDF_DT070BTFT(_mode, "dt070btft_18h", RGB666, FBF_BKLIT_DTB, args)
#define VD_DT070BTFT_18L(_mode, args...)		VDF_DT070BTFT(_mode, "dt070btft_18l", RGB666, FBF_BKLIT_DTB | FBF_BKLIT_LOW_ACTIVE, args)
#define VD_DT070BTFT_24(_mode, args...)		VDF_DT070BTFT(_mode, "dt070btft_24", RGB24, 0, args)
#define VD_DT070BTFT_24H(_mode, args...)		VDF_DT070BTFT(_mode, "dt070btft_24h", RGB24, FBF_BKLIT_DTB, args)
#define VD_DT070BTFT_24L(_mode, args...)		VDF_DT070BTFT(_mode, "dt070btft_24l", RGB24, FBF_BKLIT_DTB | FBF_BKLIT_LOW_ACTIVE, args)
#define VD_PM9598(_mode, args...)		VDF_PM9598(_mode, "pm9598", RGB24, FBF_JEIDA, args)
#define VD_WSVGA(_mode, args...)		VDF_WSVGA(_mode, "wsvga", RGB666, 0, args)
#define VD_ASIT500MA6F5D(_mode, args...)	VDF_ASIT500MA6F5D(_mode, "ASIT500MA6F5D", RGB24, FBF_MODESTR, args)
#define VD_FUSION7(_mode, args...)		VDF_FUSION7(_mode, "fusion7", RGB666, FBF_MODESTR, args)
#define VD_HANNSTAR(_mode, args...)		VDF_HANNSTAR(_mode, "hannstar", RGB666, 0, args)
#define VD_TCG104XGLPAPNN(_mode, args...)	VDF_HANNSTAR(_mode, "tcg104xglpapnn", RGB24, FBF_ALT_PWM, args)
#define VD_1024_600(_mode, args...)		VDF_1024_600(_mode, "1024x600", RGB666, 0, args)
#define VD_AFK1024600A02(_mode, args...)	VDF_AFK1024600A02(_mode, "AFK1024600A02", RGB24, 0, args)
#define VD_LG9_7(_mode, args...)		VDF_LG9_7(_mode, "lg9.7", RGB666, 0, args)
#define VD_1080P60(_mode, args...)		VDF_1080P60(_mode, "1080P60", RGB24, FBF_SPLITMODE, args)
#define VD_1080P60_J(_mode, args...)		VDF_1080P60(_mode, "1080P60_J", RGB24, FBF_SPLITMODE | FBF_JEIDA, args)
#define VD_DV210FBM(_mode, args...)		VDF_DV210FBM(_mode, "dv210fbm", RGB24, FBF_SPLITMODE, args)
#define VD_LTK190L3027T(_mode, args...)		VDF_LTK190L3027T(_mode, "ltk190l3027t", RGB24, FBF_SPLITMODE | FBF_BKLIT_EN_DTB, args)
#define VD_SHARP_LQ101K1LY04(_mode, args...)	VDF_SHARP_LQ101K1LY04(_mode, "sharp-LQ101K1LY04", RGB24, FBF_JEIDA, args)
#define VD_WXGA(_mode, args...)			VDF_WXGA(_mode, "wxga", RGB24, 0, args)
#define VD_WXGA_J(_mode, args...)		VDF_WXGA(_mode, "wxga_j", RGB24, FBF_JEIDA, args)
#define VD_LS050T1SX12(_mode, args...)		VDF_LS050T1SX12(_mode, "ls050t1sx12", RGB24, FBF_LS050T1SX12, args)
#define VD_LTK080A60A004T(_mode, args...)	VDF_LTK080A60A004T(_mode, "ltk080a60a004t", RGB24, FBF_LTK080A60A004T, args)
#define VD_LXD_M8509A(_mode, args...)		VDF_LXD_M8509A(_mode, "lxd_m8509a", RGB24, FBF_LXD_M8509A, args)
#define VD_LTK080A60A004T_2(_mode, args...)	VDF_LTK080A60A004T(_mode, "ltk080a60a004t-2", RGB24, FBF_LTK080A60A004T, args)
#define VD_LTK0680YTMDB(_mode, args...)		VDF_LTK0680YTMDB(_mode, "ltk0680ytmdb", RGB24, FB_##_mode == FB_LCD ? FBF_LTK0680YTMDB_LCD : FBF_LTK0680YTMDB, args)
#define VD_LTK0680YTMDB_2(_mode, args...)	VDF_LTK0680YTMDB_2(_mode, "ltk0680ytmdb-2", RGB24, FB_##_mode == FB_LCD ? FBF_LTK0680YTMDB_LCD : FBF_LTK0680YTMDB, args)
#define VD_LTK069WXBCT02(_mode, args...)	VDF_LTK069WXBCT02(_mode, "ltk069wxbct02", RGB24, FB_##_mode == FB_LCD ? FBF_LTK069WXBCT02_LCD : FBF_LTK069WXBCT02, args)
#define VD_LTK069WXICT10(_mode, args...)	VDF_LTK069WXICT10(_mode, "ltk069wxict10", RGB24, FB_##_mode == FB_LCD ? FBF_LTK069WXICT10_LCD : FBF_LTK069WXICT10, args)
#define VD_LTK069WXICT11(_mode, args...)	VDF_LTK069WXICT11(_mode, "ltk069wxict11", RGB24, FB_##_mode == FB_LCD ? FBF_LTK069WXICT11_LCD : FBF_LTK069WXICT11, args)

#define VD_MIPI_G156HCE_L01(_mode, args...)	VDF_MIPI_G156HCE_L01(_mode, "G156HCE-L01", RGB24, FBF_G156HCE_L01, args)
#define VD_MIPI_COM50H5N03ULC(_mode, args...)	VDF_MIPI_COM50H5N03ULC(_mode, "com50h5n03ulc", RGB24, FBF_COM50H5N03ULC, args)
#define VD_MIPI_M101NWWB_NO_CMDS(_mode, args...) VDF_MIPI_M101NWWB(_mode, "m101nwwb", RGB24, FBF_M101NWWB_NO_CMDS, args)

/* name is a string, b can be B, U, E */
#define VD_MIPI_TM070JDHG30_x(name, b, _mode, args...)  VDF_TM070JDHG30(_mode, name, RGB24, FBF_ ## b ## _TM070JDHG30, args)
/* name is a string, b can be B, U, E */
#define VD_MIPI_M101NWWB_x(name, b, _mode, args...) VDF_MIPI_M101NWWB(_mode, name, RGB24, FBF_ ## b ## _M101NWWB, args)

#define VD_MIPI_AM_TFT1280X800(_mode, args...)	VDF_HANNSTAR7(_mode, "am-tft1280x800", RGB24, FBF_AM_TFT1280X800, args)
#define VD_MIPI_AM_TFT1280X800W(_mode, args...)	VDF_AM_TFT1280X800W(_mode, "am-tft1280x800w", RGB24, FBF_AM_TFT1280X800W, args)
#define VD_MIPI_MTD0900DCP27KF(_mode, args...)	VDF_MIPI_MTD0900DCP27KF(_mode, "mtd0900dcp27kf", RGB24, FBF_MTD0900DCP27KF, args)
#define VD_MIPI_DLC0350GEV06(_mode, args...)	VDF_MIPI_DLC0350GEV06(_mode, "dlc0350gev06", RGB24, FBF_DLC0350GEV06, args)
#define VD_MIPI_COM35H3R04ULY(_mode, args...)	VDF_MIPI_COM35H3R04ULY(_mode, "com35h3r04uly", RGB24, FBF_COM35H3R04ULY, args)
#define VD_MIPI_LCD133_070(_mode, args...)	VDF_MIPI_LCD133_070(_mode, "lcd133_070", RGB24, FBF_LCD133_070, args)
#define VD_MIPI_X090DTLNC01(_mode, args...)	VDF_MIPI_X090DTLNC01(_mode, "x090dtlnc01", RGB24, FBF_B_M101NWWB, args)
#define VD_MIPI_WVGA_TX23D200_24H(_mode, args...) VDF_WVGA_TX23D200(_mode, "tx23d200_24h", RGB24, FBF_MIPI_LT8912, args)
#define VD_MIPI_WVGA_TX23D200_24L(_mode, args...) VDF_WVGA_TX23D200(_mode, "tx23d200_24l", RGB24, FBF_MIPI_LT8912 | FBF_BKLIT_LOW_ACTIVE, args)
#define VD_MIPI_WVGA_TX23D200_18H(_mode, args...) VDF_WVGA_TX23D200(_mode, "tx23d200_18h", RGB666, FBF_MIPI_LT8912, args)
#define VD_MIPI_WVGA_TX23D200_18L(_mode, args...) VDF_WVGA_TX23D200(_mode, "tx23d200_18l", RGB666, FBF_MIPI_LT8912 | FBF_BKLIT_LOW_ACTIVE, args)
#define VD_MIPI_AM_1280800P2TZQW(_mode, args...) VDF_AM_1280800P2TZQW(_mode, "AM-1280800P2TZQW", RGB24, FBF_MIPI_LT8912, args)
#define VD_MIPI_DT070BTFT_18H(_mode, args...)	VDF_DT070BTFT(_mode, "dt070btft_18h", RGB666, FBF_MIPI_LT8912, args)
#define VD_MIPI_DT070BTFT_18L(_mode, args...)	VDF_DT070BTFT(_mode, "dt070btft_18l", RGB666, FBF_MIPI_LT8912 | FBF_BKLIT_LOW_ACTIVE, args)
#define VD_MIPI_DT070BTFT_24H(_mode, args...)	VDF_DT070BTFT(_mode, "dt070btft_24h", RGB24, FBF_MIPI_LT8912, args)
#define VD_MIPI_DT070BTFT_24L(_mode, args...)	VDF_DT070BTFT(_mode, "dt070btft_24l", RGB24, FBF_MIPI_LT8912 | FBF_BKLIT_LOW_ACTIVE, args)

#define VD_LD070WSVGA(_mode, args...)		VDF_LD070WSVGA(_mode, "ld070wsvga", RGB24, 0, args)
#define VD_SVGA(_mode, args...)			VDF_SVGA(_mode, "svga", RGB666, FBF_MODESTR, args)
#define VD_WVGA_TX23D200_24(_mode, args...)	VDF_WVGA_TX23D200(_mode, "tx23d200_24", RGB24, 0, args)
#define VD_WVGA_TX23D200_24H(_mode, args...)	VDF_WVGA_TX23D200(_mode, "tx23d200_24h", RGB24, FBF_BKLIT_DTB, args)
#define VD_WVGA_TX23D200_24L(_mode, args...)	VDF_WVGA_TX23D200(_mode, "tx23d200_24l", RGB24, FBF_BKLIT_DTB | FBF_BKLIT_LOW_ACTIVE, args)
#define VD_WVGA_TX23D200_18(_mode, args...)	VDF_WVGA_TX23D200(_mode, "tx23d200_18", RGB666, 0, args)
#define VD_WVGA_TX23D200_18H(_mode, args...)	VDF_WVGA_TX23D200(_mode, "tx23d200_18h", RGB666, FBF_BKLIT_DTB, args)
#define VD_WVGA_TX23D200_18L(_mode, args...)	VDF_WVGA_TX23D200(_mode, "tx23d200_18l", RGB666, FBF_BKLIT_DTB | FBF_BKLIT_LOW_ACTIVE, args)
#define VD_WVGA(_mode, args...)			VDF_WVGA(_mode, "wvga", RGB666, 0, args)
#define VD_WVGA_J(_mode, args...)		VDF_WVGA(_mode, "wvga_j", RGB24, FBF_JEIDA, args)
#define VD_AA065VE11(_mode, args...)		VDF_AA065VE11(_mode, "AA065VE11", RGB24, 0, args)
#define VD_VGA(_mode, args...)			VDF_VGA(_mode, "vga", RGB24, 0, args)
#define VD_LSA40AT9001(_mode, args...)		VDF_LSA40AT9001(_mode, "LSA40AT9001", RGB24, FBF_MODESTR, args)
#define VD_AUO_G050(_mode, args...)		VDF_AUO_G050(_mode, "AUO_G050", RGB24, FBF_MODESTR | FBF_SPI, args)
#define VD_OSD050T3236(_mode, args...)		VDF_OSD050T3236(_mode, "osd050t3236", RGB24, FBF_OSD050T, args)
#define VD_OSD050T3872(_mode, args...)		VDF_OSD050T3872(_mode, "osd050t3872", RGB24, FBF_OSD050T, args)
#define VD_A030JN01_UPS051(_mode, args...)	VDF_A030JN01_UPS051(_mode, "A030JN01_UPS051", UPS051, FBF_MODESTR | FBF_SPI, args)
#define VD_A030JN01_YUV720(_mode, args...)	VDF_A030JN01_YUV720(_mode, "A030JN01_YUV720", YUYV, FBF_MODESTR | FBF_SPI, args)
#define VD_KD024FM(_mode, args...)		VDF_KD024FM(_mode, "KD024FM", RGB666, FBF_MODESTR, args)

#define VD_HEADER(_mode, _fmt, _flags, _detect, _bus, _addr, args...) \
	.bus	= _bus,\
	.addr	= _addr,\
	.pixfmt	= IPU_PIX_FMT_##_fmt,\
	.detect	= _detect,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = _flags, \
	.enable_alias = {args} \

#define _to_freq(period)	(1000000000000ULL/period)

/* hdmi settings */
#define VDF_1280_800M_60(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= ((1280+8+64+8)*(800+2+39+1)*60),\
		.left_margin    = 8,\
		.right_margin   = 64,\
		.upper_margin   = 2,\
		.lower_margin   = 39,\
		.hsync_len      = 8,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_1280_720M_60(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 720,\
		.pixclock_f	= ((1280+216+72+80)*(720+22+3+5)*60),\
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

#define VDF_1920_1080M_60(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1920,\
		.yres           = 1080,\
		.pixclock_f	= ((1920+148+88+44)*(1080+36+4+5)*60),\
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

#define VDF_MIPI_CS005_0004_03(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock_f	= ((800+30+110+10)*(480+8+8+4)*60),\
		.left_margin    = 30,\
		.right_margin   = 110,\
		.upper_margin   = 8,\
		.lower_margin   = 8,\
		.hsync_len      = 10,\
		.vsync_len      = 4,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_MIPI_VTFT101RPFT20(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= ((1280+16+16+16)*(800+8+2+2)*60),\
		.left_margin    = 16,\
		.right_margin   = 16,\
		.upper_margin   = 8,\
		.lower_margin   = 2,\
		.hsync_len      = 16,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* HX8398 mipi controller */
#define VDF_MIPI_ET055FH06(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1080,\
		.yres           = 1920,\
		.pixclock_f	= 100000000, /* ((1080+30+30+50)*(1920+10+15+10)*60), */ \
		.left_margin    = 30,\
		.right_margin   = 30,\
		.upper_margin   = 10,\
		.lower_margin   = 15,\
		.hsync_len      = 50,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* HX8399C mipi controller */
#define VDF_MIPI_ZWT055AZH(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1088,\
		.yres           = 1920,\
		.pixclock_f	= ((1088+60+90+20)*(1920+3+9+4)*60), \
		.left_margin    = 60,\
		.right_margin   = 90,\
		.upper_margin   = 3,\
		.lower_margin   = 9,\
		.hsync_len      = 20,\
		.vsync_len      = 4,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_MIPI_TCXD070IBLMAT77(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.pwm_period = 500000, \
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 1280,\
		.pixclock_f	= ((800+100+20+33)*(1280+30+20+2)*60),\
		.left_margin	= 100,\
		.right_margin	= 20,\
		.upper_margin	= 30,\
		.lower_margin	= 20,\
		.hsync_len	= 33,\
		.vsync_len	= 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_1024_768M_60(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 768,\
		.pixclock_f	= ((1024+220+40+60)*(768+21+7+10)*60),\
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

#define VDF_800_600MR_60(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 600,\
		.pixclock_f	= ((800+88+40+128)*(600+23+2+3)*60),\
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

#define VDF_640_480M_60(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 640,\
		.yres           = 480,\
		.pixclock_f	= ((640+48+16+96)*(480+33+10+2)*60),\
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

#define VDF_720_480M_60(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 720,\
		.yres           = 480,\
		.pixclock_f	= ((720+60+16+62)*(480+30+9+6)*60),\
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
#define VDF_CLAA_WVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 57,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock_f	= _to_freq(37037),\
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
#define VDF_SHARP_WVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock_f	= ((800+40+40+48)*(480+31+11+3)*60),\
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

#define VDF_TFC_A9700LTWV35TC_C1(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock_f	= ((800+40+40+48)*(480+29+13+3)*60),\
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
#define VDF_800X300(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 800,\
		.yres		= 300,\
		.pixclock_f	= (800+50+1+110)*(300+8+3+1)*60,\
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

#define VDF_Q035_014(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 640,\
		.yres		= 960,\
		.pixclock_f	= 46000000,\
		.left_margin	= 35,\
		.right_margin	= 45,\
		.upper_margin	= 21,\
		.lower_margin	= 16,\
		.hsync_len	= 45,\
		.vsync_len	= 4,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/*
 * hitachi 640x240
 * vsync = 60
 * hsync = 260 * vsync = 15.6 Khz
 * pixclk = 800 * hsync = 12.48 MHz
 */
#define VDF_HITACHI_HVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 640,\
		.yres		= 240,\
		.pixclock_f	= (640+34+1+125)*(240+8+3+9)*60,\
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

#define VDF_NEON_TOUCH640X240(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 640,\
		.yres		= 240,\
		.pixclock_f	= (640+34+1+125)*(240+8+3+9)*60,\
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
#define VDF_DC050WX(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock_f	= _to_freq(33898),\
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
#define VDF_INNOLUX_WXGA_14IN_12V(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 1366,\
		.yres		= 768,\
		.pixclock_f	= (1366+108+108+10)*(768+8+8+16)*60,\
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
#define VDF_AUO_WXGA_11IN_12V(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 1366,\
		.yres		= 768,\
		.pixclock_f	= (1366+67+67+100)*(768+10+10+6)*60,\
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
#define VDF_OSD_WSVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 1024,\
		.yres		= 600,\
		.pixclock_f	= (1024+45+210+1)*(600+22+132+1)*60,\
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
#define VDF_INNOLUX_WVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 800,\
		.yres		= 480,\
		.pixclock_f	= (800+45+16+1)*(480+22+125+1)*60,\
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

#define VDF_LB043(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 57,\
		.xres           = 480,\
		.yres           = 800,\
		.pixclock_f	= _to_freq(37037),\
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

#define VDF_LXD_M8509A(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 480,\
		.yres           = 800,\
		.pixclock_f	= 28875000, /*((480+60+10+20)*(800+10+10+10)*60),*/\
		.left_margin    = 40,\
		.right_margin   = 60,\
		.upper_margin   = 10,\
		.lower_margin   = 10,\
		.hsync_len      = 20,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_OKAYA_480_272(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 57,\
		.xres		= 480,\
		.yres		= 272,\
		.pixclock_f	= _to_freq(97786),\
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

#define VDF_OKAYA_480_272_IPU(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 57,\
		.xres		= 480,\
		.yres		= 272,\
		.pixclock_f	= _to_freq(97786),\
		.left_margin	= 2,\
		.right_margin	= 1,\
		.upper_margin	= 3,\
		.lower_margin	= 2,\
		.hsync_len	= 41,\
		.vsync_len	= 10,\
		.sync		= FB_SYNC_CLK_LAT_FALL,\
		.vmode		= FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_LCM_JM430(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 480,\
		.yres		= 272,\
		.pixclock_f	= ((480+40+4+4)*(272+8+8+1)*60),\
		.left_margin	= 40,\
		.right_margin	= 4,\
		.upper_margin	= 8,\
		.lower_margin	= 8,\
		.hsync_len	= 4,\
		.vsync_len	= 1,\
		.sync		= FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode		= FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_DMT050WVNXCMI(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 480,\
		.yres		= 854,\
		.pixclock_f	= 32000000,\
		.left_margin	= 8,\
		.right_margin	= 64,\
		.upper_margin	= 20,\
		.lower_margin	= 20,\
		.hsync_len	= 8,\
		.vsync_len	= 1,\
		.sync		= FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode		= FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_DMT055FHNMCMI(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 1080,\
		.yres		= 1920,\
		.pixclock_f	= ((1080+16+16+8)*(1920+20+10+8)*60),\
		.left_margin	= 16,\
		.right_margin	= 16,\
		.upper_margin	= 20,\
		.lower_margin	= 10,\
		.hsync_len	= 8,\
		.vsync_len	= 8,\
		.sync		= FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode		= FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_ER_TFT050_MINI(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 800,\
		.yres		= 480,\
		.pixclock_f	= ((800+46+158+40)*(480+23+22+1)*60),\
		.left_margin	= 46,\
		.right_margin	= 158,\
		.upper_margin	= 23,\
		.lower_margin	= 22,\
		.hsync_len	= 40,\
		.vsync_len	= 1,\
		.sync		= FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode		= FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_LCM_JM430_MINI(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 480,\
		.yres		= 272,\
		.pixclock_f	= 13000000,\
		.left_margin	= 40,\
		.right_margin	= 4,\
		.upper_margin	= 8,\
		.lower_margin	= 8,\
		.hsync_len	= 4,\
		.vsync_len	= 1,\
		.sync		= FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode		= FB_VMODE_NONINTERLACED\
	}\
}

/* tsc2004 */
#define VDF_QVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 320,\
		.yres           = 240,\
		.pixclock_f	= ((320+38+37+30)*(240+16+15+3)*60),\
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

#define VDF_SPI_QVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 320,\
		.yres           = 240,\
		.pixclock_f	= ((320+16+20+52)*(240+16+4+2)*60),\
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
#define VDF_DT035BTFT(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 320,\
		.yres           = 240,\
		.pixclock_f	= ((320+40+18+30)*(240+10+9+3)*60),\
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

#define VDF_AT035GT_07ET3(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 320,\
		.yres           = 240,\
		.pixclock_f	= ((320+40+18+30)*(240+10+9+3)*60),\
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

/* ft5446 touch screen, Kingtech display */
#define VDF_PV03505YP54D(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 320,\
		.yres           = 240,\
		.pixclock_f	= ((320+64+20+4)*(240+16+4+2)*60),\
		.left_margin    = 64,\
		.right_margin   = 20,\
		.upper_margin   = 16,\
		.lower_margin   = 4,\
		.hsync_len      = 4,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_AM320240UTMQW(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 320,\
		.yres           = 240,\
		.pixclock_f	= ((320+69+18+1)*(240+12+10+1)*60),\
		.left_margin    = 69,\
		.right_margin   = 18,\
		.upper_margin   = 12,\
		.lower_margin   = 10,\
		.hsync_len      = 1,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ili210x touch screen */
#define VDF_AMP1024_600(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock_f	= ((1024+220+40+60)*(600+21+7+10)*60),\
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
#define VDF_ND1024_600(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock_f	= ((1024+160+80+80)*(600+19+8+8)*60),\
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

#define VDF_AM_1280800P2TZQW(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.pwm_period = 100000, \
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= ((1280+5+64+1)*(800+2+40+1)*60),\
		.left_margin    = 5,\
		.right_margin   = 64,\
		.upper_margin   = 2,\
		.lower_margin   = 40,\
		.hsync_len      = 1,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ft5x06 touch screen */
/* Tianma panel TM070JDHG30 is a 24 bit spwg panel */
/*		freq		hbp		hfp		hs		hb		vbp		vfp		vs		vb
 * TM070JDHG30	62.6:68.2:78.1	5		15:64:159	2:256		21:70:165	2:2:101-vfp	3:40:99		1:1:128		6:43:229
 * M101GWWF	70.0:72.4:76.6							130:160:190							28:38:68
 * M101NWWB	68.9:71.1:73.4							60:160:190							15:23:33
 * common	70.0:71.7:73.4	5		151		4		130:160:165	2		28		1		28:31:33
 */
#define VDF_TM070JDHG30(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.pwm_period = 32000, \
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= 71700000,\
		.left_margin    = 5,\
		.right_margin   = 151,\
		.upper_margin   = 2,\
		.lower_margin   = 28,\
		.hsync_len      = 4,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* Tianma panel TM070JDHG30 is a 24 bit spwg panel */
#define VDF_MIPI_TM070JDHG30(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.pwm_period = 32000, \
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= 74250000,\
		.left_margin    = 5,\
		.right_margin   = 67,\
		.upper_margin   = 2,\
		.lower_margin   = 39,\
		.hsync_len      = 12,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_G101EVN01(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= ((1280+64+48+16)*(800+8+6+2)*60),\
		.left_margin    = 64,\
		.right_margin   = 48,\
		.upper_margin   = 8,\
		.lower_margin   = 6,\
		.hsync_len      = 16,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_AUO_B101EW05(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= ((1280+48+48+32)*(800+8+2+6)*60),\
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

#define VDF_AM_TFT1280X800W(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= ((1280+80+48+32)*(800+15+2+21)*60),\
		.left_margin    = 80,\
		.right_margin   = 48,\
		.upper_margin   = 15,\
		.lower_margin   = 2,\
		.hsync_len      = 32,\
		.vsync_len      = 21,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ft5x06_ts */
/* lg1280x800(LP101WX1) == hannstar7 */
/* LG panel LD101WX1 is a 24 bit spwg panel */
#define VDF_HANNSTAR7(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= ((1280+80+48+32)*(800+15+2+6)*60),\
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
#define VDF_DT070BTFT(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock_f	= ((1024+220+40+60)*(600+21+4+10)*60),\
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

#define VDF_PM9598(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock_f	= ((1024+140+160+20)*(600+20+12+3)*60),\
		.left_margin    = 140,\
		.right_margin   = 160,\
		.upper_margin   = 20,\
		.lower_margin   = 12,\
		.hsync_len      = 20,\
		.vsync_len      = 3,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* ft5x06_ts */
#define VDF_WSVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock_f	= ((1024+220+40+60)*(600+21+7+10)*60),\
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
#define VDF_ASIT500MA6F5D(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.pwm_period = 100000, \
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock_f	= (800+88+40+48)*(480+32+13+3)*60,\
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
#define VDF_FUSION7(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock_f	= ((800+96+24+136)*(480+31+11+3)*60),\
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
#define VDF_HANNSTAR(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 768,\
		.pixclock_f	= ((1024+220+40+60)*(768+21+7+10)*60),\
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

#define VDF_HDA800XPT(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.pwm_period = 32000, 	/* 31.25 KHz, 1000000000/32000 ns */ \
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 768,\
		.pixclock_f	= ((1024+160+80+80)*(768+22+8+8)*60),\
		.left_margin    = 160,\
		.right_margin   = 80,\
		.upper_margin   = 22,\
		.lower_margin   = 8,\
		.hsync_len      = 80,\
		.vsync_len      = 8,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_1024_600(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock_f	= _to_freq(20408),\
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

#define VDF_AFK1024600A02(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock_f	= ((1024+160+80+80)*(600+19+8+8)*60),\
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
#define VDF_LG9_7(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 768,\
		.pixclock_f	= ((1024+480+260+250)*(768+16+6+10)*60),\
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

#define VDF_1080P60(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1920,\
		.yres           = 1080,\
		.pixclock_f	= ((1920+148+88+44)*(1080+36+4+5)*60),\
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

#define VDF_DV210FBM(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1920,\
		.yres           = 1080, /* really 132 */\
		.pixclock_f	= ((1920+120+120+40)*(1080+22+22+1)*60),\
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

#define VDF_LTK190L3027T(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 1024, \
		.pixclock_f	= 108000000 /* ((1280+200+200+8)*(1024+16+16+8)*60) */, \
		.left_margin    = 200,\
		.right_margin   = 200,\
		.upper_margin   = 16,\
		.lower_margin   = 16,\
		.hsync_len      = 8,\
		.vsync_len      = 8,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_MIPI_G156HCE_L01(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 50,\
		.xres           = 1920,\
		.yres           = 1080, \
		.pixclock_f	= ((1920+16+192+16)*(1080+3+26+1)*50),\
		.left_margin    = 16,\
		.right_margin   = 192,\
		.upper_margin   = 3,\
		.lower_margin   = 26,\
		.hsync_len      = 16,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_SHARP_LQ101K1LY04(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= ((1280+20+20+10)*(800+4+4+4)*60),\
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

#define VDF_WXGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= ((1280+40+40+10)*(800+3+80+10)*60),\
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

#define VDF_LS050T1SX12(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1088,\
		.yres           = 1920,\
		.pixclock_f	= ((1088+50+102+10)*(1920+4+4+2)*60), \
		.left_margin    = 50,\
		.right_margin   = 102,\
		.upper_margin   = 4,\
		.lower_margin   = 4,\
		.hsync_len      = 10,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_LTK0680YTMDB(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 480,\
		.yres           = 1280,\
		.pixclock_f	= ((480+160+160+24)*(1280+10+12+2)*60), \
		.left_margin    = 160,\
		.right_margin   = 160,\
		.upper_margin   = 10,\
		.lower_margin   = 12,\
		.hsync_len      = 24,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_LTK0680YTMDB_2(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 480,\
		.yres           = 1280,\
		.pixclock_f	= ((480+160+160+24)*(1280+10+12+2)*70), \
		.left_margin    = 160,\
		.right_margin   = 160,\
		.upper_margin   = 10,\
		.lower_margin   = 12,\
		.hsync_len      = 24,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* really 480, not 600 because 60 columns of black pixels on left/right of each line */
#define VDF_LTK069WXBCT02(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 600,\
		.yres           = 1280,\
		.pixclock_f	= ((600+50+50+4)*(1280+16+16+4)*60), \
		.left_margin    = 50,\
		.right_margin   = 50,\
		.upper_margin   = 16,\
		.lower_margin   = 16,\
		.hsync_len      = 4,\
		.vsync_len      = 4,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_LTK069WXICT10(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 480,\
		.yres		= 1280,\
		.pixclock_f	= ((480+50+50+4)*(1280+16+16+4)*60), \
		.left_margin	= 48,\
		.right_margin	= 48,\
		.upper_margin	= 16,\
		.lower_margin	= 16,\
		.hsync_len	= 4,\
		.vsync_len	= 4,\
		.sync		= FB_SYNC_EXT,\
		.vmode		= FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_LTK069WXICT11(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 480,\
		.yres		= 1280,\
		.pixclock_f	= ((480+28+24+4)*(1280+8+16+2)*60), \
		.left_margin	= 28,\
		.right_margin	= 24,\
		.upper_margin	= 8,\
		.lower_margin	= 16,\
		.hsync_len	= 4,\
		.vsync_len	= 2,\
		.sync		= FB_SYNC_EXT,\
		.vmode		= FB_VMODE_NONINTERLACED\
	}\
}

#if 1
/* 640x960 */
#define VDF_MIPI_DLC0350GEV06(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 640,\
		.yres		= 960,\
		.pixclock_f	= (640+80+80+12)*(960+30+20+1)*60,\
		.left_margin	= 80,\
		.right_margin	= 80,\
		.upper_margin	= 30,\
		.lower_margin	= 20,\
		.hsync_len	= 12,\
		.vsync_len	= 1,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#else
#define VDF_MIPI_DLC0350GEV06(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 720,\
		.yres		= 1280,\
		.pixclock_f	= (720+100+100+33)*(1280+30+20+2)*60,\
		.left_margin	= 100,\
		.right_margin	= 100,\
		.upper_margin	= 30,\
		.lower_margin	= 20,\
		.hsync_len	= 33,\
		.vsync_len	= 2,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}
#endif

/* COM35H3R04ULY 640x960 */
#define VDF_MIPI_COM35H3R04ULY(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name		= _name,\
		.refresh	= 60,\
		.xres		= 640,\
		.yres		= 960,\
		.pixclock_f	= (640+32+28+36)*(960+14+8+2)*60,\
		.left_margin	= 32,\
		.right_margin	= 28,\
		.upper_margin	= 14,\
		.lower_margin	= 8,\
		.hsync_len	= 36,\
		.vsync_len	= 2,\
		.sync           = FB_SYNC_EXT | FB_SYNC_CLK_LAT_FALL,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_MIPI_LCD133_070(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1200,\
		.yres           = 1920,\
		.pixclock_f	= ((1200+104+73+8)*(1920+108+74+1)*60), \
		.left_margin    = 104,\
		.right_margin   = 73,\
		.upper_margin   = 108,\
		.lower_margin   = 74,\
		.hsync_len      = 8,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_LTK080A60A004T(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1200,\
		.yres           = 1920,\
		.pixclock_f	= 120000000, /*((1200+60+42+2)*(1920+25+35+1)*60), */\
		.left_margin    = 60,\
		.right_margin   = 42,\
		.upper_margin   = 25,\
		.lower_margin   = 35,\
		.hsync_len      = 2,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_MIPI_COM50H5N03ULC(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 720,\
		.yres           = 1280,\
		.pixclock_f	= ((720+70+91+15)*(1280+3+6+3)*60),\
		.left_margin    = 70,\
		.right_margin   = 91,\
		.upper_margin   = 3,\
		.lower_margin   = 6,\
		.hsync_len      = 15,\
		.vsync_len      = 3,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_M101NWWB(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= 74250000,\
		.left_margin    = 5,\
		.right_margin   = 123,\
		.upper_margin   = 3,\
		.lower_margin   = 24,\
		.hsync_len      = 2,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/*		freq		hbp		hfp		hs		hb		vbp		vfp		vs		vb
 * M101GWWF	70.0:72.4:76.6							130:160:190							28:38:68
 * M101NWWB	68.9:71.1:73.4							60:160:190							15:23:33
 * common	70.0:71.7:73.4	8		144		8		130:160:165	2		28		1		28:31:33
 */
#define VDF_MIPI_M101NWWB(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 800,\
		.pixclock_f	= 71700000,\
		.left_margin    = 8,\
		.right_margin   = 144,\
		.upper_margin   = 2,\
		.lower_margin   = 28,\
		.hsync_len      = 8,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_MIPI_MTD0900DCP27KF(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 720,\
		.pixclock_f	= ((1280+40+20+4)*(720+12+5+1)*60),\
		.left_margin    = 40,\
		.right_margin   = 20,\
		.upper_margin   = 12,\
		.lower_margin   = 5,\
		.hsync_len      = 4,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_MIPI_X090DTLNC01(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 720,\
		.pixclock_f	= 66000000,\
		.left_margin    = 40,\
		.right_margin   = 20,\
		.upper_margin   = 12,\
		.lower_margin   = 5,\
		.hsync_len      = 4,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_LD070WSVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 55,\
		.xres           = 1024,\
		.yres           = 600,\
		.pixclock_f	= ((1024+160+160+10)*(600+23+12+3)*55),\
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

#define VDF_SVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 600,\
		.pixclock_f	= _to_freq(15385),\
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
#define VDF_WVGA_TX23D200(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock_f	= ((800+220+18+18)*(480+21+14+10)*52),\
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

#define VDF_WVGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock_f	= ((800+220+40+60)*(480+21+7+10)*60),\
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

#define VDF_AA065VE11(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 70,\
		.xres           = 640,\
		.yres           = 480,\
		.pixclock_f	= _to_freq(22858),	/* 23000 works 23100 doesn't */\
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

#define VDF_VGA(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 640,\
		.yres           = 480,\
		.pixclock_f	= ((640+48+16+96)*(480+33+10+2)*60),\
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

#define VDF_LSA40AT9001(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 600,\
		.pixclock_f	= ((800+46+210+10)*(600+23+12+1)*60),\
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
#define VDF_AUO_G050(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 480,\
		.yres           = 800,\
		.pixclock_f	= ((480+18+16+2)*(800+18+16+2)*60),\
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
#define VDF_A030JN01_UPS051(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 640,\
		.yres           = 480,\
		.pixclock_f	= ((960+40+48+20)*(480+27+18+1)*60),\
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

#define VDF_OSD050T3236(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 52,\
		.xres           = 720,\
		.yres           = 1280,\
		.pixclock_f	= ((720+31+16+1)*(1280+15+8+1)*52),\
		.left_margin    = 31,\
		.right_margin   = 16,\
		.upper_margin   = 15,\
		.lower_margin   = 8,\
		.hsync_len      = 1,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define VDF_OSD050T3872(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode	= {\
		.name           = _name,\
		.refresh        = 52,\
		.xres           = 720,\
		.yres           = 1280,\
		.pixclock_f	= ((720+31+16+1)*(1280+15+8+1)*52),\
		.left_margin    = 31,\
		.right_margin   = 16,\
		.upper_margin   = 15,\
		.lower_margin   = 8,\
		.hsync_len      = 1,\
		.vsync_len      = 1,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* 27.11 MHz pixel clock */
#define VDF_A030JN01_YUV720(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.mode   = {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 720,\
		.yres           = 480,\
		.pixclock_f	= ((720+40+98+1)*(480+27+18+1)*60),\
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

#define VDF_KD024FM(_mode, _name, _fmt, _flags, args...) \
{\
	VD_HEADER(_mode, _fmt, _flags, args),\
	.pre_enable = board_pre_enable, \
	.pwm_period = 100000, \
	.mode   = {\
		.name           = _name,\
		.refresh        = 60,\
		.xres           = 240,\
		.yres           = 320,\
		.pixclock_f	= ((240+10+38+10)*(320+4+8+4)*60),\
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
