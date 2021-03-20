/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 */

#ifndef __DW_HDMI__
#define __DW_HDMI__


struct dw_hdmi;

enum v4l2_ycbcr_encoding {
	/*
	 * Mapping of V4L2_YCBCR_ENC_DEFAULT to actual encodings for the
	 * various colorspaces:
	 *
	 * V4L2_COLORSPACE_SMPTE170M, V4L2_COLORSPACE_470_SYSTEM_M,
	 * V4L2_COLORSPACE_470_SYSTEM_BG, V4L2_COLORSPACE_SRGB,
	 * V4L2_COLORSPACE_OPRGB and V4L2_COLORSPACE_JPEG: V4L2_YCBCR_ENC_601
	 *
	 * V4L2_COLORSPACE_REC709 and V4L2_COLORSPACE_DCI_P3: V4L2_YCBCR_ENC_709
	 *
	 * V4L2_COLORSPACE_BT2020: V4L2_YCBCR_ENC_BT2020
	 *
	 * V4L2_COLORSPACE_SMPTE240M: V4L2_YCBCR_ENC_SMPTE240M
	 */
	V4L2_YCBCR_ENC_DEFAULT		= 0,

	/* ITU-R 601 -- SDTV */
	V4L2_YCBCR_ENC_601		= 1,

	/* Rec. 709 -- HDTV */
	V4L2_YCBCR_ENC_709		= 2,

	/* ITU-R 601/EN 61966-2-4 Extended Gamut -- SDTV */
	V4L2_YCBCR_ENC_XV601		= 3,

	/* Rec. 709/EN 61966-2-4 Extended Gamut -- HDTV */
	V4L2_YCBCR_ENC_XV709		= 4,

	/* BT.2020 Non-constant Luminance Y'CbCr */
	V4L2_YCBCR_ENC_BT2020		= 6,

	/* BT.2020 Constant Luminance Y'CbcCrc */
	V4L2_YCBCR_ENC_BT2020_CONST_LUM = 7,

	/* SMPTE 240M -- Obsolete HDTV */
	V4L2_YCBCR_ENC_SMPTE240M	= 8,

	/* KR=0.30, KB=0.11 or equivalent */
	V4L2_YCBCR_ENC_BT470_6M		= 9,
};

/**
 * DOC: Supported input formats and encodings
 *
 * Depending on the Hardware configuration of the Controller IP, it supports
 * a subset of the following input formats and encodings on its internal
 * 48bit bus.
 *
 * +----------------------+----------------------------------+------------------------------+
 * | Format Name          | Format Code                      | Encodings                    |
 * +----------------------+----------------------------------+------------------------------+
 * | RGB 4:4:4 8bit       | ``MEDIA_BUS_FMT_RGB888_1X24``    | ``V4L2_YCBCR_ENC_DEFAULT``   |
 * +----------------------+----------------------------------+------------------------------+
 * | RGB 4:4:4 10bits     | ``MEDIA_BUS_FMT_RGB101010_1X30`` | ``V4L2_YCBCR_ENC_DEFAULT``   |
 * +----------------------+----------------------------------+------------------------------+
 * | RGB 4:4:4 12bits     | ``MEDIA_BUS_FMT_RGB121212_1X36`` | ``V4L2_YCBCR_ENC_DEFAULT``   |
 * +----------------------+----------------------------------+------------------------------+
 * | RGB 4:4:4 16bits     | ``MEDIA_BUS_FMT_RGB161616_1X48`` | ``V4L2_YCBCR_ENC_DEFAULT``   |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:4:4 8bit     | ``MEDIA_BUS_FMT_YUV8_1X24``      | ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_XV601``  |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_XV709``  |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:4:4 10bits   | ``MEDIA_BUS_FMT_YUV10_1X30``     | ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_XV601``  |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_XV709``  |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:4:4 12bits   | ``MEDIA_BUS_FMT_YUV12_1X36``     | ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_XV601``  |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_XV709``  |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:4:4 16bits   | ``MEDIA_BUS_FMT_YUV16_1X48``     | ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_XV601``  |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_XV709``  |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:2:2 8bit     | ``MEDIA_BUS_FMT_UYVY8_1X16``     | ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:2:2 10bits   | ``MEDIA_BUS_FMT_UYVY10_1X20``    | ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:2:2 12bits   | ``MEDIA_BUS_FMT_UYVY12_1X24``    | ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:2:0 8bit     | ``MEDIA_BUS_FMT_UYYVYY8_0_5X24`` | ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:2:0 10bits   | ``MEDIA_BUS_FMT_UYYVYY10_0_5X30``| ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:2:0 12bits   | ``MEDIA_BUS_FMT_UYYVYY12_0_5X36``| ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * +----------------------+----------------------------------+------------------------------+
 * | YCbCr 4:2:0 16bits   | ``MEDIA_BUS_FMT_UYYVYY16_0_5X48``| ``V4L2_YCBCR_ENC_601``       |
 * |                      |                                  | or ``V4L2_YCBCR_ENC_709``    |
 * +----------------------+----------------------------------+------------------------------+
 */

enum {
	DW_HDMI_RES_8,
	DW_HDMI_RES_10,
	DW_HDMI_RES_12,
	DW_HDMI_RES_MAX,
};

enum dw_hdmi_phy_type {
	DW_HDMI_PHY_DWC_HDMI_TX_PHY = 0x00,
	DW_HDMI_PHY_DWC_MHL_PHY_HEAC = 0xb2,
	DW_HDMI_PHY_DWC_MHL_PHY = 0xc2,
	DW_HDMI_PHY_DWC_HDMI_3D_TX_PHY_HEAC = 0xe2,
	DW_HDMI_PHY_DWC_HDMI_3D_TX_PHY = 0xf2,
	DW_HDMI_PHY_DWC_HDMI20_TX_PHY = 0xf3,
	DW_HDMI_PHY_VENDOR_PHY = 0xfe,
};

struct dw_hdmi_mpll_config {
	unsigned long mpixelclock;
	struct {
		u16 cpce;
		u16 gmp;
	} res[DW_HDMI_RES_MAX];
};

struct dw_hdmi_curr_ctrl {
	unsigned long mpixelclock;
	u16 curr[DW_HDMI_RES_MAX];
};

struct dw_hdmi_phy_config {
	unsigned long mpixelclock;
	u16 sym_ctr;    /*clock symbol and transmitter control*/
	u16 term;       /*transmission termination value*/
	u16 vlev_ctr;   /* voltage level control */
};

struct dw_hdmi_phy_ops {
	int (*init)(struct dw_hdmi *hdmi, void *data, const struct display_timing *mode);
	void (*disable)(struct dw_hdmi *hdmi, void *data);
	int (*read_hpd)(struct dw_hdmi *hdmi, void *data);
	void (*update_hpd)(struct dw_hdmi *hdmi, void *data,
			   bool force, bool disabled, bool rxsense);
	void (*setup_hpd)(struct dw_hdmi *hdmi, void *data);
	void (*enable_audio)(struct dw_hdmi *hdmi, void *data, int channel,
			     int width, int rate, int non_pcm);
	void (*disable_audio)(struct dw_hdmi *hdmi, void *data);
};

struct dw_hdmi_plat_data {
	struct regmap *regm;
	unsigned long input_bus_format;
	unsigned long input_bus_encoding;

	/* Vendor PHY support */
	const struct dw_hdmi_phy_ops *phy_ops;
	const char *phy_name;
	void *phy_data;
	unsigned int phy_force_vendor;
	unsigned int mix_setup;
	u32	phy_min_clock;
	u32	phy_max_clock;

	/* Synopsys PHY support */
	const struct dw_hdmi_mpll_config *mpll_cfg;
	const struct dw_hdmi_curr_ctrl *cur_ctr;
	const struct dw_hdmi_phy_config *phy_config;
	int (*configure_phy)(struct dw_hdmi *hdmi,
			     const struct dw_hdmi_plat_data *pdata,
			     unsigned long mpixelclock);
};

struct dw_hdmi *dw_hdmi_probe(struct udevice *dev, const struct dw_hdmi_plat_data *plat_data);
void dw_hdmi_remove(struct dw_hdmi *hdmi);
void dw_hdmi_unbind(struct dw_hdmi *hdmi);
struct dw_hdmi *dw_hdmi_bind(struct udevice *dev, const struct dw_hdmi_plat_data *pdata);

void dw_hdmi_resume(struct dw_hdmi *hdmi);

void dw_hdmi_setup_rx_sense(struct dw_hdmi *hdmi, bool hpd, bool rx_sense);

void dw_hdmi_set_sample_non_pcm(struct dw_hdmi *hdmi, unsigned int non_pcm);
void dw_hdmi_set_sample_width(struct dw_hdmi *hdmi, unsigned int width);
void dw_hdmi_set_sample_rate(struct dw_hdmi *hdmi, unsigned int rate);
void dw_hdmi_set_channel_count(struct dw_hdmi *hdmi, unsigned int cnt);
void dw_hdmi_set_channel_allocation(struct dw_hdmi *hdmi, unsigned int ca);
void dw_hdmi_audio_enable(struct dw_hdmi *hdmi);
void dw_hdmi_audio_disable(struct dw_hdmi *hdmi);
void dw_hdmi_set_high_tmds_clock_ratio(struct dw_hdmi *hdmi);

/* PHY configuration */
void dw_hdmi_phy_i2c_set_addr(struct dw_hdmi *hdmi, u8 address);
void dw_hdmi_phy_i2c_write(struct dw_hdmi *hdmi, unsigned short data,
			   unsigned char addr);

void dw_hdmi_phy_reset(struct dw_hdmi *hdmi);

void dw_hdmi_phy_gen2_pddq(struct dw_hdmi *hdmi, u8 enable);
void dw_hdmi_phy_gen2_txpwron(struct dw_hdmi *hdmi, u8 enable);
void dw_hdmi_phy_gen2_reset(struct dw_hdmi *hdmi);

struct udevice* dw_hdmi_get_dev(struct dw_hdmi *hdmi);
int dw_hdmi_phy_read_hpd(struct dw_hdmi *hdmi, void *data);
void dw_hdmi_phy_update_hpd(struct dw_hdmi *hdmi, void *data,
			    bool force, bool disabled, bool rxsense);
void dw_hdmi_phy_setup_hpd(struct dw_hdmi *hdmi, void *data);
void dw_hdmi_bridge_enable(struct dw_hdmi *hdmi);
void dw_hdmi_bridge_disable(struct dw_hdmi *hdmi);


#endif /* __IMX_HDMI_H__ */
