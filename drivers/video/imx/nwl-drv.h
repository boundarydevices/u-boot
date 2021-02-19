/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */

#ifndef __NWL_DRV_H__
#define __NWL_DRV_H__

struct nwl_dsi_platform_data;

/* i.MX8 NWL quirks */
/* i.MX8MQ errata E11418 */
#define E11418_HS_MODE_QUIRK	    BIT(0)
/* Skip DSI bits in SRC on disable to avoid blank display on enable */
#define SRC_RESET_QUIRK		    BIT(1)

/* * DPI color coding */
#define NWL_DSI_DPI_16_BIT_565_PACKED	0
#define NWL_DSI_DPI_16_BIT_565_ALIGNED	1
#define NWL_DSI_DPI_16_BIT_565_SHIFTED	2
#define NWL_DSI_DPI_18_BIT_PACKED	3
#define NWL_DSI_DPI_18_BIT_ALIGNED	4
#define NWL_DSI_DPI_24_BIT		5


#define NWL_DSI_MAX_PLATFORM_CLOCKS 2
struct nwl_dsi_plat_clk_config {
	const char *id;
	struct clk *clk;
	bool present;
};

struct mode_config {
	unsigned			clock;
	int				crtc_clock;
	unsigned int			lanes;
	unsigned long			bitclock;
	unsigned long			phy_ref_rate;
	struct list_head		list;
};

struct nwl_dsi {
	struct mipi_dsi_host dsi_host;
	struct udevice *dev;
	union phy_configure_opts phy_cfg;
	unsigned int quirks;
	unsigned int instance;

	struct regmap *regmap;
	struct regmap *csr;
	int irq;
	struct reset_ctl *rst_byte;
	struct reset_ctl *rst_esc;
	struct reset_ctl *rst_dpi;
	struct reset_ctl *rst_pclk;
	struct mux_control *mux;

	/* DSI clocks */
	struct clk *phy_ref_clk;
	unsigned long phy_ref_rate;
	unsigned long phy_ref_enabled;
	struct clk *rx_esc_clk;
	struct clk *tx_esc_clk;
	struct clk *pll_clk;
	struct clk *lcdif_clk;
	/* Platform dependent clocks */
	struct nwl_dsi_plat_clk_config clk_config[NWL_DSI_MAX_PLATFORM_CLOCKS];

	struct list_head valid_modes;
	/* dsi lanes */
	u32 lanes;
	u32 clk_drop_lvl;
	enum mipi_dsi_pixel_format format;
	unsigned long dsi_mode_flags;

	struct mipi_dsi_device *device;
	struct display_timing timings;
	uint32_t max_data_lanes;

	struct nwl_dsi_transfer *xfer;

	const struct nwl_dsi_platform_data *pdata;
	u32 hsmult;
	u32 bitclk;
	u32 pixclock;
	u32 mipi_dsi_multiple;
	u32 min_hs_clock_multiple;

	ulong crtc_clock;
	bool use_dcss;
	struct phy phy;
};

#endif /* __NWL_DRV_H__ */
