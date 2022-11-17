// SPDX-License-Identifier: GPL-2.0+
/*
 * i.MX8 NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/io.h>
#include <clk.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/devres.h>
#include <dsi_host.h>
#include <div64.h>
#include <generic-phy.h>

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/string.h>
#include <malloc.h>
#include <mux.h>

#include <panel.h>
#include <phy-mipi-dphy.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <video_bridge.h>
#include <video_link.h>

#include "nwl-drv.h"
#include "nwl-dsi.h"

/* Possible platform specific clocks */
#define NWL_DSI_CLK_CORE	"core"
#define NWL_DSI_CLK_BYPASS	"bypass"
#define NWL_DSI_CLK_PIXEL	"pixel"

/* Maximum Video PLL frequency */
#define MAX_PLL_FREQ 1200000000

#define MBPS(x) ((x) * 1000000)
#define MIN_PHY_RATE MBPS(24)
#define MAX_PHY_RATE MBPS(30)

#define DC_ID(x)	IMX_SC_R_DC_ ## x
#define MIPI_ID(x)	IMX_SC_R_MIPI_ ## x
#define SYNC_CTRL(x)	IMX_SC_C_SYNC_CTRL ## x
#define PXL_VLD(x)	IMX_SC_C_PXL_LINK_MST ## x ## _VLD
#define PXL_ADDR(x)	IMX_SC_C_PXL_LINK_MST ## x ## _ADDR

#define IMX8ULP_DSI_CM_MASK	BIT(1)
#define IMX8ULP_DSI_CM_NORMAL	BIT(1)

enum nwl_dsi_ext_regs {
       NWL_DSI_IMX_REG_GPR = BIT(1),
};

struct nwl_dsi_platform_data {
	int (*pclk_reset)(struct nwl_dsi *dsi, bool reset);
	int (*mipi_reset)(struct nwl_dsi *dsi, bool reset);
	int (*dpi_reset)(struct nwl_dsi *dsi, bool reset);
	int (*select_input)(struct nwl_dsi *dsi);
	int (*deselect_input)(struct nwl_dsi *dsi);
	struct nwl_dsi_plat_clk_config clk_config[NWL_DSI_MAX_PLATFORM_CLOCKS];
	u32 reg_tx_ulps;
	u32 reg_pxl2dpi;
	u32 reg_cm;
	u32 max_instances;
	u32 tx_clk_rate;
	u32 rx_clk_rate;
	bool mux_present;
	bool shared_phy;
	bool use_dcnano_or_epdc;
	bool rx_clk_quirk;	/* enable rx_esc clock to access registers */
};

static unsigned long nwl_dsi_get_bit_clock(struct nwl_dsi *dsi,
		unsigned long pixclock, u32 lanes, unsigned int min_hs_clock_multiple,
		unsigned int mipi_dsi_multiple)
{
	int bpp;
	unsigned long bit_clk = 0;

	if (lanes < 1 || lanes > 4)
		return 0;

	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);
	bit_clk = (unsigned long)phy_mipi_dphy_get_hs_clk(pixclock, bpp, lanes,
			dsi->dsi_mode_flags, min_hs_clock_multiple, mipi_dsi_multiple);

	if (pixclock * min_hs_clock_multiple == bit_clk) {
		dsi->hsmult = min_hs_clock_multiple;
		debug("%s: %ld = %ld * %d\n", __func__, bit_clk, pixclock, dsi->hsmult);
	} else {
		dsi->hsmult = 0;
	}
	return bit_clk;
}

static void phyref_set_rate(struct nwl_dsi *dsi, unsigned long rate)
{
	if (dsi->phy_ref_rate != rate) {
		if (dsi->phy_ref_enabled) {
			clk_disable_unprepare(dsi->phy_ref_clk);
			clk_set_rate(dsi->phy_ref_clk, rate);
			clk_prepare_enable(dsi->phy_ref_clk);
		} else {
			clk_set_rate(dsi->phy_ref_clk, rate);
		}
		rate = clk_get_rate(dsi->phy_ref_clk);
		dsi->phy_ref_rate = rate;
	}
}

#define DCNANO_CRTC_PLL_MIN_RATE	271500000
#define DCNANO_CRTC_PLL_MAX_RATE	792000000

static unsigned long find_best_pll_rate_imx8ulp(struct nwl_dsi *dsi, unsigned long pixclock)
{
	unsigned n = (pixclock + DCNANO_CRTC_PLL_MIN_RATE - 1) / pixclock;
	unsigned long video_pll = pixclock * n;
	unsigned long min = pixclock - (pixclock >> 3);
	unsigned long best_diff = ~0;
	unsigned long best_pll_rate = 0;
	int ret;

	while (video_pll <= DCNANO_CRTC_PLL_MAX_RATE) {
		unsigned long rounded_pll_clk_rate = clk_round_rate(
				dsi->pll_clk, video_pll);
		unsigned long div = (rounded_pll_clk_rate + (pixclock >> 1)) / pixclock;
		unsigned long rate = rounded_pll_clk_rate / div;
		unsigned long diff;

		if (div > 64)
			break;
		debug("%s: requested %lu, got %lu \n", __func__,
				video_pll, rounded_pll_clk_rate);
		if (rate > pixclock)
			diff = rate - pixclock;
		else
			diff = pixclock - rate;
		if ((rate >= min) && (best_diff > diff)) {
			best_pll_rate = rounded_pll_clk_rate;
			best_diff = diff;
			debug("%s: %ld = %lu / %ld\n", __func__,
				rate, rounded_pll_clk_rate, div);
			if (diff < 5)
				break;
		}
		video_pll += pixclock;
	}

	if (!best_pll_rate) {
		debug("%s: failed to find pll clock rate\n", __func__);
		ret = -EINVAL;
	} else {
		ret = clk_set_rate(dsi->pll_clk, best_pll_rate);
	}
	if (ret < 0) {
		debug("clk_set_rate %ld failed(%d)\n", best_pll_rate, ret);
		best_pll_rate = clk_get_rate(dsi->pll_clk);
		debug("rate is %ld\n", best_pll_rate);
	}
	debug("%s: pll requested %ld, got %ld\n", __func__,
			best_pll_rate, clk_get_rate(dsi->pll_clk));
	return best_pll_rate;
}

static unsigned long find_best_pll_rate(struct nwl_dsi *dsi, unsigned long pixclock)
{
	unsigned long video_pll = pixclock;
	int ret;

	/* Video pll must be from 500MHz to 2000 MHz */
	if (video_pll < 500000000) {
		u32 n = (500000000 + video_pll - 1) / video_pll;
		int bit;

		do {
			bit = __ffs(n);
			if ((n >> bit) <= 7)
				break;
			n += (1 << bit);
		} while (1);
		video_pll *= n;
		debug("%s: %ld = %ld * %d\n", __func__, video_pll, pixclock, n);
	}
	ret = clk_set_rate(dsi->pll_clk, video_pll);
	if (ret < 0) {
		debug("clk_set_rate %ld failed(%d)\n", video_pll, ret);
		video_pll = clk_get_rate(dsi->pll_clk);
		debug("rate is %ld\n", video_pll);
	}
	return video_pll;
}

static u32 get_pixclock(struct nwl_dsi *dsi, unsigned long pixclock)
{
	unsigned long video_pll;
	unsigned long pix = -EINVAL;

	if (device_is_compatible(dsi->dev, "fsl,imx8ulp-nwl-dsi"))
		video_pll = find_best_pll_rate_imx8ulp(dsi, pixclock);
	else
		video_pll = find_best_pll_rate(dsi, pixclock);

	if (dsi->lcdif_clk) {
		struct clk *cp = clk_get_parent(dsi->lcdif_clk);

		if (!IS_ERR(cp)) {
			int ret = clk_set_rate(cp, pixclock);
			if (ret)
				debug("%s: failed to set parent clock rate\n", __func__);
		}
		debug("%s: lcdif parent requested %ld, got %ld\n", __func__,
				pixclock, clk_get_rate(cp));

		pix = clk_round_rate(dsi->lcdif_clk, pixclock);
		if (IS_ERR_VALUE(pix)) {
			clk_set_rate(dsi->lcdif_clk, pixclock);
			pix = clk_get_rate(dsi->lcdif_clk);
		}
	}
	if (IS_ERR_VALUE(pix)) {
		int n = (video_pll + (pixclock >> 1)) / pixclock;
		pix = video_pll / n;
	}
	dsi->pixclock = pix;
	debug("%s: %s: desired=%ld, got=%ld\n", __func__, dsi->lcdif_clk->dev->name, pixclock, pix);
	return pix;
}

/*
 * This function will try the required phy speed for current mode
 * If the phy speed can be achieved, the phy will save the speed
 * configuration
 */
static struct mode_config *nwl_dsi_mode_probe(struct nwl_dsi *dsi)
{
	struct mode_config *config;
	union phy_configure_opts phy_opts;
	unsigned long clock = dsi->timings.pixelclock.typ;
	unsigned long bit_clk = 0;
	unsigned long phy_ref_rate = 0;
	u32 lanes = dsi->lanes;
	int ret;

	list_for_each_entry(config, &dsi->valid_modes, list)
		if (config->clock == clock)
			return config;

	clock = phy_ref_rate = get_pixclock(dsi, clock);
	while (phy_ref_rate >= 48000000)
		phy_ref_rate >>= 1;
	while (phy_ref_rate < 24000000)
		phy_ref_rate <<= 1;
	debug("%s: phyref %ld %ld\n", __func__, dsi->phy_ref_rate, phy_ref_rate);

	dsi->phy_ref_rate = 0;
	phyref_set_rate(dsi, phy_ref_rate);

	debug("%s: pixclock %ld\n", __func__, clock);
	phy_mipi_dphy_get_default_config(clock,
		mipi_dsi_pixel_format_to_bpp(dsi->format),
		lanes, &phy_opts.mipi_dphy, dsi->dsi_mode_flags,
		dsi->min_hs_clock_multiple, dsi->mipi_dsi_multiple);
	bit_clk = phy_opts.mipi_dphy.hs_clk_rate;
	if (clock * dsi->min_hs_clock_multiple == bit_clk)
		dsi->hsmult = dsi->min_hs_clock_multiple;

	phy_opts.mipi_dphy.lp_clk_rate = clk_get_rate(dsi->tx_esc_clk);

	ret = generic_phy_validate(&dsi->phy, PHY_MODE_MIPI_DPHY, 0, &phy_opts);
	if (ret) {
		debug("Failed phy_validate %d\n", ret);
		return NULL;
	}

	dsi->bitclk = bit_clk = phy_opts.mipi_dphy.hs_clk_rate;
	config = devm_kzalloc(dsi->dev, sizeof(struct mode_config), GFP_KERNEL);
	if (config) {
		config->clock = clock;
		config->lanes = lanes;
		config->bitclock = bit_clk;
		config->phy_ref_rate = dsi->phy_ref_rate;
		list_add(&config->list, &dsi->valid_modes);
	}
	return config;
}

static int nwl_dsi_set_platform_clocks(struct nwl_dsi *dsi, bool enable)
{
	const char *id;
	struct clk *clk;
	size_t i;
	unsigned long rate;
	int ret, result = 0;

	debug("%s platform clocks\n",
			     enable ? "enabling" : "disabling");
	for (i = 0; i < ARRAY_SIZE(dsi->pdata->clk_config); i++) {
		if (!dsi->clk_config[i].present)
			continue;
		id = dsi->clk_config[i].id;
		clk = dsi->clk_config[i].clk;

		if (enable) {
			ret = clk_prepare_enable(clk);
			if (ret < 0) {
				debug("Failed to enable %s clk: %d\n",
					      id, ret);
				result = result ?: ret;
			}
			rate = clk_get_rate(clk);
			debug("Enabled %s clk @%lu Hz\n",
					     id, rate);
		} else {
			clk_disable_unprepare(clk);
			debug("Disabled %s clk\n", id);
		}
	}

	return result;
}

static void nwl_dsi_bridge_pre_enable(struct nwl_dsi *dsi)
{
	int ret;

	dsi->pdata->dpi_reset(dsi, true);
	dsi->pdata->mipi_reset(dsi, true);
	dsi->pdata->pclk_reset(dsi, true);

	if (dsi->lcdif_clk)
		clk_prepare_enable(dsi->lcdif_clk);

	if (nwl_dsi_set_platform_clocks(dsi, true) < 0)
		return;

	/*
	 * Enable rx_esc clock for some platforms to access DSI host controller
	 * and PHY registers.
	 */
	if (dsi->pdata->rx_clk_quirk)
		clk_prepare_enable(dsi->rx_esc_clk);

	/* Always use normal mode(full mode) for Type-4 display */
	if (dsi->pdata->reg_cm)
		regmap_update_bits(dsi->csr, dsi->pdata->reg_cm,
				   IMX8ULP_DSI_CM_MASK, IMX8ULP_DSI_CM_NORMAL);

	/* Perform Step 1 from DSI reset-out instructions */
	ret = dsi->pdata->pclk_reset(dsi, false);
	if (ret < 0) {
		debug("Failed to deassert PCLK: %d\n", ret);
		return;
	}

	/* Perform Step 2 from DSI reset-out instructions */
	nwl_dsi_enable(dsi);

	/* Perform Step 3 from DSI reset-out instructions */
	ret = dsi->pdata->mipi_reset(dsi, false);
	if (ret < 0) {
		debug("Failed to deassert MIPI: %d\n", ret);
		return;
	}
}

void nwl_dsi_start_frame(struct nwl_dsi *dsi)
{
	int ret;

	/* Perform Step 5 from DSI reset-out instructions */
	ret = dsi->pdata->dpi_reset(dsi, false);
	if (ret < 0)
		debug("Failed to deassert DPI: %d\n", ret);

}

static void nwl_dsi_bridge_disable(struct nwl_dsi *dsi)
{

	nwl_dsi_disable(dsi);

	/* Disable rx_esc clock as registers are not accessed any more. */
	if (dsi->pdata->rx_clk_quirk)
		clk_disable_unprepare(dsi->rx_esc_clk);

	dsi->pdata->dpi_reset(dsi, true);
	dsi->pdata->mipi_reset(dsi, true);
	dsi->pdata->pclk_reset(dsi, true);

	nwl_dsi_set_platform_clocks(dsi, false);

	if (dsi->lcdif_clk)
		clk_disable_unprepare(dsi->lcdif_clk);

	pm_runtime_put(dsi->dev);
}

static int nwl_dsi_get_dphy_params(struct nwl_dsi *dsi,
				   union phy_configure_opts *phy_opts)
{
	unsigned long rate;
	int ret;

	if (dsi->lanes < 1 || dsi->lanes > 4)
		return -EINVAL;

	/*
	 * So far the DPHY spec minimal timings work for both mixel
	 * dphy and nwl dsi host
	 */
	ret = phy_mipi_dphy_get_default_config(
		get_pixclock(dsi, dsi->timings.pixelclock.typ),
		mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes,
		&phy_opts->mipi_dphy, dsi->dsi_mode_flags,
		dsi->min_hs_clock_multiple, dsi->mipi_dsi_multiple);
	if (ret < 0)
		return ret;

	rate = clk_get_rate(dsi->tx_esc_clk);
	debug("LP clk is @%lu Hz\n", rate);
	phy_opts->mipi_dphy.lp_clk_rate = rate;

	return 0;
}

static int nwl_dsi_bridge_mode_fixup(struct nwl_dsi *dsi)
{
	struct mode_config *config;
	unsigned long pll_rate;

	debug("Fixup mode:\n");

	config = nwl_dsi_mode_probe(dsi);
	if (!config)
		return -EINVAL;

	debug("lanes=%u, data_rate=%lu\n",
			     config->lanes, config->bitclock);
	if (config->lanes < 1 || config->lanes > 4)
		return -EINVAL;

	/* Max data rate for this controller is 1.5Gbps */
	if (config->bitclock > 1500000000)
		return -EINVAL;

	/* Update the crtc_clock to be used by display controller */
	if (config->crtc_clock)
		dsi->crtc_clock = config->crtc_clock;
	else if (dsi->clk_drop_lvl) {
		int div;
		unsigned long phy_ref_rate;

		phy_ref_rate = config->phy_ref_rate;
		pll_rate = config->bitclock;
		div = DIV_ROUND_CLOSEST(pll_rate, config->clock);
		pll_rate -= phy_ref_rate * dsi->clk_drop_lvl;
		dsi->crtc_clock = (pll_rate / div);
	}

	if (!dsi->use_dcss && !dsi->pdata->use_dcnano_or_epdc) {
		/* LCDIF + NWL needs active high sync */
		dsi->timings.flags |= (DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH);
		dsi->timings.flags &= ~(DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW);
	} else {
		dsi->timings.flags &= ~(DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH);
		dsi->timings.flags |= (DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW);
	}

	return 0;
}

static int nwl_dsi_bridge_mode_valid(struct nwl_dsi *dsi)
{
	struct mode_config *config;
	int bit_rate;

	bit_rate = nwl_dsi_get_bit_clock(dsi, get_pixclock(dsi, dsi->timings.pixelclock.typ), dsi->lanes,
			dsi->min_hs_clock_multiple, dsi->mipi_dsi_multiple);

	debug("Validating mode:\n");

	if (bit_rate > MBPS(1500))
		return -EINVAL;

	if (bit_rate < MBPS(80))
		return -EINVAL;

	config = nwl_dsi_mode_probe(dsi);
	if (!config)
		return -EINVAL;

	return 0;
}

static void nwl_dsi_bridge_mode_set(struct nwl_dsi *dsi)
{
	union phy_configure_opts new_cfg;
	unsigned long phy_ref_rate;
	struct mode_config *config;
	size_t i;
	const char *id;
	struct clk *clk;
	int ret;

	debug("Setting mode:\n");

	config = nwl_dsi_mode_probe(dsi);
	/* New mode? This should NOT happen */
	if (!config) {
		debug("Unsupported mode provided:\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(dsi->pdata->clk_config); i++) {
		if (!dsi->clk_config[i].present)
			continue;
		id = dsi->clk_config[i].id;
		clk = dsi->clk_config[i].clk;

		/* Set bypass and pixel clocks to mode clock rate */
		if (!strcmp(id, NWL_DSI_CLK_BYPASS) ||
		    !strcmp(id, NWL_DSI_CLK_PIXEL))
			clk_set_rate(clk, get_pixclock(dsi, dsi->crtc_clock));
	}

	phy_ref_rate = config->phy_ref_rate;
	clk_set_rate(dsi->phy_ref_clk, phy_ref_rate);
	ret = nwl_dsi_get_dphy_params(dsi, &new_cfg);
	if (ret < 0)
		return;

	/*
	 * If hs clock is unchanged, we're all good - all parameters are
	 * derived from it atm.
	 */
	if (new_cfg.mipi_dphy.hs_clk_rate == dsi->phy_cfg.mipi_dphy.hs_clk_rate)
		return;

	debug("PHY at ref rate: %lu (actual: %lu)\n",
		phy_ref_rate, clk_get_rate(dsi->phy_ref_clk));
	/* Save the new desired phy config */
	memcpy(&dsi->phy_cfg, &new_cfg, sizeof(new_cfg));

}

static int nwl_init(struct udevice *dev,
			    struct mipi_dsi_device *device,
			    struct display_timing *timings,
			    unsigned int max_data_lanes,
			    const struct mipi_dsi_phy_ops *phy_ops)
{
	struct nwl_dsi *dsi = dev_get_priv(dev);

	debug("%s:\n", __func__);
	dsi->max_data_lanes = max_data_lanes;
	dsi->device = device;
	dsi->dsi_host.ops = &nwl_mipi_dsi_host_ops;
	device->host = &dsi->dsi_host;

	dsi->timings = *timings;
	return 0;
}

static int nwl_enable(struct udevice *dev)
{
	struct nwl_dsi *dsi = dev_get_priv(dev);
	int ret;

	debug("%s:\n", __func__);

	dsi->min_hs_clock_multiple = dsi->device->hsmult;
	dsi->mipi_dsi_multiple = dsi->device->mipi_dsi_multiple;
	dsi->max_data_lanes = dsi->device->lanes;
	debug("%s: min_hs_clock_multiple=%d lanes=%d\n", __func__,
		dsi->min_hs_clock_multiple, dsi->max_data_lanes);
	ret = nwl_dsi_bridge_mode_fixup(dsi);
	if (ret)
		return ret;

	nwl_dsi_bridge_mode_set(dsi);
	ret = nwl_dsi_bridge_mode_valid(dsi);
	if (ret)
		return ret;

	nwl_dsi_bridge_pre_enable(dsi);
	return 0;
}

static int nwl_disable(struct udevice *dev)
{
	struct nwl_dsi *dsi = dev_get_priv(dev);

	debug("%s:\n", __func__);
	nwl_dsi_bridge_disable(dsi);
	return 0;
}

struct dsi_host_ops nwl_dsi_host_ops = {
	.init = nwl_init,
	.enable = nwl_enable,
	.disable = nwl_disable,
};

/* -------------------------------------------------------------------- */

static int imx8_common_dsi_pclk_reset(struct nwl_dsi *dsi, bool reset)
{
	int ret = 0;

	if (dsi->rst_pclk) {
		if (reset)
			ret = reset_assert(dsi->rst_pclk);
		else
			ret = reset_deassert(dsi->rst_pclk);
	}

	return ret;

}

static int imx8_common_dsi_mipi_reset(struct nwl_dsi *dsi, bool reset)
{
	int ret = 0;

	if (dsi->rst_esc) {
		if (reset)
			ret = reset_assert(dsi->rst_esc);
		else
			ret = reset_deassert(dsi->rst_esc);
	}

	if (dsi->rst_byte) {
		if (reset)
			ret = reset_assert(dsi->rst_byte);
		else
			ret = reset_deassert(dsi->rst_byte);
	}

	return ret;

}

static int imx8_common_dsi_dpi_reset(struct nwl_dsi *dsi, bool reset)
{
	int ret = 0;

	if (dsi->rst_dpi) {
		if (reset)
			ret = reset_assert(dsi->rst_dpi);
		else
			ret = reset_deassert(dsi->rst_dpi);
	}

	return ret;

}


static int imx8_common_dsi_select_input(struct nwl_dsi *dsi)
{
	const char *str;
	u32 use_dcss = 1;
	u32 use_dcss_or_epdc = 1;
	int ret;

	str = ofnode_read_string(dev_ofnode(dsi->dev), "input-source");
	if (str) {
		if (strncmp(str, "lcdif", 5) == 0)
			use_dcss = use_dcss_or_epdc = 0;
		else if (strncmp(str, "dcnano", 6) == 0)
			use_dcss = use_dcss_or_epdc = 0;
		else if (strncmp(str, "epdc", 6) == 0)
			use_dcss = 0;
	}
	debug("Using %s as input source\n",
		     (use_dcss_or_epdc) ? "DCSS/epdc" : "LCDIF/dcnano");

	ret = mux_control_try_select(dsi->mux, use_dcss_or_epdc);
	if (ret < 0)
		debug("Failed to select input: %d\n", ret);

	dsi->use_dcss = use_dcss;

	return ret;
}

static int imx8_common_dsi_deselect_input(struct nwl_dsi *dsi)
{
	int ret;

	ret = mux_control_deselect(dsi->mux);
	if (ret < 0)
		debug("Failed to deselect input: %d\n", ret);

	return ret;
}

static const struct nwl_dsi_platform_data imx8mq_dev = {
	.pclk_reset = &imx8_common_dsi_pclk_reset,
	.mipi_reset = &imx8_common_dsi_mipi_reset,
	.dpi_reset = &imx8_common_dsi_dpi_reset,
	.select_input = &imx8_common_dsi_select_input,
	.deselect_input = &imx8_common_dsi_deselect_input,
	.clk_config = {
		{ .id = NWL_DSI_CLK_CORE, .present = true },
	},
	.mux_present = true,
};

static struct nwl_dsi_platform_data imx8ulp_dev = {
	.pclk_reset = &imx8_common_dsi_pclk_reset,
	.mipi_reset = &imx8_common_dsi_mipi_reset,
	.dpi_reset = &imx8_common_dsi_dpi_reset,
	.select_input = &imx8_common_dsi_select_input,
	.deselect_input = &imx8_common_dsi_deselect_input,
	.clk_config = {
		{ .id = NWL_DSI_CLK_CORE, .present = true },
	},
	.reg_cm = 0x8,
	.mux_present = true,
	.use_dcnano_or_epdc = true,
	.rx_clk_quirk = true,
};

#if !defined(CONFIG_IMX8MQ) && !defined(CONFIG_IMX8ULP)
static int imx8q_dsi_pclk_reset(struct nwl_dsi *dsi, bool reset)
{
	struct imx_sc_ipc *handle;
	u32 mipi_id, dc_id;
	u8 ctrl;
	bool shared_phy = dsi->pdata->shared_phy;
	int ret = 0;

	ret = imx_scu_get_handle(&handle);
	if (ret) {
		debug("Failed to get scu ipc handle (%d)\n", ret);
		return ret;
	}

	mipi_id = (dsi->instance)?MIPI_ID(1):MIPI_ID(0);
	dc_id = (!shared_phy && dsi->instance)?DC_ID(1):DC_ID(0);
	debug("Power %s PCLK MIPI:%u DC:%u\n",
		(reset)?"OFF":"ON", mipi_id, dc_id);

	if (shared_phy) {
		ret |= imx_sc_misc_set_control(handle,
				mipi_id, IMX_SC_C_MODE, reset);
		ret |= imx_sc_misc_set_control(handle,
				mipi_id, IMX_SC_C_DUAL_MODE, reset);
		ret |= imx_sc_misc_set_control(handle,
				mipi_id, IMX_SC_C_PXL_LINK_SEL, reset);
	}

	/* Initialize Pixel Link */
	ctrl = (shared_phy && dsi->instance)?PXL_ADDR(2):PXL_ADDR(1);
	ret |= imx_sc_misc_set_control(handle, dc_id, ctrl, reset);

	ctrl = (shared_phy && dsi->instance)?PXL_VLD(2):PXL_VLD(1);
	ret |= imx_sc_misc_set_control(handle, dc_id, ctrl, !reset);

	ctrl = (shared_phy && dsi->instance)?SYNC_CTRL(1):SYNC_CTRL(0);
	ret |= imx_sc_misc_set_control(handle, dc_id, ctrl, !reset);

	return ret;
}

static int imx8q_dsi_mipi_reset(struct nwl_dsi *dsi, bool reset)
{
	struct imx_sc_ipc *handle;
	u32 mipi_id;
	int ret = 0;

	ret = imx_scu_get_handle(&handle);
	if (ret) {
		debug("Failed to get scu ipc handle (%d)\n", ret);
		return ret;
	}

	mipi_id = (dsi->instance)?MIPI_ID(1):MIPI_ID(0);
	debug("Power %s HOST MIPI:%u\n",
		(reset)?"OFF":"ON", mipi_id);

	ret |= imx_sc_misc_set_control(handle, mipi_id,
				       IMX_SC_C_PHY_RESET, !reset);
	ret |= imx_sc_misc_set_control(handle, mipi_id,
				       IMX_SC_C_MIPI_RESET, !reset);

	return ret;
}

static int imx8q_dsi_dpi_reset(struct nwl_dsi *dsi, bool reset)
{
	struct imx_sc_ipc *handle;
	u32 mipi_id;
	int ret = 0;

	ret = imx_scu_get_handle(&handle);
	if (ret) {
		debug("Failed to get scu ipc handle (%d)\n", ret);
		return ret;
	}

	mipi_id = (dsi->instance)?MIPI_ID(1):MIPI_ID(0);
	debug("Power %s DPI MIPI:%u\n", (reset)?"OFF":"ON", mipi_id);

	regmap_write(dsi->csr, dsi->pdata->reg_tx_ulps, 0);
	regmap_write(dsi->csr, dsi->pdata->reg_pxl2dpi, NWL_DSI_DPI_24_BIT);

	ret |= imx_sc_misc_set_control(handle, mipi_id,
				       IMX_SC_C_DPI_RESET, !reset);

	return ret;
}

static const struct nwl_dsi_platform_data imx8qm_dev = {
	.pclk_reset = &imx8q_dsi_pclk_reset,
	.mipi_reset = &imx8q_dsi_mipi_reset,
	.dpi_reset = &imx8q_dsi_dpi_reset,
	.clk_config = {
		{ .id = NWL_DSI_CLK_BYPASS, .present = true },
		{ .id = NWL_DSI_CLK_PIXEL, .present = true },
	},
	.reg_tx_ulps = 0x00,
	.reg_pxl2dpi = 0x04,
	.max_instances = 2,
	.tx_clk_rate = 18000000,
	.rx_clk_rate = 72000000,
	.shared_phy = false,
};

static const struct nwl_dsi_platform_data imx8qx_dev = {
	.pclk_reset = &imx8q_dsi_pclk_reset,
	.mipi_reset = &imx8q_dsi_mipi_reset,
	.dpi_reset = &imx8q_dsi_dpi_reset,
	.clk_config = {
		{ .id = NWL_DSI_CLK_BYPASS, .present = true },
		{ .id = NWL_DSI_CLK_PIXEL, .present = true },
	},
	.reg_tx_ulps = 0x30,
	.reg_pxl2dpi = 0x40,
	.max_instances = 2,
	.tx_clk_rate = 18000000,
	.rx_clk_rate = 72000000,
	.shared_phy = true,
};
#endif

#define REV_2_0				0x20
#define REV_B1				0x21
#define IMX8MQ_SW_INFO_B1		0x40
#define IMX8MQ_SW_MAGIC_B1		0xff0055aa

#define IMX_SIP_GET_SOC_INFO		0xc2000006

static u32 imx8mq_soc_revision_from_atf(void)
{
	struct arm_smccc_res res;
	u32 rev;

	arm_smccc_smc(IMX_SIP_GET_SOC_INFO, 0, 0, 0, 0, 0, 0, 0, &res);

	rev = res.a0 & 0xff;
	if (res.a0 == SMCCC_RET_NOT_SUPPORTED)
		rev = 0;
	return rev;
}

static int nwl_get_quirks(void)
{
	u32 rev = 0;

	rev = imx8mq_soc_revision_from_atf();
	if (!rev) {
		ofnode np;
		void *ocotp_base;
		u32 magic;

		np = ofnode_by_compatible(ofnode_null(), "fsl,imx8mq-ocotp");
		if (ofnode_valid(np)) {
			ocotp_base = (void *)ofnode_get_addr(np);

			magic = readl_relaxed(ocotp_base + IMX8MQ_SW_INFO_B1);
			if (magic == IMX8MQ_SW_MAGIC_B1)
				rev = REV_B1;
		}
	}
	if (rev == REV_2_0)
		return E11418_HS_MODE_QUIRK | SRC_RESET_QUIRK;
	return 0;
}

static int nwl_dsi_ofdata_to_platdata(struct udevice *dev)
{
	struct nwl_dsi *dsi = dev_get_priv(dev);
	ofnode np = dev_ofnode(dev);
	struct clk *clk;
	const char *clk_id;
	int i;
	int id = 0;
	int ret;

	debug("%s:\n", __func__);
	dsi->pdata = (struct nwl_dsi_platform_data *)dev_get_driver_data(dev);
	dsi->dev = dev;
	dsi->quirks = nwl_get_quirks();
	INIT_LIST_HEAD(&dsi->valid_modes);

	ret = generic_phy_get_by_index_nodev(np, 0, &dsi->phy);
	if (ret) {
		debug("Could not get PHY: %d\n", ret);
		return ret;
	}

	ret = dev_read_alias_seq(dev, &id);
	if (!ret && (id > 0)) {
		if (id >= dsi->pdata->max_instances) {
			debug("Too many instances! (cur: %d, max: %d)\n",
				id, dsi->pdata->max_instances);
			return -ENODEV;
		}
		dsi->instance = id;
	}

	/* Platform dependent clocks */
	memcpy(dsi->clk_config, dsi->pdata->clk_config,
	       sizeof(dsi->pdata->clk_config));

	for (i = 0; i < ARRAY_SIZE(dsi->pdata->clk_config); i++) {
		if (!dsi->clk_config[i].present)
			continue;

		clk_id = dsi->clk_config[i].id;
		clk = devm_clk_get(dev, clk_id);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			debug("Failed to get %s clock: %d\n", clk_id, ret);
			return ret;
		}
		debug("Setup clk %s (rate: %lu)\n",
			clk_id, clk_get_rate(clk));
		dsi->clk_config[i].clk = clk;
	}

	/* DSI clocks */
	clk = devm_clk_get(dev, "phy_ref");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		debug("Failed to get phy_ref clock: %d\n", ret);
		return ret;
	}
	dsi->phy_ref_clk = clk;

	clk = devm_clk_get(dev, "rx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		debug("Failed to get rx_esc clock: %d\n", ret);
		return ret;
	}
	dsi->rx_esc_clk = clk;

	clk = devm_clk_get(dev, "tx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		debug("Failed to get tx_esc clock: %d\n", ret);
		return ret;
	}
	dsi->tx_esc_clk = clk;

	/* The video_pll clock is optional */
	clk = devm_clk_get(dev, "video_pll");
	if (!IS_ERR(clk))
		dsi->pll_clk = clk;

	/*
	 * Usually, we don't need this clock here, but due to a design issue,
	 * the MIPI Reset Synchronizer block depends on this clock. So, we will
	 * use this clock when we need to reset (or take out of reset) MIPI PHY
	 */
	dsi->lcdif_clk = devm_clk_get(dev, "lcdif");
	if (IS_ERR(dsi->lcdif_clk))
		dsi->lcdif_clk = NULL;

	if (dsi->pdata->mux_present) {
		dsi->mux = devm_mux_control_get(dev, NULL);
		if (IS_ERR(dsi->mux)) {
			ret = PTR_ERR(dsi->mux);
			debug("Failed to get mux: %d\n", ret);
			return ret;
		}
	}

	ret = regmap_init_mem(np, &dsi->regmap);
	if (ret) {
		debug("Failed to create NWL DSI regmap: %d\n", ret);
		return ret;
	}

	/* For these two regs we need a mapping to MIPI-DSI CSR */
	if (dsi->pdata->reg_tx_ulps || dsi->pdata->reg_pxl2dpi ||
	    dsi->pdata->reg_cm) {
		dsi->csr = syscon_regmap_lookup_by_phandle(dev, "csr");
		if (IS_ERR(dsi->csr)) {
			ret = PTR_ERR(dsi->csr);
			debug("Failed to get CSR regmap: %d\n", ret);
			return ret;
		}
	}

	/*
	 * We need to manage individual resets, since the order of their usage
	 * really matters to DSI Host Controller. The order of operation,
	 * should be like this:
	 * 1. Deassert pclk reset (this is needed to have access to DSI regs)
	 * 2. Configure DSI Host and DPHY and enable DPHY
	 * 3. Deassert ESC and BYTE resets (needed for Host TX operations)
	 * 4. Send DSI cmds (if the DSI peripheral needs configuration)
	 * 5. Deassert DPI reset (deasserting DPI reset, enables DPI to receive
	 *    DPI pixels and start streamming DSI data)
	 */
	dsi->rst_pclk = devm_reset_control_get(dev, "pclk");
	if (IS_ERR(dsi->rst_pclk)) {
		debug("Failed to get pclk reset: %ld\n",
			      PTR_ERR(dsi->rst_pclk));
		return PTR_ERR(dsi->rst_pclk);
	}
	dsi->rst_byte = devm_reset_control_get(dev, "byte");
	if (IS_ERR(dsi->rst_byte)) {
		debug("Failed to get byte reset: %ld\n",
			      PTR_ERR(dsi->rst_byte));
		return PTR_ERR(dsi->rst_byte);
	}
	dsi->rst_esc = devm_reset_control_get(dev, "esc");
	if (IS_ERR(dsi->rst_esc)) {
		debug("Failed to get esc reset: %ld\n",
			      PTR_ERR(dsi->rst_esc));
		return PTR_ERR(dsi->rst_esc);
	}
	dsi->rst_dpi = devm_reset_control_get(dev, "dpi");
	if (IS_ERR(dsi->rst_dpi)) {
		debug("Failed to get dpi reset: %ld\n",
			      PTR_ERR(dsi->rst_dpi));
		return PTR_ERR(dsi->rst_dpi);
	}

	ofnode_read_u32(np, "fsl,clock-drop-level", &dsi->clk_drop_lvl);
	return 0;
}

static int nwl_dsi_probe(struct udevice *dev)
{
	struct nwl_dsi *dsi = dev_get_priv(dev);
	struct clk *phy_parent;
	int ret;

	debug("%s:\n", __func__);
	if (dsi->pdata->select_input)
		dsi->pdata->select_input(dsi);

	phy_parent = devm_clk_get(dsi->dev, "phy_parent");
	if (!IS_ERR_OR_NULL(phy_parent)) {
		ret = clk_set_parent(dsi->phy_ref_clk, phy_parent);
		ret |= clk_set_parent(dsi->tx_esc_clk, phy_parent);
		ret |= clk_set_parent(dsi->rx_esc_clk, phy_parent);

		if (ret) {
			debug("Error re-parenting phy/tx/rx clocks: %d", ret);
			return ret;
		}

		if (dsi->pdata->tx_clk_rate)
			clk_set_rate(dsi->tx_esc_clk, dsi->pdata->tx_clk_rate);

		if (dsi->pdata->rx_clk_rate)
			clk_set_rate(dsi->rx_esc_clk, dsi->pdata->rx_clk_rate);
	}

	return 0;
}

static int nwl_dsi_remove(struct udevice *dev)
{
	struct nwl_dsi *dsi = dev_get_priv(dev);
	struct mode_config *config;
	struct list_head *pos, *tmp;

	list_for_each_safe(pos, tmp, &dsi->valid_modes) {
		config = list_entry(pos, struct mode_config, list);
		list_del(pos);
		devm_kfree(dsi->dev, config);
	}
	return 0;
}

static const struct udevice_id nwl_dsi_dt_ids[] = {
	{ .compatible = "fsl,imx8mq-nwl-dsi", .data = (ulong)&imx8mq_dev, },
	{ .compatible = "fsl,imx8ulp-nwl-dsi", .data = (ulong)&imx8ulp_dev, },
#if !defined(CONFIG_IMX8MQ) && !defined(CONFIG_IMX8ULP)
	{ .compatible = "fsl,imx8qm-nwl-dsi", .data = (ulong)&imx8qm_dev, },
	{ .compatible = "fsl,imx8qx-nwl-dsi", .data = (ulong)&imx8qx_dev, },
#endif
	{ /* sentinel */ }
};

U_BOOT_DRIVER(mipi_dsi_northwest) = {
	.name			= "nwl-dsi",
	.id			= UCLASS_DSI_HOST,
	.of_match		= nwl_dsi_dt_ids,
	.of_to_plat		= nwl_dsi_ofdata_to_platdata,
	.probe			= nwl_dsi_probe,
	.remove 		= nwl_dsi_remove,
	.ops			= &nwl_dsi_host_ops,
	.priv_auto		= sizeof(struct nwl_dsi),
};
