// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
 *
 * derived from imx-hdmi.c(renamed to bridge/dw_hdmi.c now)
 */

#include <common.h>
#include <backlight.h>
#include <clk.h>
#include <display.h>
#include <dm/device.h>
#include <dm/read.h>
#include <dm/device-internal.h>
#include <generic-phy.h>
#include <panel.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <video_bridge.h>
#include <video_link.h>
#include "dw_hdmi.h"
#include "imx8mp-hdmi-pavi.h"

/* GPR reg */
struct imx_hdmi_chip_data {
	int	reg_offset;
	u32	mask_bits;
	u32	shift_bit;
};

struct imx_hdmi {
	struct udevice *dev;
	struct dw_hdmi *dw_hdmi;
	struct regmap *regmap;
	const struct imx_hdmi_chip_data *chip_data;
	struct phy phy;
#define PAVI	0
#define PVI	1
#define PAI	2
	struct phy pavi[3];
	int clock_cnt;
	struct display_timing timings;
	struct clk clks[12];
};

static const char *imx8mp_clocks[] = {
	"pix_clk",
	"phy_int",
	"prep_clk",
	"skp_clk",
	"sfr_clk",
	"cec_clk",
	"apb_clk",
	"hpi_clk",
	"fdcc_ref",
	"pipe_clk",
};

static const struct dw_hdmi_mpll_config imx_mpll_cfg[] = {
	{
		45250000, {
			{ 0x01e0, 0x0000 },
			{ 0x21e1, 0x0000 },
			{ 0x41e2, 0x0000 }
		},
	}, {
		92500000, {
			{ 0x0140, 0x0005 },
			{ 0x2141, 0x0005 },
			{ 0x4142, 0x0005 },
	},
	}, {
		148500000, {
			{ 0x00a0, 0x000a },
			{ 0x20a1, 0x000a },
			{ 0x40a2, 0x000a },
		},
	}, {
		216000000, {
			{ 0x00a0, 0x000a },
			{ 0x2001, 0x000f },
			{ 0x4002, 0x000f },
		},
	}, {
		~0UL, {
			{ 0x0000, 0x0000 },
			{ 0x0000, 0x0000 },
			{ 0x0000, 0x0000 },
		},
	}
};

static const struct dw_hdmi_curr_ctrl imx_cur_ctr[] = {
	/*      pixelclk     bpp8    bpp10   bpp12 */
	{
		54000000, { 0x091c, 0x091c, 0x06dc },
	}, {
		58400000, { 0x091c, 0x06dc, 0x06dc },
	}, {
		72000000, { 0x06dc, 0x06dc, 0x091c },
	}, {
		74250000, { 0x06dc, 0x0b5c, 0x091c },
	}, {
		118800000, { 0x091c, 0x091c, 0x06dc },
	}, {
		216000000, { 0x06dc, 0x0b5c, 0x091c },
	}, {
		~0UL, { 0x0000, 0x0000, 0x0000 },
	},
};

/*
 * Resistance term 133Ohm Cfg
 * PREEMP config 0.00
 * TX/CK level 10
 */
static const struct dw_hdmi_phy_config imx6_phy_config[] = {
	/*pixelclk   symbol   term   vlev */
	{ 216000000, 0x800d, 0x0005, 0x01ad},
	{ ~0UL,      0x0000, 0x0000, 0x0000}
};

static void dw_hdmi_imx_encoder_enable(struct imx_hdmi *hdmi, int mux)
{
	if (hdmi->chip_data->reg_offset < 0)
		return;

	regmap_update_bits(hdmi->regmap, hdmi->chip_data->reg_offset,
			   hdmi->chip_data->mask_bits, mux << hdmi->chip_data->shift_bit);
}


#define IOMUXC_GPR3			0x0c
#define IMX6Q_GPR3_HDMI_MUX_CTL_MASK	(0x3 << 2)
#define IMX6Q_GPR3_HDMI_MUX_CTL_SHIFT	2

struct imx_hdmi_chip_data imx6_chip_data = {
	.reg_offset = IOMUXC_GPR3,
	.mask_bits = IMX6Q_GPR3_HDMI_MUX_CTL_MASK,
	.shift_bit = IMX6Q_GPR3_HDMI_MUX_CTL_SHIFT,
};

static struct dw_hdmi_plat_data imx6q_hdmi_drv_data = {
	.mpll_cfg   = imx_mpll_cfg,
	.cur_ctr    = imx_cur_ctr,
	.phy_config = imx6_phy_config,
	.phy_data   = &imx6_chip_data,
	.phy_min_clock =  13500000,
	.phy_max_clock = 216000000,	/* FIXME: Hardware is capable of 266MHz, but setup data is missing. */
};

static struct dw_hdmi_plat_data imx6dl_hdmi_drv_data = {
	.mpll_cfg = imx_mpll_cfg,
	.cur_ctr  = imx_cur_ctr,
	.phy_config = imx6_phy_config,
	.phy_data   = &imx6_chip_data,
	.phy_min_clock =  13500000,
	.phy_max_clock = 216000000,	/* FIXME: Hardware is capable of 266MHz, but setup data is missing. */
};

static int imx8mp_hdmi_phy_init(struct dw_hdmi *dw_hdmi, void *data, const struct display_timing *mode)
{
	struct imx_hdmi *hdmi = dev_get_priv(dw_hdmi_get_dev(dw_hdmi));
	struct imx8mp_hdmi_pavi *pavi = dev_get_priv(hdmi->pavi[PVI].dev);
	unsigned int val;

	debug("%s\n", __func__);

	dw_hdmi_phy_reset(dw_hdmi);

	pavi->flags = hdmi->timings.flags;
	/* enable PVI */
	generic_phy_power_on(&hdmi->pavi[PAVI]);
	generic_phy_power_on(&hdmi->pavi[PVI]);

	/* HDMI PHY power up */
	regmap_read(hdmi->regmap, 0x200, &val);
	val &= ~0x8;
	/* Enable CEC */
	val |= 0x2;
	regmap_write(hdmi->regmap, 0x200, val);

	if (!hdmi->phy.dev)
		return 0;

	generic_phy_power_on(&hdmi->phy);

	return 0;
}

static void imx8mp_hdmi_phy_disable(struct dw_hdmi *dw_hdmi, void *data)
{
	struct imx_hdmi *hdmi = dev_get_priv(dw_hdmi_get_dev(dw_hdmi));
	unsigned int val;

	debug("%s\n", __func__);
	if (!hdmi->phy.dev)
		return;

	/* disable PVI */
	generic_phy_power_off(&hdmi->pavi[PVI]);
	generic_phy_power_off(&hdmi->pavi[PAVI]);

	/* TODO */
	regmap_read(hdmi->regmap, 0x200, &val);
	/* Disable CEC */
	val &= ~0x2;
	/* Power down HDMI PHY */
	val |= 0x8;
	regmap_write(hdmi->regmap, 0x200, val);
}

static int imx8mp_hdmimix_setup(struct imx_hdmi *hdmi)
{
	int ret;
	int i;

	ret = device_reset(hdmi->dev);

	hdmi->clock_cnt = ARRAY_SIZE(imx8mp_clocks);
	for (i = 0; i < hdmi->clock_cnt; i++) {
		ret = clk_get_by_name(hdmi->dev, imx8mp_clocks[i],
				&hdmi->clks[i]);
		if (ret < 0) {
			printf("%s: %s clk missing %d\n", __func__,
				imx8mp_clocks[i], ret);
			return ret;
		}
	}

	for (i = 0; i < hdmi->clock_cnt; i++) {
		ret = clk_prepare_enable(&hdmi->clks[i]);
		if (ret < 0) {
			printf("%s enable %s failed %d\n", __func__,
				imx8mp_clocks[i], ret);
			return ret;
		}
	}
//	clk_set_rate(&hdmi->clks[0], rate);
	return 0;
}

void imx8mp_hdmi_enable_audio(struct dw_hdmi *dw_hdmi, void *data, int channel,
			      int width, int rate, int non_pcm)
{
	struct imx_hdmi *hdmi = dev_get_priv(dw_hdmi_get_dev(dw_hdmi));
	struct imx8mp_hdmi_pavi *pavi = dev_get_priv(hdmi->pavi[PAI].dev);

	pavi->channel = channel;
	pavi->width = width;
	pavi->rate = rate;
	pavi->non_pcm = non_pcm;
	generic_phy_power_on(&hdmi->pavi[PAI]);
}

void imx8mp_hdmi_disable_audio(struct dw_hdmi *dw_hdmi, void *data)
{
	struct imx_hdmi *hdmi = dev_get_priv(dw_hdmi_get_dev(dw_hdmi));

	generic_phy_power_off(&hdmi->pavi[PAI]);
}

static const struct dw_hdmi_phy_ops imx8mp_hdmi_phy_ops = {
	.init		= imx8mp_hdmi_phy_init,
	.disable	= imx8mp_hdmi_phy_disable,
	.read_hpd = dw_hdmi_phy_read_hpd,
	.update_hpd = dw_hdmi_phy_update_hpd,
	.setup_hpd = dw_hdmi_phy_setup_hpd,
	.enable_audio	= imx8mp_hdmi_enable_audio,
	.disable_audio  = imx8mp_hdmi_disable_audio,
};

struct imx_hdmi_chip_data imx8mp_chip_data = {
	.reg_offset = -1,
};

static const struct dw_hdmi_plat_data imx8mp_hdmi_drv_data = {
	.phy_data   = &imx8mp_chip_data,
	.phy_ops    = &imx8mp_hdmi_phy_ops,
	.phy_name   = "samsung_dw_hdmi_phy2",
	.phy_force_vendor = true,
	.mix_setup = true,
	.phy_min_clock =  13500000,
	.phy_max_clock = 297000000,
};

static const struct udevice_id dw_hdmi_imx_dt_ids[] = {
	{ .compatible = "fsl,imx6q-hdmi",
	  .data = (long)&imx6q_hdmi_drv_data
	}, {
	  .compatible = "fsl,imx6dl-hdmi",
	  .data = (long)&imx6dl_hdmi_drv_data
	}, {
	  .compatible = "fsl,imx8mp-hdmi",
	  .data = (long)&imx8mp_hdmi_drv_data
	},
	{},
};

static int dw_hdmi_imx_read_timing(struct udevice *dev, struct display_timing *timing)
{
	struct imx_hdmi *hdmi = dev_get_priv(dev);

	if (timing) {
		memcpy(timing, &hdmi->timings, sizeof(struct display_timing));
		return 0;
	}

	return -EINVAL;
}

static int dw_hdmi_imx_enable(struct udevice *dev, int panel_bpp,
		      const struct display_timing *timing)
{
	const struct dw_hdmi_plat_data *pdata =
		(struct dw_hdmi_plat_data *)dev_get_driver_data(dev);
	struct imx_hdmi *hdmi = dev_get_priv(dev);
	int ret;

	ret = pdata->phy_ops->init(hdmi->dw_hdmi, hdmi, timing);
	if (ret) {
		printf("HDMI enable failed, ret %d!\n", ret);
		return ret;
	}
	dw_hdmi_imx_encoder_enable(hdmi, 0);
	dw_hdmi_bridge_enable(hdmi->dw_hdmi);
	return 0;
}

static const char* mix_phys[] = { "pavi", "pvi", "pai" };

static int dw_hdmi_imx_parse_dt(struct imx_hdmi *hdmi)
{
	const struct dw_hdmi_plat_data *pdata =
		(struct dw_hdmi_plat_data *)dev_get_driver_data(hdmi->dev);
	ofnode np = dev_ofnode(hdmi->dev);
	int ret;
	int i;

	hdmi->regmap = syscon_regmap_lookup_by_phandle(hdmi->dev, "gpr");
	if (IS_ERR(hdmi->regmap)) {
		debug("Unable to get gpr\n");
		return PTR_ERR(hdmi->regmap);
	}

	ret = ofnode_generic_phy_get_by_name(np, "hdmi", &hdmi->phy);
	if (ret) {
		debug("%s: can't get phy: %d\n", __func__, ret);
		return ret;
	}
	if (pdata->mix_setup) {
		for (i = 0; i < ARRAY_SIZE(mix_phys); i++) {
			ret = ofnode_generic_phy_get_by_name(np, mix_phys[i],
					&hdmi->pavi[i]);
			if (ret) {
				debug("%s: can't get %s: %d\n", __func__,
					mix_phys[i], ret);
				return ret;
			}
		}
	}
	return 0;
}

static int dw_hdmi_imx_bind(struct udevice *dev)
{
	const struct dw_hdmi_plat_data *pdata =
		(struct dw_hdmi_plat_data *)dev_get_driver_data(dev);
	struct imx_hdmi *hdmi = dev_get_priv(dev);
	int ret;

	hdmi->dev = dev;
	hdmi->chip_data = pdata->phy_data;

	ret = dw_hdmi_imx_parse_dt(hdmi);
	if (ret < 0)
		return ret;

	if (pdata->mix_setup) {
		ret = imx8mp_hdmimix_setup(hdmi);
		if (ret < 0)
			return ret;
	}

	hdmi->dw_hdmi = dw_hdmi_bind(dev, pdata);
	return ret;
}

static void dw_hdmi_imx_unbind(struct udevice *dev)
{
	struct imx_hdmi *hdmi = dev_get_priv(dev);

	dw_hdmi_unbind(hdmi->dw_hdmi);
}

static int dw_hdmi_imx_probe(struct udevice *dev)
{
	debug("%s\n", __func__);
	return dw_hdmi_imx_bind(dev);
}

static int dw_hdmi_imx_remove(struct udevice *dev)
{
	struct imx_hdmi *hdmi = dev_get_priv(dev);

	dw_hdmi_bridge_disable(hdmi->dw_hdmi);
	dw_hdmi_imx_unbind(dev);
	return 0;
}

struct dm_display_ops dw_hdmi_ops = {
	.read_timing = dw_hdmi_imx_read_timing,
	.enable = dw_hdmi_imx_enable,
};

U_BOOT_DRIVER(dw_hdmi_imx) = {
	.name				= "dwhdmi-imx",
	.id				= UCLASS_DISPLAY,
	.of_match			= dw_hdmi_imx_dt_ids,
	.bind				= dm_scan_fdt_dev,
	.probe				= dw_hdmi_imx_probe,
	.remove				= dw_hdmi_imx_remove,
	.ops				= &dw_hdmi_ops,
	.priv_auto_alloc_size		= sizeof(struct imx_hdmi),
};
