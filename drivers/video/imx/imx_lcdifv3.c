// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 NXP
 */

#include <common.h>
#include <clk.h>
#include <malloc.h>
#include <video.h>
#include <video_fb.h>
#include <video_bridge.h>
#include <video_link.h>

#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <linux/err.h>
#include <asm/io.h>
#include <asm/system.h>
#include <display.h>

#include "../videomodes.h"
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/fb.h>
#include "lcdifv3-regs.h"
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/device_compat.h>
#include <reset.h>

struct lcdifv3_soc_pdata {
	bool hsync_invert;
	bool vsync_invert;
	bool de_invert;
	bool hdmimix;
	bool hvsync_high;	/* mipi needs it high */
};

struct lcdifv3_priv {
	fdt_addr_t reg_base;
	struct udevice *disp_dev;
#if CONFIG_IS_ENABLED(CLK) && IS_ENABLED(CONFIG_IMX8M)
	struct clk lcdif_pix;
	struct clk *mix_clks;
#endif
};

static int lcdifv3_set_pix_fmt(struct lcdifv3_priv *priv, unsigned int format)
{
	uint32_t ctrldescl0_5 = 0;

	ctrldescl0_5 = readl((ulong)(priv->reg_base + LCDIFV3_CTRLDESCL0_5));

	WARN_ON(ctrldescl0_5 & CTRLDESCL0_5_SHADOW_LOAD_EN);

	ctrldescl0_5 &= ~(CTRLDESCL0_5_BPP(0xf) | CTRLDESCL0_5_YUV_FORMAT(0x3));

	switch (format) {
	case GDF_16BIT_565RGB:
		ctrldescl0_5 |= CTRLDESCL0_5_BPP(BPP16_RGB565);
		break;
	case GDF_32BIT_X888RGB:
		ctrldescl0_5 |= CTRLDESCL0_5_BPP(BPP32_ARGB8888);
		break;
	default:
		printf("unsupported pixel format: %u\n", format);
		return -EINVAL;
	}

	writel(ctrldescl0_5,  (ulong)(priv->reg_base + LCDIFV3_CTRLDESCL0_5));

	return 0;
}


static void lcdifv3_set_mode(struct lcdifv3_priv *priv,
		struct display_timing *timings)
{
	u32 disp_size, hsyn_para, vsyn_para, vsyn_hsyn_width, ctrldescl0_1;
	u32 set = 0;

	/* config display timings */
	disp_size = DISP_SIZE_DELTA_Y(timings->vactive.typ) |
		    DISP_SIZE_DELTA_X(timings->hactive.typ);
	writel(disp_size, (ulong)(priv->reg_base + LCDIFV3_DISP_SIZE));

	hsyn_para = HSYN_PARA_BP_H(timings->hback_porch.typ) |
		    HSYN_PARA_FP_H(timings->hfront_porch.typ);
	writel(hsyn_para, (ulong)(priv->reg_base + LCDIFV3_HSYN_PARA));

	vsyn_para = VSYN_PARA_BP_V(timings->vback_porch.typ) |
		    VSYN_PARA_FP_V(timings->vfront_porch.typ);
	writel(vsyn_para, (ulong)(priv->reg_base + LCDIFV3_VSYN_PARA));

	vsyn_hsyn_width = VSYN_HSYN_WIDTH_PW_V(timings->vsync_len.typ) |
			  VSYN_HSYN_WIDTH_PW_H(timings->hsync_len.typ);
	writel(vsyn_hsyn_width, (ulong)(priv->reg_base + LCDIFV3_VSYN_HSYN_WIDTH));

	/* config layer size */
	/* TODO: 32bits alignment for width */
	ctrldescl0_1 = CTRLDESCL0_1_HEIGHT(timings->vactive.typ) |
		       CTRLDESCL0_1_WIDTH(timings->hactive.typ);
	writel(ctrldescl0_1, (ulong)(priv->reg_base + LCDIFV3_CTRLDESCL0_1));

	/* Polarities */
	if (timings->flags & DISPLAY_FLAGS_HSYNC_LOW)
		set |= CTRL_INV_HS;
	if (timings->flags & DISPLAY_FLAGS_VSYNC_LOW)
		set |= CTRL_INV_VS;
	if (timings->flags & DISPLAY_FLAGS_DE_LOW)
		set |= CTRL_INV_DE;
	writel(set, (ulong)(priv->reg_base + LCDIFV3_CTRL_SET));
	writel(set ^ (CTRL_INV_HS | CTRL_INV_VS | CTRL_INV_DE),
			(ulong)(priv->reg_base + LCDIFV3_CTRL_CLR));

	/* SEC MIPI DSI specific */
	writel(CTRL_INV_PXCK, (ulong)(priv->reg_base + LCDIFV3_CTRL_CLR));
	writel(CTRL_INV_DE, (ulong)(priv->reg_base + LCDIFV3_CTRL_CLR));

}

static void lcdifv3_set_bus_fmt(struct lcdifv3_priv *priv)
{
	uint32_t disp_para = 0;

	disp_para = readl((ulong)(priv->reg_base + LCDIFV3_DISP_PARA));
	disp_para &= DISP_PARA_LINE_PATTERN(0xf);

	/* Fixed to 24 bits output */
	disp_para |= DISP_PARA_LINE_PATTERN(LP_RGB888_OR_YUV444);

	/* config display mode: default is normal mode */
	disp_para &= DISP_PARA_DISP_MODE(3);
	disp_para |= DISP_PARA_DISP_MODE(0);
	writel(disp_para, (ulong)(priv->reg_base + LCDIFV3_DISP_PARA));
}

static void lcdifv3_enable_controller(struct lcdifv3_priv *priv)
{
	u32 disp_para, ctrldescl0_5;

	disp_para = readl((ulong)(priv->reg_base + LCDIFV3_DISP_PARA));
	ctrldescl0_5 = readl((ulong)(priv->reg_base + LCDIFV3_CTRLDESCL0_5));

	/* disp on */
	disp_para |= DISP_PARA_DISP_ON;
	writel(disp_para, (ulong)(priv->reg_base + LCDIFV3_DISP_PARA));

	/* enable shadow load */
	ctrldescl0_5 |= CTRLDESCL0_5_SHADOW_LOAD_EN;
	writel(ctrldescl0_5, (ulong)(priv->reg_base + LCDIFV3_CTRLDESCL0_5));

	/* enable layer dma */
	ctrldescl0_5 |= CTRLDESCL0_5_EN;
	writel(ctrldescl0_5, (ulong)(priv->reg_base + LCDIFV3_CTRLDESCL0_5));
}

static void lcdifv3_disable_controller(struct lcdifv3_priv *priv)
{
	u32 disp_para, ctrldescl0_5;

	disp_para = readl((ulong)(priv->reg_base + LCDIFV3_DISP_PARA));
	ctrldescl0_5 = readl((ulong)(priv->reg_base + LCDIFV3_CTRLDESCL0_5));

	/* dma off */
	ctrldescl0_5 &= ~CTRLDESCL0_5_EN;
	writel(ctrldescl0_5, (ulong)(priv->reg_base + LCDIFV3_CTRLDESCL0_5));

	/* disp off */
	disp_para &= ~DISP_PARA_DISP_ON;
	writel(disp_para, (ulong)(priv->reg_base + LCDIFV3_DISP_PARA));
}

static void lcdifv3_init(struct udevice *dev,
		struct display_timing *timings, unsigned int format)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct lcdifv3_priv *priv = dev_get_priv(dev);
	int ret;

	/* Kick in the LCDIF clock */
#if !CONFIG_IS_ENABLED(CLK)
	mxs_set_lcdclk(priv->reg_base, timings->pixelclock.typ / 1000);
#endif

	writel(CTRL_SW_RESET, (ulong)(priv->reg_base + LCDIFV3_CTRL_CLR));

	lcdifv3_set_mode(priv, timings);

	lcdifv3_set_bus_fmt(priv);

	ret = lcdifv3_set_pix_fmt(priv, format);
	if (ret) {
		printf("Fail to init lcdifv3, wrong format %u\n", format);
		return;
	}

	/* Set fb address to primary layer */
	writel(plat->base, (ulong)(priv->reg_base + LCDIFV3_CTRLDESCL_LOW0_4));

	writel(CTRLDESCL0_3_P_SIZE(1) |CTRLDESCL0_3_T_SIZE(1) | CTRLDESCL0_3_PITCH(timings->hactive.typ * 4),
		(ulong)(priv->reg_base + LCDIFV3_CTRLDESCL0_3));

	lcdifv3_enable_controller(priv);
}

void lcdifv3_power_down(struct lcdifv3_priv *priv)
{
	int timeout = 1000000;

	/* Disable LCDIF during VBLANK */
	writel(INT_STATUS_D0_VS_BLANK,
		(ulong)(priv->reg_base + LCDIFV3_INT_STATUS_D0));
	while (--timeout) {
		if (readl((ulong)(priv->reg_base + LCDIFV3_INT_STATUS_D0)) &
		    INT_STATUS_D0_VS_BLANK)
			break;
		udelay(1);
	}

	lcdifv3_disable_controller(priv);
}

static int lcdifv3_of_get_timings(struct udevice *dev,
			      struct display_timing *timings)
{
	int ret = 0;
	struct lcdifv3_priv *priv = dev_get_priv(dev);

	priv->disp_dev = video_link_get_next_device(dev);
	if (!priv->disp_dev ||
		(device_get_uclass_id(priv->disp_dev) != UCLASS_VIDEO_BRIDGE
		&& device_get_uclass_id(priv->disp_dev) != UCLASS_DISPLAY)) {

		printf("fail to find output device\n");
		return -ENODEV;
	}

	debug("disp_dev %s\n", priv->disp_dev->name);

	ret = video_link_get_display_timings(timings);
	if (ret) {
		printf("fail to get display timings\n");
		return ret;
	}

	return ret;
}

#if CONFIG_IS_ENABLED(CLK)
u32 pll_rates[] = {
	  24000000U,
	 135000000U,
	 148500000U,
	 360000000U,
	 361267200U,
	 364000000U,
	 384000000U,
	 393216000U,
	 452900000U,
	 453000000U,
	 497755966U,
	 519750000U,
	 594000000U,
	 650000000U,
	 756000000U,
	1039500000U,
};

static u32 get_pixclock(struct clk *pll, unsigned long pixclock, int ldb)
{
	unsigned long rate;
	unsigned long best = 0, best_n = 1, best_diff = ~0;
	unsigned long cur, cur_n, cur_diff;
	int i;
	int ret;


	for (i = 0; i < ARRAY_SIZE(pll_rates); i++) {
		rate = pll_rates[i];
		if (!ldb)
			cur_n = (rate + (pixclock >> 1)) / pixclock;
		else
			cur_n = 7;
		cur = rate / cur_n;
		if (cur >= pixclock)
			cur_diff = cur - pixclock;
		else
			cur_diff = pixclock - cur;
		if (best_diff > cur_diff) {
			best_diff = cur_diff;
			best = cur;
			best_n = cur_n;
		}
		if (!cur_diff)
			break;
	}
	cur = best * best_n;
	debug("%s: %ld = %ld * %ld\n", __func__, cur, best, best_n);
	ret = clk_set_rate(pll, cur);
	if (ret < 0) {
		debug("clk_set_rate %ld failed(%d)\n", cur, ret);
		cur = clk_get_rate(pll);
		debug("rate is %ld\n", cur);
	}
	best_n = (cur + (pixclock >> 1)) / pixclock;
	pixclock = cur / best_n;
	debug("%s: pixclock = %ld\n", __func__, pixclock);
	return pixclock;
}
#endif

const char* const mix_clocks[] = {
	"mix_apb",
	"mix_axi",
	"xtl_24m",
	"mix_pix",
	"lcdif_apb",
	"lcdif_axi",
	"lcdif_pdi",
	"lcdif_pix",
	"lcdif_spu",
	"noc_hdmi",
};

static int hdmimix_lcdif3_setup(struct lcdifv3_priv *priv, struct udevice *dev)
{
	int ret;

	debug("%s:\n", __func__);
#if CONFIG_IS_ENABLED(DM_RESET)
	device_reset(dev);
#endif
	/* enable lpcg of hdmimix lcdif and nor */
#if CONFIG_IS_ENABLED(CLK) && IS_ENABLED(CONFIG_IMX8M)
	ret = devm_clk_get_enable_bulk(dev, mix_clocks, ARRAY_SIZE(mix_clocks), 0, &priv->mix_clks);
	if (ret) {
		debug("%s: ret=%d\n", __func__, ret);
		return ret;
	}
#endif
	return 0;
}

static int lcdifv3_video_probe(struct udevice *dev)
{
	struct lcdifv3_soc_pdata *pdata = (struct lcdifv3_soc_pdata *)dev_get_driver_data(dev);
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct lcdifv3_priv *priv = dev_get_priv(dev);
	struct display_timing timings;

	ulong fb_start, fb_end;
#if CONFIG_IS_ENABLED(CLK)
	struct clk* video_pll;
	struct clk* clk_ldb;
	u32 pixclock;
#endif
	int ret;

	debug("%s() plat: base 0x%lx, size 0x%x\n",
	       __func__, plat->base, plat->size);

	priv->reg_base = dev_read_addr(dev);
	if (priv->reg_base == FDT_ADDR_T_NONE) {
		dev_err(dev, "lcdif base address is not found\n");
		return -EINVAL;
	}

	ret = lcdifv3_of_get_timings(dev, &timings);
	if (ret)
		return ret;

#if CONFIG_IS_ENABLED(CLK)
	clk_ldb = devm_clk_get_optional(dev, "ldb");
	if (IS_ERR(clk_ldb))
		return PTR_ERR(clk_ldb);

	video_pll = devm_clk_get_optional(dev, "video_pll");
	if (IS_ERR(video_pll))
		return PTR_ERR(video_pll);

	if (video_pll) {
		pixclock = get_pixclock(video_pll, timings.pixelclock.typ, clk_ldb ? 1 : 0);
	} else {
		pixclock = timings.pixelclock.typ;
	}
	ret = clk_get_by_name(dev, "pix", &priv->lcdif_pix);
	if (ret) {
		printf("Failed to get pix clk\n");
		return ret;
	}

	if (clk_ldb) {
		ret = clk_set_rate(video_pll, pixclock * 7);
		if (ret < 0) {
			printf("Failed to set pll rate(%d) %d\n", pixclock, ret);
			return ret;
		}
	}

	ret = clk_set_rate(&priv->lcdif_pix, pixclock);
	if (ret < 0) {
		printf("Failed to set pix clk rate(%d) %d\n", pixclock, ret);
		return ret;
	}
#endif
	if (priv->disp_dev) {
#if IS_ENABLED(CONFIG_VIDEO_BRIDGE)
		if (device_get_uclass_id(priv->disp_dev) == UCLASS_VIDEO_BRIDGE) {
			ret = video_bridge_attach(priv->disp_dev);
			if (ret) {
				dev_err(dev, "fail to attach bridge\n");
				return ret;
			}
		}
#endif
#ifdef CONFIG_DISPLAY
		if (device_get_uclass_id(priv->disp_dev) == UCLASS_DISPLAY) {
			display_enable(priv->disp_dev, 32, &timings);
			if (ret) {
				dev_err(dev, "display_enable failed %d\n", ret);
			}
		}
#endif
	}

	if (pdata->hvsync_high) {
		/* mipi needs high */
		timings.flags |= (DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH);
		timings.flags &= ~(DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW);
	}
	if (pdata->hsync_invert)
		timings.flags ^= (DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_HSYNC_HIGH);
	if (pdata->vsync_invert)
		timings.flags ^= (DISPLAY_FLAGS_VSYNC_LOW | DISPLAY_FLAGS_VSYNC_HIGH);
	if (pdata->de_invert)
		timings.flags ^= (DISPLAY_FLAGS_DE_LOW | DISPLAY_FLAGS_DE_HIGH);
	if (pdata->hdmimix) {
		ret = hdmimix_lcdif3_setup(priv, dev);
		if (ret < 0) {
			debug("hdmimix lcdif3 setup failed\n");
			return ret;
		}
	}

	uc_priv->bpix = VIDEO_BPP32; /* only support 32 BPP now */
	uc_priv->xsize = timings.hactive.typ;
	uc_priv->ysize = timings.vactive.typ;

	/* Enable dcache for the frame buffer */
	fb_start = plat->base & ~(MMU_SECTION_SIZE - 1);
	fb_end = plat->base + plat->size;
	fb_end = ALIGN(fb_end, 1 << MMU_SECTION_SHIFT);
	/* clear framebuffer */
	memset((void *)fb_start, 0, timings.hactive.typ * timings.vactive.typ << 2);

	mmu_set_region_dcache_behaviour(fb_start, fb_end - fb_start,
					DCACHE_WRITEBACK);
	video_set_flush_dcache(dev, true);
	gd->fb_base = plat->base;

	lcdifv3_init(dev, &timings, GDF_32BIT_X888RGB);

#if IS_ENABLED(CONFIG_VIDEO_BRIDGE)
	if (priv->disp_dev) {
		if (device_get_uclass_id(priv->disp_dev) == UCLASS_VIDEO_BRIDGE) {
			ret = video_bridge_set_backlight(priv->disp_dev, 80);
			if (ret) {
				dev_err(dev, "fail to set backlight\n");
				return ret;
			}
		}
	}
#endif

	return ret;
}

static int lcdifv3_video_bind(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);

	/* Max size supported by LCDIF, because in bind, we can't probe panel */
	plat->size = 1920 * 1080 *4 * 2;

	return 0;
}

static int lcdifv3_video_remove(struct udevice *dev)
{
	struct lcdifv3_priv *priv = dev_get_priv(dev);

	debug("%s\n", __func__);

	if (priv->disp_dev)
		device_remove(priv->disp_dev, DM_REMOVE_NORMAL);

	lcdifv3_power_down(priv);

	return 0;
}

static const struct lcdifv3_soc_pdata imx8mp_lcdif1_pdata = {
	.hsync_invert = false,
	.vsync_invert = false,
	.de_invert    = false,
	.hdmimix     = false,
	.hvsync_high = true,
};

static const struct lcdifv3_soc_pdata imx8mp_lcdif2_pdata = {
	.hsync_invert = false,
	.vsync_invert = false,
	.de_invert    = true,
	.hdmimix      = false,
};

static const struct lcdifv3_soc_pdata imx8mp_lcdif3_pdata = {
	.hsync_invert = false,
	.vsync_invert = false,
	.de_invert    = false,
	.hdmimix     = true,
};

static const struct udevice_id lcdifv3_video_ids[] = {
	{ .compatible = "fsl,imx8mp-lcdif1", .data = (ulong)&imx8mp_lcdif1_pdata,},
	{ .compatible = "fsl,imx8mp-lcdif2", .data = (ulong)&imx8mp_lcdif2_pdata,},
	{ .compatible = "fsl,imx8mp-lcdif3", .data = (ulong)&imx8mp_lcdif3_pdata,},
	{ /* sentinel */ }
};

U_BOOT_DRIVER(lcdifv3_video) = {
	.name	= "lcdifv3_video",
	.id	= UCLASS_VIDEO,
	.of_match = lcdifv3_video_ids,
	.bind	= lcdifv3_video_bind,
	.probe	= lcdifv3_video_probe,
	.remove = lcdifv3_video_remove,
	.flags	= DM_FLAG_PRE_RELOC | DM_FLAG_OS_PREPARE,
	.priv_auto = sizeof(struct lcdifv3_priv),
};
