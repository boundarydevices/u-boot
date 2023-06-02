// SPDX-License-Identifier: GPL-2.0+
/*
 * Freescale i.MX23/i.MX28 LCDIF driver
 *
 * Copyright (C) 2011-2013 Marek Vasut <marex@denx.de>
 * Copyright (C) 2014-2016 Freescale Semiconductor, Inc.
 *
 */
#include <common.h>
#include <clk.h>
#include <dm.h>
#include <env.h>
#include <log.h>
#include <asm/cache.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <malloc.h>
#include <video.h>
#include <video_fb.h>
#if CONFIG_IS_ENABLED(CLK) && IS_ENABLED(CONFIG_IMX8)
#include <clk.h>
#else
#include <asm/arch/clock.h>
#endif
#include <asm/arch/imx-regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <stdio_dev.h>

#include <asm/mach-imx/dma.h>
#include <asm/io.h>
#include <reset.h>
#include <panel.h>
#include <video_bridge.h>
#include <video_link.h>

#include "videomodes.h"
#include <linux/string.h>
#include <linux/list.h>
#include <linux/fb.h>
#include <mxsfb.h>
#include <dm/device-internal.h>


#ifdef CONFIG_VIDEO_GIS
#include <gis.h>
#endif

#define	PS2KHZ(ps)	(1000000000UL / (ps))
#define HZ2PS(hz)	(1000000000UL / ((hz) / 1000))

#define BITS_PP		18
#define BYTES_PP	4

struct mxs_dma_desc desc;

/**
 * mxsfb_system_setup() - Fine-tune LCDIF configuration
 *
 * This function is used to adjust the LCDIF configuration. This is usually
 * needed when driving the controller in System-Mode to operate an 8080 or
 * 6800 connected SmartLCD.
 */
__weak void mxsfb_system_setup(void)
{
}

#if CONFIG_IS_ENABLED(CLK)
static u32 get_pixclock(struct clk *pll, unsigned long pixclock)
{
	u32 video_pll, n;
	int ret;

	video_pll = pixclock;
	/* Video pll must be from 580MHz to 2000 MHz */
	if (video_pll < 580000000) {
		int n = (580000000 + video_pll - 1) / video_pll;
		int bit;

		do {
			bit = __ffs(n);
			if ((n >> bit) <= 7)
				break;
			n += (1 << bit);
		} while (1);
		video_pll *= n;
		debug("%s: %d = %ld * %d\n", __func__, video_pll, pixclock, n);
	}
	ret = clk_set_rate(pll, video_pll);
	if (ret < 0) {
		debug("clk_set_rate %d failed(%d)\n", video_pll, ret);
		video_pll = clk_get_rate(pll);
		debug("rate is %d\n", video_pll);
	}
	n = (video_pll + (pixclock >> 1)) / pixclock;
	pixclock = video_pll / n;
	return pixclock;
}
#endif

/*
 * ARIES M28EVK:
 * setenv videomode
 * video=ctfb:x:800,y:480,depth:18,mode:0,pclk:30066,
 *       le:0,ri:256,up:0,lo:45,hs:1,vs:1,sync:100663296,vmode:0
 *
 * Freescale mx23evk/mx28evk with a Seiko 4.3'' WVGA panel:
 * setenv videomode
 * video=ctfb:x:800,y:480,depth:24,mode:0,pclk:29851,
 *	 le:89,ri:164,up:23,lo:10,hs:10,vs:10,sync:0,vmode:0
 */

static void mxs_lcd_init(struct udevice *dev, phys_addr_t reg_base, u32 fb_addr,
			 struct display_timing *timings, int bpp, bool bridge,
			 bool enable_pol)
{
	struct mxs_lcdif_regs *regs = (struct mxs_lcdif_regs *)reg_base;
	const enum display_flags flags = timings->flags;
	uint32_t word_len = 0, bus_width = 0;
	uint8_t valid_data = 0;
	uint32_t vdctrl0;

#if CONFIG_IS_ENABLED(CLK)
	struct clk clk;
	struct clk pll;
	int ret;
	u32 pixclock = timings->pixelclock.typ;


	ret = clk_get_by_name(dev, "video_pll", &pll);
	if (!ret) {
		pixclock = get_pixclock(&pll, pixclock);
	}
	ret = clk_get_by_name(dev, "pix", &clk);
	if (ret) {
		dev_err(dev, "Failed to get mxs pix clk: %d\n", ret);
		return;
	}

	debug("%s: rate %u, bpp=%u\n", __func__, pixclock, bpp);
	ret = clk_set_rate(&clk, pixclock);
	if (ret < 0) {
		dev_err(dev, "Failed to set mxs pix clk: %d\n", ret);
		return;
	}

	ret = clk_enable(&clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable mxs pix clk: %d\n", ret);
		return;
	}

	ret = clk_get_by_name(dev, "axi", &clk);
	if (ret < 0) {
		debug("%s: Failed to get mxs axi clk: %d\n", __func__, ret);
	} else {
		ret = clk_enable(&clk);
		if (ret < 0) {
			dev_err(dev, "Failed to enable mxs axi clk: %d\n", ret);
			return;
		}
	}

	ret = clk_get_by_name(dev, "disp_axi", &clk);
	if (ret < 0) {
		debug("%s: Failed to get mxs disp_axi clk: %d\n", __func__, ret);
	} else {
		ret = clk_enable(&clk);
		if (ret < 0) {
			dev_err(dev, "Failed to enable mxs disp_axi clk: %d\n", ret);
			return;
		}
	}
#else
#if !defined(CONFIG_CMD_FBPANEL)
#if !(CONFIG_IS_ENABLED(CLK) && IS_ENABLED(CONFIG_IMX8))
	/* Kick in the LCDIF clock */
	mxs_set_lcdclk((u32)reg_base, timings->pixelclock.typ / 1000);
#endif
#endif
#endif

	/* Restart the LCDIF block */
	mxs_reset_block(&regs->hw_lcdif_ctrl_reg);

	switch (bpp) {
	case 24:
		word_len = LCDIF_CTRL_WORD_LENGTH_24BIT;
		bus_width = LCDIF_CTRL_LCD_DATABUS_WIDTH_24BIT;
		valid_data = 0x7;
		break;
	case 18:
		word_len = LCDIF_CTRL_WORD_LENGTH_24BIT;
		bus_width = LCDIF_CTRL_LCD_DATABUS_WIDTH_18BIT;
		valid_data = 0x7;
		break;
	case 16:
		word_len = LCDIF_CTRL_WORD_LENGTH_16BIT;
		bus_width = LCDIF_CTRL_LCD_DATABUS_WIDTH_16BIT;
		valid_data = 0xf;
		break;
	case 8:
		word_len = LCDIF_CTRL_WORD_LENGTH_8BIT;
		bus_width = LCDIF_CTRL_LCD_DATABUS_WIDTH_8BIT;
		valid_data = 0xf;
		break;
	}

	writel(bus_width | word_len | LCDIF_CTRL_DOTCLK_MODE |
		LCDIF_CTRL_BYPASS_COUNT | LCDIF_CTRL_LCDIF_MASTER,
		&regs->hw_lcdif_ctrl);

	writel(valid_data << LCDIF_CTRL1_BYTE_PACKING_FORMAT_OFFSET,
		&regs->hw_lcdif_ctrl1);

	if (bridge)
		writel(LCDIF_CTRL2_OUTSTANDING_REQS_REQ_16, &regs->hw_lcdif_ctrl2);

	mxsfb_system_setup();

	writel((timings->vactive.typ << LCDIF_TRANSFER_COUNT_V_COUNT_OFFSET) |
		timings->hactive.typ, &regs->hw_lcdif_transfer_count);

	vdctrl0 = LCDIF_VDCTRL0_ENABLE_PRESENT | LCDIF_VDCTRL0_ENABLE_POL |
		  LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT |
		  LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT |
		  timings->vsync_len.typ;

	if(flags & DISPLAY_FLAGS_HSYNC_HIGH)
		vdctrl0 |= LCDIF_VDCTRL0_HSYNC_POL;
	if(flags & DISPLAY_FLAGS_VSYNC_HIGH)
		vdctrl0 |= LCDIF_VDCTRL0_VSYNC_POL;
	if(flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		vdctrl0 |= LCDIF_VDCTRL0_DOTCLK_POL;
	if(flags & DISPLAY_FLAGS_DE_HIGH)
		vdctrl0 |= LCDIF_VDCTRL0_ENABLE_POL;

	if (enable_pol)
		vdctrl0 |= LCDIF_VDCTRL0_ENABLE_POL;
	else
		vdctrl0 &= ~LCDIF_VDCTRL0_ENABLE_POL;

	writel(vdctrl0, &regs->hw_lcdif_vdctrl0);
	writel(timings->vback_porch.typ + timings->vfront_porch.typ +
		timings->vsync_len.typ + timings->vactive.typ,
		&regs->hw_lcdif_vdctrl1);
	writel((timings->hsync_len.typ << LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH_OFFSET) |
		(timings->hback_porch.typ + timings->hfront_porch.typ +
		timings->hsync_len.typ + timings->hactive.typ),
		&regs->hw_lcdif_vdctrl2);
	writel(((timings->hback_porch.typ + timings->hsync_len.typ) <<
		LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT_OFFSET) |
		(timings->vback_porch.typ + timings->vsync_len.typ),
		&regs->hw_lcdif_vdctrl3);
	writel((0 << LCDIF_VDCTRL4_DOTCLK_DLY_SEL_OFFSET) | timings->hactive.typ,
		&regs->hw_lcdif_vdctrl4);

	writel(fb_addr, &regs->hw_lcdif_cur_buf);
	writel(fb_addr, &regs->hw_lcdif_next_buf);

	/* Flush FIFO first */
	writel(LCDIF_CTRL1_FIFO_CLEAR, &regs->hw_lcdif_ctrl1_set);

#ifndef CONFIG_VIDEO_MXS_MODE_SYSTEM
	/* Sync signals ON */
	setbits_le32(&regs->hw_lcdif_vdctrl4, LCDIF_VDCTRL4_SYNC_SIGNALS_ON);
#endif

	/* FIFO cleared */
	writel(LCDIF_CTRL1_FIFO_CLEAR, &regs->hw_lcdif_ctrl1_clr);
}

static int mxs_probe_common(struct udevice *dev, phys_addr_t reg_base, struct display_timing *timings,
			    int bpp, u32 fb, bool bridge, bool enable_pol)
{
	/* Start framebuffer */
	mxs_lcd_init(dev, reg_base, fb, timings, bpp, bridge, enable_pol);
	return 0;
}

static void mxs_lcd_run(void)
{
	struct mxs_lcdif_regs *regs = (struct mxs_lcdif_regs *)MXS_LCDIF_BASE;

	/* De-assert LCD Reset bit */
	writel(LCDIF_CTRL1_RESET, &regs->hw_lcdif_ctrl1_set);

	/* RUN! */
	writel(LCDIF_CTRL_RUN, &regs->hw_lcdif_ctrl_set);
	debug("%s: running\n", __func__);

#ifdef CONFIG_VIDEO_MXS_MODE_SYSTEM
	/*
	 * If the LCD runs in system mode, the LCD refresh has to be triggered
	 * manually by setting the RUN bit in HW_LCDIF_CTRL register. To avoid
	 * having to set this bit manually after every single change in the
	 * framebuffer memory, we set up specially crafted circular DMA, which
	 * sets the RUN bit, then waits until it gets cleared and repeats this
	 * infinitelly. This way, we get smooth continuous updates of the LCD.
	 */
	struct mxs_lcdif_regs *regs = (struct mxs_lcdif_regs *)reg_base;

	memset(&desc, 0, sizeof(struct mxs_dma_desc));
	desc.address = (dma_addr_t)&desc;
	desc.cmd.data = MXS_DMA_DESC_COMMAND_NO_DMAXFER | MXS_DMA_DESC_CHAIN |
			MXS_DMA_DESC_WAIT4END |
			(1 << MXS_DMA_DESC_PIO_WORDS_OFFSET);
	desc.cmd.pio_words[0] = readl(&regs->hw_lcdif_ctrl) | LCDIF_CTRL_RUN;
	desc.cmd.next = (uint32_t)&desc.cmd;

	/* Execute the DMA chain. */
	mxs_dma_circ_start(MXS_DMA_CHANNEL_AHB_APBH_LCDIF, &desc);
#endif
}

static int mxs_remove_common(phys_addr_t reg_base, u32 fb)
{
	struct mxs_lcdif_regs *regs = (struct mxs_lcdif_regs *)(reg_base);
	int timeout = 1000000;

	if (!fb)
		return -EINVAL;

	writel(fb, &regs->hw_lcdif_cur_buf_reg);
	writel(fb, &regs->hw_lcdif_next_buf_reg);
	writel(LCDIF_CTRL1_VSYNC_EDGE_IRQ, &regs->hw_lcdif_ctrl1_clr);
	while (--timeout) {
		if (readl(&regs->hw_lcdif_ctrl1_reg) &
		    LCDIF_CTRL1_VSYNC_EDGE_IRQ)
			break;
		udelay(1);
	}
	/* Assert LCD Reset bit */
	writel(LCDIF_CTRL1_RESET, &regs->hw_lcdif_ctrl1_clr);

	mxs_reset_block((struct mxs_register_32 *)&regs->hw_lcdif_ctrl_reg);

	return 0;
}

#ifndef CONFIG_DM_VIDEO

static struct graphic_device panel;
static int setup;
static struct fb_videomode fbmode;
static int depth;

int mxs_lcd_panel_setup(struct fb_videomode mode, int bpp,
	uint32_t base_addr)
{
	fbmode = mode;
	depth  = bpp;
	panel.isaBase  = base_addr;

	setup = 1;

	return 0;
}

void mxs_lcd_get_panel(struct display_panel *dispanel)
{
	dispanel->width = fbmode.xres;
	dispanel->height = fbmode.yres;
	dispanel->reg_base = panel.isaBase;
	dispanel->gdfindex = panel.gdfIndex;
	dispanel->gdfbytespp = panel.gdfBytesPP;
}

void lcdif_power_down(void)
{
	mxs_remove_common(panel.isaBase, panel.frameAdrs);
}

static void __board_video_enable(void)
{
}

void board_video_enable(void)
	__attribute__((weak, alias("__board_video_enable")));

static struct graphic_device *mxsfb_probe(int bpp, struct ctfb_res_modes *mode)
{
	struct display_timing timings;
	void *fb;
	unsigned mem_size;
	int ret;

	/* fill in Graphic device struct */
	sprintf(panel.modeIdent, "%dx%dx%d", mode->xres, mode->yres, bpp);

	panel.winSizeX = mode->xres;
	panel.winSizeY = mode->yres;
	panel.plnSizeX = mode->xres;
	panel.plnSizeY = mode->yres;

	switch (bpp) {
	case 24:
	case 18:
		panel.gdfBytesPP = 4;
		panel.gdfIndex = GDF_32BIT_X888RGB;
		break;
	case 16:
		panel.gdfBytesPP = 2;
		panel.gdfIndex = GDF_16BIT_565RGB;
		break;
	case 8:
		panel.gdfBytesPP = 1;
		panel.gdfIndex = GDF__8BIT_INDEX;
		break;
	default:
		printf("MXSFB: Invalid BPP specified! (bpp = %i)\n", bpp);
		return NULL;
	}

	mem_size = mode->xres * mode->yres * panel.gdfBytesPP;
	fb = (void *)panel.frameAdrs;
	if (fb) {
		if (panel.memSize < mem_size) {
			free(fb);
			fb = NULL;
			panel.frameAdrs = (u32)fb;
		}
	}
	if (!fb) {
		/* Allocate framebuffer */
		fb = memalign(ARCH_DMA_MINALIGN,
			roundup(mem_size, ARCH_DMA_MINALIGN));
		if (!fb) {
			printf("MXSFB: Error allocating framebuffer!\n");
			return NULL;
		}
		panel.memSize = mem_size;
		panel.frameAdrs = (u32)fb;
	}
	/* Wipe framebuffer */
	memset(fb, 0, mem_size);

	video_ctfb_mode_to_display_timing(mode, &timings);

	ret = mxs_probe_common(NULL, panel.isaBase, &timings, bpp, (u32)fb,
			false, true);
	if (ret)
		goto dealloc_fb;

#ifdef CONFIG_VIDEO_GIS
	/* Entry for GIS */
	mxc_enable_gis();
#endif

	board_video_enable();
	mxs_lcd_run();
	return (void *)&panel;

dealloc_fb:
	free(fb);

	return NULL;
}
#else /* ifndef CONFIG_DM_VIDEO */

struct mxsfb_priv {
	fdt_addr_t reg_base;
	struct udevice *disp_dev;

#if IS_ENABLED(CONFIG_DM_RESET)
	struct reset_ctl_bulk soft_resetn;
	struct reset_ctl_bulk clk_enable;
#endif

#if CONFIG_IS_ENABLED(CLK) && IS_ENABLED(CONFIG_IMX8)
	struct clk			lcdif_pix;
	struct clk			lcdif_disp_axi;
	struct clk			lcdif_axi;
#endif
};

#if IS_ENABLED(CONFIG_DM_RESET)
static int lcdif_rstc_reset(struct reset_ctl_bulk *rstc, bool assert)
{
	int ret;

	if (!rstc)
		return 0;

	ret = assert ? reset_assert_bulk(rstc)	:
		       reset_deassert_bulk(rstc);

	return ret;
}

static int lcdif_of_parse_resets(struct udevice *dev)
{
	int ret;
	ofnode parent, child;
	struct ofnode_phandle_args args;
	struct reset_ctl_bulk rstc;
	const char *compat;
	uint32_t rstc_num = 0;

	struct mxsfb_priv *priv = dev_get_priv(dev);

	ret = dev_read_phandle_with_args(dev, "resets", "#reset-cells", 0,
					 0, &args);
	if (ret)
		return ret;

	parent = args.node;
	ofnode_for_each_subnode(child, parent) {
		compat = ofnode_get_property(child, "compatible", NULL);
		if (!compat)
			continue;

		ret = reset_get_bulk_nodev(child, &rstc);
		if (ret)
			continue;

		if (!of_compat_cmp("lcdif,soft-resetn", compat, 0)) {
			priv->soft_resetn = rstc;
			rstc_num++;
		} else if (!of_compat_cmp("lcdif,clk-enable", compat, 0)) {
			priv->clk_enable = rstc;
			rstc_num++;
		}
		else
			dev_warn(dev, "invalid lcdif reset node: %s\n", compat);
	}

	if (!rstc_num) {
		dev_err(dev, "no invalid reset control exists\n");
		return -EINVAL;
	}

	return 0;
}
#endif

static int mxs_of_get_timings(struct udevice *dev,
			      struct display_timing *timings,
			      u32 *bpp)
{
	int ret = 0;
	u32 display_phandle;
	ofnode display_node;
	struct mxsfb_priv *priv = dev_get_priv(dev);

	ret = ofnode_read_u32(dev_ofnode(dev), "display", &display_phandle);
	if (ret) {
		dev_err(dev, "required display property isn't provided\n");
		return -EINVAL;
	}

	display_node = ofnode_get_by_phandle(display_phandle);
	if (!ofnode_valid(display_node)) {
		dev_err(dev, "failed to find display subnode\n");
		return -EINVAL;
	}

	ret = ofnode_read_u32(display_node, "bits-per-pixel", bpp);
	if (ret) {
		dev_err(dev,
			"required bits-per-pixel property isn't provided\n");
		return -EINVAL;
	}

	ret = ofnode_decode_display_timing(display_node, 0, timings);
	if (ret) {
		dev_info(dev, "failed to get display timings\n");
	}

	priv->disp_dev = video_link_get_next_device(dev);
	if (priv->disp_dev && ret) {
		ret = video_link_get_display_timings(timings);
		if (ret) {
			dev_err(dev, "failed to get any video link display timings\n");
			return -EINVAL;
		}
	}

	return ret;
}

static int mxs_video_probe(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct mxsfb_priv *priv = dev_get_priv(dev);

	struct display_timing timings;
	u32 bpp = 0;
	ulong fb_start, fb_end;
	int ret;
	bool enable_pol = true, enable_bridge = false;

	debug("%s() plat: base 0x%lx, size 0x%x\n",
	       __func__, plat->base, plat->size);

	priv->reg_base = dev_read_addr(dev);
	if (priv->reg_base == FDT_ADDR_T_NONE) {
		dev_err(dev, "lcdif base address is not found\n");
		return -EINVAL;
	}

	ret = mxs_of_get_timings(dev, &timings, &bpp);
	if (ret)
		return ret;

#if CONFIG_IS_ENABLED(CLK) && IS_ENABLED(CONFIG_IMX8)
	ret = clk_get_by_name(dev, "pix", &priv->lcdif_pix);
	if (ret) {
		printf("Failed to get pix clk\n");
		return ret;
	}

	ret = clk_get_by_name(dev, "disp_axi", &priv->lcdif_disp_axi);
	if (ret) {
		printf("Failed to get disp_axi clk\n");
		return ret;
	}

	ret = clk_get_by_name(dev, "axi", &priv->lcdif_axi);
	if (ret) {
		printf("Failed to get axi clk\n");
		return ret;
	}

	ret = clk_enable(&priv->lcdif_axi);
	if (ret) {
		printf("unable to enable lcdif_axi clock\n");
		return ret;
	}

	ret = clk_enable(&priv->lcdif_disp_axi);
	if (ret) {
		printf("unable to enable lcdif_disp_axi clock\n");
		return ret;
	}
#endif

#if IS_ENABLED(CONFIG_DM_RESET)
	ret = lcdif_of_parse_resets(dev);
	if (!ret) {
		ret = lcdif_rstc_reset(&priv->soft_resetn, false);
		if (ret) {
			dev_err(dev, "deassert soft_resetn failed\n");
			return ret;
		}

		ret = lcdif_rstc_reset(&priv->clk_enable, true);
		if (ret) {
			dev_err(dev, "assert clk_enable failed\n");
			return ret;
		}
	}
#endif

#if IS_ENABLED(CONFIG_VIDEO_BRIDGE)
	if (priv->disp_dev) {
		if (device_get_uclass_id(priv->disp_dev) == UCLASS_VIDEO_BRIDGE) {
			enable_bridge = true;
		}
		/* sec dsim needs enable ploarity at low, default we set to high */
		if (dev_read_bool(dev, "enable_polarity_low")) {
			enable_pol = false;
			timings.flags &= ~DISPLAY_FLAGS_DE_HIGH;
		}
	}
#endif

#if CONFIG_IS_ENABLED(CLK) && IS_ENABLED(CONFIG_IMX8)
	ret = clk_set_rate(&priv->lcdif_pix, timings.pixelclock.typ);
	if (ret < 0) {
		printf("Failed to set pix clk rate\n");
		return ret;
	}

	ret = clk_enable(&priv->lcdif_pix);
	if (ret) {
		printf("unable to enable lcdif_pix clock\n");
		return ret;
	}
#endif

	ret = mxs_probe_common(dev, priv->reg_base, &timings, bpp, plat->base, enable_bridge, enable_pol);
	if (ret)
		return ret;

	switch (bpp) {
	case 32:
	case 24:
	case 18:
		uc_priv->bpix = VIDEO_BPP32;
		break;
	case 16:
		uc_priv->bpix = VIDEO_BPP16;
		break;
	case 8:
		uc_priv->bpix = VIDEO_BPP8;
		break;
	default:
		dev_err(dev, "invalid bpp specified (bpp = %i)\n", bpp);
		return -EINVAL;
	}

	uc_priv->xsize = timings.hactive.typ;
	uc_priv->ysize = timings.vactive.typ;

	/* Enable dcache for the frame buffer */
	fb_start = plat->base;
	fb_end = plat->base + plat->size;
	/* clear framebuffer */
	memset((void *)fb_start, 0, timings.hactive.typ * timings.vactive.typ << 2);

	mmu_set_region_dcache_behaviour(fb_start, fb_end - fb_start,
					DCACHE_WRITEBACK);
	video_set_flush_dcache(dev, true);
	gd->fb_base = plat->base;

#if IS_ENABLED(CONFIG_VIDEO_BRIDGE)
	if (enable_bridge) {
		ret = video_bridge_attach(priv->disp_dev);
		if (ret) {
			dev_err(dev, "fail to attach bridge\n");
			return ret;
		}
	}
#endif

	mxs_lcd_run();
#if IS_ENABLED(CONFIG_VIDEO_BRIDGE)
	if (enable_bridge) {
		ret = video_bridge_set_backlight(priv->disp_dev, 80);
		if (ret) {
			dev_err(dev, "fail to set backlight\n");
			return ret;
		}
	}
#endif
	if (priv->disp_dev) {
		if (device_get_uclass_id(priv->disp_dev) == UCLASS_PANEL) {
			ret = panel_enable_backlight(priv->disp_dev);
			if (ret) {
				dev_err(dev, "panel %s enable backlight error %d\n",
					priv->disp_dev->name, ret);
				return ret;
			}
		}
	}

	return ret;
}

static int mxs_video_bind(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);

	/* Max size supported by LCDIF, because in bind, we can't probe panel */
	plat->size = ALIGN(1920 * 1080 *4 * 2, MMU_SECTION_SIZE);
	plat->align = MMU_SECTION_SIZE;

	return 0;
}

static int mxs_video_remove(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct mxsfb_priv *priv = dev_get_priv(dev);

	debug("%s\n", __func__);

	if (priv->disp_dev)
		device_remove(priv->disp_dev, DM_REMOVE_NORMAL);

	mxs_remove_common(priv->reg_base, plat->base);

	return 0;
}

static const struct udevice_id mxs_video_ids[] = {
	{ .compatible = "fsl,imx23-lcdif" },
	{ .compatible = "fsl,imx28-lcdif" },
	{ .compatible = "fsl,imx7ulp-lcdif" },
	{ .compatible = "fsl,imxrt-lcdif" },
	{ .compatible = "fsl,imx8mm-lcdif" },
	{ .compatible = "fsl,imx8mn-lcdif" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(mxs_video) = {
	.name	= "mxs_video",
	.id	= UCLASS_VIDEO,
	.of_match = mxs_video_ids,
	.bind	= mxs_video_bind,
	.probe	= mxs_video_probe,
	.remove = mxs_video_remove,
	.flags	= DM_FLAG_PRE_RELOC | DM_FLAG_OS_PREPARE,
	.priv_auto = sizeof(struct mxsfb_priv),
};
#endif /* ifndef CONFIG_DM_VIDEO */

void cvt_fb_videomode_to_ctfb_res_modes(const struct fb_videomode *fb, struct ctfb_res_modes *ct)
{
	ct->xres = fb->xres;
	ct->yres = fb->yres;
	ct->refresh = fb->refresh;
	ct->pixclock = fb->pixclock;
	ct->pixclock_khz = 0;
	ct->left_margin = fb->left_margin;
	ct->right_margin = fb->right_margin;
	ct->upper_margin = fb->upper_margin;
	ct->lower_margin = fb->lower_margin;
	ct->hsync_len = fb->hsync_len;
	ct->vsync_len = fb->vsync_len;
	ct->sync = fb->sync;
	ct->vmode = fb->vmode;
}

#ifndef CONFIG_DM_VIDEO
static struct fb_videomode const *gmode;
static uint32_t gpixfmt;

static struct graphic_device *mxsfb_probe2(void)
{
	struct ctfb_res_modes ct;
	int bpp = -1;

	switch (gpixfmt) {
	case IPU_PIX_FMT_RGB32:
		bpp = 32;
		break;
	case IPU_PIX_FMT_RGB24:
		bpp = 24;
		break;
	case IPU_PIX_FMT_RGB666:
		bpp = 18;
		break;
	case IPU_PIX_FMT_RGB565:
		bpp = 16;
		break;
	}
	if (setup != 1) {
		panel.isaBase  = MXS_LCDIF_BASE;
		depth  = bpp;
	}

	cvt_fb_videomode_to_ctfb_res_modes(gmode, &ct);
	return mxsfb_probe(bpp, &ct);
}

void *mxsfb_init2(void)
{
	struct graphic_device *fb = mxsfb_probe2();

	if (fb) {
		drv_video_init2(fb);
		debug("Framebuffer at 0x%x\n", (unsigned int)fb->frameAdrs);
	}
	return fb;
}

int mxsfb_init(struct fb_videomode const *mode, uint32_t pixfmt)
{
	gmode = mode;
	gpixfmt = pixfmt;
	return 0;
}

void *video_hw_init(void)
{
	char *penv;
	int bpp = -1;
	struct ctfb_res_modes mode;

	if (gmode)
		return mxsfb_probe2();
	puts("Video: ");

	if (!setup) {

		/* Suck display configuration from "videomode" variable */
		penv = env_get("videomode");
		if (!penv) {
			printf("MXSFB: 'videomode' variable not set!\n");
			return NULL;
		}

		bpp = video_get_params(&mode, penv);
		panel.isaBase  = MXS_LCDIF_BASE;
	} else {
		mode.xres = fbmode.xres;
		mode.yres = fbmode.yres;
		mode.pixclock = fbmode.pixclock;
		mode.left_margin = fbmode.left_margin;
		mode.right_margin = fbmode.right_margin;
		mode.upper_margin = fbmode.upper_margin;
		mode.lower_margin = fbmode.lower_margin;
		mode.hsync_len = fbmode.hsync_len;
		mode.vsync_len = fbmode.vsync_len;
		mode.sync = fbmode.sync;
		mode.vmode = fbmode.vmode;
		bpp = depth;
	}

	return mxsfb_probe(bpp, &mode);
}
#endif
