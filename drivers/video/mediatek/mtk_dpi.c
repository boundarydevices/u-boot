// SPDX-License-Identifier: GPL-2.0
/*
 * Mediatek Video DPI support
 *
 * Copyright (c) 2022 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <asm/io.h>
#include <asm/system.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <errno.h>
#include <edid.h>
#include <linux/err.h>
#include <video.h>
#include <video_bridge.h>

#include "mtk_ddp.h"
#include "mtk_disp_mutex.h"
#include "mtk_disp_rdma.h"

#define MTK_DPI_MAX_WIDTH  3840
#define MTK_DPI_MAX_HEIGHT 2160

#define DPI_EN		       0x00
#define EN			     BIT(0)

#define DPI_RET		       0x04
#define RST			     BIT(0)

#define DPI_CON		       0x10
#define INTL_EN			     BIT(2)

#define DPI_OUTPUT_SETTING     0x14
#define CH_SWAP			     0
#define CH_SWAP_MASK		     (0x7 << 0)
#define SWAP_RGB		     0
#define HSYNC_POL		     BIT(13)
#define VSYNC_POL		     BIT(14)
#define CK_POL			     BIT(15)

#define DPI_SIZE	       0x18
#define HSIZE			     0
#define HSIZE_MASK		     (0x1FFF << 0)
#define VSIZE			     16
#define VSIZE_MASK		     (0x1FFF << 16)

#define DPI_DDR_SETTING	       0x1C
#define DDR_EN			     BIT(0)
#define DDR_4PHASE		     BIT(2)

#define DPI_TGEN_HWIDTH	       0x20
#define HPW			     0
#define HPW_MASK		     (0xFFF << 0)

#define DPI_TGEN_HPORCH	       0x24
#define HBP			     0
#define HBP_MASK		     (0xFFF << 0)
#define HFP			     16
#define HFP_MASK		     (0xFFF << 16)

#define DPI_TGEN_VWIDTH	       0x28
#define VPW			     0
#define VPW_MASK		     (0xFFF << 0)

#define DPI_TGEN_VPORCH	       0x2C
#define VBP			     0
#define VBP_MASK		     (0xFFF << 0)
#define VFP			     16
#define VFP_MASK		     (0xFFF << 16)

#define DPI_Y_LIMIT	       0x98
#define Y_LIMINT_BOT		     0
#define Y_LIMINT_BOT_MASK	     (0xFFF << 0)
#define Y_LIMINT_TOP		     16
#define Y_LIMINT_TOP_MASK	     (0xFFF << 16)

#define DPI_C_LIMIT	       0x9C
#define C_LIMIT_BOT		     0
#define C_LIMIT_BOT_MASK	     (0xFFF << 0)
#define C_LIMIT_TOP		     16
#define C_LIMIT_TOP_MASK	     (0xFFF << 16)

#define DPI_MUTEX_SETTING      0xe0
#define H_FRE_2N		     BIT(25)

struct mtk_dpi_priv {
	void __iomem *base;
	struct udevice *dev;
	struct udevice *rdma;
	struct udevice *mutex;
	struct udevice *mmsys;
	struct udevice *bridge;
	struct clk *engine_clk;
	struct clk *pixel_clk;
	struct clk *dpi_sel_clk;
	struct clk *tvd_clk;
	bool dual_edge;
};

static void mtk_dpi_mask(struct mtk_dpi_priv *dpi, u32 offset, u32 val, u32 mask)
{
	void __iomem *addr = dpi->base + offset;
	u32 tmp = readl(addr) & ~mask;

	tmp |= (val & mask);
	writel(tmp, addr);
}

static void mtk_dpi_sw_reset(struct mtk_dpi_priv *dpi, bool reset)
{
	mtk_dpi_mask(dpi, DPI_RET, reset ? RST : 0, RST);
}

static void mtk_dpi_enable(struct mtk_dpi_priv *dpi)
{
	mtk_dpi_mask(dpi, DPI_EN, EN, EN);
}

static void mtk_dpi_config_hsync(struct mtk_dpi_priv *dpi, struct display_timing timing)
{
	mtk_dpi_mask(dpi, DPI_TGEN_HWIDTH, timing.hsync_len.typ << HPW, HPW_MASK);
	mtk_dpi_mask(dpi, DPI_TGEN_HPORCH, timing.hback_porch.typ << HBP, HBP_MASK);
	mtk_dpi_mask(dpi, DPI_TGEN_HPORCH, timing.hfront_porch.typ << HFP, HFP_MASK);
}

static void mtk_dpi_config_vsync(struct mtk_dpi_priv *dpi, struct display_timing timing)
{
	mtk_dpi_mask(dpi, DPI_TGEN_VWIDTH, timing.vsync_len.typ << VPW, VPW_MASK);
	mtk_dpi_mask(dpi, DPI_TGEN_VPORCH, timing.vback_porch.typ << VBP, VBP_MASK);
	mtk_dpi_mask(dpi, DPI_TGEN_VPORCH, timing.vfront_porch.typ << VFP, VFP_MASK);
}

static void mtk_dpi_config_pol(struct mtk_dpi_priv *dpi, struct display_timing timing)
{
	u32 val = CK_POL;

	if (timing.flags & DISPLAY_FLAGS_HSYNC_HIGH)
		val |= HSYNC_POL;

	if (timing.flags & DISPLAY_FLAGS_VSYNC_HIGH)
		val |= VSYNC_POL;

	mtk_dpi_mask(dpi, DPI_OUTPUT_SETTING, val, CK_POL | HSYNC_POL | VSYNC_POL);
}

static void mtk_dpi_config_interface(struct mtk_dpi_priv *dpi, struct display_timing timing)
{
	bool inter = (timing.flags == DISPLAY_FLAGS_INTERLACED);

	mtk_dpi_mask(dpi, DPI_CON, inter ? INTL_EN : 0, INTL_EN);
}

static void mtk_dpi_config_fb_size(struct mtk_dpi_priv *dpi, struct display_timing timing)
{
	u32 hactive = timing.hactive.typ;
	u32 vactive;

	if (timing.flags == DISPLAY_FLAGS_INTERLACED)
		vactive = timing.vactive.typ >> 1;
	else
		vactive = timing.vactive.typ;

	mtk_dpi_mask(dpi, DPI_SIZE, hactive << HSIZE, HSIZE_MASK);
	mtk_dpi_mask(dpi, DPI_SIZE, vactive << VSIZE, VSIZE_MASK);
}

static void mtk_dpi_config_channel_limit(struct mtk_dpi_priv *dpi)
{
	u32 y_bottom = 0x0010;
	u32 y_top = 0x0FE0;
	u32 c_bottom = 0x0010;
	u32 c_top = 0x0FE0;

	mtk_dpi_mask(dpi, DPI_Y_LIMIT, y_bottom << Y_LIMINT_BOT, Y_LIMINT_BOT_MASK);
	mtk_dpi_mask(dpi, DPI_Y_LIMIT, y_top << Y_LIMINT_TOP, Y_LIMINT_TOP_MASK);
	mtk_dpi_mask(dpi, DPI_C_LIMIT, c_bottom << C_LIMIT_BOT, C_LIMIT_BOT_MASK);
	mtk_dpi_mask(dpi, DPI_C_LIMIT, c_top << C_LIMIT_TOP, C_LIMIT_TOP_MASK);
}

static void mtk_dpi_config_channel_swap(struct mtk_dpi_priv *dpi)
{
	u32 val = SWAP_RGB;

	mtk_dpi_mask(dpi, DPI_OUTPUT_SETTING, val << CH_SWAP, CH_SWAP_MASK);
}

static void mtk_dpi_config_2n_h_fre(struct mtk_dpi_priv *dpi)
{
	mtk_dpi_mask(dpi, DPI_MUTEX_SETTING, H_FRE_2N, H_FRE_2N);
}

static void mtk_dpi_enable_dual_edge(struct mtk_dpi_priv *dpi)
{
	mtk_dpi_mask(dpi, DPI_DDR_SETTING, DDR_EN | DDR_4PHASE, DDR_EN | DDR_4PHASE);
}

static int mtk_dpi_calculate_factor(ulong clock)
{
	if (clock <= 27000000)
		return 8;
	else if (clock <= 167000000)
		return 4;
	else
		return 2;
}

static void mtk_dpi_set_display_mode(struct mtk_dpi_priv *dpi, struct display_timing timing)
{
	struct video_priv *priv = dev_get_uclass_priv(dpi->dev);
	int factor;
	ulong pll_rate;

	priv->xsize = timing.hactive.typ;
	priv->ysize = timing.vactive.typ;
	priv->bpix = VIDEO_BPP32;

	factor = mtk_dpi_calculate_factor(timing.pixelclock.typ);
	pll_rate = timing.pixelclock.typ * factor * (dpi->dual_edge ? 2 : 1);
	clk_set_rate(dpi->tvd_clk, pll_rate);

	mtk_dpi_sw_reset(dpi, true);
	mtk_dpi_config_pol(dpi, timing);

	mtk_dpi_config_hsync(dpi, timing);
	mtk_dpi_config_vsync(dpi, timing);

	mtk_dpi_config_interface(dpi, timing);

	mtk_dpi_config_fb_size(dpi, timing);

	mtk_dpi_config_channel_limit(dpi);
	mtk_dpi_config_channel_swap(dpi);
	mtk_dpi_config_2n_h_fre(dpi);
	if (dpi->dual_edge)
		mtk_dpi_enable_dual_edge(dpi);

	mtk_dpi_sw_reset(dpi, false);
}

static bool mtk_dpi_mode_valid(void *priv, const struct display_timing *timing)
{
	if (timing->hactive.typ >= MTK_DPI_MAX_WIDTH &&
	    timing->vactive.typ >= MTK_DPI_MAX_HEIGHT)
		return false;
	else
		return true;
}

static int mtk_dpi_get_display_timing(struct mtk_dpi_priv *dpi, struct display_timing *timing)
{
	u8 buf[EDID_SIZE];
	int panel_bits_per_colourp;
	int ret;

	ret = video_bridge_read_edid(dpi->bridge, buf, EDID_SIZE);
	if (ret != EDID_SIZE) {
		dev_err(dpi->dev, "failed to read edid\n");
		return ret;
	}

	return edid_get_timing_validate(buf, EDID_SIZE, timing, &panel_bits_per_colourp,
					mtk_dpi_mode_valid, NULL);
}

static int mtk_dpi_power_on(struct mtk_dpi_priv *dpi)
{
	int ret;

	ret = clk_enable(dpi->dpi_sel_clk);
	if (ret) {
		dev_err(dpi->dev, "failed to enable dpi_sel clk: %d\n", ret);
		return ret;
	}

	ret = clk_enable(dpi->engine_clk);
	if (ret) {
		dev_err(dpi->dev, "failed to enable engine clk: %d\n", ret);
		return ret;
	}

	ret = clk_enable(dpi->pixel_clk);
	if (ret) {
		dev_err(dpi->dev, "failed to enable pixel clk: %d\n", ret);
		return ret;
	}

	ret = clk_enable(dpi->tvd_clk);
	if (ret) {
		dev_err(dpi->dev, "failed to enable tvd_clk clk: %d\n", ret);
		return ret;
	}

	ret = mtk_rdma_enable(dpi->rdma);
	if (ret) {
		dev_err(dpi->dev, "failed to enable rdma: %d\n", ret);
		return ret;
	}

	mtk_dpi_enable(dpi);
	return 0;
}

static int mtk_dpi_power_off(struct mtk_dpi_priv *dpi)
{
	int ret;

	ret = clk_disable(dpi->engine_clk);
	if (ret) {
		dev_err(dpi->dev, "failed to disable engine clk: %d\n", ret);
		return ret;
	}

	ret = clk_disable(dpi->pixel_clk);
	if (ret) {
		dev_err(dpi->dev, "failed to disable pixel clk: %d\n", ret);
		return ret;
	}

	ret = clk_disable(dpi->tvd_clk);
	if (ret) {
		dev_err(dpi->dev, "failed to disable tvd_clk clk: %d\n", ret);
		return ret;
	}

	ret = clk_disable(dpi->dpi_sel_clk);
	if (ret) {
		dev_err(dpi->dev, "failed to disable dpi_sel clk: %d\n", ret);
		return ret;
	}

	ret = mtk_rdma_disable(dpi->rdma);
	if (ret) {
		dev_err(dpi->dev, "failed to disable rdma: %d\n", ret);
		return ret;
	}

	return 0;
}

static int mtk_dpi_probe(struct udevice *dev)
{
	struct mtk_dpi_priv *dpi = dev_get_priv(dev);
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct display_timing timing;
	int ret;

	/* Before relocation we don't need to do anything */
	if (!(gd->flags & GD_FLG_RELOC))
		return 0;

	dpi->dev = dev;

	dpi->base = dev_remap_addr(dev);
	if (IS_ERR(dpi->base))
		return PTR_ERR(dpi->base);

	dpi->dual_edge = ofnode_read_bool(dev_ofnode(dev), "dpi_dual_edge");

	dpi->engine_clk = devm_clk_get(dev, "engine");
	if (IS_ERR(dpi->engine_clk))
		return PTR_ERR(dpi->engine_clk);

	dpi->pixel_clk = devm_clk_get(dev, "pixel");
	if (IS_ERR(dpi->pixel_clk))
		return PTR_ERR(dpi->pixel_clk);

	dpi->tvd_clk = devm_clk_get(dev, "pll");
	if (IS_ERR(dpi->tvd_clk))
		return PTR_ERR(dpi->tvd_clk);

	dpi->dpi_sel_clk = devm_clk_get(dev, "dpi_sel");
	if (IS_ERR(dpi->dpi_sel_clk))
		return PTR_ERR(dpi->dpi_sel_clk);

	ret = uclass_get_device_by_phandle(UCLASS_MISC, dev, "mediatek,rdma", &dpi->rdma);
	if (ret) {
		dev_err(dev, "cannot get rdma device: %s\n", errno_str(ret));
		return ret;
	}

	ret = uclass_get_device_by_phandle(UCLASS_MISC, dev, "mediatek,mutex", &dpi->mutex);
	if (ret) {
		dev_err(dev, "cannot get mutex device: %s\n", errno_str(ret));
		return ret;
	}

	ret = uclass_get_device_by_phandle(UCLASS_MISC, dev, "mediatek,mmsys", &dpi->mmsys);
	if (ret) {
		dev_err(dev, "cannot get mmsys device: %s\n", errno_str(ret));
		return ret;
	}

	ret = uclass_first_device(UCLASS_VIDEO_BRIDGE, &dpi->bridge);
	if (ret) {
		dev_err(dev, "video bridge not found: %s\n", errno_str(ret));
		return ret;
	}

	ret = mtk_dpi_get_display_timing(dpi, &timing);
	if (ret) {
		dev_err(dev, "failed to get display timing\n");
		return ret;
	}

	ret = mtk_dpi_power_off(dpi);
	if (ret) {
		dev_err(dev, "failed to power off\n");
		return ret;
	}

	mtk_disp_mutex_rdma_dpi_enable(dpi->mutex);

	mtk_ddp_rdma_to_dpi(dpi->mmsys);

	mtk_rdma_config(dpi->rdma, timing);

	ret = mtk_dpi_power_on(dpi);
	if (ret) {
		dev_err(dev, "failed to power on\n");
		return ret;
	}

	mtk_rdma_start(dpi->rdma);

	mtk_dpi_set_display_mode(dpi, timing);

	mtk_rdma_layer(dpi->rdma, plat, timing);

	ret = video_bridge_attach(dpi->bridge);
	if (ret) {
		dev_err(dpi->bridge, "failed to attach bridge\n");
		return ret;
	}

	mmu_set_region_dcache_behaviour(plat->base, ALIGN(plat->size, MMU_SECTION_SIZE),
					DCACHE_WRITEBACK);
	video_set_flush_dcache(dev, true);

	return 0;
}

static int mtk_dpi_bind(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);

	plat->size = MTK_DPI_MAX_WIDTH * MTK_DPI_MAX_HEIGHT * VNBYTES(VIDEO_BPP32);

	return 0;
}

static const struct udevice_id mtk_dpi_ids[] = {
	{ .compatible = "mediatek,mt8365-dpi" },
	{}
};

U_BOOT_DRIVER(mtk_dpi) = {
	.name	   = "mtk_dpi",
	.id	   = UCLASS_VIDEO,
	.of_match  = mtk_dpi_ids,
	.bind	   = mtk_dpi_bind,
	.probe	   = mtk_dpi_probe,
	.priv_auto = sizeof(struct mtk_dpi_priv),
	.flags	   = DM_FLAG_PRE_RELOC | DM_FLAG_LEAVE_PD_ON,
};
