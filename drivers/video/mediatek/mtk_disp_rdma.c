// SPDX-License-Identifier: GPL-2.0
/*
 * Mediatek Video RDMA support
 *
 * Copyright (c) 2022 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <asm/io.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <errno.h>
#include <video.h>

#include "mtk_smi.h"

#define DISP_REG_RDMA_GLOBAL_CON      0x0010
#define RDMA_ENGINE_EN			      BIT(0)
#define RDMA_MODE_MEMORY		      BIT(1)

#define DISP_REG_RDMA_SIZE_CON_0      0x0014
#define RDMA_MATRIX_ENABLE		      BIT(17)

#define DISP_REG_RDMA_SIZE_CON_1      0x0018

#define DISP_RDMA_MEM_CON	      0x0024
#define MEM_MODE_INPUT_FORMAT_RGBA8888	      (2 << 4)

#define DISP_RDMA_MEM_SRC_PITCH	      0x002c

#define DISP_RDMA_MEM_START_ADDR      0x0f00

struct mtk_disp_rdma_priv {
	void __iomem *base;
	struct udevice *dev;
	struct udevice *larb;
	struct clk clk;
};

static void mtk_rdma_mask(struct mtk_disp_rdma_priv *rdma, u32 offset, u32 val, u32 mask)
{
	void __iomem *addr = rdma->base + offset;
	u32 tmp = readl(addr) & ~mask;

	tmp |= (val & mask);
	writel(tmp, addr);
}

static void mtk_rdma_write(struct mtk_disp_rdma_priv *rdma, u32 offset, u32 val)
{
	void __iomem *addr = rdma->base + offset;

	writel(val, addr);
}

void mtk_rdma_config(struct udevice *dev, struct display_timing timing)
{
	struct mtk_disp_rdma_priv *rdma = dev_get_priv(dev);
	u32 width = timing.hactive.typ;
	u32 height = timing.vactive.typ;

	mtk_rdma_mask(rdma, DISP_REG_RDMA_SIZE_CON_0, width, 0xfff);
	mtk_rdma_mask(rdma, DISP_REG_RDMA_SIZE_CON_1, height, 0xfffff);
}

void mtk_rdma_start(struct udevice *dev)
{
	struct mtk_disp_rdma_priv *rdma = dev_get_priv(dev);

	mtk_rdma_mask(rdma, DISP_REG_RDMA_GLOBAL_CON, RDMA_ENGINE_EN, RDMA_ENGINE_EN);
}

void mtk_rdma_layer(struct udevice *dev, struct video_uc_plat *plat, struct display_timing timing)
{
	struct mtk_disp_rdma_priv *rdma = dev_get_priv(dev);
	u32 width = timing.hactive.typ;

	mtk_rdma_write(rdma, DISP_RDMA_MEM_CON, MEM_MODE_INPUT_FORMAT_RGBA8888);

	mtk_rdma_mask(rdma, DISP_REG_RDMA_SIZE_CON_0, 0, RDMA_MATRIX_ENABLE);

	mtk_rdma_write(rdma, DISP_RDMA_MEM_SRC_PITCH, width * VNBYTES(VIDEO_BPP32));

	mtk_rdma_write(rdma, DISP_RDMA_MEM_START_ADDR, plat->base);

	mtk_rdma_mask(rdma, DISP_REG_RDMA_GLOBAL_CON, RDMA_MODE_MEMORY, RDMA_MODE_MEMORY);
}

int mtk_rdma_enable(struct udevice *dev)
{
	struct mtk_disp_rdma_priv *rdma = dev_get_priv(dev);
	int ret;

	ret = mtk_smi_larb_enable(rdma->larb);
	if (ret) {
		dev_err(rdma->dev, "failed to enable larb: %s\n", errno_str(ret));
		return ret;
	}

	ret = clk_enable(&rdma->clk);
	if (ret) {
		dev_err(rdma->dev, "failed to enable clk: %s\n", errno_str(ret));
		return ret;
	}

	return 0;
}

int mtk_rdma_disable(struct udevice *dev)
{
	struct mtk_disp_rdma_priv *rdma = dev_get_priv(dev);
	int ret;

	ret = mtk_smi_larb_disable(rdma->larb);
	if (ret) {
		dev_err(rdma->dev, "failed to disable larb: %s\n", errno_str(ret));
		return ret;
	}

	ret = clk_disable(&rdma->clk);
	if (ret) {
		dev_err(rdma->dev, "failed to disable clk: %s\n", errno_str(ret));
		return ret;
	}

	return 0;
}

static int mtk_disp_rdma_probe(struct udevice *dev)
{
	struct mtk_disp_rdma_priv *rdma = dev_get_priv(dev);
	int ret;

	rdma->dev = dev;

	rdma->base = dev_remap_addr(dev);
	if (IS_ERR(rdma->base))
		return PTR_ERR(rdma->base);

	ret = clk_get_by_index(dev, 0, &rdma->clk);
	if (ret)
		return ret;

	ret = uclass_get_device_by_phandle(UCLASS_MISC, dev, "mediatek,larb", &rdma->larb);
	if (ret) {
		dev_err(dev, "cannot get larb device: %s\n", errno_str(ret));
		return ret;
	}

	return 0;
}

static const struct udevice_id mtk_disp_rdma_ids[] = {
	{ .compatible = "mediatek,mt8365-disp-rdma1" },
	{}
};

U_BOOT_DRIVER(mtk_disp_rdma) = {
	.name	   = "mtk_disp_rdma",
	.id	   = UCLASS_MISC,
	.of_match  = mtk_disp_rdma_ids,
	.probe	   = mtk_disp_rdma_probe,
	.priv_auto = sizeof(struct mtk_disp_rdma_priv),
};
