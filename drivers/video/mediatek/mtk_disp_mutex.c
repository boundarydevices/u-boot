// SPDX-License-Identifier: GPL-2.0
/*
 * Mediatek Display MUTEX
 *
 * Copyright (c) 2022 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <asm/io.h>
#include <dm.h>
#include <dm/device_compat.h>

#define DISP_MUTEX_CFG		  0x08
#define MUTEX_DISABLE_CLK_GATING	0

#define DISP_MUTEX0_EN		  0x20
#define MUTEX_EN			BIT(0)

#define DISP_MUTEX0_CTL		  0x2c
#define MUTEX_EOF_DPI0			BIT(7)
#define MUTEX_SOF_DPI0			BIT(1)

#define DISP_MUTEX0_MOD0	  0x30
#define MUTEX_MOD_DISP_RDMA1		BIT(10)

struct mtk_disp_mutex_priv {
	void __iomem *base;
};

void mtk_disp_mutex_rdma_dpi_enable(struct udevice *dev)
{
	struct mtk_disp_mutex_priv *mutex = dev_get_priv(dev);

	writel(MUTEX_DISABLE_CLK_GATING, mutex->base + DISP_MUTEX_CFG);

	writel(MUTEX_MOD_DISP_RDMA1, mutex->base + DISP_MUTEX0_MOD0);

	writel(MUTEX_EOF_DPI0 | MUTEX_SOF_DPI0, mutex->base + DISP_MUTEX0_CTL);

	writel(MUTEX_EN, mutex->base + DISP_MUTEX0_EN);
}

static int mtk_disp_mutex_probe(struct udevice *dev)
{
	struct mtk_disp_mutex_priv *mutex = dev_get_priv(dev);

	mutex->base = dev_remap_addr(dev);
	if (IS_ERR(mutex->base))
		return PTR_ERR(mutex->base);

	return 0;
}

static const struct udevice_id mtk_disp_mutex_ids[] = {
	{ .compatible = "mediatek,mt8365-disp-mutex" },
	{}
};

U_BOOT_DRIVER(mtk_disp_mutex) = {
	.name	   = "mtk_disp_mutex",
	.id	   = UCLASS_MISC,
	.of_match  = mtk_disp_mutex_ids,
	.probe	   = mtk_disp_mutex_probe,
	.priv_auto = sizeof(struct mtk_disp_mutex_priv),
};
