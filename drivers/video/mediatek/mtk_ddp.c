// SPDX-License-Identifier: GPL-2.0
/*
 * Mediatek Display Data Path
 *
 * Copyright (c) 2022 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <asm/io.h>
#include <dm.h>
#include <dm/device_compat.h>

#define DISP_REG_CONFIG_DISP_RDMA1_SOUT_SEL   0xfd0
#define DISP_REG_CONFIG_DISP_DPI0_SEL_IN      0xfd8
#define DISP_REG_CONFIG_DISP_LVDS_SYS_CFG_00  0xfdc

struct mtk_ddp_priv {
	void __iomem *base;
};

void mtk_ddp_rdma_to_dpi(struct udevice *dev)
{
	struct mtk_ddp_priv *ddp = dev_get_priv(dev);

	writel(BIT(0), ddp->base + DISP_REG_CONFIG_DISP_RDMA1_SOUT_SEL);

	writel(0, ddp->base + DISP_REG_CONFIG_DISP_DPI0_SEL_IN);

	writel(BIT(0), ddp->base + DISP_REG_CONFIG_DISP_LVDS_SYS_CFG_00);
}

static int mtk_ddp_probe(struct udevice *dev)
{
	struct mtk_ddp_priv *ddp = dev_get_priv(dev);

	ddp->base = dev_remap_addr(dev);
	if (IS_ERR(ddp->base))
		return PTR_ERR(ddp->base);

	return 0;
}

static const struct udevice_id mtk_ddp_ids[] = {
	{ .compatible = "mediatek,mt8365-mmsys" },
	{}
};

U_BOOT_DRIVER(mtk_ddp) = {
	.name	   = "mtk_ddp",
	.id	   = UCLASS_MISC,
	.of_match  = mtk_ddp_ids,
	.probe	   = mtk_ddp_probe,
	.priv_auto = sizeof(struct mtk_ddp_priv),
};
