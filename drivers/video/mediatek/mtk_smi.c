// SPDX-License-Identifier: GPL-2.0
/*
 * Mediatek Memory Controller module
 *
 * Copyright (c) 2022 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <asm/io.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <errno.h>

#define SMI_LARB_NONSEC_CON(id)	 (0x380 + ((id) * 4))
#define MTK_M4U_ID(larb, port)	 (((larb) << 5) | (port))

#define M4U_PORT_DISP_RDMA1	 MTK_M4U_ID(0, 4)

struct mtk_smi_common_priv {
	struct udevice *dev;
	struct clk *common_clk;
	struct clk *comm0_clk;
	struct clk *comm1_clk;
};

static int mtk_smi_common_enable(struct udevice *dev)
{
	struct mtk_smi_common_priv *smi = dev_get_priv(dev);
	int ret;

	ret = clk_enable(smi->common_clk);
	if (ret) {
		dev_err(smi->dev, "failed to enable common clk: %s\n", errno_str(ret));
		return ret;
	}

	ret = clk_enable(smi->comm0_clk);
	if (ret) {
		dev_err(smi->dev, "failed to enable comm0 clk: %s\n", errno_str(ret));
		return ret;
	}

	ret = clk_enable(smi->comm1_clk);
	if (ret) {
		dev_err(smi->dev, "failed to enable comm1 clk: %s\n", errno_str(ret));
		return ret;
	}

	return 0;
}

static int mtk_smi_common_disable(struct udevice *dev)
{
	struct mtk_smi_common_priv *smi = dev_get_priv(dev);
	int ret;

	ret = clk_disable(smi->common_clk);
	if (ret) {
		dev_err(smi->dev, "failed to disable common clk: %s\n", errno_str(ret));
		return ret;
	}

	ret = clk_disable(smi->comm0_clk);
	if (ret) {
		dev_err(smi->dev, "failed to disable comm0 clk: %s\n", errno_str(ret));
		return ret;
	}

	ret = clk_disable(smi->comm1_clk);
	if (ret) {
		dev_err(smi->dev, "failed to disable comm1 clk: %s\n", errno_str(ret));
		return ret;
	}

	return 0;
}

static int mtk_smi_common_probe(struct udevice *dev)
{
	struct mtk_smi_common_priv *smi = dev_get_priv(dev);

	smi->dev = dev;

	smi->common_clk = devm_clk_get(dev, "common");
	if (IS_ERR(smi->common_clk))
		return PTR_ERR(smi->common_clk);

	smi->comm0_clk = devm_clk_get(dev, "comm0");
	if (IS_ERR(smi->comm0_clk))
		return PTR_ERR(smi->comm0_clk);

	smi->comm1_clk = devm_clk_get(dev, "comm1");
	if (IS_ERR(smi->comm1_clk))
		return PTR_ERR(smi->comm1_clk);

	return 0;
}

static const struct udevice_id mtk_smi_common_ids[] = {
	{ .compatible = "mediatek,mt8365-smi-common" },
	{}
};

U_BOOT_DRIVER(mtk_smi_common) = {
	.name	   = "mtk_smi_common",
	.id	   = UCLASS_MISC,
	.of_match  = mtk_smi_common_ids,
	.probe	   = mtk_smi_common_probe,
	.priv_auto = sizeof(struct mtk_smi_common_priv),
};

struct mtk_smi_larb_priv {
	void __iomem *base;
	struct udevice *dev;
	struct udevice *smi;
	struct clk clk;
};

int mtk_smi_larb_enable(struct udevice *dev)
{
	struct mtk_smi_larb_priv *larb = dev_get_priv(dev);
	int ret;

	ret = mtk_smi_common_enable(larb->smi);
	if (ret) {
		dev_err(larb->dev, "failed to enable smi common: %s\n", errno_str(ret));
		return ret;
	}

	ret = clk_enable(&larb->clk);
	if (ret) {
		dev_err(larb->dev, "failed to enable larb clk: %s\n", errno_str(ret));
		return ret;
	}

	/* RDMA1: Agent transaction to Physical Address */
	writel(0, larb->base + SMI_LARB_NONSEC_CON(M4U_PORT_DISP_RDMA1));

	return 0;
}

int mtk_smi_larb_disable(struct udevice *dev)
{
	struct mtk_smi_larb_priv *larb = dev_get_priv(dev);
	int ret;

	ret = clk_disable(&larb->clk);
	if (ret) {
		dev_err(larb->dev, "failed to disable larb clk: %s\n", errno_str(ret));
		return ret;
	}

	ret = mtk_smi_common_disable(larb->smi);
	if (ret) {
		dev_err(larb->dev, "failed to disable smi common: %s\n", errno_str(ret));
		return ret;
	}

	return 0;
}

static int mtk_smi_larb_probe(struct udevice *dev)
{
	struct mtk_smi_larb_priv *larb = dev_get_priv(dev);
	int ret;

	larb->dev = dev;

	larb->base = dev_remap_addr(dev);
	if (IS_ERR(larb->base))
		return PTR_ERR(larb->base);

	ret = clk_get_by_index(dev, 0, &larb->clk);
	if (ret)
		return ret;

	ret = uclass_get_device_by_phandle(UCLASS_MISC, dev, "mediatek,smi", &larb->smi);
	if (ret) {
		dev_err(dev, "cannot get smi device: %s\n", errno_str(ret));
		return ret;
	}

	return 0;
}

static const struct udevice_id mtk_smi_larb_ids[] = {
	{ .compatible = "mediatek,mt8365-smi-larb" },
	{}
};

U_BOOT_DRIVER(mtk_smi_larb) = {
	.name	   = "mtk_smi_larb",
	.id	   = UCLASS_MISC,
	.of_match  = mtk_smi_larb_ids,
	.probe	   = mtk_smi_larb_probe,
	.priv_auto = sizeof(struct mtk_smi_larb_priv),
};
