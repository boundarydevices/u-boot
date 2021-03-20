// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2020 NXP
 *
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <dt-bindings/reset/imx-hdmimix-reset.h>
#include <linux/io.h>
#include <reset.h>
#include <reset-uclass.h>

#define IMX_HDMIMIX_RESET_CTL0_REG	0x20
#define IMX_HDMIMIX_RESET_CTL0_REG_SET	0x24
#define IMX_HDMIMIX_RESET_CTL0_REG_CLR	0x28

#define IMX_HDMIMIX_RESET_CTL0_TX_TRNG_RESETN							(1 << 20)
#define IMX_HDMIMIX_RESET_CTL0_VID_LINK_SLV_RESETN						(1 << 22)
#define IMX_HDMIMIX_RESET_CTL0_PAI_RESETN								(1 << 18)
#define IMX_HDMIMIX_RESET_CTL0_IRQ_STEER_RESETN							(1 << 16)
#define IMX_HDMIMIX_RESET_CTL0_TX_KSV_MEM_RESETN						(1 << 13)
#define IMX_HDMIMIX_RESET_CTL0_TX_PHY_PRESETN							(1 << 12)
#define IMX_HDMIMIX_RESET_CTL0_TX_APBRSTZ								(1 << 11)
#define IMX_HDMIMIX_RESET_CTL0_TX_RSTZ									(1 << 10)
#define IMX_HDMIMIX_RESET_CTL0_FDCC_HDMI_RESETN							(1 << 7)
#define IMX_HDMIMIX_RESET_CTL0_FDCC_RESETN								(1 << 6)
#define IMX_HDMIMIX_RESET_CTL0_LCDIF_APB_RESETN							(1 << 5)
#define IMX_HDMIMIX_RESET_CTL0_LCDIF_ASYNC_RESETN						(1 << 4)
#define IMX_HDMIMIX_RESET_CTL0_NOC_RESETN								(1 << 0)

struct imx_hdmimix_reset_data {
	void __iomem *base;
};

static int imx_hdmimix_reset_set(struct reset_ctl *rst, bool assert)
{
	struct imx_hdmimix_reset_data *drvdata =
		(struct imx_hdmimix_reset_data *)dev_get_priv(rst->dev);

	void __iomem *reg_addr = drvdata->base;
	unsigned int val;
	unsigned long id = rst->id;

	switch (id) {
	case IMX_HDMIMIX_HDMI_TX_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_TX_APBRSTZ |
				IMX_HDMIMIX_RESET_CTL0_TX_RSTZ |
				IMX_HDMIMIX_RESET_CTL0_FDCC_HDMI_RESETN |
				IMX_HDMIMIX_RESET_CTL0_FDCC_RESETN;
		break;
	case IMX_HDMIMIX_HDMI_PHY_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_TX_PHY_PRESETN;
		break;
	case IMX_HDMIMIX_HDMI_PAI_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_PAI_RESETN;
		break;
	case IMX_HDMIMIX_HDMI_PVI_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_VID_LINK_SLV_RESETN;
		break;
	case IMX_HDMIMIX_HDMI_TRNG_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_TX_TRNG_RESETN;
		break;
	case IMX_HDMIMIX_IRQ_STEER_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_IRQ_STEER_RESETN;
		break;
	case IMX_HDMIMIX_HDMI_HDCP_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_TX_KSV_MEM_RESETN;
		break;
	case IMX_HDMIMIX_LCDIF_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_LCDIF_APB_RESETN |
				IMX_HDMIMIX_RESET_CTL0_LCDIF_ASYNC_RESETN;
		break;
	default:
		return -EINVAL;
	}
	debug("%s: %s id=%ld, val=%x, %p\n", __func__,
		assert ? "assert" : "release", id, val, reg_addr +
		(assert ? IMX_HDMIMIX_RESET_CTL0_REG_CLR :
			IMX_HDMIMIX_RESET_CTL0_REG_SET));
#if 0
	if (assert) {
		writel(val, reg_addr + IMX_HDMIMIX_RESET_CTL0_REG_CLR);
	} else {
		writel(val, reg_addr + IMX_HDMIMIX_RESET_CTL0_REG_SET);
	}
#endif
	return 0;
}

static int imx_hdmimix_reset_assert(struct reset_ctl *rst)
{
	return imx_hdmimix_reset_set(rst, true);
}

static int imx_hdmimix_reset_deassert(struct reset_ctl *rst)
{
	return imx_hdmimix_reset_set(rst, false);
}

static int imx_hdmimix_reset_free(struct reset_ctl *rst)
{
	return 0;
}

static int imx_hdmimix_reset_request(struct reset_ctl *rst)
{
	return 0;
}

static const struct reset_ops imx_hdmimix_reset_ops = {
	.request = imx_hdmimix_reset_request,
	.rfree = imx_hdmimix_reset_free,
	.rst_assert   = imx_hdmimix_reset_assert,
	.rst_deassert = imx_hdmimix_reset_deassert,
};

static int imx_hdmimix_reset_probe(struct udevice *dev)
{
	struct imx_hdmimix_reset_data *drvdata = (struct imx_hdmimix_reset_data *)dev_get_priv(dev);

	drvdata->base = (void *)dev_read_addr(dev);
	return 0;
}

static const struct udevice_id imx_hdmimix_reset_dt_ids[] = {
	{ .compatible = "fsl,imx8mp-hdmimix-reset", },
	{ /* sentinel */ },
};

U_BOOT_DRIVER(imx_hdmimix_reset) = {
	.name = "imx_hdmimix_reset",
	.id = UCLASS_RESET,
	.of_match = imx_hdmimix_reset_dt_ids,
	.ops = &imx_hdmimix_reset_ops,
	.probe = imx_hdmimix_reset_probe,
	.priv_auto_alloc_size = sizeof(struct imx_hdmimix_reset_data),
};
