/*
 * Copyright 2017 NXP
 *
 * FSL i.MX8M USB HOST xHCI Controller
 *
 * Author: Jun Li <jun.li@nxp.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm-generic/gpio.h>
#include <clk.h>
#include <usb.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/compat.h>
#include <linux/usb/dwc3.h>
#include <asm/arch/sys_proto.h>
#include <usb/xhci.h>
#include <dm.h>
#include <power/regulator.h>

/* Declare global data pointer */
DECLARE_GLOBAL_DATA_PTR;

#define USBMIX_PHY_OFFSET		0xF0040

#define PHY_CTRL0_REF_SSP_EN		BIT(2)

#define PHY_CTRL1_RESET			BIT(0)
#define PHY_CTRL1_ATERESET		BIT(3)
#define PHY_CTRL1_VDATSRCENB0		BIT(19)
#define PHY_CTRL1_VDATDETENB0		BIT(20)

#define PHY_CTRL2_TXENABLEN0		BIT(8)

struct imx8m_usbmix {
	u32 phy_ctrl0;
	u32 phy_ctrl1;
	u32 phy_ctrl2;
	u32 phy_ctrl3;
	u32 phy_ctrl4;
	u32 phy_ctrl5;
	u32 phy_ctrl6;
};

struct imx8m_xhci {
	struct xhci_hccr *hcd;
	struct dwc3 *dwc3_reg;
	struct imx8m_usbmix *usbmix_reg;
};

struct imx8m_usbctrl_data {
	u32 usb_id;
	unsigned long ctr_addr;
};


#if CONFIG_IS_ENABLED(DM_USB)
struct xhci_imx8m_priv {
	struct xhci_ctrl xhci;
	struct clk_bulk clks;
#ifdef CONFIG_DM_GPIO
	struct gpio_desc reset_gpio;
#endif
	struct udevice *vbus_supply;
};
#else
static struct imx8m_usbctrl_data ctr_data[] = {
	{0, USB1_BASE_ADDR},
	{1, USB2_BASE_ADDR},
};
#endif

#ifdef CONFIG_IMX8MP
#define HSIO_GPR_BASE                               (0x32F10000U)
#define HSIO_GPR_REG_0                              (HSIO_GPR_BASE)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT    (1)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN          (0x1U << HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT)
#endif

static void imx8m_usb_phy_init(struct imx8m_usbmix *usbmix_reg)
{
	u32 reg;

#ifdef CONFIG_IMX8MP
	/* enable usb clock via hsio gpr */
	reg = readl(HSIO_GPR_REG_0);
	reg |= HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN;
	writel(reg, HSIO_GPR_REG_0);

	/* USB3.0 PHY signal fsel for 24M ref */
	reg = readl(&usbmix_reg->phy_ctrl0);
	reg = (reg & 0xfffff81f) | (0x2a<<5);
	writel(reg, &usbmix_reg->phy_ctrl0);

	reg = readl(&usbmix_reg->phy_ctrl6);
	reg &=~0x1;
	writel(reg, &usbmix_reg->phy_ctrl6);
#endif

	reg = readl(&usbmix_reg->phy_ctrl1);
	reg &= ~(PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0);
	reg |= PHY_CTRL1_RESET | PHY_CTRL1_ATERESET;
	writel(reg, &usbmix_reg->phy_ctrl1);

	reg = readl(&usbmix_reg->phy_ctrl0);
	reg |= PHY_CTRL0_REF_SSP_EN;
	writel(reg, &usbmix_reg->phy_ctrl0);

	reg = readl(&usbmix_reg->phy_ctrl2);
	reg |= PHY_CTRL2_TXENABLEN0;
	writel(reg, &usbmix_reg->phy_ctrl2);

	reg = readl(&usbmix_reg->phy_ctrl1);
	reg &= ~(PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	writel(reg, &usbmix_reg->phy_ctrl1);
}

static void imx8m_xhci_set_suspend_clk(struct dwc3 *dwc3_reg)
{
	u32 reg;

	/* Set suspend_clk to be 32KHz */
	reg = readl(&dwc3_reg->g_ctl);
	reg &= ~(DWC3_GCTL_PWRDNSCALE_MASK);
	reg |= DWC3_GCTL_PWRDNSCALE(2);

	writel(reg, &dwc3_reg->g_ctl);
}

static int imx8m_xhci_core_init(struct imx8m_xhci *imx8m_xhci)
{
	int ret = 0;

	imx8m_usb_phy_init(imx8m_xhci->usbmix_reg);

	ret = dwc3_core_init(imx8m_xhci->dwc3_reg);
	if (ret) {
		debug("%s:failed to initialize core\n", __func__);
		return ret;
	}

	imx8m_xhci_set_suspend_clk(imx8m_xhci->dwc3_reg);

	/* We are hard-coding DWC3 core to Host Mode */
	dwc3_set_mode(imx8m_xhci->dwc3_reg, DWC3_GCTL_PRTCAP_HOST);

	/* Set GFLADJ_30MHZ as 20h as per XHCI spec default value */
	dwc3_set_fladj(imx8m_xhci->dwc3_reg, GFLADJ_30MHZ_DEFAULT);

	return ret;
}

int imx8m_usb_common_init(void *base, struct xhci_hccr **hccr, struct xhci_hcor **hcor)
{
	int ret;
	struct imx8m_xhci ctx;

	ctx.hcd = (struct xhci_hccr *)base;
	ctx.dwc3_reg = (struct dwc3 *)(base + DWC3_REG_OFFSET);
	ctx.usbmix_reg = (struct imx8m_usbmix *)(base + USBMIX_PHY_OFFSET);

	ret = imx8m_xhci_core_init(&ctx);
	if (ret < 0) {
		puts("Failed to initialize imx8m xhci\n");
		return ret;
	}

	*hccr = (struct xhci_hccr *)ctx.hcd;
	*hcor = (struct xhci_hcor *)((uintptr_t) *hccr
				+ HC_LENGTH(xhci_readl(&(*hccr)->cr_capbase)));

	debug("imx8m-xhci: init hccr %lx and hcor %lx hc_length %lx\n",
	      (uintptr_t)*hccr, (uintptr_t)*hcor,
	      (uintptr_t)HC_LENGTH(xhci_readl(&(*hccr)->cr_capbase)));

	return ret;
}

#if CONFIG_IS_ENABLED(DM_USB)
static int imx8_of_clk_init(struct udevice *dev,
		struct xhci_imx8m_priv *priv)
{
	int ret;

	ret = clk_get_bulk(dev, &priv->clks);
	if (ret == -ENOSYS)
		return 0;
	if (ret)
		return ret;

#if CONFIG_IS_ENABLED(CLK)
	ret = clk_enable_bulk(&priv->clks);
	if (ret) {
		clk_release_bulk(&priv->clks);
		return ret;
	}
#endif

	return 0;
}

#ifdef CONFIG_DM_GPIO
static void imx8_of_reset_gpio_init(struct udevice *dev,
		struct xhci_imx8m_priv *priv)
{
	gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset_gpio,
			GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	if (dm_gpio_is_valid(&priv->reset_gpio)) {
		udelay(500);
		/* release reset */
		dm_gpio_set_value(&priv->reset_gpio, 0);
	}
}
#else
static void imx8_of_reset_gpio_init(struct udevice *dev,
		struct xhci_imx8m_priv *priv)
{
}
#endif

static int imx8m_xhci_probe(struct udevice *dev)
{
	struct xhci_imx8m_priv *priv = dev_get_priv(dev);
	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	fdt_addr_t hcd_base;
	int index;

	int ret = 0;

	/*
	 * Get the base address for XHCI controller from the device node
	 */
	hcd_base = dev_read_addr_index(dev, 0);
	if (hcd_base == FDT_ADDR_T_NONE) {
		printf("Can't get the XHCI register base address\n");
		return -ENXIO;
	}
	index = hcd_base == USB1_BASE_ADDR ? 0 : 1;
	ret = board_usb_init(index, USB_INIT_HOST);
	if (ret != 0) {
		imx8m_usb_power(index, false);
		puts("Failed to initialize board for imx8m USB\n");
		return ret;
	}
#if CONFIG_IS_ENABLED(DM_REGULATOR)
	ret = device_get_supply_regulator(dev, "vbus-supply",
					  &priv->vbus_supply);
	if (ret)
		debug("%s: No vbus supply\n", dev->name);
	if (priv->vbus_supply) {
		ret = regulator_set_enable(priv->vbus_supply, true);
		if (ret && ret != -ENOSYS) {
			printf("Error enabling VBUS supply (ret=%i)\n", ret);
			return ret;
		}
	}
#endif

	imx8_of_clk_init(dev, priv);
	imx8_of_reset_gpio_init(dev, priv);
	ret = imx8m_usb_common_init((void *)hcd_base, &hccr, &hcor);
	if (ret < 0)
		return ret;

	return xhci_register(dev, hccr, hcor);
}

static int imx8m_xhci_remove(struct udevice *dev)
{
	return xhci_deregister(dev);
}

static const struct udevice_id xhci_usb_ids[] = {
	{ .compatible = "fsl,imx8mq-dwc3", },
	{ }
};

U_BOOT_DRIVER(imx8m_xhci) = {
	.name	= "imx8m_xhci",
	.id	= UCLASS_USB,
	.of_match = xhci_usb_ids,
	.probe = imx8m_xhci_probe,
	.remove = imx8m_xhci_remove,
	.ops	= &xhci_usb_ops,
	.plat_auto = sizeof(struct usb_plat),
	.priv_auto = sizeof(struct xhci_imx8m_priv),
	.flags	= DM_FLAG_ALLOC_PRIV_DMA,
};
#else
int xhci_hcd_init(int index, struct xhci_hccr **hccr, struct xhci_hcor **hcor)
{
	int ret = 0;

	ret = board_usb_init(ctr_data[index].usb_id, USB_INIT_HOST);
	if (ret != 0) {
		imx8m_usb_power(ctr_data[index].usb_id, false);
		puts("Failed to initialize board for imx8m USB\n");
		return ret;
	}
	return imx8m_usb_common_init((void*)ctr_data[index].ctr_addr, hccr, hcor);
}

void xhci_hcd_stop(int index)
{
	board_usb_cleanup(ctr_data[index].usb_id, USB_INIT_HOST);
}
#endif
