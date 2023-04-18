// SPDX-License-Identifier: GPL-2.0
/*
 * Generic DWC3 Glue layer
 *
 * Copyright (C) 2016 - 2018 Xilinx, Inc.
 *
 * Based on dwc3-omap.c.
 */

#include <common.h>
#include <cpu_func.h>
#include <log.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <dwc3-uboot.h>
#include <generic-phy.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <power/regulator.h>
#include <malloc.h>
#include <usb.h>
#include "core.h"
#include "gadget.h"
#include <reset.h>
#include <clk.h>
#include <usb/xhci.h>
#include <asm/gpio.h>

struct dwc3_glue_data {
	struct clk_bulk		clks;
	struct reset_ctl_bulk	resets;
	fdt_addr_t regs;
	u32	usb_ctrl0_clr;
	u32	usb_ctrl0_set;
	u32	usb_ctrl1_clr;
	u32	usb_ctrl1_set;
	int	vbus_oc_combined;
};

struct dwc3_generic_plat {
	fdt_addr_t base;
	u32 maximum_speed;
	enum usb_dr_mode dr_mode;
#if CONFIG_IS_ENABLED(DM_REGULATOR)
	struct udevice		*vbus_supply;
#endif
};

struct dwc3_generic_priv {
	void *base;
	struct dwc3 dwc3;
	struct phy_bulk phys;
	struct gpio_desc ulpi_reset;
};

struct dwc3_generic_host_priv {
	struct xhci_ctrl xhci_ctrl;
	struct dwc3_generic_priv gen_priv;
};

static int dwc3_generic_probe(struct udevice *dev,
			      struct dwc3_generic_priv *priv)
{
	int rc;
	struct dwc3_generic_plat *plat = dev_get_plat(dev);
	struct dwc3 *dwc3 = &priv->dwc3;
	struct dwc3_glue_data *glue = dev_get_plat(dev->parent);

	dwc3->dev = dev;
	dwc3->maximum_speed = plat->maximum_speed;
	dwc3->dr_mode = plat->dr_mode;
#if CONFIG_IS_ENABLED(OF_CONTROL)
	dwc3_of_parse(dwc3);
#endif

	/*
	 * It must hold whole USB3.0 OTG controller in resetting to hold pipe
	 * power state in P2 before initializing TypeC PHY on RK3399 platform.
	 */
	if (device_is_compatible(dev->parent, "rockchip,rk3399-dwc3")) {
		reset_assert_bulk(&glue->resets);
		udelay(1);
	}

	rc = dwc3_setup_phy(dev, &priv->phys);
	if (rc && rc != -ENOTSUPP)
		return rc;

	if (CONFIG_IS_ENABLED(DM_GPIO) &&
	    device_is_compatible(dev->parent, "xlnx,zynqmp-dwc3")) {
		rc = gpio_request_by_name(dev->parent, "reset-gpios", 0,
					  &priv->ulpi_reset, GPIOD_ACTIVE_LOW);
		if (rc)
			return rc;

		/* Toggle ulpi to reset the phy. */
		rc = dm_gpio_set_value(&priv->ulpi_reset, 1);
		if (rc)
			return rc;

		mdelay(5);

		rc = dm_gpio_set_value(&priv->ulpi_reset, 0);
		if (rc)
			return rc;

		mdelay(5);
	}

	if (device_is_compatible(dev->parent, "rockchip,rk3399-dwc3"))
		reset_deassert_bulk(&glue->resets);

	priv->base = map_physmem(plat->base, DWC3_OTG_REGS_END, MAP_NOCACHE);
	dwc3->regs = priv->base + DWC3_GLOBALS_REGS_START;


	rc =  dwc3_init(dwc3);
	if (rc) {
		unmap_physmem(priv->base, MAP_NOCACHE);
		return rc;
	}

	return 0;
}

static int dwc3_generic_remove(struct udevice *dev,
			       struct dwc3_generic_priv *priv)
{
	struct dwc3 *dwc3 = &priv->dwc3;

	if (CONFIG_IS_ENABLED(DM_GPIO) &&
	    device_is_compatible(dev->parent, "xlnx,zynqmp-dwc3")) {
		struct gpio_desc *ulpi_reset = &priv->ulpi_reset;

		dm_gpio_free(ulpi_reset->dev, ulpi_reset);
	}

	dwc3_remove(dwc3);
	dwc3_shutdown_phy(dev, &priv->phys);
	unmap_physmem(dwc3->regs, MAP_NOCACHE);

	return 0;
}

static int dwc3_generic_of_to_plat(struct udevice *dev)
{
	struct dwc3_generic_plat *plat = dev_get_plat(dev);
	ofnode node = dev_ofnode(dev);
#if CONFIG_IS_ENABLED(DM_REGULATOR)
	int ret;
#endif

	if (!strncmp(dev->name, "port", 4) || !strncmp(dev->name, "hub", 3)) {
		/* This is a leaf so check the parent */
		plat->base = dev_read_addr(dev->parent);
	} else {
		plat->base = dev_read_addr(dev);
	}

	plat->maximum_speed = usb_get_maximum_speed(node);
	if (plat->maximum_speed == USB_SPEED_UNKNOWN) {
		pr_info("No USB maximum speed specified. Using super speed\n");
		plat->maximum_speed = USB_SPEED_SUPER;
	}

	if (plat->dr_mode == USB_DR_MODE_UNKNOWN)
		plat->dr_mode = usb_get_dr_mode(node);
	if (plat->dr_mode == USB_DR_MODE_UNKNOWN) {
		/* might be a leaf so check the parent for mode */
		node = dev_ofnode(dev->parent);
		plat->dr_mode = usb_get_dr_mode(node);
		if (plat->dr_mode == USB_DR_MODE_UNKNOWN) {
			pr_err("Invalid usb mode setup\n");
			return -ENODEV;
		}
	}
#if CONFIG_IS_ENABLED(DM_REGULATOR)
	ret = device_get_supply_regulator(dev, "vbus-supply",
					  &plat->vbus_supply);
	if (ret && ret != -ENOENT) {
		pr_err("Failed to get PHY regulator\n");
		return ret;
	}
#endif

	return 0;
}

struct dwc3_glue_ops {
	void (*glue_configure)(struct udevice *dev, int index,
			       enum usb_dr_mode mode);
	enum usb_dr_mode (*get_dr_mode)(struct udevice *dev);
};

static void dwc3_glue_configure(struct udevice *dev, int index, enum usb_dr_mode dr_mode)
{
	struct dwc3_glue_ops *ops = (struct dwc3_glue_ops *)dev_get_driver_data(dev);

	if (ops && ops->glue_configure)
		ops->glue_configure(dev, index, dr_mode);
}

#if CONFIG_IS_ENABLED(DM_USB_GADGET)
static int dwc3_generic_peripheral_handle_interrupts(struct udevice *dev)
{
	struct dwc3_generic_priv *priv = dev_get_priv(dev);
	struct dwc3 *dwc3 = &priv->dwc3;

	dwc3_gadget_uboot_handle_interrupt(dwc3);

	return 0;
}

static int dwc3_generic_peripheral_probe(struct udevice *dev)
{
	struct dwc3_generic_plat *plat = dev_get_plat(dev);
	struct dwc3_generic_priv *priv = dev_get_priv(dev);

#if CONFIG_IS_ENABLED(DM_REGULATOR)
	if (plat->vbus_supply)
		regulator_set_enable(plat->vbus_supply, false);
#endif
	dwc3_glue_configure(dev->parent, 0, USB_DR_MODE_PERIPHERAL);
	return dwc3_generic_probe(dev, priv);
}

static int dwc3_generic_peripheral_remove(struct udevice *dev)
{
	struct dwc3_generic_priv *priv = dev_get_priv(dev);

	return dwc3_generic_remove(dev, priv);
}

U_BOOT_DRIVER(dwc3_generic_peripheral) = {
	.name	= "dwc3-generic-peripheral",
	.id	= UCLASS_USB_GADGET_GENERIC,
	.of_to_plat = dwc3_generic_of_to_plat,
	.probe = dwc3_generic_peripheral_probe,
	.remove = dwc3_generic_peripheral_remove,
	.handle_interrupts = dwc3_generic_peripheral_handle_interrupts,
	.priv_auto	= sizeof(struct dwc3_generic_priv),
	.plat_auto	= sizeof(struct dwc3_generic_plat),
};
#endif

#if defined(CONFIG_SPL_USB_HOST) || \
	!defined(CONFIG_SPL_BUILD) && defined(CONFIG_USB_HOST)
static int dwc3_generic_host_probe(struct udevice *dev)
{
	struct xhci_hcor *hcor;
	struct xhci_hccr *hccr;
	struct dwc3_generic_plat *plat = dev_get_plat(dev);
	struct dwc3_generic_host_priv *priv = dev_get_priv(dev);
	int rc;

	dwc3_glue_configure(dev->parent, 0, USB_DR_MODE_HOST);
	rc = dwc3_generic_probe(dev, &priv->gen_priv);
	if (rc)
		return rc;

	hccr = (struct xhci_hccr *)priv->gen_priv.base;
	hcor = (struct xhci_hcor *)(priv->gen_priv.base +
			HC_LENGTH(xhci_readl(&(hccr)->cr_capbase)));

	rc = xhci_register(dev, hccr, hcor);
#if CONFIG_IS_ENABLED(DM_REGULATOR)
	if (plat->vbus_supply) {
		regulator_set_enable(plat->vbus_supply, true);
	}
#endif
	return rc;
}

static int dwc3_generic_host_remove(struct udevice *dev)
{
	struct dwc3_generic_host_priv *priv = dev_get_priv(dev);
	int rc;

	rc = xhci_deregister(dev);
	if (rc)
		return rc;

	return dwc3_generic_remove(dev, &priv->gen_priv);
}

U_BOOT_DRIVER(dwc3_generic_host) = {
	.name	= "dwc3-generic-host",
	.id	= UCLASS_USB,
	.of_to_plat = dwc3_generic_of_to_plat,
	.probe = dwc3_generic_host_probe,
	.remove = dwc3_generic_host_remove,
	.priv_auto	= sizeof(struct dwc3_generic_host_priv),
	.plat_auto	= sizeof(struct dwc3_generic_plat),
	.ops = &xhci_usb_ops,
	.flags = DM_FLAG_ALLOC_PRIV_DMA,
};
#endif

void dwc3_imx8mp_ctrl0_1(struct udevice *dev, u32 clr0, u32 set0,
			 u32 clr1, u32 set1)
{
/* USB glue registers */
#define USB_CTRL0		0x00
#define USB_CTRL1		0x04
#define PHY_CTRL2		0x48

#define USB_CTRL0_PORTPWR_EN	BIT(12) /* 1 - PPC enabled (default) */
#define USB_CTRL0_USB3_FIXED	BIT(22) /* 1 - USB3 permanent attached */
#define USB_CTRL0_USB2_FIXED	BIT(23) /* 1 - USB2 permanent attached */

#define USB_CTRL1_OC_POLARITY	BIT(16) /* 0 - HIGH / 1 - LOW */
#define USB_CTRL1_PWR_POLARITY	BIT(17) /* 0 - HIGH / 1 - LOW */

#define PHY_CTRL2_ID_PULLUP	BIT(14)

	fdt_addr_t regs = dev_read_addr_index(dev, 1);
	void *base = map_physmem(regs, 0x8, MAP_NOCACHE);
	u32 value;

	if (clr0 || set0) {
		value = readl(base + USB_CTRL0);
		value &= ~clr0;
		value |= set0;
		writel(value, base + USB_CTRL0);
	}
	if (clr1 || set1) {
		value = readl(base + USB_CTRL1);
		value &= ~clr1;
		value |= set1;
		writel(value, base + USB_CTRL1);
	}
	debug("%s: base=%p %x %x\n", __func__, base, clr1, set1);
	unmap_physmem(base, MAP_NOCACHE);
}

void dwc3_glue_dt_parse(struct udevice *dev, struct dwc3_glue_data *glue)
{
	u32 clr0 = 0, set0 = 0;
	u32 clr1 = 0, set1 = 0;
	u32 vbus_oc_combined = 0;

	if (dev_read_bool(dev, "fsl,permanently-attached"))
		set0 = (USB_CTRL0_USB2_FIXED | USB_CTRL0_USB3_FIXED);
	else
		clr0 = (USB_CTRL0_USB2_FIXED | USB_CTRL0_USB3_FIXED);

	if (dev_read_bool(dev, "fsl,disable-port-power-control"))
		clr0 |= USB_CTRL0_PORTPWR_EN;
	else
		set0 |= USB_CTRL0_PORTPWR_EN;

	if (dev_read_bool(dev, "fsl,over-current-active-low"))
		set1 = USB_CTRL1_OC_POLARITY;
	else
		clr1 = USB_CTRL1_OC_POLARITY;

	if (dev_read_bool(dev, "fsl,power-active-low"))
		set1 |= USB_CTRL1_PWR_POLARITY;
	else
		clr1 |= USB_CTRL1_PWR_POLARITY;

	if (dev_read_bool(dev, "fsl,vbus-oc-combined"))
		vbus_oc_combined = 1;

	glue->usb_ctrl0_clr = clr0;
	glue->usb_ctrl0_set = set0;
	glue->usb_ctrl1_clr = clr1;
	glue->usb_ctrl1_set = set1;
	glue->vbus_oc_combined = vbus_oc_combined;
	debug("%s: %s: %x %x, %x %x, %d\n", __func__, ofnode_get_name(dev_ofnode(dev)), clr0, set0, clr1, set1, vbus_oc_combined);
}

void dwc3_imx8mp_glue_configure(struct udevice *dev, int index,
				enum usb_dr_mode mode)
{
	struct dwc3_glue_data *glue = dev_get_plat(dev);
	u32 clr1 = glue->usb_ctrl1_clr;
	u32 set1 = glue->usb_ctrl1_set;

	debug("%s: %s: %d, %d\n", __func__, ofnode_get_name(dev_ofnode(dev)), mode, glue->vbus_oc_combined);
	if ((mode == USB_DR_MODE_PERIPHERAL) && glue->vbus_oc_combined) {
		clr1 ^= USB_CTRL1_OC_POLARITY;
		set1 ^= USB_CTRL1_OC_POLARITY;
	}
	dwc3_imx8mp_ctrl0_1(dev, glue->usb_ctrl0_clr, glue->usb_ctrl0_set, clr1, set1);
}

enum usb_dr_mode dwc3_imx8mp_get_dr_mode(struct udevice *dev)
{
	fdt_addr_t regs0 = dev_read_addr_index(dev, 0);
	fdt_addr_t regs1 = dev_read_addr_index(dev, 1);
	void *base0 = map_physmem(regs0, 0x8, MAP_NOCACHE);
	void *base1 = map_physmem(regs1, 0x20, MAP_NOCACHE);
	u32 ctrl2, status;

#define USB_WAKEUP_STATUS		0x04
#define USB_WAKEUP_STATUS_IDDIG0	BIT(6)

	ctrl2 = readl(base1 + PHY_CTRL2);
	writel(ctrl2 | PHY_CTRL2_ID_PULLUP, base1 + PHY_CTRL2);
	udelay(1);

	status = readl(base0 + USB_WAKEUP_STATUS);
	writel(ctrl2, base1 + PHY_CTRL2);	/* Restore value */
	if (status & USB_WAKEUP_STATUS_IDDIG0) {
		debug("%s: peripheral\n", __func__);
		return USB_DR_MODE_PERIPHERAL;
	}
	debug("%s: host\n", __func__);
	return USB_DR_MODE_HOST;
}

struct dwc3_glue_ops imx8mp_ops = {
	.glue_configure = dwc3_imx8mp_glue_configure,
	.get_dr_mode = dwc3_imx8mp_get_dr_mode,
};

void dwc3_ti_glue_configure(struct udevice *dev, int index,
			    enum usb_dr_mode mode)
{
#define USBOTGSS_UTMI_OTG_STATUS		0x0084
#define USBOTGSS_UTMI_OTG_OFFSET		0x0480

/* UTMI_OTG_STATUS REGISTER */
#define USBOTGSS_UTMI_OTG_STATUS_SW_MODE	BIT(31)
#define USBOTGSS_UTMI_OTG_STATUS_POWERPRESENT	BIT(9)
#define USBOTGSS_UTMI_OTG_STATUS_TXBITSTUFFENABLE BIT(8)
#define USBOTGSS_UTMI_OTG_STATUS_IDDIG		BIT(4)
#define USBOTGSS_UTMI_OTG_STATUS_SESSEND	BIT(3)
#define USBOTGSS_UTMI_OTG_STATUS_SESSVALID	BIT(2)
#define USBOTGSS_UTMI_OTG_STATUS_VBUSVALID	BIT(1)
enum dwc3_omap_utmi_mode {
	DWC3_OMAP_UTMI_MODE_UNKNOWN = 0,
	DWC3_OMAP_UTMI_MODE_HW,
	DWC3_OMAP_UTMI_MODE_SW,
};

	u32 use_id_pin;
	u32 host_mode;
	u32 reg;
	u32 utmi_mode;
	u32 utmi_status_offset = USBOTGSS_UTMI_OTG_STATUS;

	struct dwc3_glue_data *glue = dev_get_plat(dev);
	void *base = map_physmem(glue->regs, 0x10000, MAP_NOCACHE);

	if (device_is_compatible(dev, "ti,am437x-dwc3"))
		utmi_status_offset += USBOTGSS_UTMI_OTG_OFFSET;

	utmi_mode = dev_read_u32_default(dev, "utmi-mode",
					 DWC3_OMAP_UTMI_MODE_UNKNOWN);
	if (utmi_mode != DWC3_OMAP_UTMI_MODE_HW) {
		debug("%s: OTG is not supported. defaulting to PERIPHERAL\n",
		      dev->name);
		mode = USB_DR_MODE_PERIPHERAL;
	}

	switch (mode)  {
	case USB_DR_MODE_PERIPHERAL:
		use_id_pin = 0;
		host_mode = 0;
		break;
	case USB_DR_MODE_HOST:
		use_id_pin = 0;
		host_mode = 1;
		break;
	case USB_DR_MODE_OTG:
	default:
		use_id_pin = 1;
		host_mode = 0;
		break;
	}

	reg = readl(base + utmi_status_offset);

	reg &= ~(USBOTGSS_UTMI_OTG_STATUS_SW_MODE);
	if (!use_id_pin)
		reg |= USBOTGSS_UTMI_OTG_STATUS_SW_MODE;

	writel(reg, base + utmi_status_offset);

	reg &= ~(USBOTGSS_UTMI_OTG_STATUS_SESSEND |
		USBOTGSS_UTMI_OTG_STATUS_VBUSVALID |
		USBOTGSS_UTMI_OTG_STATUS_IDDIG);

	reg |= USBOTGSS_UTMI_OTG_STATUS_SESSVALID |
		USBOTGSS_UTMI_OTG_STATUS_POWERPRESENT;

	if (!host_mode)
		reg |= USBOTGSS_UTMI_OTG_STATUS_IDDIG |
			USBOTGSS_UTMI_OTG_STATUS_VBUSVALID;

	writel(reg, base + utmi_status_offset);

	unmap_physmem(base, MAP_NOCACHE);
}

struct dwc3_glue_ops ti_ops = {
	.glue_configure = dwc3_ti_glue_configure,
};

static int dwc3_glue_bind(struct udevice *parent)
{
	struct dwc3_glue_ops *ops = (struct dwc3_glue_ops *)dev_get_driver_data(parent);
	ofnode node;
	int ret;
	enum usb_dr_mode dr_mode;

	dr_mode = usb_get_dr_mode(dev_ofnode(parent));

	ofnode_for_each_subnode(node, dev_ofnode(parent)) {
		const char *name = ofnode_get_name(node);
		struct udevice *dev;
		struct dwc3_generic_plat *plat;
		const char *driver = NULL;

		debug("%s: subnode name: %s\n", __func__, name);

		/* if the parent node doesn't have a mode check the leaf */
		if (!dr_mode)
			dr_mode = usb_get_dr_mode(node);

		switch (dr_mode) {
		case USB_DR_MODE_OTG:
			if (ops && ops->get_dr_mode) {
				dr_mode = ops->get_dr_mode(parent);
				if (dr_mode == USB_DR_MODE_HOST)
					goto host;
			}
		case USB_DR_MODE_PERIPHERAL:
#if CONFIG_IS_ENABLED(DM_USB_GADGET)
			debug("%s: dr_mode: OTG or Peripheral\n", __func__);
			driver = "dwc3-generic-peripheral";
#endif
			break;
#if defined(CONFIG_SPL_USB_HOST) || !defined(CONFIG_SPL_BUILD)
		case USB_DR_MODE_HOST:
host:
			debug("%s: dr_mode: HOST\n", __func__);
			driver = "dwc3-generic-host";
			break;
#endif
		default:
			debug("%s: unsupported dr_mode\n", __func__);
			return -ENODEV;
		};

		if (!driver)
			continue;

		ret = device_bind_driver_to_node(parent, driver, name,
						 node, &dev);
		if (ret) {
			debug("%s: not able to bind usb device mode\n",
			      __func__);
			return ret;
		}
		plat = dev_get_plat(dev);
		plat->dr_mode = dr_mode;
	}

	return 0;
}

static int dwc3_glue_reset_init(struct udevice *dev,
				struct dwc3_glue_data *glue)
{
	int ret;

	ret = reset_get_bulk(dev, &glue->resets);
	if (ret == -ENOTSUPP || ret == -ENOENT)
		return 0;
	else if (ret)
		return ret;

	ret = reset_deassert_bulk(&glue->resets);
	if (ret) {
		reset_release_bulk(&glue->resets);
		return ret;
	}

	return 0;
}

static int dwc3_glue_clk_init(struct udevice *dev,
			      struct dwc3_glue_data *glue)
{
	int ret;

	ret = clk_get_bulk(dev, &glue->clks);
	if (ret == -ENOSYS || ret == -ENOENT)
		return 0;
	if (ret)
		return ret;

#if CONFIG_IS_ENABLED(CLK)
	ret = clk_enable_bulk(&glue->clks);
	if (ret) {
		clk_release_bulk(&glue->clks);
		return ret;
	}
#endif

	return 0;
}

static int dwc3_glue_probe(struct udevice *dev)
{
	struct dwc3_glue_data *glue = dev_get_plat(dev);
	struct udevice *child = NULL;
	int index = 0;
	int ret;
	struct phy phy;

	ret = generic_phy_get_by_name(dev, "usb3-phy", &phy);
	if (!ret) {
		ret = generic_phy_init(&phy);
		if (ret)
			return ret;
	} else if (ret != -ENOENT && ret != -ENODATA) {
		debug("could not get phy (err %d)\n", ret);
		return ret;
	} else {
		phy.dev = NULL;
	}

	glue->regs = dev_read_addr(dev);

	ret = dwc3_glue_clk_init(dev, glue);
	if (ret)
		return ret;

	ret = dwc3_glue_reset_init(dev, glue);
	if (ret)
		return ret;

	if (phy.dev) {
		ret = generic_phy_power_on(&phy);
		if (ret)
			return ret;
	}

	ret = device_find_first_child(dev, &child);
	if (ret)
		return ret;

	if (glue->resets.count == 0) {
		ret = dwc3_glue_reset_init(child, glue);
		if (ret)
			return ret;
	}

	dwc3_glue_dt_parse(dev, glue);

	while (child) {
		enum usb_dr_mode dr_mode;

		dr_mode = usb_get_dr_mode(dev_ofnode(child));
		device_find_next_child(&child);
		dwc3_glue_configure(dev, index, dr_mode);
		index++;
	}

	return 0;
}

static int dwc3_glue_remove(struct udevice *dev)
{
	struct dwc3_glue_data *glue = dev_get_plat(dev);

	reset_release_bulk(&glue->resets);

	clk_release_bulk(&glue->clks);

	return 0;
}

static const struct udevice_id dwc3_glue_ids[] = {
	{ .compatible = "xlnx,zynqmp-dwc3" },
	{ .compatible = "xlnx,versal-dwc3" },
	{ .compatible = "ti,keystone-dwc3"},
	{ .compatible = "ti,dwc3", .data = (ulong)&ti_ops },
	{ .compatible = "ti,am437x-dwc3", .data = (ulong)&ti_ops },
	{ .compatible = "ti,am654-dwc3" },
	{ .compatible = "rockchip,rk3328-dwc3" },
	{ .compatible = "rockchip,rk3399-dwc3" },
	{ .compatible = "qcom,dwc3" },
	{ .compatible = "fsl,imx8mp-dwc3", .data = (ulong)&imx8mp_ops },
	{ .compatible = "fsl,imx8mq-dwc3" },
	{ .compatible = "intel,tangier-dwc3" },
	{ }
};

U_BOOT_DRIVER(dwc3_generic_wrapper) = {
	.name	= "dwc3-generic-wrapper",
	.id	= UCLASS_NOP,
	.of_match = dwc3_glue_ids,
	.bind = dwc3_glue_bind,
	.probe = dwc3_glue_probe,
	.remove = dwc3_glue_remove,
	.plat_auto	= sizeof(struct dwc3_glue_data),

};
