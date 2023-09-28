// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * Author: Chunfeng Yun <chunfeng.yun@mediatek.com>
 */

#include <common.h>
#include <asm-generic/gpio.h>
#include <dm/lists.h>
#include <linux/iopoll.h>

#include "mtu3.h"
#include "mtu3_dr.h"

void ssusb_set_force_mode(struct ssusb_mtk *ssusb,
			  enum mtu3_dr_force_mode mode)
{
	u32 value;

	value = mtu3_readl(ssusb->ippc_base, SSUSB_U2_CTRL(0));
	switch (mode) {
	case MTU3_DR_FORCE_DEVICE:
		value |= SSUSB_U2_PORT_FORCE_IDDIG | SSUSB_U2_PORT_RG_IDDIG;
		break;
	case MTU3_DR_FORCE_HOST:
		value |= SSUSB_U2_PORT_FORCE_IDDIG;
		value &= ~SSUSB_U2_PORT_RG_IDDIG;
		break;
	case MTU3_DR_FORCE_NONE:
		value &= ~(SSUSB_U2_PORT_FORCE_IDDIG | SSUSB_U2_PORT_RG_IDDIG);
		break;
	default:
		return;
	}
	mtu3_writel(ssusb->ippc_base, SSUSB_U2_CTRL(0), value);
}

void ssusb_set_vbusvalid_state(struct ssusb_mtk *ssusb)
{
	enum usb_dr_mode dr_mode = ssusb->dr_mode;
	u32 u2ctl, u2ctl_old;
	u32 misc, misc_old;

	if (!ssusb->force_vbus && !ssusb->force_vbus_peripheral)
		return;

	misc_old = misc = mtu3_readl(ssusb->mac_base, U3D_MISC_CTRL);
	misc &= ~(VBUS_FRC_EN | VBUS_ON);

	switch (dr_mode) {
	case USB_DR_MODE_HOST:
		if (ssusb->force_vbus)
			misc |= VBUS_FRC_EN | VBUS_ON;
		break;
	case USB_DR_MODE_OTG:
	case USB_DR_MODE_PERIPHERAL:
		misc |= VBUS_FRC_EN | VBUS_ON;
		u2ctl = mtu3_readl(ssusb->ippc_base, SSUSB_U2_CTRL(0));
		u2ctl &= ~SSUSB_U2_PORT_OTG_SEL;
		mtu3_writel(ssusb->ippc_base, SSUSB_U2_CTRL(0), u2ctl);
		break;
	case USB_DR_MODE_UNKNOWN:
		u2ctl_old = u2ctl = mtu3_readl(ssusb->ippc_base, SSUSB_U2_CTRL(0));
		u2ctl |= SSUSB_U2_PORT_OTG_SEL;
		if (u2ctl != u2ctl_old) {
			pr_debug("%s: mode=%d u2 was %x, now %x %d %d\n", __func__, dr_mode, u2ctl_old, u2ctl,
				ssusb->force_vbus, ssusb->force_vbus_peripheral);
			mtu3_writel(ssusb->ippc_base, SSUSB_U2_CTRL(0), u2ctl);
		}
		break;
	}
	if (misc != misc_old) {
		pr_debug("%s: role=%d was %x, now %x %d %d\n", __func__, dr_mode, misc_old, misc,
			ssusb->force_vbus, ssusb->force_vbus_peripheral);
		mtu3_writel(ssusb->mac_base, U3D_MISC_CTRL, misc);
	}
}

/* u2-port0 should be powered on and enabled; */
int ssusb_check_clocks(struct ssusb_mtk *ssusb, u32 ex_clks)
{
	void __iomem *ibase = ssusb->ippc_base;
	u32 value, check_val;
	int ret;

	check_val = ex_clks | SSUSB_SYS125_RST_B_STS | SSUSB_SYSPLL_STABLE |
			SSUSB_REF_RST_B_STS;

	ret = readl_poll_timeout(ibase + U3D_SSUSB_IP_PW_STS1, value,
				 ((value & check_val) == check_val), 10000);
	if (ret) {
		dev_err(ssusb->dev, "clks of sts1 are not stable!\n");
		return ret;
	}

	ret = readl_poll_timeout(ibase + U3D_SSUSB_IP_PW_STS2, value,
				 (value & SSUSB_U2_MAC_SYS_RST_B_STS), 10000);
	if (ret) {
		dev_err(ssusb->dev, "mac2 clock is not stable\n");
		return ret;
	}

	return 0;
}

int ssusb_phy_setup(struct ssusb_mtk *ssusb)
{
	struct udevice *dev = ssusb->dev;
	struct phy_bulk *phys = &ssusb->phys;
	int ret;

	ret = generic_phy_get_bulk(dev, phys);
	if (ret)
		return ret;

	ret = generic_phy_init_bulk(phys);
	if (ret)
		return ret;

	ret = generic_phy_power_on_bulk(phys);
	if (ret)
		generic_phy_exit_bulk(phys);

	return ret;
}

void ssusb_phy_shutdown(struct ssusb_mtk *ssusb)
{
	generic_phy_power_off_bulk(&ssusb->phys);
	generic_phy_exit_bulk(&ssusb->phys);
}

static int ssusb_rscs_init(struct ssusb_mtk *ssusb)
{
	int ret = 0;

	ret = regulator_set_enable(ssusb->vusb33_supply, true);
	if (ret < 0 && ret != -ENOSYS) {
		dev_err(ssusb->dev, "failed to enable vusb33\n");
		goto vusb33_err;
	}

	ret = clk_enable_bulk(&ssusb->clks);
	if (ret)
		goto clks_err;

	ret = ssusb_phy_setup(ssusb);
	if (ret) {
		dev_err(ssusb->dev, "failed to setup phy\n");
		goto phy_err;
	}

	return 0;

phy_err:
	clk_disable_bulk(&ssusb->clks);
clks_err:
	regulator_set_enable(ssusb->vusb33_supply, false);
vusb33_err:
	return ret;
}

static void ssusb_rscs_exit(struct ssusb_mtk *ssusb)
{
	clk_disable_bulk(&ssusb->clks);
	regulator_set_enable(ssusb->vusb33_supply, false);
	ssusb_phy_shutdown(ssusb);
}

static void ssusb_ip_sw_reset(struct ssusb_mtk *ssusb)
{
	/* reset whole ip (xhci & u3d) */
	mtu3_setbits(ssusb->ippc_base, U3D_SSUSB_IP_PW_CTRL0, SSUSB_IP_SW_RST);
	udelay(1);
	mtu3_clrbits(ssusb->ippc_base, U3D_SSUSB_IP_PW_CTRL0, SSUSB_IP_SW_RST);
}

static int get_ssusb_rscs(struct udevice *dev, struct ssusb_mtk *ssusb)
{
	struct udevice *child = NULL;
	ofnode node = dev_ofnode(dev);
	const char *name;
	ofnode cnode;
	int ret;

	ret = device_get_supply_regulator(dev, "vusb33-supply",
					  &ssusb->vusb33_supply);
	if (ret)	/* optional, ignore error */
		dev_warn(dev, "can't get optional vusb33 %d\n", ret);

	ret = device_get_supply_regulator(dev, "vbus-supply",
					  &ssusb->vbus_supply);
	if (ret)	/* optional, ignore error */
		dev_warn(dev, "can't get optional vbus regulator %d!\n", ret);

	ret = clk_get_bulk(dev, &ssusb->clks);
	if (ret) {
		dev_err(dev, "failed to get clocks %d!\n", ret);
		return ret;
	}

	ssusb->ippc_base = devfdt_remap_addr_name(dev, "ippc");
	if (!ssusb->ippc_base) {
		dev_err(dev, "error mapping memory for ippc\n");
		return -ENODEV;
	}

	ssusb->force_vbus = dev_read_bool(dev, "mediatek,force-vbus");
	ssusb->force_vbus_peripheral = dev_read_bool(dev, "mediatek,force-vbus-peripheral");

	ssusb->dr_mode = usb_get_dr_mode(node);
	if (ssusb->dr_mode == USB_DR_MODE_UNKNOWN ||
		ssusb->dr_mode == USB_DR_MODE_OTG)
		ssusb->dr_mode = USB_DR_MODE_PERIPHERAL;

	ssusb->mac_base = devfdt_remap_addr_name(dev, "mac");
	if (!ssusb->mac_base) {
		dev_err(dev, "error mapping memory for mac\n");
		return -ENODEV;
	}

	if ((ssusb->dr_mode == USB_DR_MODE_HOST)) {
		cnode = ofnode_get_child_by_compatible(node, "mediatek,mtk-xhci");
		if (!ofnode_valid(cnode))
			return -ENODEV;
		ret = device_find_by_ofnode(cnode, &child);
	} else {
		name = ofnode_get_name(node);
		ret = device_find_child_by_name(dev, name, &child);
	}
	if (ret || !child) {
		dev_err(dev, "failed to get child %d! %s\n", ret, ofnode_get_name(node));
		return ret;
	}

	dev_info(dev, "dr_mode: %d, ippc: 0x%p, mac: 0x%p\n",
		 ssusb->dr_mode, ssusb->ippc_base, ssusb->mac_base);

	return 0;
}

static int mtu3_probe(struct udevice *dev)
{
	struct ssusb_mtk *ssusb = dev_get_priv(dev);
	int ret = -ENOMEM;
	struct gpio_desc *reset;

	ssusb->dev = dev;

	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	if (IS_ERR(reset))
		return PTR_ERR(reset);

	ret = get_ssusb_rscs(dev, ssusb);
	if (ret)
		return ret;

	ret = ssusb_rscs_init(ssusb);
	if (ret)
		return ret;

	ssusb_ip_sw_reset(ssusb);
	dm_gpio_set_value(reset, 0);
	ssusb->reset = reset;

	return 0;
}

static int mtu3_remove(struct udevice *dev)
{
	struct ssusb_mtk *ssusb = dev_to_ssusb(dev);

	ssusb_rscs_exit(ssusb);
	dm_gpio_set_value(ssusb->reset, 1);
	return 0;
}

#if CONFIG_IS_ENABLED(DM_USB_GADGET)
int mtu3_gadget_handle_interrupts(struct udevice *dev)
{
	struct mtu3 *mtu = dev_get_priv(dev);

	mtu3_irq(0, mtu);

	return 0;
}

static int mtu3_gadget_probe(struct udevice *dev)
{
	struct ssusb_mtk *ssusb = dev_to_ssusb(dev->parent);
	struct mtu3 *mtu = dev_get_priv(dev);

	mtu->dev = dev;
	ssusb->u3d = mtu;
	return ssusb_gadget_init(ssusb);
}

static int mtu3_gadget_remove(struct udevice *dev)
{
	struct mtu3 *mtu = dev_get_priv(dev);

	ssusb_gadget_exit(mtu->ssusb);
	return 0;
}

U_BOOT_DRIVER(mtu3_peripheral) = {
	.name = "mtu3-peripheral",
	.id = UCLASS_USB_GADGET_GENERIC,
	.probe = mtu3_gadget_probe,
	.remove = mtu3_gadget_remove,
	.handle_interrupts = mtu3_gadget_handle_interrupts,
	.priv_auto	= sizeof(struct mtu3),
};
#endif

#if defined(CONFIG_SPL_USB_HOST) || \
	(!defined(CONFIG_SPL_BUILD) && defined(CONFIG_USB_HOST))
static int mtu3_host_probe(struct udevice *dev)
{
	struct ssusb_mtk *ssusb = dev_to_ssusb(dev->parent);
	struct mtu3_host *u3h = dev_get_priv(dev);
	struct udevice *vbus_supply;
	struct xhci_hcor *hcor;
	fdt_addr_t hcd_base;
	int rc;

	hcd_base = dev_read_addr(dev);
	if (hcd_base == FDT_ADDR_T_NONE) {
		dev_err(dev, "error mapping memory for mac\n");
		return -ENODEV;
	}
	u3h->hcd = (void *)hcd_base;

	u3h->dev = dev;
	ssusb->u3h = u3h;
	rc = ssusb_host_init(ssusb);
	if (rc)
		return rc;

	rc = device_get_supply_regulator(dev, "vbus-supply",
					  &vbus_supply);
	if (rc) {
		debug("can't get vbus regulator %d %s!\n", rc, ofnode_get_name(dev_ofnode(dev)));
	} else {
		regulator_set_enable(vbus_supply, true);
		u3h->vbus_supply = vbus_supply;
	}

	u3h->ctrl.quirks = XHCI_MTK_HOST;
	hcor = (struct xhci_hcor *)((uintptr_t)u3h->hcd +
			HC_LENGTH(xhci_readl(&u3h->hcd->cr_capbase)));

	return xhci_register(dev, u3h->hcd, hcor);
}

static int mtu3_host_remove(struct udevice *dev)
{
	struct mtu3_host *u3h = dev_get_priv(dev);

	xhci_deregister(dev);
	ssusb_host_exit(u3h->ssusb);
	regulator_set_enable(u3h->vbus_supply, false);
	return 0;
}

static const struct udevice_id mtu3_host_of_match[] = {
	{.compatible = "mediatek,mtk-xhci",},
	{},
};

U_BOOT_DRIVER(mtu3_host) = {
	.name = "mtu3-host",
	.id = UCLASS_USB,
	.of_match = mtu3_host_of_match,
	.probe = mtu3_host_probe,
	.remove = mtu3_host_remove,
	.priv_auto	= sizeof(struct mtu3_host),
	.ops = &xhci_usb_ops,
	.flags = DM_FLAG_ALLOC_PRIV_DMA,
};
#endif

static int mtu3_glue_bind(struct udevice *parent)
{
	ofnode node = dev_ofnode(parent);
	struct udevice *dev;
	enum usb_dr_mode dr_mode;
	const char *driver;
	const char *name;
	int ret;

	dr_mode = usb_get_dr_mode(node);

	switch (dr_mode) {
#if CONFIG_IS_ENABLED(DM_USB_GADGET)
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		dev_dbg(parent, "%s: dr_mode: peripheral\n", __func__);
		driver = "mtu3-peripheral";
		name = ofnode_get_name(node);
		ret = device_bind_driver(parent, driver, name, &dev);
		break;
#endif

#if defined(CONFIG_SPL_USB_HOST) || \
	(!defined(CONFIG_SPL_BUILD) && defined(CONFIG_USB_HOST))
	case USB_DR_MODE_HOST:
		dev_dbg(parent, "%s: dr_mode: host\n", __func__);
		node = ofnode_get_child_by_compatible(node, "mediatek,mtk-xhci");
		if (!ofnode_valid(node))
			return -ENODEV;
		driver = "mtu3-host";
		if (!ofnode_valid(node))
			return -ENODEV;
		name = ofnode_get_name(node);
		ret = device_bind_driver_to_node(parent, driver, name, node, &dev);
		break;
#endif
	default:
		dev_err(parent, "%s: unsupported dr_mode %d %s\n",
			__func__, dr_mode, ofnode_get_name(node));
		return -ENODEV;
	};

	dev_dbg(parent, "%s: node name: %s, driver %s, dr_mode %d\n",
		__func__, name, driver, dr_mode);

	if (ret)
		dev_err(parent, "%s: not able to bind usb device mode\n",
			__func__);

	return ret;
}

static const struct udevice_id mtu3_of_match[] = {
	{.compatible = "mediatek,mtu3",},
	{},
};

U_BOOT_DRIVER(mtu3) = {
	.name = "mtu3",
	.id = UCLASS_NOP,
	.of_match = mtu3_of_match,
	.bind = mtu3_glue_bind,
	.probe = mtu3_probe,
	.remove = mtu3_remove,
	.priv_auto	= sizeof(struct ssusb_mtk),
};
