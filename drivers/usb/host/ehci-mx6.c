// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2009 Daniel Mack <daniel@caiaq.de>
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 */

#include <common.h>
#include <clk.h>
#include <log.h>
#include <usb.h>
#include <errno.h>
#include <wait_bit.h>
#include <asm/global_data.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <usb/ehci-ci.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/regs-usbphy.h>
#include <asm/mach-imx/sys_proto.h>
#include <dm.h>
#include <asm/mach-types.h>
#include <power/regulator.h>
#include <linux/usb/otg.h>
#include <linux/usb/phy.h>
#include <power-domain.h>

#include "ehci.h"
#if CONFIG_IS_ENABLED(POWER_DOMAIN)
#include <power-domain.h>
#endif
#include <clk.h>
#include <usb/usb_mx6_common.h>

#if CONFIG_IS_ENABLED(PHY) && !defined(CONFIG_IMX8) && !defined(CONFIG_IMX8ULP) && !defined(CONFIG_IMX8MM) && !defined(CONFIG_IMX8MN)
#define USE_USB_PHY	1
#endif

DECLARE_GLOBAL_DATA_PTR;

#define USB_OTGREGS_OFFSET	0x000
#define USB_H1REGS_OFFSET	0x200
#define USB_H2REGS_OFFSET	0x400
#define USB_H3REGS_OFFSET	0x600
#define USB_OTHERREGS_OFFSET	0x800

#define USB_H1_CTRL_OFFSET	0x04

#define USBNC_OFFSET		0x200

/* If this is not defined, assume MX6/MX7/MX8M SoC default */
#ifndef CONFIG_MXC_USB_PORTSC
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#endif

static void ehci_mx6_powerup_fixup(struct ehci_ctrl *ctrl, uint32_t *status_reg,
				   uint32_t *reg)
{
	uint32_t result;
	int usec = 2000;

	mdelay(50);

	do {
		result = ehci_readl(status_reg);
		udelay(5);
		if (!(result & EHCI_PS_PR))
			break;
		usec--;
	} while (usec > 0);

	*reg = ehci_readl(status_reg);
}

#if !CONFIG_IS_ENABLED(DM_USB)
/**
 * board_usb_phy_mode - override usb phy mode
 * @port:	usb host/otg port
 *
 * Target board specific, override usb_phy_mode.
 * When usb-otg is used as usb host port, iomux pad usb_otg_id can be
 * left disconnected in this case usb_phy_mode will not be able to identify
 * the phy mode that usb port is used.
 * Machine file overrides board_usb_phy_mode.
 *
 * Return: USB_INIT_DEVICE or USB_INIT_HOST
 */
int __weak board_usb_phy_mode(int port)
{
	return usb_phy_mode(port);
}

/**
 * board_ehci_hcd_init - set usb vbus voltage
 * @port:      usb otg port
 *
 * Target board specific, setup iomux pad to setup supply vbus voltage
 * for usb otg port. Machine board file overrides board_ehci_hcd_init
 *
 * Return: 0 Success
 */
int __weak board_ehci_hcd_init(int port)
{
	return 0;
}

/**
 * board_ehci_power - enables/disables usb vbus voltage
 * @port:      usb otg port
 * @on:        on/off vbus voltage
 *
 * Enables/disables supply vbus voltage for usb otg port.
 * Machine board file overrides board_ehci_power
 *
 * Return: 0 Success
 */
int __weak board_ehci_power(int port, int on)
{
	return 0;
}

static const struct ehci_ops mx6_ehci_ops = {
	.powerup_fixup		= ehci_mx6_powerup_fixup,
};

int ehci_hcd_init(int index, enum usb_init_type init,
		struct ehci_hccr **hccr, struct ehci_hcor **hcor)
{
	enum usb_init_type type;
	struct ehci_mx6_phy_data phy_data;
	memset(&phy_data, 0, sizeof(phy_data));

#if defined(CONFIG_MX6) || defined(CONFIG_IMXRT)
	if (index > 3)
			return -EINVAL;

	u32 controller_spacing = 0x200;
	phy_data.anatop_addr = (void __iomem *)ANATOP_BASE_ADDR;
	phy_data.misc_addr = (void __iomem *)(USB_BASE_ADDR +
			USB_OTHERREGS_OFFSET);
	phy_data.phy_addr = (void __iomem *)get_usb_mx6_phy_base(index);
	if (!phy_data.phy_addr)
		return -EINVAL;

#elif defined(CONFIG_USB_EHCI_MX7)
	if (index > 1)
		return -EINVAL;

	u32 controller_spacing = 0x10000;
	phy_data.misc_addr = (void __iomem *)(ulong)(USB_BASE_ADDR +
			(0x10000 * index) + USBNC_OFFSET);
#elif defined(CONFIG_MX7ULP) || defined(CONFIG_IMX8) || defined(CONFIG_IMX8ULP)
	u32 controller_spacing = is_imx8ulp()? 0x20000: 0x10000;
	phy_data.phy_addr = (void __iomem *)get_usb_mx6_phy_base(index);
	if (!phy_data.phy_addr)
		return -EINVAL;
	phy_data.misc_addr = (void __iomem *)(ulong)(USB_BASE_ADDR +
			(controller_spacing * index) + USBNC_OFFSET);
#endif
	struct usb_ehci *ehci = (struct usb_ehci *)(ulong)(USB_BASE_ADDR +
		(controller_spacing * index));
	int ret;

	if (CONFIG_IS_ENABLED(IMX_MODULE_FUSE)) {
		if (usb_fused((ulong)ehci)) {
			printf("SoC fuse indicates USB@0x%lx is unavailable.\n",
			       (ulong)ehci);
			return	-ENODEV;
		}
	}

	enable_usboh3_clk(1);
	mdelay(1);

	/* Do board specific initialization */
	ret = board_ehci_hcd_init(index);
	if (ret) {
		enable_usboh3_clk(0);
		return ret;
	}

	ehci_mx6_phy_init(ehci, &phy_data, index);

	ehci_set_controller_priv(index, NULL, &mx6_ehci_ops);

	type = board_usb_phy_mode(index);

	if (hccr && hcor) {
		*hccr = (struct ehci_hccr *)((uintptr_t)&ehci->caplength);
		*hcor = (struct ehci_hcor *)((uintptr_t)*hccr +
				HC_LENGTH(ehci_readl(&(*hccr)->cr_capbase)));
	}

	if ((type == init) || (type == USB_INIT_DEVICE))
		board_ehci_power(index, (type == USB_INIT_DEVICE) ? 0 : 1);
	if (type != init)
		return -ENODEV;

	if (is_mx6dqp() || is_mx6dq() || is_mx6sdl() ||
		((is_mx6sl() || is_mx6sx()) && type == USB_INIT_HOST))
		setbits_le32(&ehci->usbmode, SDIS);

	if (type == USB_INIT_DEVICE)
		return 0;

	setbits_le32(&ehci->usbmode, CM_HOST);
	writel(CONFIG_MXC_USB_PORTSC, &ehci->portsc);
	setbits_le32(&ehci->portsc, USB_EN);

	mdelay(10);

	return 0;
}

int ehci_hcd_stop(int index)
{
	return 0;
}
#else
struct ehci_mx6_priv_data {
	struct ehci_ctrl ctrl;
	struct usb_ehci *ehci;
	struct udevice *vbus_supply;
	struct clk clk;
	struct phy phy;
	enum usb_init_type init_type;
	enum usb_phy_interface phy_type;
	int portnr;
#if !defined(USE_USB_PHY)
	struct udevice phy_dev;
	struct ehci_mx6_phy_data phy_data;
#if CONFIG_IS_ENABLED(POWER_DOMAIN)
	struct power_domain phy_pd;
#endif
#if CONFIG_IS_ENABLED(CLK)
	struct clk phy_clk;
#endif
#endif
	struct gpio_desc *gd_reset;
	struct gpio_desc gds_reset;
};

static u32 mx6_portsc(enum usb_phy_interface phy_type)
{
	switch (phy_type) {
	case USBPHY_INTERFACE_MODE_UTMI:
		return PORT_PTS_UTMI;
	case USBPHY_INTERFACE_MODE_UTMIW:
		return PORT_PTS_UTMI | PORT_PTS_PTW;
	case USBPHY_INTERFACE_MODE_ULPI:
		return PORT_PTS_ULPI;
	case USBPHY_INTERFACE_MODE_SERIAL:
		return PORT_PTS_SERIAL;
	case USBPHY_INTERFACE_MODE_HSIC:
		return PORT_PTS_HSIC;
	default:
		return CONFIG_MXC_USB_PORTSC;
	}
}

#if CONFIG_IS_ENABLED(DM_USB) && !CONFIG_IS_ENABLED(DM_REGULATOR)
int __weak board_usb_power_gpio(int port)
{
	return 0;
}

int board_usb_power(int port, enum usb_init_type init)
{
	int gp = board_usb_power_gpio(port);

	if (gp) {
		debug("%s: %d %d\n", __func__, port, init);
		gpio_direction_output(gp, (init == USB_INIT_DEVICE) ? 0 : 1);
	}
	return 0;
}
#endif

static int mx6_init_after_reset(struct ehci_ctrl *dev)
{
	struct ehci_mx6_priv_data *priv = dev->priv;
	enum usb_init_type type = priv->init_type;
	struct usb_ehci *ehci = priv->ehci;
	int ret;

	if (priv->gd_reset) {
		dm_gpio_set_value(priv->gd_reset, 1);	/* activate reset */
		mdelay(2);
		dm_gpio_set_value(priv->gd_reset, 0);
	}
	ret = board_usb_init(priv->portnr, priv->init_type);
	if (ret) {
		printf("Failed to initialize board for USB\n");
		return ret;
	}

#if !defined(USE_USB_PHY)
	ehci_mx6_phy_init(ehci, &priv->phy_data, priv->portnr);
#endif

#if CONFIG_IS_ENABLED(DM_REGULATOR)
	if (priv->vbus_supply) {
		int ret;
		ret = regulator_set_enable(priv->vbus_supply,
					   (type == USB_INIT_DEVICE) ?
					   false : true);
		if (ret && ret != -ENOSYS) {
			printf("Error enabling VBUS supply (ret=%i)\n", ret);
			return ret;
		}
	}
#else
	ret = board_usb_power(priv->portnr, priv->init_type);
#endif

	if (is_mx6dqp() || is_mx6dq() || is_mx6sdl() ||
		((is_mx6sl() || is_mx6sx()) && type == USB_INIT_HOST))
		setbits_le32(&ehci->usbmode, SDIS);

	if (type == USB_INIT_DEVICE)
		return 0;

	setbits_le32(&ehci->usbmode, CM_HOST);
	writel(mx6_portsc(priv->phy_type), &ehci->portsc);
	setbits_le32(&ehci->portsc, USB_EN);

	mdelay(10);

	return 0;
}

static const struct ehci_ops mx6_ehci_ops = {
	.powerup_fixup		= ehci_mx6_powerup_fixup,
	.init_after_reset 	= mx6_init_after_reset
};

/**
 * board_ehci_usb_phy_mode - override usb phy mode
 * @port:	usb host/otg port
 *
 * Target board specific, override usb_phy_mode.
 * When usb-otg is used as usb host port, iomux pad usb_otg_id can be
 * left disconnected in this case usb_phy_mode will not be able to identify
 * the phy mode that usb port is used.
 * Machine file overrides board_usb_phy_mode.
 * When the extcon property is set in DTB, machine must provide this function, otherwise
 * it will default return HOST.
 *
 * Return: USB_INIT_DEVICE or USB_INIT_HOST
 */
int __weak board_ehci_usb_phy_mode(struct udevice *dev)
{
	return USB_INIT_HOST;
}

int __weak board_usb_otg_mode(struct udevice *dev)
{
	return -ENODEV;
}

static int ehci_usb_phy_mode(struct udevice *dev)
{
	struct ehci_mx6_priv_data *priv = dev_get_priv(dev);
	void *__iomem addr = (void *__iomem)ofnode_get_addr(dev_ofnode(dev));
	void *__iomem phy_ctrl, *__iomem phy_status;
	struct ofnode_phandle_args arg;
	int ret;
	u32 val;

	ret = board_usb_otg_mode(dev);
	if (ret >= 0) {
		priv->init_type = ret;
		if (ret != USB_INIT_UNKNOWN)
			return 0;
	}
	/*
	 * About fsl,usbphy, Refer to
	 * Documentation/devicetree/bindings/usb/ci-hdrc-usb2.txt.
	 */
	if (is_mx6() || is_mx7ulp() || is_imxrt() || is_imx8() || is_imx8ulp()) {
		ret = dev_read_phandle_with_args(dev, "fsl,usbphy",
				NULL, 0, 0, &arg);
		if (ret) {
			ret = dev_read_phandle_with_args(dev, "phys",
					NULL, 0, 0, &arg);
			if (ret)
				return -EINVAL;
		}

		addr = (void __iomem *)ofnode_get_addr(arg.node);
		if ((fdt_addr_t)addr == FDT_ADDR_T_NONE)
			return -EINVAL;

		phy_ctrl = (void __iomem *)(addr + USBPHY_CTRL);
		val = readl(phy_ctrl);

		if (val & USBPHY_CTRL_OTG_ID)
			priv->init_type = USB_INIT_DEVICE;
		else
			priv->init_type = USB_INIT_HOST;
	} else if (is_mx7() || is_imx8mm() || is_imx8mn()) {
		phy_status = (void __iomem *)(addr +
					      USBNC_PHY_STATUS_OFFSET);
		val = readl(phy_status);

		if (val & USBNC_PHYSTATUS_ID_DIG)
			priv->init_type = USB_INIT_DEVICE;
		else
			priv->init_type = USB_INIT_HOST;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int ehci_usb_of_to_plat(struct udevice *dev)
{
	struct usb_plat *plat = dev_get_plat(dev);
	struct ehci_mx6_priv_data *priv = dev_get_priv(dev);
	enum usb_dr_mode dr_mode;
	const struct fdt_property *extcon;
	int ret;

	extcon = fdt_get_property(gd->fdt_blob, dev_of_offset(dev),
			"extcon", NULL);
	if (extcon) {
		priv->init_type = board_ehci_usb_phy_mode(dev);
		goto check_type;
	}

	dr_mode = usb_get_dr_mode(dev_ofnode(dev));

	switch (dr_mode) {
	case USB_DR_MODE_HOST:
		priv->init_type = USB_INIT_HOST;
		break;
	case USB_DR_MODE_PERIPHERAL:
		priv->init_type = USB_INIT_DEVICE;
		break;
	default:
		priv->init_type = USB_INIT_UNKNOWN;
	};

check_type:
	if (priv->init_type != USB_INIT_UNKNOWN && priv->init_type != plat->init_type) {
		debug("Request USB type is %u, board forced type is %u\n",
			plat->init_type, priv->init_type);
		return -ENODEV;
	}

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->gds_reset,
				   GPIOD_IS_OUT);
	if (ret) {
		debug("%s: Warning: cannot get reset-gpios: ret=%d\n",
		      __func__, ret);
		if (ret != -ENOENT)
			return ret;
	} else {
		priv->gd_reset = &priv->gds_reset;
	}
	return 0;
}

static int mx6_parse_dt_addrs(struct udevice *dev)
{
	struct ehci_mx6_priv_data *priv = dev_get_priv(dev);
	ofnode phy;
	ofnode misc;
	int ret;

	ret = ofnode_parse_phandle(dev_ofnode(dev), "fsl,usbphy", &phy);
	if (ret < 0) {
		ret = ofnode_parse_phandle(dev_ofnode(dev), "phys", &phy);
		if (ret < 0)
			return ret;
	}
#if !defined(USE_USB_PHY)
	dev_set_ofnode(&priv->phy_dev, phy);
#endif

	ret = ofnode_parse_phandle(dev_ofnode(dev), "fsl,usbmisc", &misc);
	if (ret < 0)
		return ret;

	priv->portnr = dev_seq(dev);

#if !defined(USE_USB_PHY)
	void *__iomem addr;
	struct ehci_mx6_phy_data *phy_data = &priv->phy_data;
	addr = (void __iomem *)ofnode_get_addr(phy);
	if ((fdt_addr_t)addr == FDT_ADDR_T_NONE)
		addr = NULL;

	phy_data->phy_addr = addr;

	addr = (void __iomem *)ofnode_get_addr(misc);
	if ((fdt_addr_t)addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	phy_data->misc_addr = addr;

#if defined(CONFIG_MX6)
	ofnode anatop;

	/* Resolve ANATOP offset through USB PHY node */
	ret = ofnode_parse_phandle(phy, "fsl,anatop", &anatop);
	if (ret < 0)
		return ret;

	addr = (void __iomem *)ofnode_get_addr(anatop);
	if ((fdt_addr_t)addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	phy_data->anatop_addr = addr;
#endif
#endif
	return 0;
}

#if !defined(USE_USB_PHY)
static int ehci_mx6_phy_prepare(struct ehci_mx6_priv_data *priv)
{
#if CONFIG_IS_ENABLED(POWER_DOMAIN)
	/* Need to power on the PHY before access it */
	if (!power_domain_get(&priv->phy_dev, &priv->phy_pd)) {
		if (power_domain_on(&priv->phy_pd))
			return -EINVAL;
	}
#endif

#if CONFIG_IS_ENABLED(CLK)
	int ret;

	ret = clk_get_by_index(&priv->phy_dev, 0, &priv->phy_clk);
	if (ret) {
		printf("Failed to get phy_clk\n");
		return ret;
	}

	ret = clk_enable(&priv->phy_clk);
	if (ret) {
		printf("Failed to enable phy_clk\n");
		return ret;
	}
#endif

	return 0;
}

static int ehci_mx6_phy_remove(struct ehci_mx6_priv_data *priv)
{
	int ret = 0;

#if CONFIG_IS_ENABLED(CLK)
	if (priv->phy_clk.dev) {
		ret = clk_disable(&priv->phy_clk);
		if (ret)
			return ret;

		ret = clk_free(&priv->phy_clk);
		if (ret)
			return ret;
	}
#endif

#if CONFIG_IS_ENABLED(POWER_DOMAIN)
	if (priv->phy_pd.dev) {
		ret = power_domain_off(&priv->phy_pd);
		if (ret)
			printf("Power down USB PHY failed! (error = %d)\n", ret);
	}
#endif

	return ret;
}
#endif

int ehci_is_host(struct udevice *dev)
{
	struct ehci_mx6_priv_data *priv = dev_get_priv(dev);

	return (priv->init_type == USB_INIT_HOST) ? 1 : 0;
}

static int ehci_usb_probe(struct udevice *dev)
{
	struct usb_plat *plat = dev_get_plat(dev);
	struct usb_ehci *ehci = dev_read_addr_ptr(dev);
	struct ehci_mx6_priv_data *priv = dev_get_priv(dev);
	enum usb_init_type type;
	struct ehci_hccr *hccr;
	struct ehci_hcor *hcor;
	int ret;

	if (CONFIG_IS_ENABLED(IMX_MODULE_FUSE)) {
		if (usb_fused((ulong)ehci)) {
			printf("SoC fuse indicates USB@0x%lx is unavailable.\n",
			       (ulong)ehci);
			return -ENODEV;
		}
	}

	ret = mx6_parse_dt_addrs(dev);
	if (ret)
		return ret;

	priv->ehci = ehci;
#if defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MN)
	enable_usboh3_clk(1);
	imx8m_usb_power(priv->portnr, true);
#endif
	type = plat->init_type;

	priv->phy_type = usb_get_phy_mode(dev_ofnode(dev));

	/* Init usb board level according to the requested init type */
	ret = board_usb_init(priv->portnr, type);
	if (ret) {
		printf("Failed to initialize board for USB\n");
		return ret;
	}

#if !defined(USE_USB_PHY)
	ret = ehci_mx6_phy_prepare(priv);
	if (ret) {
		printf("Failed to prepare USB phy\n");
		return ret;
	}
#endif

#if CONFIG_IS_ENABLED(CLK)
	ret = clk_get_by_index(dev, 0, &priv->clk);
	if (ret < 0)
		return ret;

	ret = clk_enable(&priv->clk);
	if (ret)
		return ret;
#else
	/* Compatibility with DM_USB and !CLK */
	enable_usboh3_clk(1);
	mdelay(1);
#endif

#if CONFIG_IS_ENABLED(DM_REGULATOR)
	ret = device_get_supply_regulator(dev, "vbus-supply",
					  &priv->vbus_supply);
	if (ret)
		debug("%s: No vbus supply\n", dev->name);
#endif

#if !defined(USE_USB_PHY)
	ehci_mx6_phy_init(ehci, &priv->phy_data, priv->portnr);
#else
	ret = ehci_setup_phy(dev, &priv->phy, priv->portnr);
	if (ret)
		goto err_clk;
#endif

	/* If the init_type is unknown due to it is not forced in DTB, we use USB ID to detect */
	if (priv->init_type == USB_INIT_UNKNOWN) {
		ret = ehci_usb_phy_mode(dev);
		if (ret)
			goto err_clk;
		if (priv->init_type != type) {
			ret = -ENODEV;
			goto err_clk;
		}
	}

#if CONFIG_IS_ENABLED(DM_REGULATOR)
	if (priv->vbus_supply) {
		ret = regulator_set_enable(priv->vbus_supply,
					   (priv->init_type == USB_INIT_DEVICE) ?
					   false : true);
		if (ret && ret != -ENOSYS) {
			printf("Error enabling VBUS supply (ret=%i)\n", ret);
			goto err_phy;
		}
	}
#endif

	if (priv->init_type == USB_INIT_HOST) {
		setbits_le32(&ehci->usbmode, CM_HOST);
		writel(mx6_portsc(priv->phy_type), &ehci->portsc);
		setbits_le32(&ehci->portsc, USB_EN);
	}

	mdelay(10);

	hccr = (struct ehci_hccr *)((uintptr_t)&ehci->caplength);
	hcor = (struct ehci_hcor *)((uintptr_t)hccr +
			HC_LENGTH(ehci_readl(&(hccr)->cr_capbase)));

	ret = ehci_register(dev, hccr, hcor, &mx6_ehci_ops, 0, priv->init_type);
	if (ret)
		goto err_regulator;

	return ret;


err_regulator:
#if CONFIG_IS_ENABLED(DM_REGULATOR)
	if (priv->vbus_supply)
		regulator_set_enable(priv->vbus_supply, false);
err_phy:
#endif
#if defined(USE_USB_PHY)
	ehci_shutdown_phy(dev, &priv->phy);
#endif
err_clk:
#if CONFIG_IS_ENABLED(CLK)
	clk_disable(&priv->clk);
#endif
#if !defined(USE_USB_PHY)
	ehci_mx6_phy_remove(priv);
#endif

	return ret;
}

int ehci_usb_remove(struct udevice *dev)
{
	struct ehci_mx6_priv_data *priv = dev_get_priv(dev);
	struct usb_plat *plat = dev_get_plat(dev);
	int ret;

	ehci_deregister(dev);
	if (priv->gd_reset) {
		dm_gpio_free(dev, priv->gd_reset);
		priv->gd_reset = NULL;
	}
	ret = board_usb_cleanup(dev_seq(dev), priv->init_type);

#if defined(USE_USB_PHY)
	ehci_shutdown_phy(dev, &priv->phy);
#endif

#if CONFIG_IS_ENABLED(DM_REGULATOR)
	if (priv->vbus_supply)
		regulator_set_enable(priv->vbus_supply, false);
#endif

#if CONFIG_IS_ENABLED(CLK)
	clk_disable(&priv->clk);
#endif

#if !defined(USE_USB_PHY)
	ehci_mx6_phy_remove(priv);
#endif

	plat->init_type = 0; /* Clean the requested usb type to host mode */
	return ret;
}

static const struct udevice_id mx6_usb_ids[] = {
	{ .compatible = "fsl,imx27-usb" },
	{ .compatible = "fsl,imx7d-usb" },
	{ .compatible = "fsl,imxrt-usb" },
	{ }
};

U_BOOT_DRIVER(usb_mx6) = {
	.name	= "ehci_mx6",
	.id	= UCLASS_USB,
	.of_match = mx6_usb_ids,
	.of_to_plat = ehci_usb_of_to_plat,
	.probe	= ehci_usb_probe,
	.remove = ehci_usb_remove,
	.ops	= &ehci_usb_ops,
	.plat_auto	= sizeof(struct usb_plat),
	.priv_auto	= sizeof(struct ehci_mx6_priv_data),
	.flags	= DM_FLAG_ALLOC_PRIV_DMA,
};
#endif
