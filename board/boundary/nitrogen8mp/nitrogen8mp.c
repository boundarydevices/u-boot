// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 Ezurio
 */

#include <efi_loader.h>
#include <env.h>
#include <errno.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/delay.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mp_pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <spl.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <mmc.h>
#include "../common/nitrogen_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static const iomux_v3_cfg_t uart_pads[] = {
	MX8MP_PAD_UART2_RXD__UART2_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX8MP_PAD_UART2_TXD__UART2_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static const iomux_v3_cfg_t wdog_pads[] = {
	MX8MP_PAD_GPIO1_IO02__WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	return 0;
}
#endif

#ifdef CONFIG_USB_DWC3

#define USB_PHY_CTRL0			0xF0040
#define USB_PHY_CTRL0_REF_SSP_EN	BIT(2)

#define USB_PHY_CTRL1			0xF0044
#define USB_PHY_CTRL1_RESET		BIT(0)
#define USB_PHY_CTRL1_COMMONONN		BIT(1)
#define USB_PHY_CTRL1_ATERESET		BIT(3)
#define USB_PHY_CTRL1_VDATSRCENB0	BIT(19)
#define USB_PHY_CTRL1_VDATDETENB0	BIT(20)

#define USB_PHY_CTRL2			0xF0048
#define USB_PHY_CTRL2_TXENABLEN0	BIT(8)

#define USB_PHY_CTRL6			0xF0058

#define HSIO_GPR_BASE					(0x32F10000U)
#define HSIO_GPR_REG_0					(HSIO_GPR_BASE)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT	(1)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN \
	(0x1U << HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT)

static struct dwc3_device dwc3_device_data = {
#ifdef CONFIG_SPL_BUILD
	.maximum_speed = USB_SPEED_HIGH,
#else
	.maximum_speed = USB_SPEED_SUPER,
#endif
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
	.power_down_scale = 2,
};

int dm_usb_gadget_handle_interrupts(struct udevice *dev)
{
	dwc3_uboot_handle_interrupt(dev);
	return 0;
}

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 regdata;

	/* enable usb clock via hsio gpr */
	regdata = readl(HSIO_GPR_REG_0);
	regdata |= HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN;
	writel(regdata, HSIO_GPR_REG_0);

	/* USB3.0 PHY signal fsel for 100M ref */
	regdata = readl(dwc3->base + USB_PHY_CTRL0);
	regdata = (regdata & 0xfffff81f) | (0x2a << 5);
	writel(regdata, dwc3->base + USB_PHY_CTRL0);

	regdata = readl(dwc3->base + USB_PHY_CTRL6);
	regdata &= ~0x1;
	writel(regdata, dwc3->base + USB_PHY_CTRL6);

	regdata = readl(dwc3->base + USB_PHY_CTRL1);
	regdata &= ~(USB_PHY_CTRL1_VDATSRCENB0 | USB_PHY_CTRL1_VDATDETENB0 |
			USB_PHY_CTRL1_COMMONONN);
	regdata |= USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET;
	writel(regdata, dwc3->base + USB_PHY_CTRL1);

	regdata = readl(dwc3->base + USB_PHY_CTRL0);
	regdata |= USB_PHY_CTRL0_REF_SSP_EN;
	writel(regdata, dwc3->base + USB_PHY_CTRL0);

	regdata = readl(dwc3->base + USB_PHY_CTRL2);
	regdata |= USB_PHY_CTRL2_TXENABLEN0;
	writel(regdata, dwc3->base + USB_PHY_CTRL2);

	regdata = readl(dwc3->base + USB_PHY_CTRL1);
	regdata &= ~(USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET);
	writel(regdata, dwc3->base + USB_PHY_CTRL1);
}
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
#define USB2_PWR_EN IMX_GPIO_NR(1, 14)
int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	if (index == 0 && init == USB_INIT_DEVICE) {
		imx8m_usb_power(index, true);
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	} else if (index == 0 && init == USB_INIT_HOST) {
		return ret;
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;

	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_uboot_exit(index);
		imx8m_usb_power(index, false);
	} else if (index == 0 && init == USB_INIT_HOST) {
	}

	return ret;
}
#endif

int board_init(void)
{
#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
	init_usb_clk();
#endif

	return 0;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

int board_late_init(void)
{
	unsigned char mac_address[6];

	imx_get_mac_from_fuse(0, mac_address);
	if (is_valid_ethaddr(mac_address)) {
		if (!env_get("serial#")) {
			bd_setserialnumber(mac_address);
		};
	}

	return 0;
}

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void)
{
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif

#ifdef CONFIG_SPL_MMC
#define UBOOT_RAW_SECTOR_OFFSET 0x40
unsigned long spl_mmc_get_uboot_raw_sector(struct mmc *mmc, unsigned long raw_sect)
{
	u32 boot_dev = spl_boot_device();

	switch (boot_dev) {
	case BOOT_DEVICE_MMC2:
		return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR - UBOOT_RAW_SECTOR_OFFSET;
	default:
		return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;
	}
}
#endif

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /* TODO */
}
#endif /* CONFIG_ANDROID_RECOVERY */
#endif /* CONFIG_FSL_FASTBOOT */
