// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 NXP
 * Copyright 2020 Boundary Devices
 */

#include <common.h>
#include <command.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx8mp_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/io.h>
#include <asm/mach-imx/dma.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm-generic/gpio.h>

#include <display_detect.h>
#include <dwc3-uboot.h>
#include <errno.h>
#include <linux/delay.h>
#include <miiphy.h>
#include <mmc.h>
#include <netdev.h>
#include <power/pmic.h>
#include <spl.h>
#include <usb.h>
#include "../common/padctrl.h"
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const init_pads[] = {
#define GP_PWM1_MIPI		IMX_GPIO_NR(1, 0)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO01, 0x100),
	IOMUX_PAD_CTRL(GPIO1_IO02__WDOG1_WDOG_B, WDOG_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_RXD__UART2_DCE_RX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_TXD__UART2_DCE_TX, UART_PAD_CTRL),

#define GPIRQ_TS_GT911	IMX_GPIO_NR(3, 10)
#define GP_TS_GT911_IRQ	IMX_GPIO_NR(3, 10)
	IOMUX_PAD_CTRL(NAND_DATA04__GPIO3_IO10, 0x1c0),
#define GP_TS_GT911_RESET	IMX_GPIO_NR(3, 9)
	IOMUX_PAD_CTRL(NAND_DATA03__GPIO3_IO09, 0x119),

#define GP_LTK700WS08_DISPLAY_EN	IMX_GPIO_NR(3, 14)
	IOMUX_PAD_CTRL(NAND_DQS__GPIO3_IO14, 0x100),

#define GP_BACKLIGHT_MIPI_EN	IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO09, 0x106),
	/* eqos */
#define GP_EQOS_MII_MDC		IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(ENET_MDC__ENET_QOS_MDC, 0x3),
#define GP_EQOS_MII_MDIO	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(ENET_MDIO__ENET_QOS_MDIO, 0x3),
	/* fec */

#define GP_USB3_1_HUB_RESET	IMX_GPIO_NR(1, 6)
	IOMUX_PAD_CTRL(GPIO1_IO06__GPIO1_IO06, WEAK_PULLDN_OUTPUT),
#define GP_REG_USB_OTG2_VBUS	IMX_GPIO_NR(1, 12)
	IOMUX_PAD_CTRL(GPIO1_IO12__GPIO1_IO12, 0x100),
	IOMUX_PAD_CTRL(GPIO1_IO13__HSIOMIX_usb1_OTG_OC,	0x116),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	set_wdog_reset(wdog);
	init_uart_clk(1);

	return 0;
}

#if !CONFIG_IS_ENABLED(USB_DWC3_GENERIC) && (defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M))
int board_usb_hub_gpio_init(void)
{
	return GP_USB3_1_HUB_RESET;
}
#endif

#ifdef CONFIG_CMD_FBPANEL
#ifdef CONFIG_VIDEO_IMX8MP_HDMI
int board_detect_hdmi(struct display_info_t const *di)
{
	int ret =  display_detect_by_node_name("hdmi_hpd");

	return (ret > 0) ? 1 : 0;
}
#endif
int board_detect_gt911(struct display_info_t const *di)
{
	return board_detect_gt911_common(di, 0, 0, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
}

static const struct display_info_t displays[] = {
	/* hdmi */
#ifdef CONFIG_VIDEO_IMX8MP_HDMI
	VD_1920_1080M_60(HDMI, board_detect_hdmi, 0, 0x50),
#else
	VD_1920_1080M_60(HDMI, NULL, 0, 0x50),
#endif
	VD_1280_800M_60(HDMI, NULL, 0, 0x50),
	VD_1280_720M_60(HDMI, NULL, 0, 0x50),
	VD_1024_768M_60(HDMI, NULL, 0, 0x50),
	VD_640_480M_60(HDMI, NULL, 0, 0x50),

	/* mipi */
	VD_LTK069WXICT10(MIPI, board_detect_gt911, fbp_bus_gp(2, GP_LTK700WS08_DISPLAY_EN, GP_LTK700WS08_DISPLAY_EN, 0), 0x5d, FBTS_GOODIX),
	VD_LTK069WXBCT02(MIPI, NULL, fbp_bus_gp(2, GP_LTK700WS08_DISPLAY_EN, GP_LTK700WS08_DISPLAY_EN, 0), 0x5d, FBTS_GOODIX),
	VD_LTK0680YTMDB(MIPI, NULL, fbp_bus_gp(2, GP_LTK700WS08_DISPLAY_EN, GP_LTK700WS08_DISPLAY_EN, 0), 0x5d, FBTS_GOODIX),
	VD_LTK080A60A004T(MIPI, NULL, fbp_bus_gp(2, GP_LTK700WS08_DISPLAY_EN, GP_LTK700WS08_DISPLAY_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
	gpio_request(GP_TS_GT911_RESET, "gt11_reset");
	gpio_request(GPIRQ_TS_GT911, "gt11_irq");
	gpio_request(GP_EQOS_MII_MDC, "eqos_mdc");
	gpio_request(GP_EQOS_MII_MDIO, "eqos_mdio");
#if !CONFIG_IS_ENABLED(USB_DWC3_GENERIC)
	gpio_request(GP_USB3_1_HUB_RESET, "usb1_hub_reset");
#endif
	gpio_direction_output(GP_TS_GT911_RESET, 0);
#if !CONFIG_IS_ENABLED(USB_DWC3_GENERIC)
	gpio_direction_output(GP_USB3_1_HUB_RESET, 0);
#endif

#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
#if !CONFIG_IS_ENABLED(USB_DWC3_GENERIC) && (defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M))
	board_usb_reset(0, USB_INIT_DEVICE);
#endif
	return 0;
}
