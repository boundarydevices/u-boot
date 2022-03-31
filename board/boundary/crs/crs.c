// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 Boundary Devices
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
	IOMUX_PAD_CTRL(GPIO1_IO02__WDOG1_WDOG_B, WDOG_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_RXD__UART2_DCE_RX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_TXD__UART2_DCE_TX, UART_PAD_CTRL),
#define GP_USB_UART_SUSPEND	IMX_GPIO_NR(4, 14)
	IOMUX_PAD_CTRL(SAI1_TXD2__GPIO4_IO14, 0x140),
#define GP_USB_UART_RESET	IMX_GPIO_NR(4, 6)
	IOMUX_PAD_CTRL(SAI1_RXD4__GPIO4_IO06, 0x140),

	/* eqos */
#define GP_EQOS_MII_MDC		IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(ENET_MDC__GPIO1_IO16, 0x3),
#define GP_EQOS_MII_MDIO	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(ENET_MDIO__GPIO1_IO17, 0x3),

#define GP_LED1_BLUE		IMX_GPIO_NR(4, 19)
	IOMUX_PAD_CTRL(SAI1_TXD7__GPIO4_IO19, 0x140),
#define GP_LED1_GREEN		IMX_GPIO_NR(4, 0)
	IOMUX_PAD_CTRL(SAI1_RXFS__GPIO4_IO00, 0x140),
#define GP_LED1_RED		IMX_GPIO_NR(4, 1)
	IOMUX_PAD_CTRL(SAI1_RXC__GPIO4_IO01, 0x140),

#define GP_LED2_BLUE		IMX_GPIO_NR(4, 16)
	IOMUX_PAD_CTRL(SAI1_TXD4__GPIO4_IO16, 0x140),
#define GP_LED2_GREEN		IMX_GPIO_NR(4, 17)
	IOMUX_PAD_CTRL(SAI1_TXD5__GPIO4_IO17, 0x140),
#define GP_LED2_RED		IMX_GPIO_NR(4, 18)
	IOMUX_PAD_CTRL(SAI1_TXD6__GPIO4_IO18, 0x140),

#define GP_LED3_BLUE		IMX_GPIO_NR(4, 3)
	IOMUX_PAD_CTRL(SAI1_RXD1__GPIO4_IO03, 0x140),
#define GP_LED3_GREEN		IMX_GPIO_NR(4, 4)
	IOMUX_PAD_CTRL(SAI1_RXD2__GPIO4_IO04, 0x140),
#define GP_LED3_RED		IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(SAI1_RXD3__GPIO4_IO05, 0x140),

#define GP_LED4_BLUE		IMX_GPIO_NR(4, 7)
	IOMUX_PAD_CTRL(SAI1_RXD5__GPIO4_IO07, 0x140),
#define GP_LED4_GREEN		IMX_GPIO_NR(4, 8)
	IOMUX_PAD_CTRL(SAI1_RXD6__GPIO4_IO08, 0x140),
#define GP_LED4_RED		IMX_GPIO_NR(4, 9)
	IOMUX_PAD_CTRL(SAI1_RXD7__GPIO4_IO09, 0x140),

#define GP_USB3_1_HUB_RESET	IMX_GPIO_NR(5, 6)
	IOMUX_PAD_CTRL(ECSPI1_SCLK__GPIO5_IO06, 0x100),
	IOMUX_PAD_CTRL(GPIO1_IO12__HSIOMIX_usb1_OTG_PWR, 0x100),
	IOMUX_PAD_CTRL(GPIO1_IO13__HSIOMIX_usb1_OTG_OC,	0x140),
};

static const struct gpio_reserve gpios_to_reserve[] = {
	{ GP_LED1_BLUE, GPIOD_OUT_HIGH, 0, "led1-blue", },
	{ GP_LED1_GREEN, GPIOD_OUT_HIGH, 0, "led1-green", },
	{ GP_LED1_RED, GPIOD_OUT_HIGH, 0, "led1-red", },
	{ GP_LED2_BLUE, GPIOD_OUT_HIGH, 0, "led2-blue", },
	{ GP_LED2_GREEN, GPIOD_OUT_HIGH, 0, "led2-green", },
	{ GP_LED2_RED, GPIOD_OUT_HIGH, 0, "led2-red", },
	{ GP_LED3_BLUE, GPIOD_OUT_HIGH, 0, "led3-blue", },
	{ GP_LED3_GREEN, GPIOD_OUT_HIGH, 0, "led3-green", },
	{ GP_LED3_RED, GPIOD_OUT_HIGH, 0, "led3-red", },
	{ GP_LED4_BLUE, GPIOD_OUT_HIGH, 0, "led4-blue", },
	{ GP_LED4_GREEN, GPIOD_OUT_HIGH, 0, "led4-green", },
	{ GP_LED4_RED, GPIOD_OUT_HIGH, 0, "led4-red", },
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	gpios_reserve(gpios_to_reserve, ARRAY_SIZE(gpios_to_reserve));
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	set_wdog_reset(wdog);
	init_uart_clk(1);

	return 0;
}

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
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
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
	gpios_reserve(gpios_to_reserve, ARRAY_SIZE(gpios_to_reserve));
	gpio_request(GP_EQOS_MII_MDC, "eqos_mdc");
	gpio_request(GP_EQOS_MII_MDIO, "eqos_mdio");
	gpio_request(GP_USB3_1_HUB_RESET, "usb3_1_hub_reset");
	gpio_direction_output(GP_USB3_1_HUB_RESET, 0);

#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	board_usb_reset(0, USB_INIT_DEVICE);

	return 0;
}
