/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/video.h>
#include <linux/delay.h>
#include <video_fb.h>
#include <spl.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <dm.h>
#include "../common/padctrl.h"
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

static iomux_v3_cfg_t const init_pads[] = {
	IMX8MQ_PAD_GPIO1_IO02__WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
	IMX8MQ_PAD_UART1_RXD__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_UART1_TXD__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),

#define GPIRQ_GT911 			IMX_GPIO_NR(3, 12)
	IMX8MQ_PAD_NAND_DATA06__GPIO3_IO12 | MUX_PAD_CTRL(0xd6),
#define GP_EXC3000_RESET		IMX_GPIO_NR(3, 13)
#define GP_GT911_RESET			IMX_GPIO_NR(3, 13)
#define GP_ST1633_RESET			IMX_GPIO_NR(3, 13)
	IMX8MQ_PAD_NAND_DATA07__GPIO3_IO13 | MUX_PAD_CTRL(0x49),

#define GP_GPIOKEY_POWER			IMX_GPIO_NR(1, 7)
	IMX8MQ_PAD_GPIO1_IO07__GPIO1_IO7 | MUX_PAD_CTRL(WEAK_PULLUP),

	/* Old */
#define GPIRQ_I2C2_SN65DSI83_REV0	IMX_GPIO_NR(4, 28)
	IMX8MQ_PAD_SAI3_RXFS__GPIO4_IO28 | MUX_PAD_CTRL(0x44),
#define GP_I2C2_SN65DSI83_EN_REV0	IMX_GPIO_NR(3, 14)
	IMX8MQ_PAD_NAND_DQS__GPIO3_IO14 | MUX_PAD_CTRL(0x6),

	/* New */
#define GPIRQ_I2C2_SN65DSI83		IMX_GPIO_NR(3, 24)
	IMX8MQ_PAD_SAI5_RXD3__GPIO3_IO24 | MUX_PAD_CTRL(0x44),
#define GP_I2C2_SN65DSI83_EN		IMX_GPIO_NR(3, 22)
	IMX8MQ_PAD_SAI5_RXD1__GPIO3_IO22 | MUX_PAD_CTRL(0x6),

#define GP_EMMC_RESET			IMX_GPIO_NR(2, 10)
	IMX8MQ_PAD_SD1_RESET_B__GPIO2_IO10 | MUX_PAD_CTRL(0x41),

#define GP_CSI1_MIPI_PWDN		IMX_GPIO_NR(4, 25)
	IMX8MQ_PAD_SAI2_TXC__GPIO4_IO25 | MUX_PAD_CTRL(0x61),
#define GP_CSI1_MIPI_RESET		IMX_GPIO_NR(4, 24)
	IMX8MQ_PAD_SAI2_TXFS__GPIO4_IO24 | MUX_PAD_CTRL(0x61),

	/* New, LDO2_VSEL needs to be high */
	IMX8MQ_PAD_NAND_CE1_B__GPIO3_IO2 | MUX_PAD_CTRL(WEAK_PULLUP),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	set_wdog_reset(wdog);

	gpio_direction_output(GP_EMMC_RESET, 1);
	gpio_direction_output(GP_I2C2_SN65DSI83_EN, 0);
	gpio_direction_output(GP_I2C2_SN65DSI83_EN_REV0, 0);
	gpio_direction_output(GP_CSI1_MIPI_PWDN, 1);
	gpio_direction_output(GP_CSI1_MIPI_RESET, 0);

	return 0;
}

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
int __weak board_usb_hub_gpio_init(void)
{
#define GP_USB1_HUB_RESET	IMX_GPIO_NR(1, 14)
	imx_iomux_v3_setup_pad(IMX8MQ_PAD_GPIO1_IO14__GPIO1_IO14 |
			MUX_PAD_CTRL(WEAK_PULLUP));
	return GP_USB1_HUB_RESET;
}
#endif

#ifdef CONFIG_CMD_FBPANEL

int board_detect_hdmi(struct display_info_t const *di)
{
	return hdmi_hpd_status() ? 1 : 0;
}

int board_detect_gt911(struct display_info_t const *di)
{
	return board_detect_gt911_common(di, (di->bus_num >> 4) | 4, 0, GP_GT911_RESET, GPIRQ_GT911);
}

static const struct display_info_t displays[] = {
	/* hdmi */
	VD_1920_1080M_60(HDMI, board_detect_hdmi, 0, 0x50),
	VD_1280_720M_60(HDMI, NULL, 0, 0x50),
	VD_MIPI_AM_TFT1280X800W(MIPI, board_detect_pca9540, fbp_bus_gp(3 | (0 << 4), GP_EXC3000_RESET, 0, 100), 0x2a, FBP_MIPI_TO_LVDS, FBTS_EXC3000),
	VD_MIPI_AM_TFT1280X800(MIPI, NULL, fbp_bus_gp(3 | (0 << 4), GP_EXC3000_RESET, 0, 80), 0x2a, FBP_MIPI_TO_LVDS, FBTS_EXC3000),
#if 0
	VD_LTK0680YTMDB(MIPI, board_detect_gt911, fbp_bus_gp(3 | (0 << 4), GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d, FBTS_GOODIX),
	VD_MIPI_COM50H5N03ULC(MIPI, NULL, fbp_bus_gp(3, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x00),
#endif
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_I2C2_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_I2C2_SN65DSI83_EN_REV0, "sn65dsi83_enable_rev0");
#endif
	gpio_request(GP_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_GT911, "gt911_irq");
	gpio_direction_output(GP_GT911_RESET, 0);
#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	board_usb_reset(0, USB_INIT_DEVICE);

	return 0;
}


int board_fastboot_key_pressed(void)
{
	gpio_request(GP_GPIOKEY_POWER, "fastboot_key");
	gpio_direction_input(GP_GPIOKEY_POWER);
	return !gpio_get_value(GP_GPIOKEY_POWER);
}
