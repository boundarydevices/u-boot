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
	IOMUX_PAD_CTRL(SAI1_RXD3__ENET1_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(SAI1_RXD2__ENET1_MDC, PAD_CTRL_ENET_MDC),

#define GPIRQ_SN65DSI83		IMX_GPIO_NR(4, 0)
#define GP_LTK08_MIPI_EN	IMX_GPIO_NR(4, 0)
	IOMUX_PAD_CTRL(SAI1_RXFS__GPIO4_IO00, 0x04),

#define GPIRQ_TS_GT911		IMX_GPIO_NR(4, 25)
#define GP_TS_GT911_IRQ		IMX_GPIO_NR(4, 25)
#define GPIRQ_TS_FT5X06		IMX_GPIO_NR(4, 25)
#define GP_TS_FT5X06_WAKE	IMX_GPIO_NR(4, 25)
	IOMUX_PAD_CTRL(SAI2_TXC__GPIO4_IO25, 0x180),

#define GP_TS_GT911_RESET	IMX_GPIO_NR(4, 24)
#define GP_ST1633_RESET		IMX_GPIO_NR(4, 24)
#define GP_TS_FT5X06_RESET	IMX_GPIO_NR(4, 24)
	IOMUX_PAD_CTRL(SAI2_TXFS__GPIO4_IO24, 0x100),

#define GP_TC358762_EN		IMX_GPIO_NR(4, 27)
#define GP_SC18IS602B_RESET	IMX_GPIO_NR(4, 27)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(4, 27)
#define GP_MIPI_RESET		IMX_GPIO_NR(4, 27)
	/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE	IMX_GPIO_NR(4, 27)
	IOMUX_PAD_CTRL(SAI2_MCLK__GPIO4_IO27, 0x106),

#define GP_BACKLIGHT_LVDS	IMX_GPIO_NR(4, 22)
	IOMUX_PAD_CTRL(SAI2_RXC__GPIO4_IO22, 0x106),	/* enable */

#define GP_BACKLIGHT_LVDS2	IMX_GPIO_NR(4, 21)
	IOMUX_PAD_CTRL(SAI2_RXFS__GPIO4_IO21, 0x106),	/* enable */

#define GP_LVDS_PWM		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO09, 0x106),	/* PWM */
#define GP_LVDS2_PWM		IMX_GPIO_NR(3, 21)
	IOMUX_PAD_CTRL(SAI5_RXD0__GPIO3_IO21, 0x106),	/* PWM */
	/* eqos */
#define GP_EQOS_MII_MDC		IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(ENET_MDC__ENET_QOS_MDC, 0x3),
#define GP_EQOS_MII_MDIO	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(ENET_MDIO__ENET_QOS_MDIO, 0x3),
	/* fec */
#define GP_FEC_MII_MDC		IMX_GPIO_NR(4, 4)
	IOMUX_PAD_CTRL(SAI1_RXD2__ENET1_MDC, 0x3),
#define GP_FEC_MII_MDIO		IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(SAI1_RXD3__ENET1_MDIO, 0x3),

#define GP_USB3_1_HUB_RESET	IMX_GPIO_NR(1, 6)
	IOMUX_PAD_CTRL(GPIO1_IO06__GPIO1_IO06, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(GPIO1_IO12__GPIO1_IO12, 0x100),
	IOMUX_PAD_CTRL(GPIO1_IO13__HSIOMIX_usb1_OTG_OC,	0x116),
	IOMUX_PAD_CTRL(SAI1_RXD0__GPIO4_IO02, 0x00),	/* float for OTG1 mode */
#define GP_FASTBOOT_KEY		IMX_GPIO_NR(5, 9)
	IOMUX_PAD_CTRL(ECSPI1_SS0__GPIO5_IO09, WEAK_PULLUP),
	IOMUX_PAD_CTRL(ECSPI1_MISO__GPIO5_IO08, WEAK_PULLDN_OUTPUT),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	gpio_request(GP_PWM1_MIPI, "pwm1");
	gpio_direction_output(GP_PWM1_MIPI, 0);
	gpio_free(GP_PWM1_MIPI);

	gpio_request(GP_SN65DSI83_EN, "sn65en");
	gpio_direction_output(GP_SN65DSI83_EN, 0);
	gpio_free(GP_SN65DSI83_EN);

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
	return board_detect_gt911_common(di, 1 << (di->bus_num >> 4), 0, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
}

int board_detect_gt911_sn65(struct display_info_t const *di)
{
	return board_detect_gt911_sn65_common(di, 1 << (di->bus_num >> 4), 0, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
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

#ifdef CONFIG_VIDEO_IMX_SEC_DSI
	/* mipi */
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-1",	B, MIPI, board_detect_gt911_sn65, fbp_bus_gp((1 | (3 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-2",	U, MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-3",	E, MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-1",	B, MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-2",	U, MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-3",	E, MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),

	VD_MIPI_TM070JDHG30_x("tm070jdhg30-4",	B, MIPI, board_detect_pca9546_sn65, fbp_bus_gp((1 | (3 << 4)), GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_M101NWWB_x("m101nwwb-4",	B, MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),

	VD_MIPI_MTD0900DCP27KF(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x41, FBP_MIPI_TO_LVDS, FBTS_ILI251X),
	VD_DMT050WVNXCMI(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), GP_SC18IS602B_RESET, 0, 30), fbp_addr_gp(0x2f, 0, 6, 0), FBP_SPI_LCD, FBTS_GOODIX),
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp((1 | (3 << 4)), GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
	VD_LCM_JM430_MINI(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), GP_ST1633_RESET, GP_TC358762_EN, 30), fbp_addr_gp(0x55, 0, 0, 0), FBTS_ST1633I),	/* Sitronix touch */
	VD_LTK0680YTMDB_2(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d, FBTS_GOODIX),
	VD_MIPI_COM50H5N03ULC(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x00),
	/* 0x3e is the TPS65132 power chip on our adapter board */
	VD_MIPI_LCD133_070(MIPI, board_detect_lcd133, fbp_bus_gp((1 | (3 << 4)), GP_LCD133_070_ENABLE, GP_LCD133_070_ENABLE, 1), fbp_addr_gp(0x3e, 0, 0, 0), FBTS_FT7250),
	VD_MIPI_640_480M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_800M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_720M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),

	/* Looking for the max7323 gpio chip on the Lontium daughter board */
	VD_MIPI_1920_1080M_60(MIPI, board_detect_pca9546_2, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1024_768M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_800_600MR_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_720_480M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_LXD_M8509A(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x66, FBP_BACKLIGHT_MIPI_ALT, FBTS_FT5X06_3),

	VD_MIPI_VTFT101RPFT20(MIPI, NULL, (1 | (3 << 4)), 0x70, FBP_PCA9540),
#else
	VD_LTK190L3027T(LVDS, NULL, 0, 0, 0),
#endif

	/* lvds */
	/* goodix */
	VD_M101NWWB(LVDS, fbp_detect_i2c, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_M101NWWB_14(LVDS, fbp_detect_i2c, fbp_bus_gp(3, 0, 0, 0), 0x14, FBTS_GOODIX2, FBTS_LVDS_GOODIX),
	VD_M101NWWB_5D(LVDS, fbp_detect_i2c, fbp_bus_gp(3, 0, 0, 0), 0x5d, FBTS_GOODIX3, FBTS_LVDS_GOODIX2),
	VD_VTFT101RPFT20(LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_HANNSTAR7(	LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_PM9598(	LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_AUO_B101EW05(LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_LG1280_800(	LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_DT070BTFT(	LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_WSVGA(	LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_TM070JDHG30(	LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_TM070JDHG30_14(LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x14, FBTS_GOODIX2, FBTS_LVDS_GOODIX),
	VD_TM070JDHG30_5D(LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x5d, FBTS_GOODIX3, FBTS_LVDS_GOODIX2),
	VD_ND1024_600(	LVDS, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),

	VD_G101EVN01(	LVDS, fbp_detect_i2c, fbp_bus_gp(3, 0, 0, 0), 0x4a, FBTS_ATMEL_MT,  FBTS_LVDS_ATMEL_MT),

	VD_M101NWWB(	LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_M101NWWB_14( LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x14, FBTS_GOODIX2, FBTS_LVDS_GOODIX),
	VD_M101NWWB_5D( LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x5d, FBTS_GOODIX3, FBTS_LVDS_GOODIX2),
	VD_VTFT101RPFT20(LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_HANNSTAR7(	LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_PM9598(	LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_AUO_B101EW05(LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_LG1280_800(	LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_DT070BTFT(	LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_WSVGA(	LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_TM070JDHG30(	LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),
	VD_TM070JDHG30_14(LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x14, FBTS_GOODIX2, FBTS_LVDS_GOODIX),
	VD_TM070JDHG30_5D(LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x5d, FBTS_GOODIX3, FBTS_LVDS_GOODIX2),
	VD_ND1024_600(	LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x38, FBTS_FT5X06_2, FBTS_LVDS_FT5X06),

	/* ili251x */
	VD_HDA800XPT(LVDS, fbp_detect_i2c, fbp_bus_gp(3, 0, 0, 0), fbp_addr_gp(0x41, GP_BACKLIGHT_LVDS, 0, 0), FBTS_ILI251X, FBTS_LVDS_ILI251X),

	VD_HDA800XPT(LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), fbp_addr_gp(0x41, GP_BACKLIGHT_LVDS2, 0, 0), FBTS_ILI251X, FBTS_LVDS_ILI251X),

	/* egalax_ts */
	VD_HANNSTAR(LVDS, fbp_detect_i2c, fbp_bus_gp(3, 0, 0, 0), 0x04, FBTS_EGALAX, FBTS_LVDS_EGALAX),

	VD_HANNSTAR(LVDS2, NULL, fbp_bus_gp(3, 0, 0, 0), 0x04, FBTS_EGALAX, FBTS_LVDS_EGALAX),
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
//	gpio_request(GP_SN65DSI83_EN, "sn65en");
//	gpio_request(GP_LTK08_MIPI_EN, "ltk08_mipi_en");
	gpio_request(GP_EQOS_MII_MDC, "eqos_mdc");
	gpio_request(GP_EQOS_MII_MDIO, "eqos_mdio");
	gpio_request(GP_FEC_MII_MDC, "fec_mdc");
	gpio_request(GP_FEC_MII_MDIO, "fec_mdio");
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

#if defined(CONFIG_CMD_FASTBOOT) || defined(CONFIG_CMD_DFU)
int board_fastboot_key_pressed(void)
{
	gpio_request(GP_FASTBOOT_KEY, "fastboot_key");
	gpio_direction_input(GP_FASTBOOT_KEY);
	return !gpio_get_value(GP_FASTBOOT_KEY);
}
#endif

static void check_usb_mux(void)
{
	struct udevice *bus;
	struct i2c_msg msg;
	struct dm_i2c_ops *ops;
	u8 buf[4];
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, 3, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return;
	}

	ret = i2c_probe_chip(bus, 0x60, 0);
	if (ret)
		return;

	buf[0] = 0x0a;
	buf[1] = 0x08;
	ops = i2c_get_ops(bus);
	msg.addr = 0x60;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;
	ret = ops->xfer(bus, &msg, 1);
	if (ret < 0)
		printf("tusb320 reset failed(%d)\n", ret);

	/*
	 * The 1st few boards used a TUSB320 instead of hd3ss3220
	 */
	env_set("cmd_board", "fdt set usb_mux reg <0x60>");
}

static int board_carrier = -1;
static int board_rv = -1;

static void check_board_env(void)
{
	if (board_rv == 20)
		env_set("board_rv", "_r20");
#ifdef CONFIG_FEC_MXC	/* bm has only 1 phy as well */
	if (board_carrier >= 0)
		env_set("board_carrier", board_carrier ? "-enc" : NULL);
#endif
}

void board_eth_type(int index, int ksz)
{
	if (index == 0) {
		board_rv = ksz ? 20 : 0;
		if (board_carrier < 0)
			board_carrier = 1;
	} else if (index == 1) {
		board_carrier = 0;	/* EVK */
	}
	check_board_env();
}

void board_env_init(void)
{
	check_usb_mux();
	check_board_env();
}
