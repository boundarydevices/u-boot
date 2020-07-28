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
	IOMUX_PAD_CTRL(SAI1_RXD3__ENET1_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(SAI1_RXD2__ENET1_MDC, PAD_CTRL_ENET_MDC),

#define GPIRQ_SN65DSI83		IMX_GPIO_NR(4, 0)
#define GP_LCM_JM430_BKL_EN	IMX_GPIO_NR(4, 0)
#define GP_LTK08_MIPI_EN	IMX_GPIO_NR(4, 0)
	IOMUX_PAD_CTRL(SAI1_RXFS__GPIO4_IO00, 0x04),

#define GPIRQ_TS_GT911		IMX_GPIO_NR(4, 25)
#define GP_TS_GT911_IRQ		IMX_GPIO_NR(4, 25)
#define GPIRQ_TS_FT5X06		IMX_GPIO_NR(4, 25)
#define GP_TS_FT5X06_WAKE	IMX_GPIO_NR(4, 25)
	IOMUX_PAD_CTRL(SAI2_TXC__GPIO4_IO25, 0x106),

#define GP_TS_GT911_RESET	IMX_GPIO_NR(4, 24)
#define GP_ST1633_RESET		IMX_GPIO_NR(4, 24)
#define GP_TS_FT5X06_RESET	IMX_GPIO_NR(4, 24)
	IOMUX_PAD_CTRL(SAI2_TXFS__GPIO4_IO24, 0x109),

#define GP_TC358762_EN		IMX_GPIO_NR(4, 27)
#define GP_SC18IS602B_RESET	IMX_GPIO_NR(4, 27)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(4, 27)
#define GP_MIPI_RESET		IMX_GPIO_NR(4, 27)
	/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE	IMX_GPIO_NR(4, 27)
	IOMUX_PAD_CTRL(SAI2_MCLK__GPIO4_IO27, 0x106),

#define GP_LVDS_BKL_EN		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO09, 0x106),
#define GP_LVDS2_BKL_EN		IMX_GPIO_NR(3, 21)
	IOMUX_PAD_CTRL(SAI5_RXD0__GPIO3_IO21, 0x106),
	/* eqos */
#define GP_EQOS_MII_MDC		IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(ENET_MDC__GPIO1_IO16, 0x3),
#define GP_EQOS_MII_MDIO	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(ENET_MDIO__GPIO1_IO17, 0x3),
	/* fec */
#define GP_FEC_MII_MDC		IMX_GPIO_NR(4, 4)
	IOMUX_PAD_CTRL(SAI1_RXD2__GPIO4_IO04, 0x3),
#define GP_FEC_MII_MDIO		IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(SAI1_RXD3__GPIO4_IO05, 0x3),

#define GP_USB1_HUB_RESET	IMX_GPIO_NR(1, 6)
	IOMUX_PAD_CTRL(GPIO1_IO06__GPIO1_IO06, WEAK_PULLUP),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	gpio_request(GP_SN65DSI83_EN, "sn65en");
	gpio_direction_output(GP_SN65DSI83_EN, 0);

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	set_wdog_reset(wdog);
	init_uart_clk(1);

	return 0;
}

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
int board_usb_hub_gpio_init(void)
{
	return GP_USB1_HUB_RESET;
}
#endif

#ifdef CONFIG_CMD_FBPANEL
int board_detect_gt911(struct display_info_t const *di)
{
	int ret;
	struct udevice *dev, *chip;

	if (di->bus_gp)
		gpio_direction_output(di->bus_gp, 1);
	gpio_set_value(GP_TS_GT911_RESET, 0);
	mdelay(20);
	gpio_direction_output(GPIRQ_TS_GT911, di->addr_num == 0x14 ? 1 : 0);
	udelay(100);
	gpio_set_value(GP_TS_GT911_RESET, 1);
	mdelay(6);
	gpio_set_value(GPIRQ_TS_GT911, 0);
	mdelay(50);
	gpio_direction_input(GPIRQ_TS_GT911);
	ret = uclass_get_device(UCLASS_I2C, di->bus_num, &dev);
	if (ret)
		return 0;

	ret = dm_i2c_probe(dev, di->addr_num, 0x0, &chip);
	if (ret && di->bus_gp)
		gpio_direction_input(di->bus_gp);
	return (ret == 0);
}

static const struct display_info_t displays[] = {
	/* hdmi */
	VD_1920_1080M_60(HDMI, NULL, 0, 0x50),
	VD_1280_720M_60(HDMI, NULL, 0, 0x50),

	/* mipi */
	VD_MIPI_M101NWWB(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), GP_SN65DSI83_EN, 0, 0), 0x2c, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_MTD0900DCP27KF(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x41, FBP_MIPI_TO_LVDS, FBTS_ILI251X),
	VD_DMT050WVNXCMI(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), GP_SC18IS602B_RESET, 0, 30), fbp_addr_gp(0x2f, 0, 6, 0), FBP_SPI_LCD, FBTS_GOODIX),
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp((1 | (3 << 4)), GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
	VD_LCM_JM430_MINI(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), GP_ST1633_RESET, GP_TC358762_EN, 30), fbp_addr_gp(0x55, GP_LCM_JM430_BKL_EN, 0, 0), FBTS_ST1633I),	/* Sitronix touch */
	VD_LTK0680YTMDB(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d, FBTS_GOODIX),
	VD_MIPI_COM50H5N03ULC(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x00),
	/* 0x3e is the TPS65132 power chip on our adapter board */
	VD_MIPI_LCD133_070(MIPI, board_detect_lcd133, fbp_bus_gp((1 | (3 << 4)), GP_LCD133_070_ENABLE, GP_LCD133_070_ENABLE, 1), fbp_addr_gp(0x3e, 0, 0, 0), FBTS_FT7250),
	VD_MIPI_640_480M_60(MIPI, board_detect_pca9546_2, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_800M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_720M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1920_1080M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1024_768M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_800_600MR_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_720_480M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_VTFT101RPFT20(MIPI, NULL, (1 | (3 << 4)), 0x70, FBP_PCA9540),

	/* lvds */
	VD_HANNSTAR7(LVDS, fbp_detect_i2c, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_PM9598(LVDS2, NULL, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_HANNSTAR7(LVDS2, NULL, fbp_bus_gp(3, 0, GP_LVDS2_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_PM9598(LVDS, NULL, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_AUO_B101EW05(LVDS, NULL, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_AUO_B101EW05(LVDS2, NULL, fbp_bus_gp(3, 0, GP_LVDS2_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_LG1280_800(LVDS, NULL, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_LG1280_800(LVDS2, NULL, fbp_bus_gp(3, 0, GP_LVDS2_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_M101NWWB(LVDS, NULL, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_M101NWWB(LVDS2, NULL, fbp_bus_gp(3, 0, GP_LVDS2_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_DT070BTFT(LVDS, NULL, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_DT070BTFT(LVDS2, NULL, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_WSVGA(LVDS, NULL, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_TM070JDHG30(LVDS, NULL, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_ND1024_600(LVDS, fbp_detect_i2c, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),

	/* ili210x */
	VD_AMP1024_600(LVDS, fbp_detect_i2c, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x41, FBTS_ILI210X),

	/* egalax_ts */
	VD_HANNSTAR(LVDS, fbp_detect_i2c, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x04, FBTS_EGALAX),
	VD_HANNSTAR(LVDS2, NULL, fbp_bus_gp(3, 0, GP_LVDS2_BKL_EN, 0), 0x04, FBTS_EGALAX),
	VD_LG9_7(LVDS, NULL, fbp_bus_gp(3, 0, GP_LVDS_BKL_EN, 0), 0x04, FBTS_EGALAX),

	/* fusion7 specific touchscreen */
	VD_FUSION7(LCD, fbp_detect_i2c, 3, 0x10, FBTS_FUSION7),

	VD_SHARP_LQ101K1LY04(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_WXGA(LVDS, NULL, 0, 0x00),
	VD_LD070WSVGA(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_WVGA(LVDS, NULL, 0, 0x00),
	VD_AA065VE11(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_VGA(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),

	/* uses both lvds connectors */
	VD_1080P60(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_1080P60_J(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
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
	gpio_request(GP_SN65DSI83_EN, "sn65en");
	gpio_request(GP_LTK08_MIPI_EN, "ltk08_mipi_en");
	gpio_request(GP_EQOS_MII_MDC, "eqos_mdc");
	gpio_request(GP_EQOS_MII_MDIO, "eqos_mdio");
	gpio_request(GP_FEC_MII_MDC, "fec_mdc");
	gpio_request(GP_FEC_MII_MDIO, "fec_mdio");
	gpio_request(GP_USB1_HUB_RESET, "usb1_hub_reset");
	gpio_direction_output(GP_TS_GT911_RESET, 0);
	gpio_direction_output(GP_USB1_HUB_RESET, 0);

#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif
