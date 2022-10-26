/*
 * Copyright 2022 Boundary Devices LLC
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/clock.h>
#if defined(CONFIG_IMX8MM)
#include <asm/arch/imx8mm_pins.h>
#elif defined(CONFIG_IMX8MN)
#include <asm/arch/imx8mn_pins.h>
#endif
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/dma.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/video.h>
#include <dm.h>
#include <env.h>
#include <i2c.h>
#include <linux/delay.h>
#include <spl.h>
#include "../common/padctrl.h"
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

static iomux_v3_cfg_t const init_pads[] = {
	IOMUX_PAD_CTRL(GPIO1_IO02__WDOG1_WDOG_B, WDOG_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_RXD__UART2_DCE_RX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_TXD__UART2_DCE_TX, UART_PAD_CTRL),

#define GP_B_MODEM_COEX1_RXD		IMX_GPIO_NR(5, 28)
	IOMUX_PAD_CTRL(UART4_RXD__GPIO5_IO28, 0x100),	/* B_MODEM_COEX1 pull-down */
#define GP_B_MODEM_COEX2_TXD		IMX_GPIO_NR(5, 29)
	IOMUX_PAD_CTRL(UART4_TXD__GPIO5_IO29, 0x100),	/* B_MODEM_COEX2 pull-down */

#define GPIRQ_SN65DSI83			IMX_GPIO_NR(1, 1)
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 1)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x146),
#define GPIRQ_TS_GT911 			IMX_GPIO_NR(1, 6)
#define GPIRQ_TS_FT5X06			IMX_GPIO_NR(1, 6)
	IOMUX_PAD_CTRL(GPIO1_IO06__GPIO1_IO6, 0x180),
#define GP_TS_GT911_RESET		IMX_GPIO_NR(1, 7)
#define GP_ST1633_RESET			IMX_GPIO_NR(1, 7)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(1, 7)
#define GP_TS_FT7250_RESET		IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO1_IO07__GPIO1_IO7, 0x100),
#define GP_TC358762_EN		IMX_GPIO_NR(1, 9)
#define GP_SC18IS602B_RESET	IMX_GPIO_NR(1, 9)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(1, 9)
#define GP_MIPI_RESET		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO9, 0x100),
#define GP_BACKLIGHT_MIPI		IMX_GPIO_NR(5, 3)
	IOMUX_PAD_CTRL(SPDIF_TX__GPIO5_IO3, 0x116),

#define GP_PCIE0_RESET		IMX_GPIO_NR(4, 31)
	IOMUX_PAD_CTRL(SAI3_TXFS__GPIO4_IO31, 0x100),
#define GP_PCIE0_DISABLE	IMX_GPIO_NR(3, 21)
	IOMUX_PAD_CTRL(SAI5_RXD0__GPIO3_IO21, 0x100),

#ifdef CONFIG_FEC_MXC
#ifdef CONFIG_FEC_PHY_BITBANG
#define GP_MII_MDIO	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(ENET_MDIO__GPIO1_IO17, 0),
#define GP_MII_MDC	IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(ENET_MDC__GPIO1_IO16, 0x140),
#else
	IOMUX_PAD_CTRL(ENET_MDIO__ENET1_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(ENET_MDC__ENET1_MDC, 0x140),
#endif
	IOMUX_PAD_CTRL(ENET_TX_CTL__ENET1_RGMII_TX_CTL, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD0__ENET1_RGMII_TD0, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD1__ENET1_RGMII_TD1, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD2__ENET1_RGMII_TD2, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD3__ENET1_RGMII_TD3, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TXC__ENET1_RGMII_TXC, PAD_CTRL_ENET_TX),
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(SAI3_RXFS__GPIO4_IO28, 0),
#define GP_RGMII_PHY_RESET2	IMX_GPIO_NR(3, 15)
	IOMUX_PAD_CTRL(NAND_RE_B__GPIO3_IO15, 0),	/* Old rev */
#endif

#define GP_OTG2_HUB_RESET	IMX_GPIO_NR(5, 0)
	IOMUX_PAD_CTRL(SAI3_TXC__GPIO5_IO0, WEAK_PULLUP),

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IOMUX_PAD_CTRL(SD1_RESET_B__GPIO2_IO10, 0x100),
#define GP_5V_BOOST_EN	IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(SAI5_RXFS__GPIO3_IO19, 0x100),
};

int board_usb_hub_gpio_init(void)
{
	gpio_direction_output(GP_5V_BOOST_EN, 1);
	return GP_OTG2_HUB_RESET;
}

static const struct gpio_reserve gpios_to_reserve[] = {
	{ GP_BACKLIGHT_MIPI, GPIOD_OUT_LOW, GRF_FREE, "backlight_mipi", },
#ifdef CONFIG_DM_VIDEO
	{ GP_SN65DSI83_EN, GPIOD_OUT_LOW, GRF_FREE, "sn65en", },
	{ GP_LTK08_MIPI_EN, GPIOD_OUT_LOW, GRF_FREE, "lkt08_mipi_en", },
#else
	{ GP_SN65DSI83_EN, GPIOD_OUT_LOW, 0, "sn65en", },
	{ GP_LTK08_MIPI_EN, GPIOD_OUT_LOW, 0, "lkt08_mipi_en", },
#endif
	{ GP_B_MODEM_COEX1_RXD, GPIOD_IN, 0, "modem_coex1_rxd", },
	{ GP_B_MODEM_COEX2_TXD, GPIOD_OUT_LOW, 0, "modem_coex2_txd", },
	{ GP_PCIE0_RESET, GPIOD_OUT_LOW, 0, "pcie0_reset", },
	{ GP_PCIE0_DISABLE, GPIOD_OUT_LOW, 0, "pcie0_disable", },
	{ GP_5V_BOOST_EN, GPIOD_OUT_LOW, 0, "5v_boost", },
	{ GP_OTG2_HUB_RESET, GPIOD_OUT_LOW, GRF_FREE, "otg2_hub_reset", },
	{ GP_EMMC_RESET, GPIOD_OUT_HIGH, 0, "emmc_reset", },
#ifdef CONFIG_FEC_PHY_BITBANG
	{ GP_MII_MDC, GPIOD_IN, 0, "mii_mdc", },
	{ GP_MII_MDIO, GPIOD_IN, 0, "mii_mdio", },
#endif
	{ GP_TS_GT911_RESET, GPIOD_OUT_LOW, 0, "gt911_reset", },
	{ GPIRQ_TS_GT911, GPIOD_IN, 0, "gt911_irq", },
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	gpios_reserve(gpios_to_reserve, ARRAY_SIZE(gpios_to_reserve));
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	set_wdog_reset(wdog);
	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
int board_detect_gt911(struct display_info_t const *di)
{
	return board_detect_gt911_common(di, 0, 0, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
}

int board_detect_gt911_sn65(struct display_info_t const *di)
{
	return board_detect_gt911_sn65_common(di, 0, 0, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
}

static const struct display_info_t displays[] = {
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-1",	B, MIPI, board_detect_gt911_sn65, fbp_bus_gp(2, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-2",	U, MIPI, NULL, fbp_bus_gp(2, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-3",	E, MIPI, NULL, fbp_bus_gp(2, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-1",	B, MIPI, NULL, fbp_bus_gp(2, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-2",	U, MIPI, NULL, fbp_bus_gp(2, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-3",	E, MIPI, NULL, fbp_bus_gp(2, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),

	VD_MIPI_TM070JDHG30_x("tm070jdhg30-4",	B, MIPI, board_detect_sn65_and_ts, fbp_bus_gp(2, GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-5",	U, MIPI, NULL, fbp_bus_gp(2, GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-6",	E, MIPI, NULL, fbp_bus_gp(2, GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_M101NWWB_x("m101nwwb-4",	B, MIPI, NULL, fbp_bus_gp(2, GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_M101NWWB_x("m101nwwb-5",	U, MIPI, NULL, fbp_bus_gp(2, GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_M101NWWB_x("m101nwwb-6",	E, MIPI, NULL, fbp_bus_gp(2, GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),

	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp(2, GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
	gpios_reserve(gpios_to_reserve, ARRAY_SIZE(gpios_to_reserve));
#if defined(CONFIG_MXC_SPI) && !defined(CONFIG_DM_SPI)
	setup_spi();
#endif
#ifdef CONFIG_MAX77975
	max77975_init();
	max77975_set_chrgin_limit(8000);	/* 8 amps */
#endif
#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	return 0;
}

void board_env_init(void)
{
	/*
	 * If touchscreen reset is low, display will not initialize, but runs fine
	 * after init independent of gpio level
	 */
	gpio_direction_output(GP_TS_FT7250_RESET, 1);
}

void board_poweroff(void)
{
	struct snvs_regs *snvs = (struct snvs_regs *)(SNVS_BASE_ADDR);

	writel(0x60, &snvs->lpcr);
	mdelay(500);
}

static int _do_poweroff(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	board_poweroff();
	return 0;
}

U_BOOT_CMD(
	poweroff, 70, 0, _do_poweroff,
	"power down board",
	""
);
