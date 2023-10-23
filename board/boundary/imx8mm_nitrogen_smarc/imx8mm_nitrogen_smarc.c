/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
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
#include <env.h>
#include <dm.h>
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

#define GPIRQ_I2C2_SN65DSI83		IMX_GPIO_NR(3, 24)
#define GP_CS005_0004_03_DISPLAY_EN	IMX_GPIO_NR(3, 24)
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(3, 24)
#define GP_LS050T1SX12_EN		IMX_GPIO_NR(3, 24)
#define GP_LT8912_DISPLAY_EN		IMX_GPIO_NR(3, 24)
	IOMUX_PAD_CTRL(SAI5_RXD3__GPIO3_IO24, 0x100),		/* MIPI_IRQ - SM_CARRIER_STANDBY */

#define GP_LVDS_BACKLIGHT_EN		IMX_GPIO_NR(1, 1)
#define GPIRQ_TS_GT911 			IMX_GPIO_NR(1, 1)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x180),		/* MIPI_TS_IRQ  - SM_LCD0_BKLT_EN */

#ifdef CONFIG_IMX8MN
#define MCP23018 8
#define GP_TS_ATMEL_RESET		IMX_GPIO_NR(MCP23018, 4)
#define GP_TS_GT911_RESET		IMX_GPIO_NR(MCP23018, 4)
#define GP_ST1633_RESET			IMX_GPIO_NR(MCP23018, 4)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(MCP23018, 4)
#define GP_TS_ILI251X_RESET		IMX_GPIO_NR(MCP23018, 4)
#else
#define GP_TS_ATMEL_RESET		IMX_GPIO_NR(4, 5)
#define GP_TS_GT911_RESET		IMX_GPIO_NR(4, 5)
#define GP_ST1633_RESET			IMX_GPIO_NR(4, 5)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(4, 5)
#define GP_TS_ILI251X_RESET		IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(SAI1_RXD3__GPIO4_IO5, 0x100),		/* MIPI_TS_RESET - SM_GPIO7 */
#endif

#define GP_TC358762_EN		IMX_GPIO_NR(1, 3)
#define GP_SC18IS602B_RESET	IMX_GPIO_NR(1, 3)
#define GP_DMT055FHNMCMI_EN	IMX_GPIO_NR(1, 3)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(1, 3)
#define GP_MIPI_ENABLE		IMX_GPIO_NR(1, 3)
#define	GP_LT8912_RESET		IMX_GPIO_NR(1, 3)
/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE		IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO1_IO03__GPIO1_IO3, 0x06),		/* MIPI_ENABLE - SM_LCD0_VDD_EN */

#define GP_TS_LVDS_GT911_RESET	IMX_GPIO_NR(1, 10)
#define GP_TS_LVDS_FT5X06_RESET	IMX_GPIO_NR(1, 10)
	IOMUX_PAD_CTRL(GPIO1_IO10__GPIO1_IO10, 0x06),		/* SM_ESPI_ALERT0 */
#define GPIRQ_TS_LVDS_GT911	IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(SAI5_RXD1__GPIO3_IO22, 0x06),		/* SM_SMB_ALERT */

#define GP_SN65DSI83_LVDS_EN	IMX_GPIO_NR(3, 25)
	IOMUX_PAD_CTRL(SAI5_MCLK__GPIO3_IO25, 0x06),		/* on som  */

#define GPIRQ_RV3028		IMX_GPIO_NR(3, 23)
	IOMUX_PAD_CTRL(SAI5_RXD2__GPIO3_IO23, 0x1c0),		/* SM_GPIO12 */

	/* pcie */
#define GP_PCIE0_RESET		IMX_GPIO_NR(3, 2)
	IOMUX_PAD_CTRL(NAND_CE1_B__GPIO3_IO2, 0x100),		/* SM_PCIE_A_RST */

#ifdef CONFIG_FEC_MXC
	/* PHY - Micrel KSZ9031 */
	IOMUX_PAD_CTRL(ENET_MDIO__ENET1_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(ENET_MDC__ENET1_MDC, PAD_CTRL_ENET_MDC),
	IOMUX_PAD_CTRL(ENET_TX_CTL__ENET1_RGMII_TX_CTL, 0x10),
	IOMUX_PAD_CTRL(ENET_TD0__ENET1_RGMII_TD0, 0x10),
	IOMUX_PAD_CTRL(ENET_TD1__ENET1_RGMII_TD1, 0x10),
	IOMUX_PAD_CTRL(ENET_TD2__ENET1_RGMII_TD2, 0x10),
	IOMUX_PAD_CTRL(ENET_TD3__ENET1_RGMII_TD3, 0x10),
	IOMUX_PAD_CTRL(ENET_TXC__ENET1_RGMII_TXC, 0x12),
#endif

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IOMUX_PAD_CTRL(SD1_RESET_B__GPIO2_IO10, 0x41),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	gpio_request(GP_SN65DSI83_EN, "sn65en");
	gpio_direction_output(GP_SN65DSI83_EN, 0);
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	gpio_direction_output(GP_EMMC_RESET, 1);
	set_wdog_reset(wdog);
	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
int board_detect_lvds_gt911(struct display_info_t const *di)
{
	return board_detect_gt911_common(di, 0, 0, GP_TS_LVDS_GT911_RESET, GPIRQ_TS_LVDS_GT911);
}

int board_detect_gt911_x71(struct display_info_t const *di)
{
	int ret;

	gpio_request(GPIRQ_TS_GT911, "gt911_irq");
	ret = board_detect_gt911_common(di, 0x7102, (1 << (di->bus_num >> 4)) | 0x7300, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
	gpio_free(GPIRQ_TS_GT911);
	return ret;
}

int board_detect_gt911_sn65_x71(struct display_info_t const *di)
{
	int ret;

	gpio_request(GPIRQ_TS_GT911, "gt911_irq");
	ret = board_detect_gt911_sn65_common(di, 0x7102, (1 << (di->bus_num >> 4)) | 0x7300, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
	gpio_free(GPIRQ_TS_GT911);
	return ret;
}

int board_detect_pca9546_sn65_x71(struct display_info_t const *di)
{
	return detect_common(di, 0x7102, (1 << (di->bus_num >> 4)) | 0x7300, -1, 0x0, -1, 0x0, 0, 0, 0x2c);
}

int board_detect_lcd133_x71(struct display_info_t const *di)
{
	/* 0 - 0xf : VPOS 5.5V output */
	/* 1 - 0xf : VNEG -5.5V output */
	return detect_common(di, 0x7102, (1 << (di->bus_num >> 4)) | 0x7300, 0, 0xf, 1, 0xf, 0, 0, 0);
}

static const struct display_info_t displays[] = {
	/* mipi */
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-1",	U, MIPI, board_detect_gt911_sn65_x71, fbp_bus_gp((0 | (2 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-2",	B, MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-3",	E, MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-1",	B, MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-2",	U, MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-3",	E, MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),

	VD_MIPI_TM070JDHG30_x("tm070jdhg30-4",	B, MIPI, board_detect_pca9546_sn65_x71, fbp_bus_gp((2 | (2 << 4)), GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_M101NWWB_x("m101nwwb-4",	B, MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),

	VD_LTK080A60A004T(MIPI, board_detect_gt911_x71, fbp_bus_gp((0 | (2 << 4)), GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
	VD_LTK0680YTMDB_2(MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), GP_MIPI_ENABLE, GP_MIPI_ENABLE, 0), 0x5d, FBTS_GOODIX),
	VD_MIPI_COM50H5N03ULC(MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), GP_MIPI_ENABLE, GP_MIPI_ENABLE, 0), 0x00),
	/* 0x3e is the TPS65132 power chip on our adapter board */
	VD_MIPI_LCD133_070(MIPI, board_detect_lcd133_x71, fbp_bus_gp((0 | (2 << 4)), GP_LCD133_070_ENABLE, GP_LCD133_070_ENABLE, 1), fbp_addr_gp(0x3e, 0, 0, 0), FBTS_FT7250),
	VD_MIPI_640_480M_60(MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_800M_60(MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_720M_60(MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),

	/* Looking for the max7323 gpio chip on the Lontium daughter board */
	VD_MIPI_1920_1080M_60(MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1024_768M_60(MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_800_600MR_60(MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_720_480M_60(MIPI, NULL, fbp_bus_gp((0 | (2 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),

	VD_MIPI_VTFT101RPFT20(MIPI, NULL, (0 | (2 << 4)), 0x70, FBP_PCA9540),

	/* on som mipi-to-lvds coverter */
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-5", UT, MIPI, board_detect_lvds_gt911, fbp_bus_gp(3, 0, GP_SN65DSI83_LVDS_EN, 0), 0x14, FBP_BACKLIGHT_MIPI_ALT, FBTS_LVDS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-6", BT, MIPI, NULL, fbp_bus_gp(3, 0, GP_SN65DSI83_LVDS_EN, 0), 0x14, FBP_BACKLIGHT_MIPI_ALT, FBTS_LVDS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-7", ET, MIPI, NULL, fbp_bus_gp(3, 0, GP_SN65DSI83_LVDS_EN, 0), 0x14, FBP_BACKLIGHT_MIPI_ALT, FBTS_LVDS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-8", UT, MIPI, board_detect_lvds_gt911, fbp_bus_gp(3, 0, GP_SN65DSI83_LVDS_EN, 0), 0x5d, FBP_BACKLIGHT_MIPI_ALT, FBTS_LVDS_GOODIX2),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-9", BT, MIPI, fbp_detect_i2c, fbp_bus_gp(3, GP_TS_LVDS_FT5X06_RESET, GP_SN65DSI83_LVDS_EN, 0), 0x38, FBP_BACKLIGHT_MIPI_ALT, FBTS_LVDS_FT5X06),
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
	gpio_request(GPIRQ_RV3028, "rv3028_irq");
	gpio_request(GP_TS_GT911_RESET, "gt911_reset");
	gpio_request(GP_TS_LVDS_GT911_RESET, "lvds_gt911_reset");
	gpio_request(GPIRQ_TS_LVDS_GT911, "lvds_gt911_irq");
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_LVDS_SN65DSI83_EN, "lvds_sn65dsi83_enable");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
#endif
	gpio_direction_output(GP_TS_GT911_RESET, 0);
	gpio_direction_output(GP_TS_LVDS_GT911_RESET, 0);
#if defined(CONFIG_MXC_SPI) && !defined(CONFIG_DM_SPI)
	setup_spi();
#endif

#ifdef CONFIG_FSL_FSPI
	board_qspi_init();
#endif

#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	return 0;
}

static iomux_v3_cfg_t const rtcirq_pulldn_pads[] = {
	IOMUX_PAD_CTRL(SAI5_RXD2__GPIO3_IO23, 0x180),		/* SM_GPIO12 */
};

static iomux_v3_cfg_t const rtcirq_pullup_pads[] = {
	IOMUX_PAD_CTRL(SAI5_RXD2__GPIO3_IO23, 0x1c0),		/* SM_GPIO12 */
};

static void check_pcie_clk(void)
{
	int val;

	imx_iomux_v3_setup_multiple_pads(rtcirq_pulldn_pads, ARRAY_SIZE(rtcirq_pulldn_pads));
	val = gpio_get_value(GPIRQ_RV3028);
	imx_iomux_v3_setup_multiple_pads(rtcirq_pullup_pads, ARRAY_SIZE(rtcirq_pullup_pads));
	if (val) {
		/*
		 * pullup(R70) on rtc irq means an external PCIe clock
		 * is provided to the processor
		 */
		env_set("cmd_board", "fdt set pcie-phy fsl,refclk-pad-mode <1>");
	} else {
		val = gpio_get_value(GPIRQ_RV3028);
		if (!val) {
			printf("Warning, rtc irq seems to be pending, pcie clk may be set incorrectly\n");
		} else {
			/*
			 * No pullup(NS R70) on rtc irq means the processor
			 * provides the PCIe clock.
			 */
			env_set("cmd_board",
				"fdt set pcie-phy fsl,refclk-pad-mode <2>");
		}

	}
}

void board_env_init(void)
{
	check_pcie_clk();
}
