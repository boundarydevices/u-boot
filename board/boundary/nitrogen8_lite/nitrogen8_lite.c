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

#define GPIRQ_SN65DSI83			IMX_GPIO_NR(1, 1)
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 1)
#define GP_LCD133_070_RESET		IMX_GPIO_NR(1, 1)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x146),

#define GPIRQ_GT911 			IMX_GPIO_NR(1, 6)
#define GPIRQ_TS_FT5X06			IMX_GPIO_NR(1, 6)
	IOMUX_PAD_CTRL(GPIO1_IO06__GPIO1_IO6, 0xd6),
#define GP_GT911_RESET			IMX_GPIO_NR(1, 7)
#define GP_ST1633_RESET			IMX_GPIO_NR(1, 7)
#define GP_TS_FT7250_RESET		IMX_GPIO_NR(1, 7)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO1_IO07__GPIO1_IO7, 0x49),

#define GP_TC358762_EN		IMX_GPIO_NR(1, 9)
#define GP_SC18IS602B_RESET	IMX_GPIO_NR(1, 9)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(1, 9)
#define GP_LT8912_2_RESET	IMX_GPIO_NR(1, 9)
#define GP_MIPI_RESET		IMX_GPIO_NR(1, 9)
/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO9, 0x06),

#define GP_BACKLIGHT_MIPI		IMX_GPIO_NR(5, 3)
	IOMUX_PAD_CTRL(SPDIF_TX__GPIO5_IO3, 0x116),

#define GPIRQ_CSI1_TC3587		IMX_GPIO_NR(5, 5)
#define GP_CSI1_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(5, 5)
	IOMUX_PAD_CTRL(SPDIF_EXT_CLK__GPIO5_IO5, 0x141),
#define GP_CSI1_OV5640_MIPI_RESET	IMX_GPIO_NR(1, 11)
	IOMUX_PAD_CTRL(GPIO1_IO11__GPIO1_IO11, 0x101),

#define GPIRQ_RV3028		<&gpio3 23 IRQ_TYPE_LEVEL_LOW>
	IOMUX_PAD_CTRL(SAI5_RXD2__GPIO3_IO23, 0x1c0),

#define GP_J46P2	IMX_GPIO_NR(1, 4)
	IOMUX_PAD_CTRL(GPIO1_IO04__GPIO1_IO4, 0x141),
#define GP_J46P3	IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO1_IO08__GPIO1_IO8, 0x141),

	/* pcie */
#define GP_PCIE0_RESET		IMX_GPIO_NR(4, 27)
	IOMUX_PAD_CTRL(SAI2_MCLK__GPIO4_IO27, 0x100),
#define GP_PCIE0_DISABLE	IMX_GPIO_NR(4, 24)
	IOMUX_PAD_CTRL(SAI2_TXFS__GPIO4_IO24, 0x100),

	/* sound - wm8960 */
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(1, 10)
	IOMUX_PAD_CTRL(GPIO1_IO10__GPIO1_IO10, 0x80),

#ifdef CONFIG_FEC_MXC
	/* PHY - AR8035 */
#ifdef CONFIG_FEC_PHY_BITBANG
#define GP_MII_MDIO	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(ENET_MDIO__GPIO1_IO17, 0),
#define GP_MII_MDC	IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(ENET_MDC__GPIO1_IO16, 0),
#else
	IOMUX_PAD_CTRL(ENET_MDIO__ENET1_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(ENET_MDC__ENET1_MDC, PAD_CTRL_ENET_MDC),
#endif
	IOMUX_PAD_CTRL(ENET_TX_CTL__ENET1_RGMII_TX_CTL, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD0__ENET1_RGMII_TD0, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD1__ENET1_RGMII_TD1, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD2__ENET1_RGMII_TD2, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD3__ENET1_RGMII_TD3, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TXC__ENET1_RGMII_TXC, PAD_CTRL_ENET_TX),
#endif

#ifdef CONFIG_IMX8MM
	IOMUX_PAD_CTRL(GPIO1_IO14__USB2_OTG_PWR, 0x16),
	IOMUX_PAD_CTRL(GPIO1_IO15__USB2_OTG_OC, WEAK_PULLUP),
#endif

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IOMUX_PAD_CTRL(SD1_RESET_B__GPIO2_IO10, 0x41),

#define GP_GPIOLEDS_RED		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(SAI5_RXD1__GPIO3_IO22, WEAK_PULLDN_OUTPUT),
#define GP_GPIOLEDS_GREEN	IMX_GPIO_NR(3, 21)
	IOMUX_PAD_CTRL(SAI5_RXD0__GPIO3_IO21, WEAK_PULLDN_OUTPUT),
#define GP_GPIOLEDS_YELLOW	IMX_GPIO_NR(4, 23)
	IOMUX_PAD_CTRL(SAI2_RXD0__GPIO4_IO23, WEAK_PULLDN_OUTPUT),
#define GP_GPIOLEDS_DRY		IMX_GPIO_NR(4, 29)
	IOMUX_PAD_CTRL(SAI3_RXC__GPIO4_IO29, WEAK_PULLDN_OUTPUT),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	gpio_direction_output(GP_BACKLIGHT_MIPI, 0);
	gpio_request(GP_SN65DSI83_EN, "sn65en");
	gpio_direction_output(GP_SN65DSI83_EN, 0);

	gpio_direction_output(GP_GPIOLEDS_RED, 0);
	gpio_direction_output(GP_GPIOLEDS_GREEN, 0);
	gpio_direction_output(GP_GPIOLEDS_YELLOW, 0);
	gpio_direction_output(GP_GPIOLEDS_DRY, 0);
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	gpio_direction_output(GP_EMMC_RESET, 1);
	set_wdog_reset(wdog);
	gpio_direction_output(GP_SN65DSI83_EN, 0);
	gpio_free(GP_SN65DSI83_EN);
	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
int board_detect_gt911(struct display_info_t const *di)
{
	return board_detect_gt911_common(di, 0, 0, GP_GT911_RESET, GPIRQ_GT911);
}

int board_detect_gt911_sn65(struct display_info_t const *di)
{
	board_disable_i2c_mux(di->bus_num & 0x0f);
	return board_detect_gt911_sn65_common(di, 0, 0, GP_GT911_RESET, GPIRQ_GT911);
}

int board_detect_pca9546_gt911(struct display_info_t const *di)
{
	return board_detect_gt911_common(di, 1 << (di->bus_num >> 4), 0, GP_GT911_RESET, GPIRQ_GT911);
}

static const struct display_info_t displays[] = {
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-1",	B, MIPI, board_detect_gt911_sn65, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-1",	B, MIPI, NULL, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-2",	U, MIPI, NULL, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-3",	E, MIPI, NULL, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),

	VD_MIPI_TM070JDHG30_x("tm070jdhg30-4",	B, MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x2c, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_M101NWWB_x("m101nwwb-4",	B, MIPI, NULL, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x2c, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_MTD0900DCP27KF(MIPI, fbp_detect_i2c, fbp_bus_gp(1, 0, 0, 0), 0x41, FBP_MIPI_TO_LVDS, FBTS_ILI251X),
	VD_DMT050WVNXCMI(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_SC18IS602B_RESET, 0, 30), fbp_addr_gp(0x2f, 0, 6, 0), FBP_SPI_LCD, FBTS_GOODIX),
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp(1, GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
	VD_LCM_JM430_MINI(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_ST1633_RESET, GP_TC358762_EN, 30), fbp_addr_gp(0x55, 0, 0, 0), FBTS_ST1633I),	/* Sitronix touch */

	VD_LS050T1SX12(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d, FBTS_GOODIX),
	VD_LTK0680YTMDB(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d, FBTS_GOODIX),
	VD_MIPI_COM50H5N03ULC(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x00),

	/* 0x3e is the TPS65132 power chip on our adapter board */
	VD_MIPI_LCD133_070(MIPI, board_detect_lcd133, fbp_bus_gp(1, GP_LCD133_070_ENABLE, GP_LCD133_070_ENABLE, 1), fbp_addr_gp(0x3e, 0, 0, 0), FBTS_FT7250),

	VD_MIPI_TM070JDHG30_LT8912(MIPI, board_detect_pca9546_gt911, fbp_bus_gp((1 | (2 << 4)), 0, 0, 0), 0x5d, FBP_PCA9546, FBTS_GOODIX2),
	VD_MIPI_TM070JDHG30_LT8912_2(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (2 << 4)), 0, GP_TS_FT5X06_RESET, 0), 0x38, FBP_PCA9546, FBTS_FT5X06_2),
	VD_MIPI_VTFT101RPFT20(MIPI, NULL, fbp_bus_gp((1 | (2 << 4)), 0, 0, 0), 0x5d, FBP_PCA9546, FBTS_GOODIX2),
	VD_MIPI_VTFT101RPFT20_2(MIPI, NULL, fbp_bus_gp((1 | (2 << 4)), 0, GP_TS_FT5X06_RESET, 0), 0x38, FBP_PCA9546, FBTS_FT5X06_2),
	/* Looking for the max7323 gpio chip on the Lontium daughter board */
	VD_MIPI_1920_1080M_60(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_800M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_720M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1024_768M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_800_600MR_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_720_480M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_640_480M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),

	VD_MIPI_VTFT101RPFT20_3(MIPI, board_detect_pca9540, fbp_bus_gp((1 | (1 << 4)), GP_LT8912_2_RESET, 0, 0), 0x48, FBP_PCA9540),
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
#ifdef CONFIG_FEC_PHY_BITBANG
	gpio_request(GP_MII_MDC, "mii_mdc");
	gpio_request(GP_MII_MDIO, "mii_mdio");
#endif
	gpio_request(GP_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_GT911, "gt911_irq");
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
#else
	gpio_request(GP_LCD133_070_RESET, "lcd133_070_reset");
#endif
	gpio_request(GP_J46P2, "J64P2");
	gpio_request(GP_J46P3, "J64P3");
	gpio_request(GPIRQ_CSI1_TC3587, "csi1_mipi_pwdn");
	gpio_request(GP_CSI1_OV5640_MIPI_RESET, "csi1_mipi_reset");
	gpio_direction_output(GP_GT911_RESET, 0);
	gpio_direction_output(GP_J46P2, 0);
	gpio_direction_output(GP_J46P3, 0);
	gpio_direction_input(GPIRQ_CSI1_TC3587);
	gpio_direction_output(GP_CSI1_OV5640_MIPI_RESET, 0);
	/* An unmodified panel has reset connected directly to 1.8V, so make input */
	gpio_direction_input(GP_LCD133_070_RESET);
#ifdef CONFIG_DM_VIDEO
	gpio_free(GP_LCD133_070_RESET);
#endif
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

void board_env_init(void)
{
	/*
	 * If touchscreen reset is low, display will not initialize, but runs fine
	 * after init independent of gpio level
	 */
	gpio_direction_output(GP_TS_FT7250_RESET, 1);
}
