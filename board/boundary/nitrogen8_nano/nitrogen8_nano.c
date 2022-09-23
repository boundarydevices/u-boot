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
#define GP_LT8912_DISPLAY_EN		IMX_GPIO_NR(1, 1)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x116),

#define GPIRQ_TS_GT911 			IMX_GPIO_NR(1, 6)
	IOMUX_PAD_CTRL(GPIO1_IO06__GPIO1_IO6, 0x180),
#define GP_TS_GT911_RESET			IMX_GPIO_NR(1, 7)
#define GP_ST1633_RESET			IMX_GPIO_NR(1, 7)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO1_IO07__GPIO1_IO7, 0x100),

#define GP_TC358762_EN		IMX_GPIO_NR(1, 9)
#define GP_SC18IS602B_RESET	IMX_GPIO_NR(1, 9)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(1, 9)
#define GP_MIPI_RESET		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO9, 0x06),

#define GPIRQ_RV4162		<&gpio4 22 IRQ_TYPE_LEVEL_LOW>
	IOMUX_PAD_CTRL(GPIO1_IO03__GPIO1_IO3, 0x1c0),

#define GP_CSI1_MIPI_PWDN	IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO1_IO08__GPIO1_IO8, 0x141),
#define GP_CSI1_MIPI_RESET	IMX_GPIO_NR(3, 5)
	IOMUX_PAD_CTRL(NAND_CLE__GPIO3_IO5, 0x101),

	/* sound - wm8960 */
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(1, 10)
	IOMUX_PAD_CTRL(GPIO1_IO10__GPIO1_IO10, 0x80),
#define GP_WM8960_HP_DET	IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(SAI3_RXFS__GPIO4_IO28, 0x80),

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
	gpio_direction_output(GP_SN65DSI83_EN, 0);
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
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-1",	B, MIPI, board_detect_gt911_sn65, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-2",	U, MIPI, NULL, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-3",	E, MIPI, NULL, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-1",	B, MIPI, NULL, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-2",	U, MIPI, NULL, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-3",	E, MIPI, NULL, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),

	VD_MIPI_M101NWWB_x("m101nwwb-4",	B, MIPI, board_detect_sn65_and_ts, fbp_bus_gp(1, GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_TM070JDHG30_x("tm070jdhg30-4",	B, MIPI, NULL, fbp_bus_gp(1, GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),

	VD_DMT050WVNXCMI(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_SC18IS602B_RESET, 0, 30), fbp_addr_gp(0x2f, 0, 6, 0), FBP_SPI_LCD, FBTS_GOODIX),
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp(1, 0, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
	VD_LCM_JM430_MINI(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_ST1633_RESET, GP_TC358762_EN, 30), fbp_addr_gp(0x55, 0, 0, 0), FBTS_ST1633I),	/* Sitronix touch */

	VD_LTK0680YTMDB(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d, FBTS_GOODIX),
	VD_MIPI_COM50H5N03ULC(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x00),

	/* Looking for the max7323 gpio chip on the Lontium daughter board */
	VD_MIPI_1920_1080M_60(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_800M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_720M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1024_768M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_800_600MR_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_720_480M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_640_480M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_TM070JDHG30_LT8912(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), fbp_addr_gp(0x68, GP_LT8912_DISPLAY_EN, 0, 0), FBP_PCA9546),

	VD_MIPI_VTFT101RPFT20(MIPI, fbp_detect_i2c, 1, 0x70, FBP_PCA9540),
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
	gpio_request(GP_TS_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_TS_GT911, "gt911_irq");
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
#endif
	gpio_request(GP_CSI1_MIPI_PWDN, "csi1_mipi_pwdn");
	gpio_request(GP_CSI1_MIPI_RESET, "csi1_mipi_reset");
	gpio_direction_output(GP_TS_GT911_RESET, 0);
	gpio_direction_output(GP_CSI1_MIPI_PWDN, 1);
	gpio_direction_output(GP_CSI1_MIPI_RESET, 0);
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
