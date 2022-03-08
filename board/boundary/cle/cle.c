/*
 * Copyright 2022 Boundary Devices
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
#include <env.h>
#include <i2c.h>
#include <linux/delay.h>
#include <spl.h>
#include "../common/padctrl.h"
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

static iomux_v3_cfg_t const init_pads[] = {
	IOMUX_PAD_CTRL(GPIO1_IO02__WDOG1_WDOG_B, 0x140),
	IOMUX_PAD_CTRL(UART2_RXD__UART2_DCE_RX, 0x140),
	IOMUX_PAD_CTRL(UART2_TXD__UART2_DCE_TX, 0x140),

#define GP_BT_RFKILL_RESET	IMX_GPIO_NR(4, 25)
	IOMUX_PAD_CTRL(SAI2_TXC__GPIO4_IO25, 0x100),

#define GP_ECSPI2_CS0	IMX_GPIO_NR(5, 13)
	IOMUX_PAD_CTRL(ECSPI2_SS0__GPIO5_IO13, 0x140),
	IOMUX_PAD_CTRL(ECSPI2_MISO__ECSPI2_MISO, 0x14),
	IOMUX_PAD_CTRL(ECSPI2_SCLK__ECSPI2_SCLK, 0x14),
	IOMUX_PAD_CTRL(ECSPI2_MOSI__ECSPI2_MOSI, 0x14),

#ifdef CONFIG_FEC_MXC
	/* PHY - KSZ9031 */
	IOMUX_PAD_CTRL(ENET_MDC__ENET1_MDC, PAD_CTRL_ENET_MDC),
	IOMUX_PAD_CTRL(ENET_MDIO__ENET1_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(ENET_TX_CTL__ENET1_RGMII_TX_CTL, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD0__ENET1_RGMII_TD0, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD1__ENET1_RGMII_TD1, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD2__ENET1_RGMII_TD2, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD3__ENET1_RGMII_TD3, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TXC__ENET1_RGMII_TXC, PAD_CTRL_ENET_TX),
#define GP_FEC1_RESET	IMX_GPIO_NR(3, 15)
	IOMUX_PAD_CTRL(NAND_RE_B__GPIO3_IO15, 0x100),
#define GPIRQ_FEC1_PHY	IMX_GPIO_NR(3, 16)
	IOMUX_PAD_CTRL(NAND_READY_B__GPIO3_IO16, 0x1c0),
#endif

	IOMUX_PAD_CTRL(SD2_WP__GPIO2_IO20, 0x140),	/* TP2 */
	IOMUX_PAD_CTRL(SD2_RESET_B__GPIO2_IO19, 0x140),	/* TP7 */
	IOMUX_PAD_CTRL(SAI5_RXC__GPIO3_IO20, 0x140),	/* TP9 */
	IOMUX_PAD_CTRL(SAI5_RXD1__GPIO3_IO22, 0x140),	/* BT_HOST_WAKE */
	IOMUX_PAD_CTRL(GPIO1_IO11__GPIO1_IO11, 0x140),	/* J15 pin 32 */
	IOMUX_PAD_CTRL(GPIO1_IO08__GPIO1_IO8, 0x140),	/* J15 pin 34 */
	IOMUX_PAD_CTRL(GPIO1_IO05__GPIO1_IO5, 0x140),	/* J15 pin 36 */
	IOMUX_PAD_CTRL(GPIO1_IO03__GPIO1_IO3, 0x140),	/* J15 pin 38 */

#define GP_I2C1_PF8100_EWARN	IMX_GPIO_NR(3, 3)
	IOMUX_PAD_CTRL(NAND_CE2_B__GPIO3_IO3, 0x1c0),	/* EWARN */
#define GP_I2C1_PF8100_FAULT	IMX_GPIO_NR(3, 4)
	IOMUX_PAD_CTRL(NAND_CE3_B__GPIO3_IO4, 0x1c0),	/* FAULT */

#define GPIRQ_SN65DSI83			IMX_GPIO_NR(1, 1)
#define GPIRQ_LT8912_2			IMX_GPIO_NR(1, 1)
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 1)
#define GP_LCD133_070_RESET		IMX_GPIO_NR(1, 1)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x140),

#define GPIRQ_TS_GT911 			IMX_GPIO_NR(1, 6)
#define GPIRQ_TS_FT5X06			IMX_GPIO_NR(1, 6)
	IOMUX_PAD_CTRL(GPIO1_IO06__GPIO1_IO6, 0x1c0),
#define GP_TS_GT911_RESET		IMX_GPIO_NR(1, 7)
#define GP_ST1633_RESET			IMX_GPIO_NR(1, 7)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(1, 7)
#define GP_TS_FT7250_RESET		IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO1_IO07__GPIO1_IO7, 0x100),

#define GP_SN65DSI83_EN		IMX_GPIO_NR(1, 9)
#define GP_LT8912_2_RESET	IMX_GPIO_NR(1, 9)
#define GP_TC358762_EN		IMX_GPIO_NR(1, 9)
#define GP_SC18IS602B_RESET	IMX_GPIO_NR(1, 9)
#define GP_MIPI_RESET		IMX_GPIO_NR(1, 9)
/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO9, 0x100),
	IOMUX_PAD_CTRL(SAI2_RXC__GPIO4_IO22, PAD_CTL_DSE1 | PAD_CTL_ODE),

#define GP_BACKLIGHT_MIPI_PWM	IMX_GPIO_NR(5, 3)
	IOMUX_PAD_CTRL(SPDIF_TX__GPIO5_IO3, 0x100),

#define GPIRQ_RV4162		IMX_GPIO_NR(3, 25)
	IOMUX_PAD_CTRL(SAI5_MCLK__GPIO3_IO25, 0x1c0),

#define GP_REG_USDHC2_VSEL	IMX_GPIO_NR(3, 2)
	IOMUX_PAD_CTRL(NAND_CE1_B__GPIO3_IO2, 0x100),

#define GP_REG_WLAN_VMMC	IMX_GPIO_NR(4, 26)
	IOMUX_PAD_CTRL(SAI2_TXD0__GPIO4_IO26, 0x100),

	/* sound - wm8960 */
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(1, 10)
	IOMUX_PAD_CTRL(GPIO1_IO10__GPIO1_IO10, 0x80),
#define GP_WM8960_HP_DET	IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(SAI3_RXFS__GPIO4_IO28, 0x80),

	IOMUX_PAD_CTRL(UART1_RXD__UART1_DCE_RX, 0x140),
	IOMUX_PAD_CTRL(UART1_TXD__UART1_DCE_TX, 0x140),
	IOMUX_PAD_CTRL(UART3_RXD__UART1_CTS_B, 0x140),
	IOMUX_PAD_CTRL(UART3_TXD__UART1_RTS_B, 0x140),

	IOMUX_PAD_CTRL(ECSPI1_SCLK__UART3_RX, 0x140),
	IOMUX_PAD_CTRL(ECSPI1_MOSI__UART3_TX, 0x140),

	IOMUX_PAD_CTRL(UART4_RXD__UART4_RX, 0x140),
	IOMUX_PAD_CTRL(UART4_TXD__UART4_TX, 0x140),

	IOMUX_PAD_CTRL(GPIO1_IO12__USB1_OTG_PWR, 0x100),
	IOMUX_PAD_CTRL(GPIO1_IO13__USB1_OTG_OC, 0x1c0),

#define GP_OTG2_HUB_RESET	IMX_GPIO_NR(5, 0)
	IOMUX_PAD_CTRL(SAI3_TXC__GPIO5_IO0, 0x100),

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IOMUX_PAD_CTRL(SD1_RESET_B__GPIO2_IO10, 0x140),
#define GP_USDHC2_CD	IMX_GPIO_NR(2, 12)
	IOMUX_PAD_CTRL(SD2_CD_B__GPIO2_IO12, 0x1c0),
	/* Bluetooth slow clock */
#if 0
	IOMUX_PAD_CTRL(GPIO1_IO00__ANAMIX_REF_CLK_32K, 0x03),
#endif
	IOMUX_PAD_CTRL(SAI2_TXFS__GPIO4_IO24, 0x140),	/* WL_IRQ */
	IOMUX_PAD_CTRL(SAI2_RXFS__GPIO4_IO21, 0x140),	/* CLK_REQ */
};

static const struct gpio_reserve gpios_to_reserve[] = {
	{ GP_BT_RFKILL_RESET, GPIOD_OUT_LOW, 0, "bt-rfkill-reset", },
	{ GP_ECSPI2_CS0, GPIOD_OUT_HIGH, 0, "ecspi2_cs", },
	{ GP_FEC1_RESET, GPIOD_OUT_LOW, 0, "fec1-reset", },
	{ GPIRQ_FEC1_PHY, GPIOD_IN, 0, "irq-fec1-phy", },
	{ GP_I2C1_PF8100_EWARN, GPIOD_IN, 0, "ewarn", },
	{ GP_I2C1_PF8100_FAULT, GPIOD_IN, 0, "fault", },
	{ GP_SN65DSI83_EN, GPIOD_OUT_LOW, GRF_FREE, "sn65-en", },
	{ GPIRQ_SN65DSI83, GPIOD_IN, GRF_FREE, "irq-sn65", },
	{ GPIRQ_TS_GT911, GPIOD_IN, 0, "irq-touch", },
	{ GP_TS_GT911_RESET, GPIOD_OUT_LOW, 0, "touch-reset", },
	{ GP_BACKLIGHT_MIPI_PWM, GPIOD_OUT_LOW, 0, "backlight-pwm", },
	{ GPIRQ_RV4162, GPIOD_IN, 0, "irq-rv4162", },
	{ GP_REG_USDHC2_VSEL, GPIOD_OUT_LOW, GRF_FREE, "usdhc2-vsel", },
	{ GP_REG_WLAN_VMMC, GPIOD_OUT_LOW, 0, "wlan-en", },
	{ GP_WM8960_MIC_DET, GPIOD_IN, 0, "mic-det", },
	{ GP_WM8960_HP_DET, GPIOD_IN, 0, "hp-det", },
	{ GP_OTG2_HUB_RESET, GPIOD_OUT_LOW, 0, "otg2-hub-reset", },
	{ GP_EMMC_RESET, GPIOD_OUT_HIGH, GRF_FREE, "emmc-reset", },
	{ GP_USDHC2_CD, GPIOD_IN, GRF_FREE, "usdhc2-cd", },
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

int board_detect_pca9540_gt911(struct display_info_t const *di)
{
	return board_detect_gt911_common(di, (di->bus_num >> 4) ? 5 : 4, 0, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
}

int board_detect_gt911_sn65(struct display_info_t const *di)
{
	return board_detect_gt911_sn65_common(di, 0, 0, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
}

static const struct display_info_t displays[] = {
	VD_MIPI_M101NWWB_2(MIPI, board_detect_gt911_sn65, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB(MIPI, board_detect_sn65_and_ts, fbp_bus_gp(1, GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_MTD0900DCP27KF(MIPI, fbp_detect_i2c, fbp_bus_gp(1, 0, 0, 0), 0x41, FBP_MIPI_TO_LVDS, FBTS_ILI251X),
	VD_DMT050WVNXCMI(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_SC18IS602B_RESET, 0, 30), fbp_addr_gp(0x2f, 0, 6, 0), FBP_SPI_LCD, FBTS_GOODIX),
	VD_ER_TFT050_MINI(MIPI, board_detect_pca9540_gt911, fbp_bus_gp(1, 0, GP_TC358762_EN, 0), fbp_addr_gp(0x5d, 0, 0, 0), FBP_PCA9540_2),
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp(1, GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
	VD_LCM_JM430_MINI(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_ST1633_RESET, GP_TC358762_EN, 30), fbp_addr_gp(0x55, 0, 0, 0), FBTS_ST1633I),	/* Sitronix touch */

	VD_LS050T1SX12(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d, FBTS_GOODIX),
	VD_LTK0680YTMDB(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d, FBTS_GOODIX),
	VD_MIPI_COM50H5N03ULC(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x00),

	/* 0x3e is the TPS65132 power chip on our adapter board */
	VD_MIPI_LCD133_070(MIPI, board_detect_lcd133, fbp_bus_gp(1, GP_LCD133_070_ENABLE, GP_LCD133_070_ENABLE, 1), fbp_addr_gp(0x3e, 0, 0, 0), FBTS_FT7250),

	/* Looking for the max7323 gpio chip on the Lontium daughter board */
	VD_MIPI_1920_1080M_60(MIPI, board_detect_pca9546, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_800M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1280_720M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1024_768M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_800_600MR_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_720_480M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_640_480M_60(MIPI, NULL, fbp_bus_gp((1 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),

	VD_MIPI_VTFT101RPFT20(MIPI, fbp_detect_i2c, 1, 0x70, FBP_PCA9540),
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
