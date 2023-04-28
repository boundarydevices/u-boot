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
	IOMUX_PAD_CTRL(GPIO1_IO07__GPIO1_IO7, 0x49),

#define GP_TC358762_EN		IMX_GPIO_NR(1, 9)
#define GP_SC18IS602B_RESET	IMX_GPIO_NR(1, 9)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(1, 9)
#define GP_MIPI_RESET		IMX_GPIO_NR(1, 9)
/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO9, 0x06),

#define GP_BACKLIGHT_MIPI		IMX_GPIO_NR(5, 3)
	IOMUX_PAD_CTRL(SPDIF_TX__GPIO5_IO3, 0x116),

#define GPIRQ_CSI1_TC3587		IMX_GPIO_NR(4, 28)
#define GP_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(SAI3_RXFS__GPIO4_IO28, 0x141),
#define GP_OV5640_MIPI_RESET	IMX_GPIO_NR(1, 11)
	IOMUX_PAD_CTRL(GPIO1_IO11__GPIO1_IO11, 0x101),

#define GP_OTG2_HUB_RESET	IMX_GPIO_NR(5, 5)
	IOMUX_PAD_CTRL(SPDIF_EXT_CLK__GPIO5_IO5, 0x100),

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
#ifdef CONFIG_FEC_RESET_PULLUP
#define RESET_PINCTRL	WEAK_PULLUP_OUTPUT
#else
#define RESET_PINCTRL	WEAK_PULLDN_OUTPUT
#endif
	IOMUX_PAD_CTRL(GPIO1_IO00__GPIO1_IO0, RESET_PINCTRL),
#endif

#ifdef CONFIG_IMX8MM
	IOMUX_PAD_CTRL(GPIO1_IO12__USB1_OTG_PWR, 0x16),
	IOMUX_PAD_CTRL(GPIO1_IO13__USB1_OTG_OC, WEAK_PULLUP),
#endif

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IOMUX_PAD_CTRL(SD1_RESET_B__GPIO2_IO10, 0x41),

#define GP_REG_3P7V    IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(SAI1_MCLK__GPIO4_IO20, 0x100),
	/* EC21 - Modem pins */
#define GP_EC21_RESET		IMX_GPIO_NR(4, 2)
	IOMUX_PAD_CTRL(SAI1_RXD0__GPIO4_IO2, 0x100),	/*  98, LTE_RESET_N */
#define GP_EC21_USB_BOOT	IMX_GPIO_NR(4, 27)
	IOMUX_PAD_CTRL(SAI2_MCLK__GPIO4_IO27, 0x100),	/* 123, LTE_USB_BOOT */
#define GP_EC21_POWER_KEY	IMX_GPIO_NR(4, 10)
	IOMUX_PAD_CTRL(SAI1_TXFS__GPIO4_IO10, 0x140),	/* 106, LTE_ON_OFF */
#define GP_EC21_USIM_DETECT	IMX_GPIO_NR(3, 20)
	IOMUX_PAD_CTRL(SAI5_RXC__GPIO3_IO20, 0x100),	/*  84, LTE_USIM_DETECT */
#define GP_EC21_WAKE_UP		IMX_GPIO_NR(4, 11)
	IOMUX_PAD_CTRL(SAI1_TXC__GPIO4_IO11 , 0x100),	/* 107, LTE_WAKE_UP */
#define GP_EC21_AP_READY	IMX_GPIO_NR(4, 19)
	IOMUX_PAD_CTRL(SAI1_TXD7__GPIO4_IO19, 0x100),	/* 115, LTE_AP_READY */
#define GP_EC21_ACTIVE_STATUS	IMX_GPIO_NR(4, 1)
	IOMUX_PAD_CTRL(SAI1_RXC__GPIO4_IO1, 0x80),	/*  97, LTE_STAT */
#if 0
/* EC21 only on very 1st prototype of board */
#define GP_EC21_NET_STAT	IMX_GPIO_NR(1, 0)
	IOMUX_PAD_CTRL(GPIO1_IO00__GPIO1_IO0, 0x80),	/*   0, LTE_NET_STAT */
#endif
#define GP_EC21_NET_MODE	IMX_GPIO_NR(4, 12)
	IOMUX_PAD_CTRL(SAI1_TXD0__GPIO4_IO12, 0x80),	/* 108, LTE_NET_MODE */
#define GP_EC21_RI		IMX_GPIO_NR(4, 0)
	IOMUX_PAD_CTRL(SAI1_RXFS__GPIO4_IO0, 0x80),	/*  96, LTE_RI */
#define GP_12VEN		IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(SAI1_RXD3__GPIO4_IO5, 0x140),
#define GP_RELAY1_EN		IMX_GPIO_NR(4, 8)
	IOMUX_PAD_CTRL(SAI1_RXD6__GPIO4_IO8, 0x140),
#define GP_RELAY2_EN		IMX_GPIO_NR(4, 7)
	IOMUX_PAD_CTRL(SAI1_RXD5__GPIO4_IO7, 0x140),
#define GP_RELAY3_EN		IMX_GPIO_NR(4, 6)
	IOMUX_PAD_CTRL(SAI1_RXD4__GPIO4_IO6, 0x140),
#define GP_J55P1		IMX_GPIO_NR(4, 31)
	IOMUX_PAD_CTRL(SAI3_TXFS__GPIO4_IO31, 0x100),	/* J55, Pin 1 12V off */
#define GP_J55P2		IMX_GPIO_NR(5, 0)
	IOMUX_PAD_CTRL(SAI3_TXC__GPIO5_IO0, 0x100),	/* J55, Pin 2 12V off */
#define GP_J55P3		IMX_GPIO_NR(5, 1)
	IOMUX_PAD_CTRL(SAI3_TXD__GPIO5_IO1, 0x100),	/* J55, Pin 3 12V off */
};

static const struct gpio_reserve gpios_to_reserve[] = {
	{ GP_BACKLIGHT_MIPI, GPIOD_OUT_LOW, 0, "backlight_mipi", },
	{ GP_SN65DSI83_EN, GPIOD_OUT_LOW, GRF_FREE, "sn65en", },
	{ GP_REG_3P7V, GPIOD_OUT_LOW, 0, "3p7v", },
	{ GP_EC21_RESET, GPIOD_OUT_LOW, 0, "ec21-reset", },
	{ GP_EC21_USB_BOOT, GPIOD_OUT_LOW, 0, "ec21-usb-boot", },
	{ GP_EC21_POWER_KEY, GPIOD_IN, 0, "ec21-power-key", },
	{ GP_EC21_USIM_DETECT, GPIOD_OUT_LOW, 0, "ec21-usim-detect", },
	{ GP_EC21_WAKE_UP, GPIOD_OUT_LOW, 0, "ec21-wake-up", },
	{ GP_EC21_AP_READY, GPIOD_OUT_LOW, 0, "ec21-ap-ready", },
	{ GP_EMMC_RESET, GPIOD_OUT_HIGH, 0, "emmc-reset", },
	{ GP_EC21_ACTIVE_STATUS, GPIOD_IN, 0, "ec21-active-status", },
#if 0
	{ GP_EC21_NET_STAT, GPIOD_IN, 0, "ec21-net-stat", },
#endif
	{ GP_EC21_NET_MODE, GPIOD_IN, 0, "ec21-net-mode", },
	{ GP_EC21_RI, GPIOD_IN, 0, "ec21-ri", },
#ifdef CONFIG_FEC_PHY_BITBANG
	{ GP_MII_MDC, GPIOD_IN, 0, "mii_mdc", },
	{ GP_MII_MDIO, GPIOD_IN, 0, "mii_mdio", },
#endif
	{ GP_GT911_RESET, GPIOD_OUT_LOW, 0, "gt911_reset", },
	{ GPIRQ_GT911, GPIOD_IN, 0, "gt911_irq", },
	/* An unmodified panel has reset connected directly to 1.8V, so make input */
	{ GP_LCD133_070_RESET, GPIOD_IN, GRF_FREE, "lcd133_070_reset", },
	{ GPIRQ_CSI1_TC3587, GPIOD_IN, 0, "tc3587_irq", },	/* also "csi1_mipi_pwdn" */
	{ GP_OV5640_MIPI_RESET, GPIOD_OUT_LOW, 0, "csi1_mipi_reset", },
	{ GP_12VEN, GPIOD_OUT_HIGH, 0, "12VEN", },
	{ GP_RELAY1_EN, GPIOD_OUT_HIGH, 0, "RELAY1_EN", },
	{ GP_RELAY2_EN, GPIOD_OUT_HIGH, 0, "RELAY2_EN", },
	{ GP_RELAY3_EN, GPIOD_OUT_HIGH, 0, "RELAY3_EN", },
	{ GP_J55P1, GPIOD_OUT_LOW, 0, "J55P1", },
	{ GP_J55P2, GPIOD_OUT_LOW, 0, "J55P2", },
	{ GP_J55P3, GPIOD_OUT_LOW, 0, "J55P3", },
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
	return board_detect_gt911_common(di, 0, 0, GP_GT911_RESET, GPIRQ_GT911);
}

static const struct display_info_t displays[] = {
#ifdef CONFIG_BOARD_GENO
	VD_MIPI_X090DTLNC01(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x2c, FBP_MIPI_TO_LVDS, FBTS_ILI251X),
#endif
	VD_MIPI_M101NWWB_x("m101nwwb-1", B, MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_SN65DSI83_EN, 0, 0), 0x2c, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_MTD0900DCP27KF(MIPI, fbp_detect_i2c, fbp_bus_gp(1, 0, 0, 0), 0x41, FBP_MIPI_TO_LVDS, FBTS_ILI251X),
	VD_DMT050WVNXCMI(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_SC18IS602B_RESET, 0, 30), fbp_addr_gp(0x2f, 0, 6, 0), FBP_SPI_LCD, FBTS_GOODIX),
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

void board_env_init(void)
{
	/*
	 * If touchscreen reset is low, display will not initialize, but runs fine
	 * after init independent of gpio level
	 */
	gpio_direction_output(GP_TS_FT7250_RESET, 1);
}
