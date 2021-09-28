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
#include <spl.h>
#include <usb.h>
#include "../common/padctrl.h"
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

static iomux_v3_cfg_t const init_pads[] = {
	IOMUX_PAD_CTRL(GPIO1_IO02__WDOG1_WDOG_B, WDOG_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_RXD__UART2_DCE_RX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_TXD__UART2_DCE_TX, UART_PAD_CTRL),
#define GP_BT_RFKILL_RESET	IMX_GPIO_NR(3, 14)
	IOMUX_PAD_CTRL(NAND_DQS__GPIO3_IO14, 0x119),
#define GP_ECSPI2_CS0		IMX_GPIO_NR(5, 13)
	IOMUX_PAD_CTRL(ECSPI2_SS0__GPIO5_IO13, 0x140),

#define GP_FEC1_RESET	IMX_GPIO_NR(3, 15)
	IOMUX_PAD_CTRL(NAND_RE_B__GPIO3_IO15, 0x119),
#define GPIRQ_FEC1_PHY	IMX_GPIO_NR(3, 16)
	IOMUX_PAD_CTRL(NAND_READY_B__GPIO3_IO16, 0x159),

#define GP_GPIOKEY_POWER_FAIL	IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO1_IO03__GPIO1_IO3, 0x140),

#define GP_GPIOLEDS_RED1	IMX_GPIO_NR(5, 3)
	IOMUX_PAD_CTRL(SPDIF_TX__GPIO5_IO3, 0x140),
#define GP_GPIOLEDS_GREEN1	IMX_GPIO_NR(5, 4)
	IOMUX_PAD_CTRL(SPDIF_RX__GPIO5_IO4, 0x140),
#define GP_GPIOLEDS_BLUE1	IMX_GPIO_NR(5, 5)
	IOMUX_PAD_CTRL(SPDIF_EXT_CLK__GPIO5_IO5, 0x140),

#define GP_GPIOLEDS_RED2	IMX_GPIO_NR(3, 25)
	IOMUX_PAD_CTRL(SAI5_MCLK__GPIO3_IO25, 0x140),
#define GP_GPIOLEDS_GREEN2	IMX_GPIO_NR(3, 20)
	IOMUX_PAD_CTRL(SAI5_RXC__GPIO3_IO20, 0x140),
#define GP_GPIOLEDS_BLUE2	IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(SAI5_RXFS__GPIO3_IO19, 0x140),

#define GP_GPIOLEDS_RED3	IMX_GPIO_NR(3, 21)
	IOMUX_PAD_CTRL(SAI5_RXD0__GPIO3_IO21, 0x140),
#define GP_GPIOLEDS_GREEN3	IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(SAI5_RXD1__GPIO3_IO22, 0x140),
#define GP_GPIOLEDS_BLUE3	IMX_GPIO_NR(3, 23)
	IOMUX_PAD_CTRL(SAI5_RXD2__GPIO3_IO23, 0x140),

#define GPIRQ_RV3028		IMX_GPIO_NR(1, 1)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x1c0),
#define GP_RV3028_EVI		IMX_GPIO_NR(1, 5)
	IOMUX_PAD_CTRL(GPIO1_IO05__GPIO1_IO5, 0x100),

#define GP_PCIE0_RESET		IMX_GPIO_NR(4, 31)
	IOMUX_PAD_CTRL(SAI3_TXFS__GPIO4_IO31, 0x100),
#define GP_PCIE0_DISABLE	IMX_GPIO_NR(1, 4)
	IOMUX_PAD_CTRL(GPIO1_IO04__GPIO1_IO4, 0x100),

#define GP_REG_3P7V	IMX_GPIO_NR(5, 0)
	IOMUX_PAD_CTRL(SAI3_TXC__GPIO5_IO0, 0x116),

#define GP_USDHC2_VSEL	IMX_GPIO_NR(3, 2)
	IOMUX_PAD_CTRL(NAND_CE1_B__GPIO3_IO2, 0x116),

#define GP_REG_WLAN_VMMC	IMX_GPIO_NR(4, 18)
	IOMUX_PAD_CTRL(SAI1_TXD6__GPIO4_IO18, 0x116),

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IOMUX_PAD_CTRL(SD1_RESET_B__GPIO2_IO10, 0x111),

#define GP_USDHC2_CD	IMX_GPIO_NR(2, 12)
	IOMUX_PAD_CTRL(SD2_CD_B__GPIO2_IO12, 0x1c4),

#define GP_ZWAVE_RESET		IMX_GPIO_NR(4, 30)
	IOMUX_PAD_CTRL(SAI3_RXD__GPIO4_IO30, 0x100),


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

	IOMUX_PAD_CTRL(GPIO1_IO14__USB2_OTG_PWR, 0x16),
	IOMUX_PAD_CTRL(GPIO1_IO15__USB2_OTG_OC, WEAK_PULLUP),
	/* EC21 - Modem pins */
#define GP_EC21_RESET		IMX_GPIO_NR(4, 13)
	IOMUX_PAD_CTRL(SAI1_TXD1__GPIO4_IO13, 0x100),	/* RESET_N */
#define GP_EC21_USB_BOOT	IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(SAI1_TXD3__GPIO4_IO15, 0x100),	/* USB_BOOT */
#define GP_EC21_POWER_KEY	IMX_GPIO_NR(4, 11)
	IOMUX_PAD_CTRL(SAI1_TXC__GPIO4_IO11, 0x140),	/* ON_OFF */
#define GP_EC21_USIM_DETECT	IMX_GPIO_NR(4, 12)
	IOMUX_PAD_CTRL(SAI1_TXD0__GPIO4_IO12, 0x100),	/* USIM_DETECT */
#define GP_EC21_WAKE_UP		IMX_GPIO_NR(4, 14)
	IOMUX_PAD_CTRL(SAI1_TXD2__GPIO4_IO14 , 0x100),	/* WAKE_UP */
#define GP_EC21_AP_READY	IMX_GPIO_NR(4, 17)
	IOMUX_PAD_CTRL(SAI1_TXD5__GPIO4_IO17, 0x100),	/* AP_READY */
#define GP_EC21_ACTIVE_STATUS	IMX_GPIO_NR(4, 0)
	IOMUX_PAD_CTRL(SAI1_RXFS__GPIO4_IO0, 0x80),	/* STAT */
#define GP_EC21_NET_STAT	IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(SAI1_MCLK__GPIO4_IO20, 0x80),	/* NET_STAT */
#define GP_EC21_NET_MODE	IMX_GPIO_NR(4, 10)
	IOMUX_PAD_CTRL(SAI1_TXFS__GPIO4_IO10, 0x80),	/* NET_MODE */
#define GP_EC21_RI		IMX_GPIO_NR(4, 16)
	IOMUX_PAD_CTRL(SAI1_TXD4__GPIO4_IO16, 0x80),	/* RI */
};

static const struct gpio_reserve gpios_to_reserve[] = {
	{ GP_ECSPI2_CS0, GPIOD_OUT_HIGH, 0, "ecspi2-cs0", },
	{ GP_GPIOLEDS_RED1, GPIOD_OUT_HIGH, 0, "red1", },
	{ GP_GPIOLEDS_GREEN1, GPIOD_OUT_HIGH, 0, "green1", },
	{ GP_GPIOLEDS_BLUE1, GPIOD_OUT_HIGH, 0, "blue1", },
	{ GP_GPIOLEDS_RED2, GPIOD_OUT_HIGH, 0, "red2", },
	{ GP_GPIOLEDS_GREEN2, GPIOD_OUT_HIGH, 0, "green2", },
	{ GP_GPIOLEDS_BLUE2, GPIOD_OUT_HIGH, 0, "blue2", },
	{ GP_GPIOLEDS_RED3, GPIOD_OUT_HIGH, 0, "red3", },
	{ GP_GPIOLEDS_GREEN3, GPIOD_OUT_HIGH, 0, "green3", },
	{ GP_GPIOLEDS_BLUE3, GPIOD_OUT_HIGH, 0, "blue3", },
	{ GP_RV3028_EVI, GPIOD_OUT_HIGH, 0, "rv3028-evi", },
	{ GPIRQ_FEC1_PHY, GPIOD_IN, 0, "irq-fec1-phy", },
	{ GPIRQ_RV3028, GPIOD_IN, 0, "irq-rv3028", },
	{ GP_GPIOKEY_POWER_FAIL, GPIOD_IN, 0, "power-fail", },
	{ GP_USDHC2_CD, GPIOD_IN, GRF_FREE, "usdhc2-cd", },
#ifdef CONFIG_FEC_PHY_BITBANG
	{ GP_MII_MDC, GPIOD_IN, 0, "mii_mdc", },
	{ GP_MII_MDIO, GPIOD_IN, 0, "mii_mdio", },
#endif
	{ GP_BT_RFKILL_RESET, GPIOD_OUT_LOW, 0, "bt-rfkill-reset", },
	{ GP_FEC1_RESET, GPIOD_OUT_LOW, 0, "fec1-reset", },
	{ GP_PCIE0_RESET, GPIOD_OUT_LOW, 0, "pcie0-reset", },
	{ GP_PCIE0_DISABLE, GPIOD_OUT_LOW, 0, "pcie0-disable", },
	{ GP_REG_3P7V, GPIOD_OUT_LOW, 0, "3p7v", },
	{ GP_EC21_RESET, GPIOD_OUT_LOW, 0, "ec21-reset", },
	{ GP_EC21_USB_BOOT, GPIOD_OUT_LOW, 0, "ec21-usb-boot", },
	{ GP_EC21_POWER_KEY, GPIOD_IN, 0, "ec21-power-key", },
	{ GP_EC21_USIM_DETECT, GPIOD_OUT_LOW, 0, "ec21-usim-detect", },
	{ GP_EC21_WAKE_UP, GPIOD_OUT_LOW, 0, "ec21-wake-up", },
	{ GP_EC21_AP_READY, GPIOD_OUT_LOW, 0, "ec21-ap-ready", },
	{ GP_EC21_ACTIVE_STATUS, GPIOD_IN, 0, "ec21-active-status", },
	{ GP_EC21_NET_STAT, GPIOD_IN, 0, "ec21-net-stat", },
	{ GP_EC21_NET_MODE, GPIOD_IN, 0, "ec21-net-mode", },
	{ GP_EC21_RI, GPIOD_IN, 0, "ec21-ri", },
	{ GP_USDHC2_VSEL, GPIOD_OUT_LOW, GRF_FREE, "usdhc2-vsel", },
	{ GP_REG_WLAN_VMMC, GPIOD_OUT_LOW, GRF_FREE, "reg-wlan-vmmc", },
	{ GP_EMMC_RESET, GPIOD_OUT_LOW, GRF_FREE, "emmc-reset", },
	{ GP_ZWAVE_RESET, GPIOD_OUT_LOW, 0, "zwave-reset", },
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	gpios_reserve(gpios_to_reserve, ARRAY_SIZE(gpios_to_reserve));
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	gpio_direction_output(GP_EMMC_RESET, 1);
	set_wdog_reset(wdog);
	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
static const struct display_info_t displays[] = {
	VD_MIPI_640_480M_60(MIPI, fbp_detect_i2c, 1, 0x70),
	VD_MIPI_1280_720M_60(MIPI, NULL, 1, 0x70),
	VD_MIPI_1920_1080M_60(MIPI, NULL, 1, 0x70),
	VD_MIPI_1024_768M_60(MIPI, NULL, 1, 0x70),
	VD_MIPI_800_600MR_60(MIPI, NULL, 1, 0x70),
	VD_MIPI_720_480M_60(MIPI, NULL, 1, 0x70),
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
	gpios_reserve(gpios_to_reserve, ARRAY_SIZE(gpios_to_reserve));
#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	return 0;
}
