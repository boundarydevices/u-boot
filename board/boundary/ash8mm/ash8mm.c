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

#define GP_BACKLIGHT_MIPI_EN	IMX_GPIO_NR(4, 18)
	IOMUX_PAD_CTRL(SAI1_TXD6__GPIO4_IO18, 0x100),	/* U5(MIC2601) enable pin */
#define GP_DISPLAY_EN		IMX_GPIO_NR(1, 1)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x100),	/* J3 Pin 5 */
#define GP_BT_RFKILL_RESET	IMX_GPIO_NR(4, 25)
	IOMUX_PAD_CTRL(SAI2_TXC__GPIO4_IO25, 0x100),

#ifdef CONFIG_FEC_MXC
	/* PHY - KSZ9031 */
	IOMUX_PAD_CTRL(ENET_MDIO__ENET1_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(ENET_MDC__ENET1_MDC, PAD_CTRL_ENET_MDC),
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

#define GP_GPIOKEY_SW1	IMX_GPIO_NR(3, 21)
	IOMUX_PAD_CTRL(SAI5_RXD0__GPIO3_IO21, 0x1c0),
#define GP_GPIOKEY_SW2	IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(SAI5_RXD1__GPIO3_IO22, 0x1c0),
#define GP_GPIOKEY_SW3	IMX_GPIO_NR(3, 23)
	IOMUX_PAD_CTRL(SAI5_RXD2__GPIO3_IO23, 0x1c0),
#define GP_GPIOKEY_SW4	IMX_GPIO_NR(3, 24)
	IOMUX_PAD_CTRL(SAI5_RXD3__GPIO3_IO24, 0x1c0),
#define GP_GPIOKEY_SW5	IMX_GPIO_NR(5, 2)
	IOMUX_PAD_CTRL(SAI3_MCLK__GPIO5_IO2, 0x1c0),
#define GP_GPIOKEY_SW6	IMX_GPIO_NR(4, 31)
	IOMUX_PAD_CTRL(SAI3_TXFS__GPIO4_IO31, 0x1c0),
#define GP_GPIOKEY_SW7	IMX_GPIO_NR(5, 0)
	IOMUX_PAD_CTRL(SAI3_TXC__GPIO5_IO0, 0x1c0),
#define GP_GPIOKEY_SW8	IMX_GPIO_NR(5, 1)
	IOMUX_PAD_CTRL(SAI3_TXD__GPIO5_IO1, 0x1c0),

#define GP_BUTTON_LED	IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO1_IO08__GPIO1_IO8, 0x140),

	IOMUX_PAD_CTRL(SAI2_RXC__GPIO4_IO22, 0x1c0),	/* BT_HOST_WAKE */
	IOMUX_PAD_CTRL(SD2_WP__GPIO2_IO20, 0x140),	/* TP2 */
	IOMUX_PAD_CTRL(SD2_RESET_B__GPIO2_IO19, 0x140),	/* TP7 */
	IOMUX_PAD_CTRL(SAI2_MCLK__GPIO4_IO27, 0x140),	/* TP11 */
	IOMUX_PAD_CTRL(SAI2_RXD0__GPIO4_IO23, 0x140),	/* TP12 */
	IOMUX_PAD_CTRL(SAI3_RXFS__GPIO4_IO28, 0x140),	/* TP13 */
	IOMUX_PAD_CTRL(SAI5_RXFS__GPIO3_IO19, 0x140),	/* TP22 */
	IOMUX_PAD_CTRL(NAND_DATA01__GPIO3_IO7, 0x140),	/* TP23 */
	IOMUX_PAD_CTRL(ECSPI2_SCLK__ECSPI2_SCLK, 0x140), /* TP26 */
	IOMUX_PAD_CTRL(ECSPI2_MISO__ECSPI2_MISO, 0x140), /* TP27 */
	IOMUX_PAD_CTRL(SAI5_RXC__GPIO3_IO20, 0x140),	/* TP30 */
	IOMUX_PAD_CTRL(NAND_DATA03__GPIO3_IO9, 0x140),	/* TP31 */
	IOMUX_PAD_CTRL(SAI1_RXD2__GPIO4_IO4, 0x140),	/* TP32 */
	IOMUX_PAD_CTRL(SAI1_RXD4__GPIO4_IO6, 0x140),	/* TP33 */
	IOMUX_PAD_CTRL(GPIO1_IO14__GPIO1_IO14, 0x140),	/* TP34 */
	IOMUX_PAD_CTRL(GPIO1_IO15__GPIO1_IO15, 0x140),	/* TP36 */
	IOMUX_PAD_CTRL(UART3_TXD__GPIO5_IO27, 0x140),	/* TP37 */
	IOMUX_PAD_CTRL(GPIO1_IO03__GPIO1_IO3, 0x140),	/* TP38 */
	IOMUX_PAD_CTRL(SPDIF_RX__PWM2_OUT, 0x140),	/* TP39 */

#define GP_LT8912_RESET	IMX_GPIO_NR(3, 6)
	IOMUX_PAD_CTRL(NAND_DATA00__GPIO3_IO6, 0x100),
#define GPIRQ_LT8912	IMX_GPIO_NR(3, 8)
	IOMUX_PAD_CTRL(NAND_DATA02__GPIO3_IO8, 0x1c0),

#define GP_I2C1_PF8100_EWARN	IMX_GPIO_NR(3, 3)
	IOMUX_PAD_CTRL(NAND_CE2_B__GPIO3_IO3, 0x1c0),
#define GP_I2C1_PF8100_FAULT	IMX_GPIO_NR(3, 4)
	IOMUX_PAD_CTRL(NAND_CE3_B__GPIO3_IO4, 0x1c0),


#define GPIRQ_TS_GT911 		IMX_GPIO_NR(1, 6)
#define GPIRQ_TS_FT5X06		IMX_GPIO_NR(1, 6)
	IOMUX_PAD_CTRL(GPIO1_IO06__GPIO1_IO6, 0x1c0),
#define GP_TS_GT911_RESET	IMX_GPIO_NR(1, 7)
#define GP_ST1633_RESET		IMX_GPIO_NR(1, 7)
#define GP_TS_FT5X06_RESET	IMX_GPIO_NR(1, 7)
#define GP_TS_FT7250_RESET	IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO1_IO07__GPIO1_IO7, 0x100),

#define GPIRQ_RV4162		IMX_GPIO_NR(3, 25)
	IOMUX_PAD_CTRL(SAI5_MCLK__GPIO3_IO25, 0x1c0),

#define GP_BACKLIGHT_MIPI_PWM	IMX_GPIO_NR(5, 3)
	IOMUX_PAD_CTRL(SPDIF_TX__GPIO5_IO3, 0x100),

#define GP_REG_USDHC2_VSEL	IMX_GPIO_NR(3, 2)
	IOMUX_PAD_CTRL(NAND_CE1_B__GPIO3_IO2, 0x100),

#define GP_REG_WLAN_VMMC	IMX_GPIO_NR(4, 26)
	IOMUX_PAD_CTRL(SAI2_TXD0__GPIO4_IO26, 0x100),

	/* sound - wm8960 */
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(1, 10)
	IOMUX_PAD_CTRL(GPIO1_IO10__GPIO1_IO10, 0x80),
#define GP_TDA7991_MUTE		IMX_GPIO_NR(4, 13)
	IOMUX_PAD_CTRL(SAI1_TXD1__GPIO4_IO13, 0x100),
#define GP_TDA7991_G0		IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(SAI1_TXD3__GPIO4_IO15, 0x100),
#define GP_TDA7991_G1		IMX_GPIO_NR(4, 14)
	IOMUX_PAD_CTRL(SAI1_TXD2__GPIO4_IO14, 0x100),

	IOMUX_PAD_CTRL(UART1_RXD__UART1_DCE_RX, 0x140),
	IOMUX_PAD_CTRL(UART1_TXD__UART1_DCE_TX, 0x140),
#define GP_UART1_RXD_LED	IMX_GPIO_NR(4, 0)
	IOMUX_PAD_CTRL(SAI1_RXFS__GPIO4_IO0, 0x140),
#define GP_UART1_TXD_LED	IMX_GPIO_NR(4, 1)
	IOMUX_PAD_CTRL(SAI1_RXC__GPIO4_IO1, 0x140),

	IOMUX_PAD_CTRL(ECSPI1_SCLK__UART3_RX, 0x140),
	IOMUX_PAD_CTRL(ECSPI1_MOSI__UART3_TX, 0x140),
	IOMUX_PAD_CTRL(ECSPI1_SS0__UART3_RTS_B, 0x140),
	IOMUX_PAD_CTRL(ECSPI1_MISO__UART3_CTS_B, 0x140),

	IOMUX_PAD_CTRL(UART4_RXD__UART4_RX, 0x140),
	IOMUX_PAD_CTRL(UART4_TXD__UART4_TX, 0x140),

	IOMUX_PAD_CTRL(GPIO1_IO12__USB1_OTG_PWR, 0x100),
	IOMUX_PAD_CTRL(GPIO1_IO13__USB1_OTG_OC, 0x1c0),

#define GP_OTG2_HUB_RESET	IMX_GPIO_NR(5, 13)
	IOMUX_PAD_CTRL(ECSPI2_SS0__GPIO5_IO13, 0x100),

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IOMUX_PAD_CTRL(SD1_RESET_B__GPIO2_IO10, 0x41),
#define GP_USDHC2_CD	IMX_GPIO_NR(2, 12)
	IOMUX_PAD_CTRL(SD2_CD_B__GPIO2_IO12, 0x1c4),
	/* Bluetooth slow clock */
#if 0
	IOMUX_PAD_CTRL(GPIO1_IO00__ANAMIX_REF_CLK_32K, 0x03),
#endif
	IOMUX_PAD_CTRL(SAI2_TXFS__GPIO4_IO24, 0x140),	/* WL_IRQ */
	IOMUX_PAD_CTRL(SAI2_RXFS__GPIO4_IO21, 0x140),	/* CLK_REQ */
};

static const struct gpio_reserve gpios_to_reserve[] = {
	{ GP_BACKLIGHT_MIPI_EN, GPIOD_OUT_LOW, GRF_FREE, "backlight-en", },
	{ GP_DISPLAY_EN, GPIOD_OUT_LOW, GRF_FREE, "display-en", },
	{ GP_BT_RFKILL_RESET, GPIOD_OUT_LOW, 0, "bt-rfkill-reset", },
	{ GP_FEC1_RESET, GPIOD_OUT_LOW, 0, "fec1-reset", },
	{ GPIRQ_FEC1_PHY, GPIOD_IN, 0, "irq-fec1-phy", },
	{ GP_GPIOKEY_SW1, GPIOD_IN, 0, "sw1", },
	{ GP_GPIOKEY_SW2, GPIOD_IN, 0, "sw2", },
	{ GP_GPIOKEY_SW3, GPIOD_IN, 0, "sw3", },
	{ GP_GPIOKEY_SW4, GPIOD_IN, 0, "sw4", },
	{ GP_GPIOKEY_SW5, GPIOD_IN, 0, "sw5", },
	{ GP_GPIOKEY_SW6, GPIOD_IN, 0, "sw6", },
	{ GP_GPIOKEY_SW7, GPIOD_IN, 0, "sw7", },
	{ GP_GPIOKEY_SW8, GPIOD_IN, 0, "sw8", },
	{ GP_BUTTON_LED, GPIOD_OUT_HIGH, 0, "button-led", },
	{ GP_LT8912_RESET, GPIOD_OUT_LOW, GRF_FREE, "lt8912-reset", },
	{ GPIRQ_LT8912, GPIOD_IN, GRF_FREE, "irq-lt8912", },
	{ GP_I2C1_PF8100_EWARN, GPIOD_IN, 0, "ewarn", },
	{ GP_I2C1_PF8100_FAULT, GPIOD_IN, 0, "fault", },
	{ GPIRQ_TS_GT911, GPIOD_IN, 0, "irq-touch", },
	{ GP_TS_GT911_RESET, GPIOD_OUT_LOW, 0, "touch-reset", },
	{ GPIRQ_RV4162, GPIOD_IN, 0, "irq-rv4162", },
	{ GP_BACKLIGHT_MIPI_PWM, GPIOD_OUT_LOW, 0, "backlight-pwm", },
	{ GP_REG_USDHC2_VSEL, GPIOD_OUT_LOW, GRF_FREE, "usdhc2-vsel", },
	{ GP_REG_WLAN_VMMC, GPIOD_OUT_LOW, 0, "wlan-en", },
	{ GP_WM8960_MIC_DET, GPIOD_IN, 0, "mic-det", },
	{ GP_TDA7991_MUTE, GPIOD_OUT_LOW, 0, "mute", },
	{ GP_TDA7991_G0, GPIOD_OUT_LOW, 0, "g0", },
	{ GP_TDA7991_G1, GPIOD_OUT_LOW, 0, "g1", },
	{ GP_UART1_RXD_LED, GPIOD_OUT_LOW, GRF_FREE, "rxd-led", },
	{ GP_UART1_TXD_LED, GPIOD_OUT_LOW, GRF_FREE, "txd-led", },
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
static int sw_vals = -1;

static ulong swm00[] = { CONFIG_SWM_DISPLAY_00_A, CONFIG_SWM_DISPLAY_00_B };
static ulong swm24[] = { CONFIG_SWM_DISPLAY_24_A, CONFIG_SWM_DISPLAY_24_B };
static ulong swm38[] = { CONFIG_SWM_DISPLAY_38_A, CONFIG_SWM_DISPLAY_38_B };

int board_detect_display(const struct display_info_t *di)
{
	unsigned sw = sw_vals & 0x3f;
	unsigned swl = sw >> 5;
	ulong swm = 1 << (sw & 0x1f);
	ulong mask;
	char buf[16];

	snprintf(buf, sizeof(buf), "swm%02x_%c", di->addr_num, 'a' + swl);

	mask = env_get_hex(buf, (di->addr_num == 0x24) ?
			swm24[swl] : (di->addr_num == 0x38) ? swm38[swl] : swm00[swl]);
	return (swm & mask) ? 1 : 0;
}

static const struct display_info_t displays[] = {
	/* lt8912 mipi to lvds */
	VD_WVGA_TX23D200_18L(LVDS, board_detect_display, fbp_bus_gp(1, 0, 0, 0), 0x00),
	VD_WVGA_TX23D200_18H(LVDS, NULL, fbp_bus_gp(1, 0, 0, 0), 0x00),
	VD_WVGA_TX23D200_24L(LVDS, NULL, fbp_bus_gp(1, 0, 0, 0), 0x00),
	VD_WVGA_TX23D200_24H(LVDS, NULL, fbp_bus_gp(1, 0, 0, 0), 0x00),
	VD_AM_1280800P2TZQW(LVDS, board_detect_display, fbp_bus_gp(1, 0, 0, 0), 0x24, FBTS_CYTTSP5),
	VD_DT070BTFT_24H(LVDS, board_detect_display, fbp_bus_gp(1, 0, 0, 0), 0x38, FBTS_FT5X06),
	VD_DT070BTFT_24L(LVDS, NULL, fbp_bus_gp(1, 0, 0, 0), 0x38, FBTS_FT5X06),
	VD_DT070BTFT_18H(LVDS, NULL, fbp_bus_gp(1, 0, 0, 0), 0x38, FBTS_FT5X06),
	VD_DT070BTFT_18L(LVDS, NULL, fbp_bus_gp(1, 0, 0, 0), 0x38, FBTS_FT5X06),
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

const struct button_key board_buttons[] = {
	{"sw1",		GP_GPIOKEY_SW1,		'1', 1},
	{"sw2",		GP_GPIOKEY_SW2,		'2', 1},
	{"sw3",		GP_GPIOKEY_SW3,		'3', 1},
	{"sw4",		GP_GPIOKEY_SW4,		'4', 1},
	{"sw5",		GP_GPIOKEY_SW5,		'5', 1},
	{"sw6",		GP_GPIOKEY_SW6,		'6', 1},
	{"sw7",		GP_GPIOKEY_SW7,		'7', 1},
	{"sw8",		GP_GPIOKEY_SW8,		'8', 1},
	{NULL, 0, 0, 0},
};

static int read_keys_int(int cnt)
{
	int i = 0;
	const struct button_key *bb = board_buttons;
	unsigned short gp;
	int mask = 0;

	while (i < cnt) {
		if (!bb->name)
			break;
		gp = bb->gpnum;
		if (gpio_get_value(gp) ^ bb->active_low) {
			mask |= 1 << i;
		}
		i++;
		bb++;
	}
	return mask;
}

int board_init(void)
{
	sw_vals = read_keys_int(6);
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
	env_set_hex("switch", sw_vals);
}
