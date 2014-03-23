/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/spi.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <spi.h>
#include <input.h>
#include <usb/ehci-ci.h>
#include "../common/bd_common.h"
#include "../common/padctrl.h"

DECLARE_GLOBAL_DATA_PTR;

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL	(PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC4_PAD_CTRL	(PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

/*
 *
 */
static const iomux_v3_cfg_t init_pads[] = {
	/* ECSPI1 */
	IOMUX_PAD_CTRL(EIM_D17__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D18__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D16__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(EIM_D19__GPIO3_IO19, WEAK_PULLUP),

	/* ENET pads that don't change for PHY reset */
	IOMUX_PAD_CTRL(ENET_MDIO__ENET_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(ENET_MDC__ENET_MDC, PAD_CTRL_ENET_MDC),
	IOMUX_PAD_CTRL(RGMII_TXC__RGMII_TXC, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD0__RGMII_TD0, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD1__RGMII_TD1, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD2__RGMII_TD2, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD3__RGMII_TD3, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TX_CTL__RGMII_TX_CTL, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_REF_CLK__ENET_TX_CLK, PAD_CTRL_ENET_TX),
	/* pin 42 PHY nRST */
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(1, 27)
	IOMUX_PAD_CTRL(ENET_RXD0__GPIO1_IO27, WEAK_PULLUP),

	/* GPIO_KEYS */
#define GP_S2	IMX_GPIO_NR(3, 3)
	IOMUX_PAD_CTRL(EIM_DA3__GPIO3_IO03, WEAK_PULLUP),	/* S2 */
#define GP_S4	IMX_GPIO_NR(3, 4)
	IOMUX_PAD_CTRL(EIM_DA4__GPIO3_IO04, WEAK_PULLUP),	/* S4 */
#define GP_S3	IMX_GPIO_NR(3, 5)
	IOMUX_PAD_CTRL(EIM_DA5__GPIO3_IO05, WEAK_PULLUP),	/* S3 */

	/* Misc inputs */
	IOMUX_PAD_CTRL(GPIO_18__GPIO7_IO13, WEAK_PULLUP),	/* bidirectional - NXP P0-8 */
	IOMUX_PAD_CTRL(NANDF_WP_B__GPIO6_IO09, WEAK_PULLUP),	/* bidirectional - NXP P0-14 */
	IOMUX_PAD_CTRL(KEY_COL2__GPIO4_IO10, WEAK_PULLUP),	/* bidirectional - NXP P0-21 */
	IOMUX_PAD_CTRL(KEY_ROW2__GPIO4_IO11, WEAK_PULLUP),	/* bidirectional - NXP P0-21 */
	IOMUX_PAD_CTRL(CSI0_DATA_EN__GPIO5_IO20, WEAK_PULLUP),	/* bidirectional - NXP P2-10 */
	IOMUX_PAD_CTRL(SD1_DAT1__GPIO1_IO17, WEAK_PULLUP),	/* bidirectional - NXP RST_OUT */
	/* Misc outputs */
	IOMUX_PAD_CTRL(EIM_DA0__GPIO3_IO00, WEAK_PULLUP),	/* TPS3823 - reset generator for NXP active hi */
	IOMUX_PAD_CTRL(EIM_DA1__GPIO3_IO01, WEAK_PULLUP),	/* output - to NXP P4-30 */
	IOMUX_PAD_CTRL(EIM_DA2__GPIO3_IO02, WEAK_PULLUP),	/* 74LVC1G32 - WDT active low - reset to I.MX */
	IOMUX_PAD_CTRL(EIM_OE__GPIO2_IO25, WEAK_PULLUP), 	/* output - to NXP P4-31 */
	IOMUX_PAD_CTRL(EIM_D20__GPIO3_IO20, WEAK_PULLUP),	/* output - to NXP P1-10 */

	/* Backlight on LVDS connector */
#define GP_BACKLIGHT_LVDS IMX_GPIO_NR(1, 18)
	IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, WEAK_PULLUP),

	/* UART1 */
	/* UART2 */
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),

	/* USDHC3 - sdcard */
	IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT4__SD3_DATA4, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT5__SD3_DATA5, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT6__SD3_DATA6, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT7__SD3_DATA7, USDHC_PAD_CTRL),

	/* USDHC4 - sdcard */
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK, USDHC4_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD, USDHC4_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC4_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC4_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC4_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC4_PAD_CTRL),
#define GP_USDHC4_CD		IMX_GPIO_NR(2, 6)
	IOMUX_PAD_CTRL(NANDF_D6__GPIO2_IO06, WEAK_PULLUP), /* CD */
};

static const struct i2c_pads_info i2c_pads[] = {
	/* I2C1, SGTL5000, J15:pins 5-6 */
	I2C_PADS_INFO_ENTRY(I2C1, EIM_D21, 3, 21, EIM_D28, 3, 28, I2C_PAD_CTRL),
	/* I2C2, HDMI EDID, RTC */
	I2C_PADS_INFO_ENTRY(I2C2, KEY_COL3, 4, 12, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
	/* I2C3,  Touch screen, FDC6301 */
	I2C_PADS_INFO_ENTRY(I2C3, GPIO_5, 1, 05, GPIO_16, 7, 11, I2C_PAD_CTRL),
};
#define I2C_BUS_CNT	3

int board_ehci_hcd_init(int port)
{
	return 0;
}

#ifdef CONFIG_FSL_ESDHC_IMX
struct fsl_esdhc_cfg board_usdhc_cfg[] = {
	{.esdhc_base = USDHC4_BASE_ADDR, .bus_width = 4,
			.gp_cd = GP_USDHC4_CD},
	{.esdhc_base = USDHC3_BASE_ADDR, .bus_width = 8,},
};
#endif

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : -1;
}

void board_enable_lvds(const struct display_info_t *di, int enable)
{
	gpio_direction_output(GP_BACKLIGHT_LVDS, enable);
}

static const struct display_info_t displays[] = {
	VD_WXGA_J(LVDS, NULL, 0, 0x00),

	/* hdmi */
	VD_1280_720M_60(HDMI, fbp_detect_i2c, 1, 0x50),
	VD_1920_1080M_60(HDMI, NULL, 1, 0x50),
	VD_1024_768M_60(HDMI, NULL, 1, 0x50),
};
#define display_cnt	ARRAY_SIZE(displays)

static const unsigned short gpios_out_low[] = {
	GP_RGMII_PHY_RESET,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,
};

static const unsigned short gpios_in[] = {
	GP_BACKLIGHT_LVDS,
	GP_S2,
	GP_S4,
	GP_S3,
};

int board_early_init_f(void)
{
	set_gpios_in(gpios_in, ARRAY_SIZE(gpios_in));
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	SETUP_IOMUX_PADS(init_pads);
	return 0;
}

int board_init(void)
{
	common_board_init(i2c_pads, I2C_BUS_CNT, IOMUXC_GPR1_OTG_ID_GPIO1,
			displays, display_cnt, 0);
	return 0;
}

const struct button_key board_buttons[] = {
	{"S2",	GP_S2,	'2', 1},
	{"S4",	GP_S4,	'4', 1},
	{"S3",	GP_S3,	'3', 1},
	{NULL, 0, 0, 0},
};

#ifdef CONFIG_CMD_BMODE
const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif
