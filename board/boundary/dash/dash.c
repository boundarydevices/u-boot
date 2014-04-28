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
#include <asm/mach-imx/sata.h>
#include <asm/mach-imx/spi.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <i2c.h>
#include <input.h>
#include <splash.h>
#include <usb/ehci-ci.h>
#include "../common/bd_common.h"
#include "../common/padctrl.h"

DECLARE_GLOBAL_DATA_PTR;

#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

/*
 *
 */
static iomux_v3_cfg_t const init_pads[] = {
	/* bt_rfkill */
#define GP_BT_RFKILL_RESET	IMX_GPIO_NR(6, 16)
	IOMUX_PAD_CTRL(NANDF_CS3__GPIO6_IO16, WEAK_PULLDN),

	/* ECSPI1 */
	IOMUX_PAD_CTRL(EIM_D17__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D18__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D16__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(EIM_D19__GPIO3_IO19,  WEAK_PULLUP),

	/* USB otg power */
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, OUTPUT_40OHM),
	IOMUX_PAD_CTRL(NANDF_D5__GPIO2_IO05, OUTPUT_40OHM),
	IOMUX_PAD_CTRL(NANDF_WP_B__GPIO6_IO09, OUTPUT_40OHM),
	IOMUX_PAD_CTRL(GPIO_8__GPIO1_IO08, OUTPUT_40OHM),
	IOMUX_PAD_CTRL(GPIO_6__GPIO1_IO06, OUTPUT_40OHM),

	/* gpio_Keys - Button assignments for J14 */
	/* Menu */
#define GP_GPIOKEY_BACK		IMX_GPIO_NR(2, 2)
	IOMUX_PAD_CTRL(NANDF_D2__GPIO2_IO02, BUTTON_PAD_CTRL),
#define GP_GPIOKEY_HOME		IMX_GPIO_NR(2, 4)
	IOMUX_PAD_CTRL(NANDF_D4__GPIO2_IO04, BUTTON_PAD_CTRL),
#define GP_GPIOKEY_MENU		IMX_GPIO_NR(2, 1)
	IOMUX_PAD_CTRL(NANDF_D1__GPIO2_IO01, BUTTON_PAD_CTRL),
	/* Labeled Search (mapped to Power under Android) */
#define GP_GPIOKEY_POWER	IMX_GPIO_NR(2, 3)
	IOMUX_PAD_CTRL(NANDF_D3__GPIO2_IO03, BUTTON_PAD_CTRL),
#define GP_GPIOKEY_VOL_DOWN	IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(GPIO_19__GPIO4_IO05, BUTTON_PAD_CTRL),
#define GP_GPIOKEY_VOL_UP	IMX_GPIO_NR(7, 13)
	IOMUX_PAD_CTRL(GPIO_18__GPIO7_IO13, BUTTON_PAD_CTRL),

	/* PWM4 - Backlight on LVDS connector: J6 */
#define GP_BACKLIGHT_LVDS	IMX_GPIO_NR(1, 18)
	IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, WEAK_PULLDN),
	IOMUX_PAD_CTRL(EIM_DA2__GPIO3_IO02, WEAK_PULLUP),	/* LVDS_BCKLT_EN */
	IOMUX_PAD_CTRL(EIM_DA1__GPIO3_IO01, WEAK_PULLUP),	/* LVDS_PWR_EN */

	/* reg_usbotg_vbus */
#define GP_REG_USBOTG		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLDN),

	/* reg_wlan_en */
#define GP_REG_WLAN_EN		IMX_GPIO_NR(6, 15)
	IOMUX_PAD_CTRL(NANDF_CS2__GPIO6_IO15, WEAK_PULLDN),

	/* UART1 */
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),

	/* UART2 */
#ifndef CONFIG_SILENT_UART
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),
#else
	IOMUX_PAD_CTRL(EIM_D26__GPIO3_IO26, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__GPIO3_IO27, UART_PAD_CTRL),
#endif

	/* USBH1 */
	IOMUX_PAD_CTRL(EIM_D30__USB_H1_OC, WEAK_PULLUP),
#define GP_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
	IOMUX_PAD_CTRL(GPIO_17__GPIO7_IO12, WEAK_PULLDN),

	/* USBOTG */
	IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, WEAK_PULLUP),
	IOMUX_PAD_CTRL(KEY_COL4__USB_OTG_OC, WEAK_PULLUP),

	/* USDHC2 - TiWi wl1271 */
	IOMUX_PAD_CTRL(SD2_CLK__SD2_CLK,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__SD2_CMD,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT0__SD2_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT1__SD2_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT2__SD2_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT3__SD2_DATA3, USDHC_PAD_CTRL),

	/* USDHC4 - sdcard */
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL),
#define GP_USDHC4_CD		IMX_GPIO_NR(2, 6)
	IOMUX_PAD_CTRL(NANDF_D6__GPIO2_IO06,    WEAK_PULLUP),

	/* wl1271 pads  */
#define GPIRQ_WL1271_WL	IMX_GPIO_NR(6, 14)
	IOMUX_PAD_CTRL(NANDF_CS1__GPIO6_IO14, WEAK_PULLDN),
};

static const struct i2c_pads_info i2c_pads[] = {
	/* I2C1, ISL1208 RTC */
	I2C_PADS_INFO_ENTRY(I2C1, EIM_D21, 3, 21, EIM_D28, 3, 28, I2C_PAD_CTRL),
	/* I2C2 MPU-9150, BH1780GLI */
	I2C_PADS_INFO_ENTRY(I2C2, KEY_COL3, 4, 12, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
	/* I2C3, J15 - LVDS EDID on J4 */
	I2C_PADS_INFO_ENTRY(I2C3, GPIO_5, 1, 05, GPIO_16, 7, 11, I2C_PAD_CTRL),
};
#define I2C_BUS_CNT	3

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	if (port) {
		/* Reset USB hub */
		gpio_direction_output(GP_USB_HUB_RESET, 0);
		mdelay(2);
		gpio_set_value(GP_USB_HUB_RESET, 1);
	}
	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port)
		return 0;
	gpio_set_value(GP_REG_USBOTG, on);
	return 0;
}

#endif

#ifdef CONFIG_FSL_ESDHC_IMX
struct fsl_esdhc_cfg board_usdhc_cfg[] = {
	{.esdhc_base = USDHC4_BASE_ADDR, .bus_width = 4,
			.gp_cd = GP_USDHC4_CD},
};
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : -1;
}
#endif

void board_enable_lvds(const struct display_info_t *di, int enable)
{
	gpio_direction_output(GP_BACKLIGHT_LVDS, enable);
}

static const struct display_info_t displays[] = {
	VD_WXGA_J(LVDS, NULL, 0, 0x00),
	VD_WXGA(LVDS, NULL, 0, 0x00),
	VD_HANNSTAR(LVDS, fbp_detect_i2c, 2, 0x04, FBTS_EGALAX),
	VD_LG9_7(LVDS, NULL, 2, 0x04),
	VD_WSVGA(LVDS, fbp_detect_i2c, 2, 0x38, FBTS_FT5X06),
};
#define display_cnt	ARRAY_SIZE(displays)

static const unsigned short gpios_out_low[] = {
	/* Disable wl1271 */
	GP_REG_WLAN_EN,
	GP_BT_RFKILL_RESET,
	GP_REG_USBOTG,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,
};

static const unsigned short gpios_in[] = {
	GP_BACKLIGHT_LVDS,
	GPIRQ_WL1271_WL,
	GP_USDHC4_CD,
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
	{"back",	GP_GPIOKEY_BACK,	'B', 1},
	{"home",	GP_GPIOKEY_HOME,	'H', 1},
	{"menu",	GP_GPIOKEY_MENU,	'M', 1},
	{"search",	GP_GPIOKEY_POWER,	'S', 1},
	{"volup",	GP_GPIOKEY_VOL_UP,	'V', 1},
	{"voldown",	GP_GPIOKEY_VOL_DOWN,	'v', 1},
	{NULL, 0, 0, 0},
};

#ifdef CONFIG_CMD_BMODE
const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif
