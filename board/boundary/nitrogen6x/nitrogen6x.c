// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 */

#include <common.h>
#include <command.h>
#include <env.h>
#include <init.h>
#include <net.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/sata.h>
#include <asm/mach-imx/spi.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <fsl_esdhc_imx.h>
#include <splash.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <usb/ehci-ci.h>
#include "spi_display.h"
#include "../common/bd_common.h"
#include "../common/padctrl.h"

DECLARE_GLOBAL_DATA_PTR;

#define BUTTON_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define RGB_PAD_CTRL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL	(PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

static const iomux_v3_cfg_t init_pads[] = {
/* bt_rfkill */
#define GP_BT_RFKILL_RESET	IMX_GPIO_NR(6, 16)
		IOMUX_PAD_CTRL(NANDF_CS3__GPIO6_IO16, WEAK_PULLDN),

		/* ECSPI1 */
		IOMUX_PAD_CTRL(EIM_D17__ECSPI1_MISO, SPI_PAD_CTRL),
		IOMUX_PAD_CTRL(EIM_D18__ECSPI1_MOSI, SPI_PAD_CTRL),
		IOMUX_PAD_CTRL(EIM_D16__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(3, 19)
		IOMUX_PAD_CTRL(EIM_D19__GPIO3_IO19, WEAK_PULLUP),

		/* ECSPI2 */
		IOMUX_PAD_CTRL(CSI0_DAT10__ECSPI2_MISO, SPI_PAD_CTRL),
		IOMUX_PAD_CTRL(CSI0_DAT9__ECSPI2_MOSI, SPI_PAD_CTRL),
		IOMUX_PAD_CTRL(CSI0_DAT8__ECSPI2_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI2_CS		IMX_GPIO_NR(5, 29)
		IOMUX_PAD_CTRL(CSI0_DAT11__GPIO5_IO29, WEAK_PULLUP), /* for spi displays */
#define GP_SPI_DISPLAY_RESET	IMX_GPIO_NR(4, 20)
		IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, WEAK_PULLUP),

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
		/* Sabrelite has different reset pin*/
#define GP_RGMII2_PHY_RESET	IMX_GPIO_NR(3, 23)
		IOMUX_PAD_CTRL(EIM_D23__GPIO3_IO23, WEAK_PULLUP),
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(1, 27)
		IOMUX_PAD_CTRL(ENET_RXD0__GPIO1_IO27, WEAK_PULLUP),
#define GPIRQ_ENET_PHY		IMX_GPIO_NR(1, 28)
		IOMUX_PAD_CTRL(ENET_TX_EN__GPIO1_IO28, WEAK_PULLUP),

		/* gpio_Keys - Button assignments for J14 */
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

		/* i2c1_isl1208 */
#define GPIRQ_RTC_ISL1208	IMX_GPIO_NR(6, 7)
		IOMUX_PAD_CTRL(NANDF_CLE__GPIO6_IO07, WEAK_PULLUP),

		/* i2c1_SGTL5000 sys_mclk */
		IOMUX_PAD_CTRL(GPIO_0__CCM_CLKO1, OUTPUT_40OHM),
		/* Needed if inappropriately used with SOM2 carrier board */
#define GP_SGTL5000_HP_MUTE	IMX_GPIO_NR(3, 29)		/* Low is muted */
		IOMUX_PAD_CTRL(EIM_D29__GPIO3_IO29, WEAK_PULLUP),

		/* i2c2 ov5640 mipi Camera controls */
#define GP_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(6, 9)
		IOMUX_PAD_CTRL(NANDF_WP_B__GPIO6_IO09, WEAK_PULLUP),

		/* i2c2 TC358743 interrupt */
#define GPIRQ_TC3587		IMX_GPIO_NR(2, 5)
		IOMUX_PAD_CTRL(NANDF_D5__GPIO2_IO05, WEAK_PULLDN),

		/* i2c2 ov5642 Camera controls, J5 */
		IOMUX_PAD_CTRL(GPIO_3__CCM_CLKO2, OUTPUT_40OHM), /* mclk */
#define GP_OV5642_POWER_DOWN	IMX_GPIO_NR(1, 6)
		IOMUX_PAD_CTRL(GPIO_6__GPIO1_IO06, WEAK_PULLUP),
#define GP_OV5642_RESET		IMX_GPIO_NR(1, 8)
		IOMUX_PAD_CTRL(GPIO_8__GPIO1_IO08, WEAK_PULLDN),

		/* PWM1 - Backlight on RGB connector: J15 */
#define GP_BACKLIGHT_RGB	IMX_GPIO_NR(1, 21)
		IOMUX_PAD_CTRL(SD1_DAT3__GPIO1_IO21, WEAK_PULLDN),

		/* PWM4 - Backlight on LVDS connector: J6 */
#define GP_BACKLIGHT_LVDS	IMX_GPIO_NR(1, 18)
		IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, WEAK_PULLDN),
#define GP_LVDS_BKL_EN		IMX_GPIO_NR(2, 0)
		IOMUX_PAD_CTRL(NANDF_D0__GPIO2_IO00, WEAK_PULLDN),

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
		IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, WEAK_PULLUP), IOMUX_PAD_CTRL(
				KEY_COL4__USB_OTG_OC, WEAK_PULLUP),

		/* USDHC2 - TiWi wl1271 */
		IOMUX_PAD_CTRL(SD2_CLK__SD2_CLK, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD2_CMD__SD2_CMD, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD2_DAT0__SD2_DATA0, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD2_DAT1__SD2_DATA1, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD2_DAT2__SD2_DATA2, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD2_DAT3__SD2_DATA3, USDHC_PAD_CTRL),

		/* USDHC3 - sdcard */
		IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
#define GP_USDHC3_CD		IMX_GPIO_NR(7, 0)
		IOMUX_PAD_CTRL(SD3_DAT5__GPIO7_IO00, WEAK_PULLUP),

		/* USDHC4 - sdcard */
		IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL),
		IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL),
#define GP_USDHC4_CD		IMX_GPIO_NR(2, 6)
		IOMUX_PAD_CTRL(NANDF_D6__GPIO2_IO06, WEAK_PULLUP),

		/* wl1271 */
#define GPIRQ_WL1271_WL		IMX_GPIO_NR(6, 14)
		IOMUX_PAD_CTRL(NANDF_CS1__GPIO6_IO14, WEAK_PULLDN), };

#ifdef CONFIG_CMD_FBPANEL
static const iomux_v3_cfg_t rgb666_pads[] = {
	IOMUX_PAD_CTRL(DI0_DISP_CLK__IPU1_DI0_DISP_CLK, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN15__IPU1_DI0_PIN15, RGB_PAD_CTRL),	/* DRDY */
	IOMUX_PAD_CTRL(DI0_PIN2__IPU1_DI0_PIN02, RGB_PAD_CTRL),		/* HSYNC */
	IOMUX_PAD_CTRL(DI0_PIN3__IPU1_DI0_PIN03, RGB_PAD_CTRL),		/* VSYNC */
	IOMUX_PAD_CTRL(DISP0_DAT0__IPU1_DISP0_DATA00, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT1__IPU1_DISP0_DATA01, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT2__IPU1_DISP0_DATA02, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT3__IPU1_DISP0_DATA03, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT4__IPU1_DISP0_DATA04, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT5__IPU1_DISP0_DATA05, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT6__IPU1_DISP0_DATA06, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT7__IPU1_DISP0_DATA07, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT8__IPU1_DISP0_DATA08, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT9__IPU1_DISP0_DATA09, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT10__IPU1_DISP0_DATA10, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT11__IPU1_DISP0_DATA11, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT12__IPU1_DISP0_DATA12, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT13__IPU1_DISP0_DATA13, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT14__IPU1_DISP0_DATA14, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT15__IPU1_DISP0_DATA15, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT16__IPU1_DISP0_DATA16, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT17__IPU1_DISP0_DATA17, RGB_PAD_CTRL),
};

static const iomux_v3_cfg_t rgb24_pads[] = {
	IOMUX_PAD_CTRL(DISP0_DAT18__IPU1_DISP0_DATA18, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT19__IPU1_DISP0_DATA19, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT20__IPU1_DISP0_DATA20, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT21__IPU1_DISP0_DATA21, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT22__IPU1_DISP0_DATA22, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT23__IPU1_DISP0_DATA23, RGB_PAD_CTRL),
};
#endif

static const iomux_v3_cfg_t rgb_gpio_pads[] = { IOMUX_PAD_CTRL(
		DI0_DISP_CLK__GPIO4_IO16, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DI0_PIN15__GPIO4_IO17, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DI0_PIN2__GPIO4_IO18, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DI0_PIN3__GPIO4_IO19, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT0__GPIO4_IO21, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT1__GPIO4_IO22, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT2__GPIO4_IO23, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT3__GPIO4_IO24, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT4__GPIO4_IO25, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT5__GPIO4_IO26, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT6__GPIO4_IO27, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT7__GPIO4_IO28, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT8__GPIO4_IO29, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT9__GPIO4_IO30, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT10__GPIO4_IO31, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT11__GPIO5_IO05, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT12__GPIO5_IO06, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT13__GPIO5_IO07, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT14__GPIO5_IO08, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT15__GPIO5_IO09, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT16__GPIO5_IO10, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT17__GPIO5_IO11, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT18__GPIO5_IO12, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT19__GPIO5_IO13, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT20__GPIO5_IO14, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT21__GPIO5_IO15, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT22__GPIO5_IO16, WEAK_PULLUP), IOMUX_PAD_CTRL(
		DISP0_DAT23__GPIO5_IO17, WEAK_PULLUP), };

static const struct i2c_pads_info i2c_pads[] = {
/* I2C1, SGTL5000 */
I2C_PADS_INFO_ENTRY(I2C1, EIM_D21, 3, 21, EIM_D28, 3, 28, I2C_PAD_CTRL),
/* I2C2 Camera, MIPI */
I2C_PADS_INFO_ENTRY(I2C2, KEY_COL3, 4, 12, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
/* I2C3, J15 - RGB connector */
I2C_PADS_INFO_ENTRY(I2C3, GPIO_5, 1, 05, GPIO_16, 7, 11, I2C_PAD_CTRL), };
#define I2C_BUS_CNT	3

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	if (port) {
		/* Reset USB hub */
		gpio_set_value(GP_USB_HUB_RESET, 0);
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
	{.esdhc_base = USDHC3_BASE_ADDR, .bus_width = 4,
			.gp_cd = GP_USDHC3_CD},
	{.esdhc_base = USDHC4_BASE_ADDR, .bus_width = 4,
			.gp_cd = GP_USDHC4_CD},
};
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	if (bus == 0 && cs == 0)
		return GP_ECSPI1_NOR_CS;
	if (bus == 1 && cs == 0)
		return GP_ECSPI2_CS;
	if (cs >> 8)
		return (cs >> 8);
	return -1;
}
#endif

#ifdef CONFIG_CMD_FBPANEL
void board_enable_lvds(const struct display_info_t *di, int enable)
{
	gpio_direction_output(GP_BACKLIGHT_LVDS, enable);
}

void board_enable_lcd(const struct display_info_t *di, int enable)
{
	if (enable) {
		SETUP_IOMUX_PADS(rgb666_pads);
		if ((di->pixfmt == IPU_PIX_FMT_RGB24) ||
		    (di->pixfmt == IPU_PIX_FMT_BGR24))
			SETUP_IOMUX_PADS(rgb24_pads);
#ifdef CONFIG_MXC_SPI_DISPLAY
		if (di->fbflags & FBF_SPI)
			enable_spi_rgb(di);
#endif
		mdelay(100); /* let panel sync up before enabling backlight */
		gpio_direction_output(GP_BACKLIGHT_RGB, enable);
	} else {
		gpio_direction_output(GP_BACKLIGHT_RGB, enable);
		SETUP_IOMUX_PADS(rgb_gpio_pads);
	}
}

void board_pre_enable(const struct display_info_t *di)
{
	SETUP_IOMUX_PADS(rgb666_pads);
	enable_spi_rgb(di);
}

static const struct display_info_t displays[] = {
#ifdef CONFIG_DEFAULT_DISPLAY_WXGA
	VD_WXGA(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
#endif
	/* hdmi */
	VD_1280_720M_60(HDMI, fbp_detect_i2c, 1, 0x50),
	VD_1920_1080M_60(HDMI, NULL, 1, 0x50),
	VD_1024_768M_60(HDMI, NULL, 1, 0x50),
	VD_640_480M_60(HDMI, NULL, 1, 0x50),
	VD_720_480M_60(HDMI, NULL, 1, 0x50),

	/* ft5x06 */
	VD_HANNSTAR7(LVDS, fbp_detect_i2c, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_AUO_B101EW05(LVDS, NULL, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_LG1280_800(LVDS, NULL, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_M101NWWB(LVDS, NULL, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_DT070BTFT(LVDS, NULL, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_WSVGA(LVDS, NULL, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_TM070JDHG30(LVDS, NULL, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),

	/* ili210x */
	VD_AMP1024_600(LVDS, fbp_detect_i2c, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x41, FBTS_ILI210X),

	/* egalax_ts */
	VD_HANNSTAR(LVDS, fbp_detect_i2c, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x04, FBTS_EGALAX),
	VD_LG9_7(LVDS, NULL, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x04, FBTS_EGALAX),

	/* fusion7 specific touchscreen */
	VD_FUSION7(LCD, fbp_detect_i2c, 2, 0x10, FBTS_FUSION7),

	VD_SHARP_LQ101K1LY04(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_WXGA_J(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
#ifndef CONFIG_DEFAULT_DISPLAY_WXGA
	VD_WXGA(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
#endif
	VD_WVGA(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_AA065VE11(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_VGA(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),

	/* tsc2004 */
	VD_CLAA_WVGA(LCD, fbp_detect_i2c, 2, 0x48, FBTS_TSC2004),
	VD_SHARP_WVGA(LCD, NULL, 2, 0x48, FBTS_TSC2004),
	VD_DC050WX(LCD, NULL, 2, 0x48, FBTS_TSC2004),
	VD_QVGA(LCD, NULL, 2, 0x48, FBTS_TSC2004),
	VD_DT035BTFT(LCD, NULL, 2, 0x48, FBTS_TSC2004),
	VD_AT035GT_07ET3(LCD, NULL, 2, 0x48, FBTS_TSC2004),

	VD_LSA40AT9001(LCD, NULL, 0, 0x00),
#ifdef CONFIG_MXC_SPI_DISPLAY
	VD_AUO_G050(LCD, NULL, 1, 0),
	VD_A030JN01_UPS051(LCD, NULL, 1, 2),
	VD_A030JN01_YUV720(LCD, NULL, 1, 1),
	VD_KD024FM(LCD, NULL, 2, 3),
#endif
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

static const unsigned short gpios_out_low[] = {
GP_RGMII2_PHY_RESET,
GP_RGMII_PHY_RESET,
GP_BACKLIGHT_LVDS,
GP_BACKLIGHT_RGB,
/* Disable wl1271 */
GP_REG_WLAN_EN,
GP_BT_RFKILL_RESET,
GP_REG_USBOTG,
GP_OV5642_RESET,
GP_USB_HUB_RESET, };

static const unsigned short gpios_out_high[] = {
GP_ECSPI1_NOR_CS,
GP_SGTL5000_HP_MUTE,
GP_OV5642_POWER_DOWN,
GP_OV5640_MIPI_POWER_DOWN, };

static const unsigned short gpios_in[] = {
GP_GPIOKEY_BACK,
GP_GPIOKEY_HOME,
GP_GPIOKEY_MENU,
GP_GPIOKEY_POWER,
GP_GPIOKEY_VOL_DOWN,
GP_GPIOKEY_VOL_UP,
GPIRQ_ENET_PHY,
GPIRQ_RTC_ISL1208,
GPIRQ_TC3587,
GP_LVDS_BKL_EN,
GPIRQ_WL1271_WL,
GP_USDHC3_CD,
GP_USDHC4_CD, };

int board_early_init_f(void) {
	set_gpios_in(gpios_in, ARRAY_SIZE(gpios_in));
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	SETUP_IOMUX_PADS(init_pads);
	SETUP_IOMUX_PADS(rgb_gpio_pads);
	return 0;
}

#define ISL1208_ADDRESS	0x6f

void board_env_init(void) {
#ifdef CONFIG_DM_I2C
	struct udevice *bus;
	struct udevice *i2c_dev;
#else
	u8 orig_i2c_bus;
#endif
	int ret;

#ifdef CONFIG_DM_I2C
	ret = uclass_get_device_by_seq(UCLASS_I2C, 0, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return;
	}

	ret = dm_i2c_probe(bus, ISL1208_ADDRESS, 0, &i2c_dev);
#else
	orig_i2c_bus = i2c_get_bus_num();
	ret = i2c_set_bus_num(0);
	if (ret == 0)
		ret = i2c_probe(ISL1208_ADDRESS);
	i2c_set_bus_num(orig_i2c_bus);
#endif
	if (!ret) {
		/* the dts assumes a rv4162 not isl1208 */
		env_set("cmd_board", "fdt set rtc_isl1208 status okay;"
				"fdt set rtc_rv4162 status disabled");
	}
}

int board_init(void) {
	gpio_request(GP_BACKLIGHT_RGB, "rgb backlight");
	gpio_request(GP_BACKLIGHT_LVDS, "lvds backlight");
	gpio_request(GP_REG_USBOTG, "usbotg power");
	gpio_request(GP_USB_HUB_RESET, "usbh1 hub reset");
	gpio_request(GP_GPIOKEY_BACK, "back");
	gpio_request(GP_GPIOKEY_HOME, "home");
	gpio_request(GP_GPIOKEY_MENU, "menu");
	gpio_request(GP_GPIOKEY_POWER, "power");
	gpio_request(GP_GPIOKEY_VOL_UP, "volup");
	gpio_request(GP_GPIOKEY_VOL_DOWN, "voldown");

	common_board_init(i2c_pads, I2C_BUS_CNT, IOMUXC_GPR1_OTG_ID_GPIO1,
	displays, display_cnt, 0);
	return 0;
}

#ifndef CONFIG_SYS_BOARD
const char* board_get_board_type(void) {
	if (gpio_get_value(GPIRQ_WL1271_WL))
		return "nitrogen6x";
	return "sabrelite";
}
#endif

const struct button_key board_buttons[] =
		{ { "back", GP_GPIOKEY_BACK, 'B', 1 }, { "home",
				GP_GPIOKEY_HOME, 'H', 1 }, { "menu",
				GP_GPIOKEY_MENU, 'M', 1 }, { "search",
				GP_GPIOKEY_POWER, 'S', 1 }, { "volup",
				GP_GPIOKEY_VOL_UP, 'V', 1 }, { "voldown",
				GP_GPIOKEY_VOL_DOWN, 'v', 1 }, { NULL, 0, 0, 0 }, };

#ifdef CONFIG_CMD_BMODE
const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif
