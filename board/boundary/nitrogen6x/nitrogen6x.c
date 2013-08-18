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
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/fbpanel.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/spi.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <splash.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <netdev.h>
#include <usb/ehci-ci.h>
#include "spi_display.h"

DECLARE_GLOBAL_DATA_PTR;

#define BUTTON_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define OUTPUT_40OHM	(PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define RGB_PAD_CTRL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL	(PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define WEAK_PULLDN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLDN_OUTPUT (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP_OUTPUT (PAD_CTL_PUS_100K_UP |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_SLOW)

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
	IOMUX_PAD_CTRL(CSI0_DAT11__GPIO5_IO29, WEAK_PULLUP),	/* for spi displays */
#define GP_SPI_DISPLAY_RESET	IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, WEAK_PULLUP),

	/* ENET pads that don't change for PHY reset */
	IOMUX_PAD_CTRL(ENET_MDIO__ENET_MDIO, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(ENET_MDC__ENET_MDC, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TXC__RGMII_TXC, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TD0__RGMII_TD0, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TD1__RGMII_TD1, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TD2__RGMII_TD2, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TD3__RGMII_TD3, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TX_CTL__RGMII_TX_CTL, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(ENET_REF_CLK__ENET_TX_CLK, ENET_PAD_CTRL),
	/* pin 42 PHY nRST */
#define GP_ENET_PHY_RESET_SABRELITE	IMX_GPIO_NR(3, 23)
	IOMUX_PAD_CTRL(EIM_D23__GPIO3_IO23, WEAK_PULLUP),
#define GP_ENET_PHY_RESET_NITROGEN6X	IMX_GPIO_NR(1, 27)
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
	IOMUX_PAD_CTRL(GPIO_3__CCM_CLKO2, OUTPUT_40OHM),	/* mclk */
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
	IOMUX_PAD_CTRL(NANDF_CS1__GPIO6_IO14, WEAK_PULLDN),
};

static const iomux_v3_cfg_t enet_ksz9021_gpio_pads[] = {
	/* pin 35 - 1 (PHY_AD2) on reset */
#define GP_KSZ9021_AD2		IMX_GPIO_NR(6, 30)
	IOMUX_PAD_CTRL(RGMII_RXC__GPIO6_IO30, WEAK_PULLUP_OUTPUT),
	/* pin 32 - 1 - (MODE0) all */
#define GP_KSZ9021_MODE0		IMX_GPIO_NR(6, 25)
	IOMUX_PAD_CTRL(RGMII_RD0__GPIO6_IO25, WEAK_PULLUP_OUTPUT),
	/* pin 31 - 1 - (MODE1) all */
#define GP_KSZ9021_MODE1		IMX_GPIO_NR(6, 27)
	IOMUX_PAD_CTRL(RGMII_RD1__GPIO6_IO27, WEAK_PULLUP_OUTPUT),
	/* pin 28 - 1 - (MODE2) all */
#define GP_KSZ9021_MODE2		IMX_GPIO_NR(6, 28)
	IOMUX_PAD_CTRL(RGMII_RD2__GPIO6_IO28, WEAK_PULLUP_OUTPUT),
	/* pin 27 - 1 - (MODE3) all */
#define GP_KSZ9021_MODE3		IMX_GPIO_NR(6, 29)
	IOMUX_PAD_CTRL(RGMII_RD3__GPIO6_IO29, WEAK_PULLUP_OUTPUT),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
#define GP_KSZ9021_CLK125		IMX_GPIO_NR(6, 24)
	IOMUX_PAD_CTRL(RGMII_RX_CTL__GPIO6_IO24, WEAK_PULLUP_OUTPUT),
};

static const iomux_v3_cfg_t enet_ar8035_gpio_pads[] = {
	/* pin 31 - 1 (1P8_SEL) on reset */
#define GP_AR8035_1P8_SEL		IMX_GPIO_NR(6, 30)
	IOMUX_PAD_CTRL(RGMII_RXC__GPIO6_IO30, WEAK_PULLUP_OUTPUT),
	/* pin 29 - 0 - AD0 */
#define GP_AR8035_AD0		IMX_GPIO_NR(6, 25)
	IOMUX_PAD_CTRL(RGMII_RD0__GPIO6_IO25, WEAK_PULLDN_OUTPUT),
	/* pin 28 - 1 - AD1 */
#define GP_AR8035_AD1		IMX_GPIO_NR(6, 27)
	IOMUX_PAD_CTRL(RGMII_RD1__GPIO6_IO27, WEAK_PULLUP_OUTPUT),

	/* mode = 1100 - plloff mode */
	/* pin 26 - 0 - MODE1 */
#define GP_AR8035_MODE1		IMX_GPIO_NR(6, 28)
	IOMUX_PAD_CTRL(RGMII_RD2__GPIO6_IO28, WEAK_PULLDN_OUTPUT),
	/* pin 25 - 1 - MODE3 */
#define GP_AR8035_MODE3		IMX_GPIO_NR(6, 29)
	IOMUX_PAD_CTRL(RGMII_RD3__GPIO6_IO29, WEAK_PULLUP_OUTPUT),
	/* pin 30 - 0 - MODE0 */
#define GP_AR8035_MODE0		IMX_GPIO_NR(6, 24)
	IOMUX_PAD_CTRL(RGMII_RX_CTL__GPIO6_IO24, WEAK_PULLDN_OUTPUT),
};

static const iomux_v3_cfg_t enet_pads[] = {
	IOMUX_PAD_CTRL(RGMII_RXC__RGMII_RXC, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD0__RGMII_RD0, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD1__RGMII_RD1, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD2__RGMII_RD2, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD3__RGMII_RD3, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RX_CTL__RGMII_RX_CTL, ENET_PAD_CTRL),
};

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

static const iomux_v3_cfg_t rgb_gpio_pads[] = {
	IOMUX_PAD_CTRL(DI0_DISP_CLK__GPIO4_IO16, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DI0_PIN15__GPIO4_IO17, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DI0_PIN2__GPIO4_IO18, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DI0_PIN3__GPIO4_IO19, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT0__GPIO4_IO21, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT1__GPIO4_IO22, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT2__GPIO4_IO23, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT3__GPIO4_IO24, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT4__GPIO4_IO25, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT5__GPIO4_IO26, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT6__GPIO4_IO27, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT7__GPIO4_IO28, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT8__GPIO4_IO29, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT9__GPIO4_IO30, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT10__GPIO4_IO31, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT11__GPIO5_IO05, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT12__GPIO5_IO06, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT13__GPIO5_IO07, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT14__GPIO5_IO08, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT15__GPIO5_IO09, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT16__GPIO5_IO10, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT17__GPIO5_IO11, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT18__GPIO5_IO12, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT19__GPIO5_IO13, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT20__GPIO5_IO14, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT21__GPIO5_IO15, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT22__GPIO5_IO16, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT23__GPIO5_IO17, WEAK_PULLUP),
};

static struct i2c_pads_info i2c_pads[] = {
	/* I2C1, SGTL5000 */
	I2C_PADS_INFO_ENTRY(I2C1, EIM_D21, 3, 21, EIM_D28, 3, 28, I2C_PAD_CTRL),
	/* I2C2 Camera, MIPI */
	I2C_PADS_INFO_ENTRY(I2C2, KEY_COL3, 4, 12, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
	/* I2C3, J15 - RGB connector */
	I2C_PADS_INFO_ENTRY(I2C3, GPIO_5, 1, 05, GPIO_16, 7, 11, I2C_PAD_CTRL),
};

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);
	return 0;
}

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

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{.esdhc_base = USDHC3_BASE_ADDR, .max_bus_width = 4},
	{.esdhc_base = USDHC4_BASE_ADDR, .max_bus_width = 4},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int gp_cd = (cfg->esdhc_base == USDHC3_BASE_ADDR) ? GP_USDHC3_CD :
			GP_USDHC4_CD;

	return !gpio_get_value(gp_cd);
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
		case 1:
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) then supported by the board (%d)\n",
				index + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (ret)
			return ret;
	}
	return 0;
}
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

static void setup_iomux_enet(int ksz)
{
	gpio_direction_output(GP_ENET_PHY_RESET_SABRELITE, 0);
	gpio_direction_output(GP_ENET_PHY_RESET_NITROGEN6X, 0);
	if (ksz) {
		gpio_direction_output(GP_KSZ9021_AD2, 1);
		gpio_direction_output(GP_KSZ9021_MODE0, 1);
		gpio_direction_output(GP_KSZ9021_MODE1, 1);
		gpio_direction_output(GP_KSZ9021_MODE2, 1);
		gpio_direction_output(GP_KSZ9021_MODE3, 1);
		gpio_direction_output(GP_KSZ9021_CLK125, 1);
		SETUP_IOMUX_PADS(enet_ksz9021_gpio_pads);
	} else {
		gpio_direction_output(GP_AR8035_1P8_SEL, 1);
		gpio_direction_output(GP_AR8035_AD0, 0);
		gpio_direction_output(GP_AR8035_AD1, 1);
		gpio_direction_output(GP_AR8035_MODE1, 0);
		gpio_direction_output(GP_AR8035_MODE3, 1);
		gpio_direction_output(GP_AR8035_MODE0, 0);
		SETUP_IOMUX_PADS(enet_ar8035_gpio_pads);
	}

	/* Need delay 10ms according to KSZ9021 spec */
	udelay(1000 * 10);
	gpio_set_value(GP_ENET_PHY_RESET_SABRELITE, 1);
	gpio_set_value(GP_ENET_PHY_RESET_NITROGEN6X, 1);
	/* strap hold time for AR8035, 5 fails, 6 works, so 12 should be safe */
	udelay(12);

	SETUP_IOMUX_PADS(enet_pads);
	udelay(100);	/* Wait 100 us before using mii interface */
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->uid == 0x221610) {
		setup_iomux_enet(1);
		/* min rx data delay */
		ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
		/* min tx data delay */
		ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
		/* max rx/tx clock delay, min rx/tx control */
		ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);
	}
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet(0);

#ifdef CONFIG_FEC_MXC
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return -EINVAL;
	/* scan phy 4,5,6,7 */
	phydev = phy_find_by_mask(bus, (0xf << 4), PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		ret = -EINVAL;
		goto free_bus;
	}
	printf("%s at %d\n", phydev->drv->name, phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret)
		goto free_phydev;
#endif

#ifdef CONFIG_CI_UDC
	/* For otg ethernet*/
	if (!getenv("eth1addr"))
		setenv("eth1addr", getenv("usbnet_devaddr"));
	usb_eth_initialize(bis);
#endif
	return 0;

free_phydev:
	free(phydev);
free_bus:
	free(bus);
	return ret;
}


int splash_screen_prepare(void)
{
	char *env_loadsplash;

	if (!getenv("splashimage") || !getenv("splashsize")) {
		return -1;
	}

	env_loadsplash = getenv("loadsplash");
	if (env_loadsplash == NULL) {
		printf("Environment variable loadsplash not found!\n");
		return -1;
	}

	if (run_command_list(env_loadsplash, -1, 0)) {
		printf("failed to run loadsplash %s\n\n", env_loadsplash);
		return -1;
	}

	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
void board_enable_lvds(const struct display_info_t *di, int enable)
{
	gpio_direction_output(GP_BACKLIGHT_LVDS, enable);
}

void board_enable_lcd(const struct display_info_t *di, int enable)
{
	if (enable) {
		SETUP_IOMUX_PADS(rgb666_pads);
		if (di->pixfmt == IPU_PIX_FMT_RGB24)
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
	/* hdmi */
	VD_1280_720M_60(HDMI, fbp_detect_i2c, 1, 0x50),
	VD_1920_1080M_60(HDMI, NULL, 1, 0x50),
	VD_1024_768M_60(HDMI, NULL, 1, 0x50),
	VD_640_480M_60(HDMI, NULL, 1, 0x50),
	VD_720_480M_60(HDMI, NULL, 1, 0x50),

	/* ft5x06 */
	VD_HANNSTAR7(LVDS, fbp_detect_i2c, 2, 0x38),
	VD_AUO_B101EW05(LVDS, NULL, 2, 0x38),
	VD_LG1280_800(LVDS, NULL, 2, 0x38),
	VD_DT070BTFT(LVDS, NULL, 2, 0x38),
	VD_WSVGA(LVDS, NULL, 2, 0x38),
	VD_TM070JDHG30(LVDS, NULL, 2, 0x38),

	/* ili210x */
	VD_AMP1024_600(LVDS, fbp_detect_i2c, 2, 0x41),

	/* egalax_ts */
	VD_HANNSTAR(LVDS, fbp_detect_i2c, 2, 0x04),
	VD_LG9_7(LVDS, NULL, 2, 0x04),

	/* fusion7 specific touchscreen */
	VD_FUSION7(LCD, fbp_detect_i2c, 2, 0x10),

	VD_SHARP_LQ101K1LY04(LVDS, NULL, 0, 0x00),
	VD_WXGA_J(LVDS, NULL, 0, 0x00),
	VD_WXGA(LVDS, NULL, 0, 0x00),
	VD_WVGA(LVDS, NULL, 0, 0x00),
	VD_AA065VE11(LVDS, NULL, 0, 0x00),
	VD_VGA(LVDS, NULL, 0, 0x00),

	/* tsc2004 */
	VD_CLAA_WVGA(LCD, fbp_detect_i2c, 2, 0x48),
	VD_SHARP_WVGA(LCD, NULL, 2, 0x48),
	VD_DC050WX(LCD, NULL, 2, 0x48),
	VD_QVGA(LCD, NULL, 2, 0x48),
	VD_AT035GT_07ET3(LCD, NULL, 2, 0x48),

	VD_LSA40AT9001(LCD, NULL, 0, 0x00),
#ifdef CONFIG_MXC_SPI_DISPLAY
	VD_AUO_G050(LCD, NULL, 1, 0),
	VD_A030JN01_UPS051(LCD, NULL, 1, 2),
	VD_A030JN01_YUV720(LCD, NULL, 1, 1),
	VD_KD024FM(LCD, NULL, 2, 3),
#endif
};

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}
#endif

static const unsigned short gpios_out_low[] = {
	GP_BACKLIGHT_LVDS,
	GP_BACKLIGHT_RGB,
	/* Disable wl1271 */
	GP_REG_WLAN_EN,
	GP_BT_RFKILL_RESET,
	GP_REG_USBOTG,
	GP_OV5642_RESET,
	GP_USB_HUB_RESET,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,
	GP_SGTL5000_HP_MUTE,
	GP_OV5642_POWER_DOWN,
	GP_OV5640_MIPI_POWER_DOWN,
};

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
	GPIRQ_WL1271_WL,
	GP_USDHC3_CD,
	GP_USDHC4_CD,
};

static void set_gpios_in(const unsigned short *p, int cnt)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_input(*p++);
}

static void set_gpios(const unsigned short *p, int cnt, int val)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_output(*p++, val);
}

int board_early_init_f(void)
{
	set_gpios_in(gpios_in, ARRAY_SIZE(gpios_in));
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	SETUP_IOMUX_PADS(init_pads);
	SETUP_IOMUX_PADS(rgb_gpio_pads);
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_init(void)
{
	int i;
	struct i2c_pads_info *p = i2c_pads + i2c_get_info_entry_offset();
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	for (i = 0; i < 3; i++) {
	        setup_i2c(i, CONFIG_SYS_I2C_SPEED, 0x7f, p);
		p += I2C_PADS_INFO_ENTRY_SPACING;
	}
#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, ARRAY_SIZE(displays));
#endif
	return 0;
}

static char const *board_type = "uninitialized";

int checkboard(void)
{
#if defined(CONFIG_NITROGEN6X_FL)
	puts("Board: Nitrogen6X_fl\n");
	board_type = "nitrogen6x_fl";
#elif defined(CONFIG_BOARD_NAME) && defined(CONFIG_BOARD_TYPE)
	puts("Board: " CONFIG_BOARD_NAME "\n");
	board_type = CONFIG_BOARD_TYPE ;
#else
	if (gpio_get_value(GPIRQ_WL1271_WL)) {
		puts("Board: Nitrogen6X\n");
		board_type = "nitrogen6x";
	}
	else {
		puts("Board: SABRE Lite\n");
		board_type = "sabrelite";
	}
#endif
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"back",	GP_GPIOKEY_BACK,	'B'},
	{"home",	GP_GPIOKEY_HOME,	'H'},
	{"menu",	GP_GPIOKEY_MENU,	'M'},
	{"search",	GP_GPIOKEY_POWER,	'S'},
	{"volup",	GP_GPIOKEY_VOL_UP,	'V'},
	{"voldown",	GP_GPIOKEY_VOL_DOWN,	'v'},
};

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed
 */
static int read_keys(char *buf)
{
	int i, numpressed = 0;
	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (!gpio_get_value(buttons[i].gpnum))
			buf[numpressed++] = buttons[i].ident;
	}
	buf[numpressed] = '\0';
	return numpressed;
}

static int do_kbd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char envvalue[ARRAY_SIZE(buttons)+1];
	int numpressed = read_keys(envvalue);
	setenv("keybd", envvalue);
	return numpressed == 0;
}

U_BOOT_CMD(
	kbd, 1, 1, do_kbd,
	"Tests for keypresses, sets 'keybd' environment variable",
	"Returns 0 (true) to shell if key is pressed."
);

#ifdef CONFIG_PREBOOT
static char const kbd_magic_prefix[] = "key_magic";
static char const kbd_command_prefix[] = "key_cmd";

static void preboot_keys(void)
{
	int numpressed;
	char keypress[ARRAY_SIZE(buttons)+1];
	numpressed = read_keys(keypress);
	if (numpressed) {
		char *kbd_magic_keys = getenv("magic_keys");
		char *suffix;
		/*
		 * loop over all magic keys
		 */
		for (suffix = kbd_magic_keys; *suffix; ++suffix) {
			char *keys;
			char magic[sizeof(kbd_magic_prefix) + 1];
			sprintf(magic, "%s%c", kbd_magic_prefix, *suffix);
			keys = getenv(magic);
			if (keys) {
				if (!strcmp(keys, keypress))
					break;
			}
		}
		if (*suffix) {
			char cmd_name[sizeof(kbd_command_prefix) + 1];
			char *cmd;
			sprintf(cmd_name, "%s%c", kbd_command_prefix, *suffix);
			cmd = getenv(cmd_name);
			if (cmd) {
				setenv("preboot", cmd);
				return;
			}
		}
	}
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif

int misc_init_r(void)
{
#ifdef CONFIG_PREBOOT
	preboot_keys();
#endif

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	setenv_hex("reset_cause", get_imx_reset_cause());
	return 0;
}

int board_late_init(void)
{
	unsigned char mac_address[8];
	char macbuf[18];
	int cpurev = get_cpu_rev();

	setenv("cpu", get_imx_type((cpurev & 0xFF000) >> 12));
	if (!getenv("board"))
		setenv("board", board_type);
	setenv("uboot_defconfig", CONFIG_DEFCONFIG);
	if (!getenv("wlmac")) {
		imx_get_mac_from_fuse(0x800000, mac_address);
		if (is_valid_ethaddr(mac_address)) {
			snprintf(macbuf, sizeof(macbuf), "%pM", mac_address);
			setenv("wlmac", macbuf);
		}
	}
	return 0;
}
