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
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/fbpanel.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/spi.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <netdev.h>
#include <usb/ehci-fsl.h>

/* Special MXCFB sync flags are here. */
#include "../drivers/video/mxcfb.h"

DECLARE_GLOBAL_DATA_PTR;

#define AUD_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS)

#define CEC_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_PUS_22K_UP | PAD_CTL_ODE | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define CSI_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define ENET_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define HIGH_Z_SLOW	(PAD_CTL_HYS|PAD_CTL_SPEED_LOW | PAD_CTL_DSE_DISABLE)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define OUTPUT_40OHM	(PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)

#define RGB_PAD_CTRL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_CLK_PAD_CTRL (PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm | \
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL	(USDHC_CLK_PAD_CTRL | PAD_CTL_PUS_47K_UP)

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

/*
 *
 */
static const iomux_v3_cfg_t init_pads[] = {
	/* AUDMUX */
	IOMUX_PAD_CTRL(CSI0_DAT7__AUD3_RXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT4__AUD3_TXC, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT5__AUD3_TXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT6__AUD3_TXFS, AUD_PAD_CTRL),

	/* ECSPI1 */
	IOMUX_PAD_CTRL(EIM_D17__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D18__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D16__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_CS1	IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(EIM_D19__GPIO3_IO19, WEAK_PULLUP), /* SS1 */

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
#define GP_ENET_PHY_RESET	IMX_GPIO_NR(1, 27)
	IOMUX_PAD_CTRL(ENET_RXD0__GPIO1_IO27, OUTPUT_40OHM),
#define GP_ENET_PHY_INT		IMX_GPIO_NR(1, 28)
	IOMUX_PAD_CTRL(ENET_TX_EN__GPIO1_IO28, WEAK_PULLUP),	/* Micrel RGMII Phy Interrupt */

	/* GPIO_KEYS */
#define GP_HOME			IMX_GPIO_NR(1, 2)
	IOMUX_PAD_CTRL(GPIO_2__GPIO1_IO02, WEAK_PULLUP),
#define GP_BACK			IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO_3__GPIO1_IO03, WEAK_PULLUP),

	/* i2c1 SGTL5000 */
	IOMUX_PAD_CTRL(GPIO_0__CCM_CLKO1, OUTPUT_40OHM),	/* SGTL5000 sys_mclk */
#define GP_SGTL5000_MUTE	IMX_GPIO_NR(5, 4)
	IOMUX_PAD_CTRL(EIM_A24__GPIO5_IO04, WEAK_PULLDN_OUTPUT),

	/* i2c2 - rtc */
#define GPIRQ_RTC		IMX_GPIO_NR(2, 26)
	IOMUX_PAD_CTRL(EIM_RW__GPIO2_IO26, WEAK_PULLUP),

	/* I2C3 */
#define GPIRQ_TOUCH		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO_9__GPIO1_IO09, WEAK_PULLUP),
#define GP_AR1021_5_WIRE_SEL	IMX_GPIO_NR(5, 2)
	IOMUX_PAD_CTRL(EIM_A25__GPIO5_IO02, HIGH_Z_SLOW),
#define GP_PCAP_NRESET		IMX_GPIO_NR(1, 21)
	IOMUX_PAD_CTRL(SD1_DAT3__GPIO1_IO21, WEAK_PULLUP_OUTPUT),

	/* LVDS */
#define GP_LVDS_EN		IMX_GPIO_NR(7, 12)
	IOMUX_PAD_CTRL(GPIO_17__GPIO7_IO12, WEAK_PULLUP),		/* J39 - pin 19, DISP0_CONTRAST */

	/* LEDS */
#define GP_VM_GPIO_1		IMX_GPIO_NR(4, 6)
	IOMUX_PAD_CTRL(KEY_COL0__GPIO4_IO06, WEAK_PULLUP),
#define GP_VM_GPIO_2		IMX_GPIO_NR(4, 7)
	IOMUX_PAD_CTRL(KEY_ROW0__GPIO4_IO07, WEAK_PULLUP),
#define GP_VM_GPIO_3		IMX_GPIO_NR(4, 8)
	IOMUX_PAD_CTRL(KEY_COL1__GPIO4_IO08, WEAK_PULLUP),
#define GP_VM_GPIO_4		IMX_GPIO_NR(4, 9)
	IOMUX_PAD_CTRL(KEY_ROW1__GPIO4_IO09, WEAK_PULLUP),
#define GP_VM_GPIO_5		IMX_GPIO_NR(4, 10)
	IOMUX_PAD_CTRL(KEY_COL2__GPIO4_IO10, WEAK_PULLUP),
#define GP_VM_GPIO_6		IMX_GPIO_NR(4, 11)
	IOMUX_PAD_CTRL(KEY_ROW2__GPIO4_IO11, WEAK_PULLUP),
#define GP_VM_GPIO_7		IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(KEY_ROW4__GPIO4_IO15, WEAK_PULLUP),
#define GP_VM_GPIO_8		IMX_GPIO_NR(1, 4)
	IOMUX_PAD_CTRL(GPIO_4__GPIO1_IO04, WEAK_PULLUP),

	/* PWM3 */
#define GP_RGB_BACKLIGHT	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(SD1_DAT1__GPIO1_IO17, WEAK_PULLDN_OUTPUT),

	/* PWM4 */
#define GP_LVDS_BACKLIGHT	IMX_GPIO_NR(1, 18)
	IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, WEAK_PULLDN_OUTPUT),

	/* UART1  */
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),

	/* UART2 for debug */
#ifndef CONFIG_SILENT_UART
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),
#else
	IOMUX_PAD_CTRL(EIM_D26__GPIO3_IO26, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__GPIO3_IO27, UART_PAD_CTRL),
#endif
	/* UART3 - Broadcom Bluetooth*/
	IOMUX_PAD_CTRL(EIM_D24__UART3_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D25__UART3_RX_DATA, UART_PAD_CTRL),

	/* USBOTG - J3 */
	IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, WEAK_PULLUP),
	IOMUX_PAD_CTRL(KEY_COL4__USB_OTG_OC, WEAK_PULLUP),
#define GP_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLDN_OUTPUT),

	/* USDHC3 - FULL sd */
	IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK, USDHC_CLK_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
#define GP_SD3_CD	IMX_GPIO_NR(7, 0)
	IOMUX_PAD_CTRL(SD3_DAT5__GPIO7_IO00, WEAK_PULLUP),
#define GP_SD3_WP	IMX_GPIO_NR(7, 1)
	IOMUX_PAD_CTRL(SD3_DAT4__GPIO7_IO01, WEAK_PULLUP),

	/* USDHC4 - eMMC */
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK, USDHC_CLK_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT4__SD4_DATA4, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT5__SD4_DATA5, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT6__SD4_DATA6, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT7__SD4_DATA7, USDHC_PAD_CTRL),
#define GP_EMMC_RESET	IMX_GPIO_NR(2, 7)
	IOMUX_PAD_CTRL(NANDF_D7__GPIO2_IO07, OUTPUT_40OHM),
};

static const iomux_v3_cfg_t enet_pads1[] = {
	/* pin 35 - 1 (PHY_AD2) on reset */
#define GP_PHY_AD2		IMX_GPIO_NR(6, 30)
	IOMUX_PAD_CTRL(RGMII_RXC__GPIO6_IO30, OUTPUT_40OHM),
	/* pin 32 - 1 - (MODE0) all */
#define GP_PHY_MODE0		IMX_GPIO_NR(6, 25)
	IOMUX_PAD_CTRL(RGMII_RD0__GPIO6_IO25, OUTPUT_40OHM),
	/* pin 31 - 1 - (MODE1) all */
#define GP_PHY_MODE1		IMX_GPIO_NR(6, 27)
	IOMUX_PAD_CTRL(RGMII_RD1__GPIO6_IO27, OUTPUT_40OHM),
	/* pin 28 - 1 - (MODE2) all */
#define GP_PHY_MODE2		IMX_GPIO_NR(6, 28)
	IOMUX_PAD_CTRL(RGMII_RD2__GPIO6_IO28, OUTPUT_40OHM),
	/* pin 27 - 1 - (MODE3) all */
#define GP_PHY_MODE3		IMX_GPIO_NR(6, 29)
	IOMUX_PAD_CTRL(RGMII_RD3__GPIO6_IO29, OUTPUT_40OHM),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
#define GP_PHY_CLK125		IMX_GPIO_NR(6, 24)
	IOMUX_PAD_CTRL(RGMII_RX_CTL__GPIO6_IO24, OUTPUT_40OHM),
};

static const iomux_v3_cfg_t enet_pads2[] = {
	IOMUX_PAD_CTRL(RGMII_RXC__RGMII_RXC, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD0__RGMII_RD0, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD1__RGMII_RD1, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD2__RGMII_RD2, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD3__RGMII_RD3, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RX_CTL__RGMII_RX_CTL, ENET_PAD_CTRL),
};

#ifdef CONFIG_CMD_FBPANEL
static const iomux_v3_cfg_t rgb_pads[] = {
	IOMUX_PAD_CTRL(DI0_DISP_CLK__IPU1_DI0_DISP_CLK, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN15__IPU1_DI0_PIN15, RGB_PAD_CTRL),	/* DRDY */
	IOMUX_PAD_CTRL(DI0_PIN2__IPU1_DI0_PIN02, RGB_PAD_CTRL),		/* HSYNC */
	IOMUX_PAD_CTRL(DI0_PIN3__IPU1_DI0_PIN03, RGB_PAD_CTRL),		/* VSYNC */
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, RGB_PAD_CTRL),
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
	IOMUX_PAD_CTRL(DISP0_DAT18__IPU1_DISP0_DATA18, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT19__IPU1_DISP0_DATA19, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT20__IPU1_DISP0_DATA20, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT21__IPU1_DISP0_DATA21, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT22__IPU1_DISP0_DATA22, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT23__IPU1_DISP0_DATA23, RGB_PAD_CTRL),
};

static const iomux_v3_cfg_t rgb_gpio_pads[] = {
	IOMUX_PAD_CTRL(DI0_DISP_CLK__GPIO4_IO16, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DI0_PIN15__GPIO4_IO17, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DI0_PIN2__GPIO4_IO18, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DI0_PIN3__GPIO4_IO19, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, WEAK_PULLUP),
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
#endif

/*
 *
 */
static struct i2c_pads_info i2c_pads[] = {
	/* I2C1, SGTL5000, RTC */
	I2C_PADS_INFO_ENTRY(I2C1, EIM_D21, 3, 21, EIM_D28, 3, 28, I2C_PAD_CTRL),
	/* I2C2 - hdmi */
	I2C_PADS_INFO_ENTRY(I2C2, KEY_COL3, 4, 12, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
	/* I2C3, Charger, PCIe */
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
	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port)
		return 0;
	gpio_set_value(GP_USB_OTG_PWR, on);
	return 0;
}

#endif

#ifdef CONFIG_FSL_ESDHC

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

	if (cfg->esdhc_base == USDHC4_BASE_ADDR)
		return 1;	/* eMMC always present */
	gpio_direction_input(GP_SD3_CD);
	return !gpio_get_value(GP_SD3_CD);
}

static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{.esdhc_base = USDHC3_BASE_ADDR, .max_bus_width = 4},
	{.esdhc_base = USDHC4_BASE_ADDR, .max_bus_width = 8},
};

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			break;
		case 1:
			gpio_set_value(GP_EMMC_RESET, 1); /* release reset */
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
	return (bus == 0 && cs == 0) ? GP_ECSPI1_CS1 : -1;
}
#endif

int board_phy_config(struct phy_device *phydev)
{
	/* ksz9021 */
	/* min rx data delay */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
	/* min tx data delay */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
	/* max rx/tx clock delay, min rx/tx control */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

static void setup_iomux_enet(void)
{
	gpio_direction_output(GP_ENET_PHY_RESET, 0); /* PHY rst */
	gpio_direction_output(GP_PHY_AD2, 1);
	gpio_direction_output(GP_PHY_MODE0, 1);
	gpio_direction_output(GP_PHY_MODE1, 1);
	gpio_direction_output(GP_PHY_MODE2, 1);
	gpio_direction_output(GP_PHY_MODE3, 1);
	gpio_direction_output(GP_PHY_CLK125, 1);
	SETUP_IOMUX_PADS(enet_pads1);

	/* Need delay 10ms according to KSZ9021 spec */
	udelay(1000 * 10);
	gpio_set_value(GP_ENET_PHY_RESET, 1); /* PHY reset */

	SETUP_IOMUX_PADS(enet_pads2);
	udelay(100);	/* Wait 100 us before using mii interface */
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

#ifdef CONFIG_FEC_MXC
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* scan phy 4,5,6,7 */
	phydev = phy_find_by_mask(bus, (0xf << 4), PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		free(bus);
		return 0;
	}
	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
	}
#endif
#ifdef CONFIG_CI_UDC
	/* For otg ethernet*/
	usb_eth_initialize(bis);
#endif
	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
void board_enable_lvds(const struct display_info_t *di, int enable)
{
	gpio_direction_output(GP_LVDS_BACKLIGHT, enable);
	gpio_direction_output(GP_LVDS_EN, enable);
}

void board_enable_lcd(const struct display_info_t *di, int enable)
{
	if (enable)
		SETUP_IOMUX_PADS(rgb_pads);
	else
		SETUP_IOMUX_PADS(rgb_gpio_pads);
	gpio_direction_output(GP_RGB_BACKLIGHT, enable);
}

static const struct display_info_t displays[] = {
	/* hdmi */
	IMX_VD50_1280_720M_60(HDMI, 1, 1),
	IMX_VD50_1920_1080M_60(HDMI, 0, 1),
	IMX_VD50_1024_768M_60(HDMI, 0, 1),

	/* ft5x06 */
	IMX_VD38_HITACHI_HVGA(LCD, 1, 2),
	IMX_VD38_HANNSTAR7(LVDS, 0, 2),
	IMX_VD38_AUO_B101EW05(LVDS, 0, 2),
	IMX_VD38_LG1280_800(LVDS, 0, 2),
	IMX_VD38_DT070BTFT(LVDS, 0, 2),
	IMX_VD38_WSVGA(LVDS, 0, 2),

	/* ili210x */
	IMX_VD41_AMP1024_600(LVDS, 1, 2),

	/* egalax_ts */
	IMX_VD04_HANNSTAR(LVDS, 1, 2),
	IMX_VD04_LG9_7(LVDS, 0, 2),

	IMX_VD_SHARP_LQ101K1LY04(LVDS, 0, 0),
	IMX_VD_WXGA(LVDS, 0, 0),
	IMX_VD_WVGA(LVDS, 0, 0),
};
#endif

static const unsigned short gpios_out_low[] = {
	GP_ENET_PHY_RESET,
	GP_LVDS_EN,
	GP_RGB_BACKLIGHT,
	GP_LVDS_BACKLIGHT,
	GP_SGTL5000_MUTE,
	GP_USB_OTG_PWR,		/* disable USB otg power */
	GP_EMMC_RESET,		/* hold in reset */
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_CS1,		/* SS1 of spi nor */
	GP_PCAP_NRESET,		/* PCAP reset on J40 */
};

static const unsigned short gpios_in[] = {
	GP_ENET_PHY_INT,
	GP_AR1021_5_WIRE_SEL,
	GP_HOME,
	GP_BACK,
	GPIRQ_TOUCH,
	GPIRQ_RTC,
	GP_SD3_CD,
	GP_SD3_WP,
	GP_VM_GPIO_1,
	GP_VM_GPIO_2,
	GP_VM_GPIO_3,
	GP_VM_GPIO_4,
	GP_VM_GPIO_5,
	GP_VM_GPIO_6,
	GP_VM_GPIO_7,
	GP_VM_GPIO_8,
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

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	for (i = 0; i < 3; i++) {
		setup_i2c(i, CONFIG_SYS_I2C_SPEED, 0x7f, p);
		p += I2C_PADS_INFO_ENTRY_SPACING;
	}
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, ARRAY_SIZE(displays));
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: Boundary nitrogen6_vm\n");
	return 0;
}

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
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}

int board_late_init(void)
{
	int cpurev = get_cpu_rev();

	setenv("cpu", get_imx_type((cpurev & 0xFF000) >> 12));
	if (!getenv("board"))
		setenv("board", "nitrogen6_vm");
	if (!getenv("uboot_defconfig"))
		setenv("uboot_defconfig", CONFIG_DEFCONFIG);
	return 0;
}
