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
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/spi.h>
#include <asm/imx-common/video.h>
#include <asm/imx-common/boot_mode.h>
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
#include <splash.h>
#include <usb/ehci-fsl.h>

DECLARE_GLOBAL_DATA_PTR;

#define GP_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
#define GP_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
#define GP_UART5_RX_EN		IMX_GPIO_NR(6, 10)
#define GP_UART5_TX_EN		IMX_GPIO_NR(6, 7)
#define GP_UART5_RS485_EN	IMX_GPIO_NR(2, 24)
#define GP_UART5_AON		IMX_GPIO_NR(6, 8)
#define GP_WL12XX_WL_IRQ	IMX_GPIO_NR(6, 11)
#define GP_WL12XX_WL_ENABLE	IMX_GPIO_NR(6, 15)
#define GP_WL12XX_BT_ENABLE	IMX_GPIO_NR(6, 16)
#define GP_SD3_CD		IMX_GPIO_NR(7, 0)
#define GP_SD3_POWER_SEL	IMX_GPIO_NR(6, 14)
#define GP_EMMC_RESET		IMX_GPIO_NR(2, 6)
#define GP_ENET_PHY_RESET	IMX_GPIO_NR(1, 27)
#define GP_ECSPI1_SS1		IMX_GPIO_NR(3, 19)
#define GP_BUTTON_MENU		IMX_GPIO_NR(2, 1)
#define GP_BUTTON_BACK		IMX_GPIO_NR(2, 2)
#define GP_BUTTON_SEARCH	IMX_GPIO_NR(2, 3)
#define GP_BUTTON_HOME		IMX_GPIO_NR(2, 4)
#define GP_BUTTON_VOLDOWN	IMX_GPIO_NR(7, 1)
#define GP_BUTTON_VOLUP		IMX_GPIO_NR(7, 13)
#define GP_RGB_BACKLIGHT	IMX_GPIO_NR(1, 21)
#define GP_LVDS_BACKLIGHT	IMX_GPIO_NR(1, 18)
#define GP_OV5642_CSI0_PWRDN	IMX_GPIO_NR(3, 29)
#define GP_OV5642_CSI0_RESET	IMX_GPIO_NR(1, 4)
#define GP_OV5640_CSI1_PWRDN	IMX_GPIO_NR(3, 13)
#define GP_OV5640_CSI1_RESET	IMX_GPIO_NR(3, 14)
#define GP_OV5640_MIPI_PWRDN	IMX_GPIO_NR(6, 9)
#define GP_OV5640_MIPI_RESET	IMX_GPIO_NR(2, 5)
#define GP_PCIE_RESET		IMX_GPIO_NR(6, 31)
#define GP_RTC_RV4162_IRQ	IMX_GPIO_NR(4, 6)
#define GP_I2C_EN_OV5642	IMX_GPIO_NR(3, 20)
#define GP_I2C_EN_OV5640_MIPI	IMX_GPIO_NR(3, 20)
#define GP_I2C_EN_PCIE		IMX_GPIO_NR(3, 20)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLDN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)

static const iomux_v3_cfg_t nitrogen6_max_pads[] = {
	/* USB OTG */
	IOMUX_PADS(PAD_GPIO_1__USB_OTG_ID | MUX_PAD_CTRL(WEAK_PULLUP)),
	IOMUX_PADS(PAD_KEY_COL4__USB_OTG_OC | MUX_PAD_CTRL(WEAK_PULLUP)),
	IOMUX_PADS(PAD_EIM_D22__GPIO3_IO22 | MUX_PAD_CTRL(OUTPUT_40OHM)),

	/* USB host */
	IOMUX_PADS(PAD_EIM_D30__USB_H1_OC | MUX_PAD_CTRL(WEAK_PULLUP)),
	IOMUX_PADS(PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(OUTPUT_40OHM)),

	/* UART1 */
	IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),

	/* UART2 */
	IOMUX_PADS(PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),

	/* UART3 for wl1271 */
	IOMUX_PADS(PAD_EIM_D24__UART3_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D25__UART3_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D23__UART3_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D31__UART3_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL)),

	/* UART5 - ISL3330IAZ rs485/rs232 selection */
	IOMUX_PADS(PAD_KEY_COL1__UART5_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW1__UART5_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	/* RS485 RX Enable */
	IOMUX_PADS(PAD_NANDF_RB0__GPIO6_IO10 | MUX_PAD_CTRL(WEAK_PULLDN)),
	/* RS485 TX Enable */
	IOMUX_PADS(PAD_NANDF_CLE__GPIO6_IO07 | MUX_PAD_CTRL(WEAK_PULLDN)),
	/* RS485/RS232 Select 2.5V */
	IOMUX_PADS(PAD_EIM_CS1__GPIO2_IO24 | MUX_PAD_CTRL(WEAK_PULLDN)),
	/* ON - meaning depends on others */
	IOMUX_PADS(PAD_NANDF_ALE__GPIO6_IO08 | MUX_PAD_CTRL(WEAK_PULLDN)),

	/* USDHC2 - TiWi wl1271 pads */
	IOMUX_PADS(PAD_SD2_CLK__SD2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_CMD__SD2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_CS0__GPIO6_IO11 | MUX_PAD_CTRL(WEAK_PULLDN)),
	IOMUX_PADS(PAD_NANDF_CS2__GPIO6_IO15 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	IOMUX_PADS(PAD_NANDF_CS3__GPIO6_IO16 | MUX_PAD_CTRL(OUTPUT_40OHM)),

	/* USDHC3 - sdcard */
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__GPIO7_IO00 | MUX_PAD_CTRL(WEAK_PULLUP)),
	IOMUX_PADS(PAD_NANDF_CS1__SD3_VSELECT | MUX_PAD_CTRL(OUTPUT_40OHM)),

	/* USDHC4 - emmc */
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	/* EMMC_RESET */
	IOMUX_PADS(PAD_NANDF_D6__GPIO2_IO06 | MUX_PAD_CTRL(OUTPUT_40OHM)),

	/* ENET pads that don't change for PHY reset */
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* pin 42 PHY nRST */
	IOMUX_PADS(PAD_ENET_RXD0__GPIO1_IO27 | MUX_PAD_CTRL(OUTPUT_40OHM)),

	/* ECSPI1 pads */
        /* SS1 */
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),

	/* Button pad assignments for J14 */
	IOMUX_PADS(PAD_NANDF_D1__GPIO2_IO01 | MUX_PAD_CTRL(BUTTON_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D2__GPIO2_IO02 | MUX_PAD_CTRL(BUTTON_PAD_CTRL)),
	/* Labeled Search (mapped to Power under Android) */
	IOMUX_PADS(PAD_NANDF_D3__GPIO2_IO03 | MUX_PAD_CTRL(BUTTON_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D4__GPIO2_IO04 | MUX_PAD_CTRL(BUTTON_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT4__GPIO7_IO01 | MUX_PAD_CTRL(BUTTON_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(BUTTON_PAD_CTRL)),

	/* Backlight on RGB connector: J15, pin 37 */
	IOMUX_PADS(PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	/* Backlight on LVDS connector: J6, pin 20 */
	IOMUX_PADS(PAD_SD1_CMD__GPIO1_IO18 | MUX_PAD_CTRL(OUTPUT_40OHM)),

	/* Camera controls */
	IOMUX_PADS(PAD_EIM_D29__GPIO3_IO29 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	IOMUX_PADS(PAD_GPIO_4__GPIO1_IO04 | MUX_PAD_CTRL(OUTPUT_40OHM)),

	IOMUX_PADS(PAD_EIM_DA13__GPIO3_IO13),		/* pin 32 - Power */
	IOMUX_PADS(PAD_EIM_DA14__GPIO3_IO14),		/* pin 36 - Reset */

	IOMUX_PADS(PAD_NANDF_WP_B__GPIO6_IO09 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	IOMUX_PADS(PAD_NANDF_D5__GPIO2_IO05 | MUX_PAD_CTRL(OUTPUT_40OHM)),

	/* PCIe */
	IOMUX_PADS(PAD_EIM_BCLK__GPIO6_IO31 | MUX_PAD_CTRL(OUTPUT_40OHM)),

	/* rtc */
	IOMUX_PADS(PAD_KEY_COL0__GPIO4_IO06 | MUX_PAD_CTRL(WEAK_PULLUP)),

	/* i2c enables */
	/* ov5642 camera i2c enable */
	IOMUX_PADS(PAD_EIM_D20__GPIO3_IO20 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	/* ov5640_mipi camera i2c enable */
	IOMUX_PADS(PAD_KEY_ROW4__GPIO4_IO15 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	/* pcie i2c enable */
	IOMUX_PADS(PAD_EIM_OE__GPIO2_IO25 | MUX_PAD_CTRL(OUTPUT_40OHM)),

	/* Amplifier Mute */
	IOMUX_PADS(PAD_ENET_TXD1__GPIO1_IO29 | MUX_PAD_CTRL(WEAK_PULLDN)),
};

static const iomux_v3_cfg_t enet_pads1[] = {
	/* pin 35 - 1 (PHY_AD2) on reset */
	IOMUX_PADS(PAD_RGMII_RXC__GPIO6_IO30 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	/* pin 32 - 1 - (MODE0) all */
	IOMUX_PADS(PAD_RGMII_RD0__GPIO6_IO25 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	/* pin 31 - 1 - (MODE1) all */
	IOMUX_PADS(PAD_RGMII_RD1__GPIO6_IO27 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	/* pin 28 - 1 - (MODE2) all */
	IOMUX_PADS(PAD_RGMII_RD2__GPIO6_IO28 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	/* pin 27 - 1 - (MODE3) all */
	IOMUX_PADS(PAD_RGMII_RD3__GPIO6_IO29 | MUX_PAD_CTRL(OUTPUT_40OHM)),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	IOMUX_PADS(PAD_RGMII_RX_CTL__GPIO6_IO24 | MUX_PAD_CTRL(OUTPUT_40OHM)),
};

static const iomux_v3_cfg_t enet_pads2[] = {
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
};

static const iomux_v3_cfg_t rgb_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK),
	IOMUX_PADS(PAD_DI0_PIN15__IPU1_DI0_PIN15),
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02),
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03),
	IOMUX_PADS(PAD_DI0_PIN4__GPIO4_IO20),
	IOMUX_PADS(PAD_DISP0_DAT0__IPU1_DISP0_DATA00),
	IOMUX_PADS(PAD_DISP0_DAT1__IPU1_DISP0_DATA01),
	IOMUX_PADS(PAD_DISP0_DAT2__IPU1_DISP0_DATA02),
	IOMUX_PADS(PAD_DISP0_DAT3__IPU1_DISP0_DATA03),
	IOMUX_PADS(PAD_DISP0_DAT4__IPU1_DISP0_DATA04),
	IOMUX_PADS(PAD_DISP0_DAT5__IPU1_DISP0_DATA05),
	IOMUX_PADS(PAD_DISP0_DAT6__IPU1_DISP0_DATA06),
	IOMUX_PADS(PAD_DISP0_DAT7__IPU1_DISP0_DATA07),
	IOMUX_PADS(PAD_DISP0_DAT8__IPU1_DISP0_DATA08),
	IOMUX_PADS(PAD_DISP0_DAT9__IPU1_DISP0_DATA09),
	IOMUX_PADS(PAD_DISP0_DAT10__IPU1_DISP0_DATA10),
	IOMUX_PADS(PAD_DISP0_DAT11__IPU1_DISP0_DATA11),
	IOMUX_PADS(PAD_DISP0_DAT12__IPU1_DISP0_DATA12),
	IOMUX_PADS(PAD_DISP0_DAT13__IPU1_DISP0_DATA13),
	IOMUX_PADS(PAD_DISP0_DAT14__IPU1_DISP0_DATA14),
	IOMUX_PADS(PAD_DISP0_DAT15__IPU1_DISP0_DATA15),
	IOMUX_PADS(PAD_DISP0_DAT16__IPU1_DISP0_DATA16),
	IOMUX_PADS(PAD_DISP0_DAT17__IPU1_DISP0_DATA17),
	IOMUX_PADS(PAD_DISP0_DAT18__IPU1_DISP0_DATA18),
	IOMUX_PADS(PAD_DISP0_DAT19__IPU1_DISP0_DATA19),
	IOMUX_PADS(PAD_DISP0_DAT20__IPU1_DISP0_DATA20),
	IOMUX_PADS(PAD_DISP0_DAT21__IPU1_DISP0_DATA21),
	IOMUX_PADS(PAD_DISP0_DAT22__IPU1_DISP0_DATA22),
	IOMUX_PADS(PAD_DISP0_DAT23__IPU1_DISP0_DATA23),
};

/*
 *
 */
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

static struct i2c_pads_info i2c_pads[] = {
{
	/* I2C1, SGTL5000 */
	.scl = {
		.i2c_mode = MX6Q_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6Q_PAD_EIM_D21__GPIO3_IO21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_EIM_D28__I2C1_SDA | PC,
		.gpio_mode = MX6Q_PAD_EIM_D28__GPIO3_IO28 | PC,
		.gp = IMX_GPIO_NR(3, 28)
	}
}, {
	/* I2C1, SGTL5000 */
	.scl = {
		.i2c_mode = MX6DL_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6DL_PAD_EIM_D21__GPIO3_IO21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6DL_PAD_EIM_D28__I2C1_SDA | PC,
		.gpio_mode = MX6DL_PAD_EIM_D28__GPIO3_IO28 | PC,
		.gp = IMX_GPIO_NR(3, 28)
	}
}, {
	/* I2C2 Camera, MIPI */
	.scl = {
		.i2c_mode = MX6Q_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
}, {
	/* I2C2 Camera, MIPI */
	.scl = {
		.i2c_mode = MX6DL_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6DL_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6DL_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6DL_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
}, {
	/* I2C3, J15 - RGB connector */
	.scl = {
		.i2c_mode = MX6Q_PAD_GPIO_5__I2C3_SCL | PC,
		.gpio_mode = MX6Q_PAD_GPIO_5__GPIO1_IO05 | PC,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_GPIO_16__I2C3_SDA | PC,
		.gpio_mode = MX6Q_PAD_GPIO_16__GPIO7_IO11 | PC,
		.gp = IMX_GPIO_NR(7, 11)
	}
}, {
	/* I2C3, J15 - RGB connector */
	.scl = {
		.i2c_mode = MX6DL_PAD_GPIO_5__I2C3_SCL | PC,
		.gpio_mode = MX6DL_PAD_GPIO_5__GPIO1_IO05 | PC,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6DL_PAD_GPIO_16__I2C3_SDA | PC,
		.gpio_mode = MX6DL_PAD_GPIO_16__GPIO7_IO11 | PC,
		.gp = IMX_GPIO_NR(7, 11)
	}
}};

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);
//	printf("%s:%p *%p=0x%lx\n", __func__, gd, &gd->ram_size, gd->ram_size);
	return 0;
}

static void setup_iomux_enet(void)
{
	gpio_direction_output(GP_ENET_PHY_RESET, 0); /* PHY rst */
	gpio_direction_output(IMX_GPIO_NR(6, 30), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 25), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 27), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 28), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 29), 1);
	SETUP_IOMUX_PADS(enet_pads1);
	gpio_direction_output(IMX_GPIO_NR(6, 24), 1);

	/* Need delay 10ms according to KSZ9021 spec */
	udelay(1000 * 10);
	gpio_set_value(GP_ENET_PHY_RESET, 1); /* PHY reset */

	SETUP_IOMUX_PADS(enet_pads2);
	udelay(100);	/* Wait 100 us before using mii interface */
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	/* Reset USB hub */
	gpio_direction_output(GP_USB_HUB_RESET, 0);
	mdelay(2);
	gpio_set_value(GP_USB_HUB_RESET, 1);
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
static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int gp_cd = GP_SD3_CD;

	if (cfg->esdhc_base != USDHC3_BASE_ADDR)
		return 1;	/* eMMC always present */
	gpio_direction_input(gp_cd);
	return !gpio_get_value(gp_cd);
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	usdhc_cfg[0].max_bus_width = 4;
	usdhc_cfg[1].max_bus_width = 8;

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
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
	}

	return status;
}
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	int gp = (bus == 0 && cs == 0) ? GP_ECSPI1_SS1 : -1;
	printf("%s: return GP(%d) for spi bus %d, chip select %d\n", __func__, gp, bus, cs);
	return gp;
}
#endif

int board_phy_config(struct phy_device *phydev)
{
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

#if defined(CONFIG_VIDEO_IPUV3)

static void do_enable_hdmi(struct display_info_t const *dev)
{
	imx_enable_hdmi_phy();
}

static int detect_i2c(struct display_info_t const *dev)
{
	return ((0 == i2c_set_bus_num(dev->bus))
		&&
		(0 == i2c_probe(dev->addr)));
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);
	gpio_direction_output(GP_LVDS_BACKLIGHT, 1);
}

static void enable_lvds_jeida(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_JEIDA;
	writel(reg, &iomux->gpr[2]);
	gpio_direction_output(GP_LVDS_BACKLIGHT, 1);
}

static void enable_rgb(struct display_info_t const *dev)
{
	SETUP_IOMUX_PADS(rgb_pads);
	gpio_direction_output(GP_RGB_BACKLIGHT, 1);
}

struct display_info_t const displays[] = {{
	.bus	= 1,
	.addr	= 0x50,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_i2c,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds_jeida,
	.mode	= {
		.name           = "LDB-WXGA",
		.refresh        = 60,
		.xres           = 1280,
		.yres           = 800,
		.pixclock       = 14065,
		.left_margin    = 40,
		.right_margin   = 40,
		.upper_margin   = 3,
		.lower_margin   = 80,
		.hsync_len      = 10,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "LDB-WXGA-S",
		.refresh        = 60,
		.xres           = 1280,
		.yres           = 800,
		.pixclock       = 14065,
		.left_margin    = 40,
		.right_margin   = 40,
		.upper_margin   = 3,
		.lower_margin   = 80,
		.hsync_len      = 10,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x4,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x4,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "LG-9.7",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385, /* ~65MHz */
		.left_margin    = 480,
		.right_margin   = 260,
		.upper_margin   = 16,
		.lower_margin   = 6,
		.hsync_len      = 250,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x38,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "wsvga-lvds",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 600,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x48,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_i2c,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "wvga-rgb",
		.refresh        = 57,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 37037,
		.left_margin    = 40,
		.right_margin   = 60,
		.upper_margin   = 10,
		.lower_margin   = 10,
		.hsync_len      = 20,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x10,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_i2c,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "fusion7",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 33898,
		.left_margin    = 96,
		.right_margin   = 24,
		.upper_margin   = 3,
		.lower_margin   = 10,
		.hsync_len      = 72,
		.vsync_len      = 7,
		.sync           = 0x40000002,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x48,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_i2c,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "qvga",
		.refresh        = 60,
		.xres           = 320,
		.yres           = 240,
		.pixclock       = 37037,
		.left_margin    = 38,
		.right_margin   = 37,
		.upper_margin   = 16,
		.lower_margin   = 15,
		.hsync_len      = 30,
		.vsync_len      = 3,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

size_t display_count = ARRAY_SIZE(displays);

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();
	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
			|IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

	/* backlights off until needed */
	gpio_direction_input(GP_LVDS_BACKLIGHT);
	gpio_direction_input(GP_RGB_BACKLIGHT);
}
#endif


static unsigned short gpios_out_low[] = {
	GP_USB_OTG_PWR,		/* disable USB otg power */
	GP_USB_HUB_RESET,	/* disable hub */
	/* Disable wl1271 */
	GP_WL12XX_WL_ENABLE,	/* disable wireless */
	GP_WL12XX_BT_ENABLE, 	/* disable bluetooth */
	GP_EMMC_RESET,		/* hold in reset */
	GP_ENET_PHY_RESET,
	GP_OV5642_CSI0_RESET,	/* camera reset */
	GP_OV5640_CSI1_RESET,	/* camera reset */
	GP_OV5640_MIPI_RESET,	/* camera reset */
	GP_PCIE_RESET,
	GP_UART5_RX_EN,		/* power down uart5 */
	GP_UART5_TX_EN,
	GP_UART5_RS485_EN,
	GP_UART5_AON,
	GP_I2C_EN_OV5642,
	GP_I2C_EN_OV5640_MIPI,
	GP_I2C_EN_PCIE,
};

static unsigned short gpios_out_high[] = {
	GP_ECSPI1_SS1,		/* SS1 of spi nor */
	GP_OV5642_CSI0_PWRDN,	/* camera power down */
	GP_OV5640_CSI1_PWRDN,	/* camera power down */
	GP_OV5640_MIPI_PWRDN,	/* camera power down */
	GP_SD3_POWER_SEL,	/* high=3.3v */
};

static void set_gpios(unsigned short *p, int cnt, int val)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_output(*p++, val);
}

int board_early_init_f(void)
{
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	gpio_direction_input(GP_WL12XX_WL_IRQ);

	SETUP_IOMUX_PADS(nitrogen6_max_pads);
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
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
	struct i2c_pads_info *p = i2c_pads + !is_cpu_type(MXC_CPU_MX6Q);
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	for (i=0; i < ARRAY_SIZE(i2c_pads); i+= 2)
	        setup_i2c(i/2, CONFIG_SYS_I2C_SPEED, 0x7f, p+i);

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	return 0;
}


int checkboard(void)
{
	puts("Board: Nitrogen6_max\n");
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"back",	GP_BUTTON_BACK,		'B'},
	{"home",	GP_BUTTON_HOME,		'H'},
	{"menu",	GP_BUTTON_MENU,		'M'},
	{"search",	GP_BUTTON_SEARCH,	'S'},
	{"volup",	GP_BUTTON_VOLUP,	'V'},
	{"voldown",	GP_BUTTON_VOLDOWN,	'v'},
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
	{"mmc1",	MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},	/* 8-bit eMMC */
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
	return 0;
}

int board_late_init(void)
{
	int cpurev = get_cpu_rev();
	setenv("cpu",get_imx_type((cpurev & 0xFF000) >> 12));
	setenv("board","nitrogen6_max");
	return 0;
}
