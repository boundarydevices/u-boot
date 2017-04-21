/*
 * Copyright (C) 2015 Boundary Devices, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/fbpanel.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <linux/sizes.h>
#include <malloc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <usb.h>
#include <usb/ehci-fsl.h>

DECLARE_GLOBAL_DATA_PTR;

#define AUD_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define CSI_PAD_CTL	PAD_CTL_DSE_120ohm

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_HIGH | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_120ohm   | PAD_CTL_SRE_FAST)

#define ENET_RXD_DN_PAD_CTRL  (PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_HIGH | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define ENET_RXD_UP_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_HIGH | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL    (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_ODE)

#define LCDIF_PAD_CTL	PAD_CTL_DSE_120ohm

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC2_PAD_CTRL (PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC2_CLK_PAD_CTRL (PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_80ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_CLK_PAD_CTRL (PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_80ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define WEAK_PULLDN_OUTPUT (PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP_OUTPUT (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_SLOW)

static const iomux_v3_cfg_t init_pads[] = {
	/* Audmux */
	IOMUX_PAD_CTRL(SD1_DATA0__AUDMUX_AUD5_RXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA1__AUDMUX_AUD5_TXC, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA2__AUDMUX_AUD5_TXFS, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA3__AUDMUX_AUD5_TXD, AUD_PAD_CTRL),

	/* CSI */
	IOMUX_PAD_CTRL(CSI_MCLK__CSI1_MCLK, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_PIXCLK__CSI1_PIXCLK, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_VSYNC__CSI1_VSYNC, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA00__CSI1_DATA_2, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA01__CSI1_DATA_3, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA02__CSI1_DATA_4, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA03__CSI1_DATA_5, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA04__CSI1_DATA_6, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA05__CSI1_DATA_7, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA06__CSI1_DATA_8, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA07__CSI1_DATA_9, CSI_PAD_CTL),

	/* ECSPI1 (serial nor eeprom) */
	IOMUX_PAD_CTRL(KEY_COL1__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_ROW0__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_COL0__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(2, 16)
	IOMUX_PAD_CTRL(KEY_ROW1__GPIO2_IO_16, WEAK_PULLUP),

	/* ECSPI5 */
	IOMUX_PAD_CTRL(NAND_DATA00__ECSPI5_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA01__ECSPI5_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA02__ECSPI5_SCLK, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA03__ECSPI5_SS0, SPI_PAD_CTRL),

	/* enet phy */
	IOMUX_PAD_CTRL(ENET1_MDC__ENET1_MDC, WEAK_PULLUP),
	IOMUX_PAD_CTRL(ENET1_MDIO__ENET1_MDIO, WEAK_PULLUP),

	/* fec1 */
	IOMUX_PAD_CTRL(RGMII1_TD0__ENET1_TX_DATA_0, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII1_TD1__ENET1_TX_DATA_1, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII1_TD2__ENET1_TX_DATA_2, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII1_TD3__ENET1_TX_DATA_3, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII1_TXC__ENET1_RGMII_TXC, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII1_TX_CTL__ENET1_TX_EN, ENET_PAD_CTRL),
	/* PHY_AD0 */
	IOMUX_PAD_CTRL(RGMII1_RD0__ENET1_RX_DATA_0, ENET_RXD_DN_PAD_CTRL),
	/* PHY_AD1 */
	IOMUX_PAD_CTRL(RGMII1_RD1__ENET1_RX_DATA_1, ENET_RXD_DN_PAD_CTRL),
	/* MODE - 1100 */
	/* MODE 0 - 0 */
	IOMUX_PAD_CTRL(RGMII1_RX_CTL__ENET1_RX_EN, ENET_RXD_DN_PAD_CTRL),
	/* MODE 1 - 0 (PLLOFF) */
	IOMUX_PAD_CTRL(RGMII1_RD2__ENET1_RX_DATA_2, ENET_RXD_DN_PAD_CTRL),
	/* MODE 2 is LED_100, Internal pull up */
	/* MODE 3 */
	IOMUX_PAD_CTRL(RGMII1_RD3__ENET1_RX_DATA_3, ENET_RXD_UP_PAD_CTRL),
	/* 1.8V(1)/1.5V select(0) */
	IOMUX_PAD_CTRL(RGMII1_RXC__ENET1_RX_CLK, ENET_RXD_UP_PAD_CTRL),
	/* AR8035 PHY Reset */
#define GP_RGMII1_PHY_RESET	IMX_GPIO_NR(2, 7)
	IOMUX_PAD_CTRL(ENET2_CRS__GPIO2_IO_7, WEAK_PULLUP),
#define GP_RGMII1_PHY_INT	IMX_GPIO_NR(2, 4)
	IOMUX_PAD_CTRL(ENET1_RX_CLK__GPIO2_IO_4, WEAK_PULLUP),
	IOMUX_PAD_CTRL(ENET1_TX_CLK__GPIO2_IO_5, WEAK_PULLUP),

	/* fec2 */
	IOMUX_PAD_CTRL(RGMII2_TD0__ENET2_TX_DATA_0, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII2_TD1__ENET2_TX_DATA_1, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII2_TD2__ENET2_TX_DATA_2, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII2_TD3__ENET2_TX_DATA_3, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII2_TXC__ENET2_RGMII_TXC, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII2_TX_CTL__ENET2_TX_EN, ENET_PAD_CTRL),
	/* PHY_AD0 - 1 for 2nd ethernet make phyad=5*/
	IOMUX_PAD_CTRL(RGMII2_RD0__ENET2_RX_DATA_0, ENET_RXD_UP_PAD_CTRL),
	/* PHY_AD1 */
	IOMUX_PAD_CTRL(RGMII2_RD1__ENET2_RX_DATA_1, ENET_RXD_DN_PAD_CTRL),
	/* MODE - 1100 */
	/* MODE 0 - 0 */
	IOMUX_PAD_CTRL(RGMII2_RX_CTL__ENET2_RX_EN, ENET_RXD_DN_PAD_CTRL),
	/* MODE 1 - 0 (PLLOFF) */
	IOMUX_PAD_CTRL(RGMII2_RD2__ENET2_RX_DATA_2, ENET_RXD_DN_PAD_CTRL),
	/* MODE 2 is LED_100, Internal pull up */
	/* MODE 3 */
	IOMUX_PAD_CTRL(RGMII2_RD3__ENET2_RX_DATA_3, ENET_RXD_UP_PAD_CTRL),
	/* 1.8V(1)/1.5V select(0) */
	IOMUX_PAD_CTRL(RGMII2_RXC__ENET2_RX_CLK, ENET_RXD_UP_PAD_CTRL),
	/* AR8035 PHY Reset */
#define GP_RGMII2_PHY_RESET	IMX_GPIO_NR(2, 6)
	IOMUX_PAD_CTRL(ENET2_COL__GPIO2_IO_6, WEAK_PULLUP),
#define GP_RGMII2_PHY_INT	IMX_GPIO_NR(2, 8)
	IOMUX_PAD_CTRL(ENET2_RX_CLK__GPIO2_IO_8, WEAK_PULLUP),
	IOMUX_PAD_CTRL(ENET2_TX_CLK__GPIO2_IO_9, WEAK_PULLUP),

	/* flexcan1 */
	IOMUX_PAD_CTRL(QSPI1B_DQS__CAN1_TX, WEAK_PULLUP),
	IOMUX_PAD_CTRL(QSPI1A_SS1_B__CAN1_RX, WEAK_PULLUP),
#define GP_CAN1_STANDBY		IMX_GPIO_NR(4, 27)
	IOMUX_PAD_CTRL(QSPI1B_DATA3__GPIO4_IO_27, WEAK_PULLUP),

	/* flexcan2 */
	IOMUX_PAD_CTRL(QSPI1A_DQS__CAN2_TX, WEAK_PULLUP),
	IOMUX_PAD_CTRL(QSPI1B_SS1_B__CAN2_RX, WEAK_PULLUP),
#define GP_CAN2_STANDBY		IMX_GPIO_NR(4, 24)
	IOMUX_PAD_CTRL(QSPI1B_DATA0__GPIO4_IO_24, WEAK_PULLUP),

	/* hogs - expanders */
	IOMUX_PAD_CTRL(NAND_CE0_B__GPIO4_IO_1, WEAK_PULLUP),
	IOMUX_PAD_CTRL(NAND_CE1_B__GPIO4_IO_2, WEAK_PULLUP),
	IOMUX_PAD_CTRL(NAND_ALE__GPIO4_IO_0, WEAK_PULLUP),
	IOMUX_PAD_CTRL(NAND_CLE__GPIO4_IO_3, WEAK_PULLUP),
	IOMUX_PAD_CTRL(NAND_RE_B__GPIO4_IO_12, WEAK_PULLUP),
	IOMUX_PAD_CTRL(NAND_WE_B__GPIO4_IO_14, WEAK_PULLUP),
	IOMUX_PAD_CTRL(NAND_WP_B__GPIO4_IO_15, WEAK_PULLUP),
	IOMUX_PAD_CTRL(NAND_READY_B__GPIO4_IO_13, WEAK_PULLUP),
	IOMUX_PAD_CTRL(QSPI1A_DATA0__GPIO4_IO_16, WEAK_PULLUP),
	IOMUX_PAD_CTRL(QSPI1A_DATA1__GPIO4_IO_17, WEAK_PULLUP),
	IOMUX_PAD_CTRL(QSPI1A_DATA2__GPIO4_IO_18, WEAK_PULLUP),
	IOMUX_PAD_CTRL(QSPI1A_DATA3__GPIO4_IO_19, WEAK_PULLUP),
	/* hogs - Test points */
	IOMUX_PAD_CTRL(NAND_DATA04__GPIO4_IO_8, WEAK_PULLUP),
	IOMUX_PAD_CTRL(QSPI1B_DATA1__GPIO4_IO_25, WEAK_PULLUP),

	/* LVDS */
#define GP_LVDS_ENABLE	IMX_GPIO_NR(4, 21)
	IOMUX_PAD_CTRL(QSPI1A_SCLK__GPIO4_IO_21, WEAK_PULLUP),

	/* PCIe */
#define GP_PCIE_WAKE	IMX_GPIO_NR(4, 9)
	IOMUX_PAD_CTRL(NAND_DATA05__GPIO4_IO_9, WEAK_PULLUP),
#define GP_PCIE_RESET	IMX_GPIO_NR(4, 10)
	IOMUX_PAD_CTRL(NAND_DATA06__GPIO4_IO_10, WEAK_PULLUP),
#define GP_PCIE_DISABLE	IMX_GPIO_NR(4, 11)
	IOMUX_PAD_CTRL(NAND_DATA07__GPIO4_IO_11, WEAK_PULLUP),

	/* PWM2 for rgb panel */
#define GP_BACKLIGHT_RGB 	IMX_GPIO_NR(1, 11)
	IOMUX_PAD_CTRL(GPIO1_IO11__GPIO1_IO_11, WEAK_PULLDN_OUTPUT),

	/* PWM3 */
#define GP_PWM3			IMX_GPIO_NR(1, 12)
	IOMUX_PAD_CTRL(GPIO1_IO12__GPIO1_IO_12, OUTPUT_40OHM),

	/* PWM4 - for LVDS panel */
#define GP_BACKLIGHT_LVDS	IMX_GPIO_NR(1, 13)
	IOMUX_PAD_CTRL(GPIO1_IO13__GPIO1_IO_13, WEAK_PULLDN_OUTPUT),


	/* sgtl5000 */
	IOMUX_PAD_CTRL(SD1_CMD__CCM_CLKO1, OUTPUT_40OHM),
#define GP_SGTL5000_HP_DETECT	IMX_GPIO_NR(2, 0)
	IOMUX_PAD_CTRL(ENET1_COL__GPIO2_IO_0, WEAK_PULLUP),
#define GP_SGTL5000_MIC_DETECT	IMX_GPIO_NR(2, 1)
	IOMUX_PAD_CTRL(ENET1_CRS__GPIO2_IO_1, WEAK_PULLUP),
#define GP_SGTL5000_MUTE		IMX_GPIO_NR(4, 22)
	IOMUX_PAD_CTRL(QSPI1A_SS0_B__GPIO4_IO_22, WEAK_PULLDN_OUTPUT),

	/* uart1 */
	IOMUX_PAD_CTRL(GPIO1_IO04__UART1_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(GPIO1_IO05__UART1_RX, UART_PAD_CTRL),

	/* uart2 */
	IOMUX_PAD_CTRL(GPIO1_IO06__UART2_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(GPIO1_IO07__UART2_RX, UART_PAD_CTRL),

	/* uart3 */
	IOMUX_PAD_CTRL(QSPI1B_SS0_B__UART3_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(QSPI1B_SCLK__UART3_RX, UART_PAD_CTRL),

	/* uart5 */
	IOMUX_PAD_CTRL(KEY_COL3__UART5_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_ROW3__UART5_RX, UART_PAD_CTRL),

	/* USB OTG */
	IOMUX_PAD_CTRL(GPIO1_IO08__USB_OTG1_OC, WEAK_PULLUP),
#define GP_USB_OTG_PWR		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO_9, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(GPIO1_IO10__ANATOP_OTG1_ID, WEAK_PULLUP),

	/* USB OTG2 */
	/* USB Hub Reset for USB2513 3 port hub */
#define GP_USB_HUB_RESET	IMX_GPIO_NR(4, 26)
	IOMUX_PAD_CTRL(QSPI1B_DATA2__GPIO4_IO_26, OUTPUT_40OHM),

	/* usdhc2 */
	IOMUX_PAD_CTRL(SD2_CLK__USDHC2_CLK, USDHC2_CLK_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__USDHC2_CMD, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA0__USDHC2_DATA0, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA1__USDHC2_DATA1, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA2__USDHC2_DATA2, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA3__USDHC2_DATA3, USDHC2_PAD_CTRL),
#define GP_USDHC2_CD	IMX_GPIO_NR(2, 12)
	IOMUX_PAD_CTRL(KEY_COL2__GPIO2_IO_12, WEAK_PULLUP),

	/* usdhc3 - micro SD */
	IOMUX_PAD_CTRL(SD3_CLK__USDHC3_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__USDHC3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA0__USDHC3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA1__USDHC3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA2__USDHC3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA3__USDHC3_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA4__USDHC3_DATA4, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA5__USDHC3_DATA5, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA6__USDHC3_DATA6, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA7__USDHC3_DATA7, USDHC_PAD_CTRL),

	/* usdhc4 - eMMC */
	IOMUX_PAD_CTRL(SD4_CLK__USDHC4_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__USDHC4_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_RESET_B__USDHC4_RESET_B, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA0__USDHC4_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA1__USDHC4_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA2__USDHC4_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA3__USDHC4_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA4__USDHC4_DATA4, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA5__USDHC4_DATA5, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA6__USDHC4_DATA6, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA7__USDHC4_DATA7, USDHC_PAD_CTRL),
};

static const iomux_v3_cfg_t enet_gpio_pads[] = {
#define GP_PHY1_AD0	IMX_GPIO_NR(5, 0)
	IOMUX_PAD_CTRL(RGMII1_RD0__GPIO5_IO_0, WEAK_PULLDN_OUTPUT),
#define GP_PHY1_AD1	IMX_GPIO_NR(5, 1)
	IOMUX_PAD_CTRL(RGMII1_RD1__GPIO5_IO_1, WEAK_PULLDN_OUTPUT),
	/* MODE - 1100 */
#define GP_PHY1_MODE0	IMX_GPIO_NR(5, 4)
	IOMUX_PAD_CTRL(RGMII1_RX_CTL__GPIO5_IO_4, WEAK_PULLDN_OUTPUT),
#define GP_PHY1_MODE1	IMX_GPIO_NR(5, 2)
	IOMUX_PAD_CTRL(RGMII1_RD2__GPIO5_IO_2, WEAK_PULLDN_OUTPUT),
#define GP_PHY1_MODE3	IMX_GPIO_NR(5, 3)
	IOMUX_PAD_CTRL(RGMII1_RD3__GPIO5_IO_3, WEAK_PULLUP_OUTPUT),
	/* 1.8V(1)/1.5V select(0) */
#define GP_PHY1_1P8_SEL	IMX_GPIO_NR(5, 5)
	IOMUX_PAD_CTRL(RGMII1_RXC__GPIO5_IO_5, WEAK_PULLUP_OUTPUT),

#define GP_PHY2_AD0	IMX_GPIO_NR(5, 12)
	IOMUX_PAD_CTRL(RGMII2_RD0__GPIO5_IO_12, WEAK_PULLUP_OUTPUT),
#define GP_PHY2_AD1	IMX_GPIO_NR(5, 13)
	IOMUX_PAD_CTRL(RGMII2_RD1__GPIO5_IO_13, WEAK_PULLDN_OUTPUT),
	/* MODE - 1100 */
#define GP_PHY2_MODE0	IMX_GPIO_NR(5, 16)
	IOMUX_PAD_CTRL(RGMII2_RX_CTL__GPIO5_IO_16, WEAK_PULLDN_OUTPUT),
#define GP_PHY2_MODE1	IMX_GPIO_NR(5, 14)
	IOMUX_PAD_CTRL(RGMII2_RD2__GPIO5_IO_14, WEAK_PULLDN_OUTPUT),
	/* MODE 2 is LED_100, Internal pull up */
#define GP_PHY2_MODE3	IMX_GPIO_NR(5, 15)
	IOMUX_PAD_CTRL(RGMII2_RD3__GPIO5_IO_15, WEAK_PULLUP_OUTPUT),
	/* 1.8V(1)/1.5V select(0) */
#define GP_PHY2_1P8_SEL	IMX_GPIO_NR(5, 17)
	IOMUX_PAD_CTRL(RGMII2_RXC__GPIO5_IO_17, WEAK_PULLUP_OUTPUT),
};

static const iomux_v3_cfg_t enet_pads[] = {
	IOMUX_PAD_CTRL(RGMII1_RD0__ENET1_RX_DATA_0, ENET_RXD_DN_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII1_RD1__ENET1_RX_DATA_1, ENET_RXD_DN_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII1_RX_CTL__ENET1_RX_EN, ENET_RXD_DN_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII1_RD2__ENET1_RX_DATA_2, ENET_RXD_DN_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII1_RD3__ENET1_RX_DATA_3, ENET_RXD_UP_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII1_RXC__ENET1_RX_CLK, ENET_RXD_UP_PAD_CTRL),

	IOMUX_PAD_CTRL(RGMII2_RD0__ENET2_RX_DATA_0, ENET_RXD_UP_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII2_RD1__ENET2_RX_DATA_1, ENET_RXD_DN_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII2_RX_CTL__ENET2_RX_EN, ENET_RXD_DN_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII2_RD2__ENET2_RX_DATA_2, ENET_RXD_DN_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII2_RD3__ENET2_RX_DATA_3, ENET_RXD_UP_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII2_RXC__ENET2_RX_CLK, ENET_RXD_UP_PAD_CTRL),
};

#ifdef CONFIG_CMD_FBPANEL
static const iomux_v3_cfg_t rgb_pads[] = {
	/* LCDIF1 */
	IOMUX_PAD_CTRL(LCD1_CLK__LCDIF1_CLK, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_ENABLE__LCDIF1_ENABLE, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_HSYNC__LCDIF1_HSYNC, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_VSYNC__LCDIF1_VSYNC, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_RESET__LCDIF1_RESET, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA00__LCDIF1_DATA_0, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA01__LCDIF1_DATA_1, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA02__LCDIF1_DATA_2, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA03__LCDIF1_DATA_3, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA04__LCDIF1_DATA_4, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA05__LCDIF1_DATA_5, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA06__LCDIF1_DATA_6, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA07__LCDIF1_DATA_7, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA08__LCDIF1_DATA_8, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA09__LCDIF1_DATA_9, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA10__LCDIF1_DATA_10, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA11__LCDIF1_DATA_11, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA12__LCDIF1_DATA_12, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA13__LCDIF1_DATA_13, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA14__LCDIF1_DATA_14, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA15__LCDIF1_DATA_15, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA16__LCDIF1_DATA_16, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA17__LCDIF1_DATA_17, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA18__LCDIF1_DATA_18, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA19__LCDIF1_DATA_19, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA20__LCDIF1_DATA_20, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA21__LCDIF1_DATA_21, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA22__LCDIF1_DATA_22, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA23__LCDIF1_DATA_23, LCDIF_PAD_CTL),
};
#endif

static struct i2c_pads_info i2c_pads[] = {
	/* I2C1, rv4162 */
	I2C_PADS_INFO_ENTRY(I2C1, GPIO1_IO00, 1, 0, GPIO1_IO01, 1, 1, I2C_PAD_CTRL),
	I2C_PADS_INFO_ENTRY(I2C2, GPIO1_IO02, 1, 2, GPIO1_IO03, 1, 3, I2C_PAD_CTRL),
	I2C_PADS_INFO_ENTRY(I2C3, KEY_COL4, 2, 14, KEY_ROW4, 2, 19, I2C_PAD_CTRL),
};

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);
	return 0;
}

static int setup_fec(void)
{
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	int reg;

	/* Use 125MHz anatop loopback REF_CLK1 for ENET1 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK | IOMUX_GPR1_FEC2_MASK, 0);

	/* Reset AR8031 PHYs */
	gpio_direction_output(GP_RGMII1_PHY_RESET, 0);
	gpio_direction_output(GP_RGMII2_PHY_RESET, 0);
	gpio_direction_output(GP_PHY1_AD0, 0);
	gpio_direction_output(GP_PHY1_AD1, 0);
	gpio_direction_output(GP_PHY1_MODE0, 0);
	gpio_direction_output(GP_PHY1_MODE1, 0);
	gpio_direction_output(GP_PHY1_MODE3, 1);
	gpio_direction_output(GP_PHY1_1P8_SEL, 1);

	gpio_direction_output(GP_PHY2_AD0, 1);
	gpio_direction_output(GP_PHY2_AD1, 0);
	gpio_direction_output(GP_PHY2_MODE0, 0);
	gpio_direction_output(GP_PHY2_MODE1, 0);
	gpio_direction_output(GP_PHY2_MODE3, 1);
	gpio_direction_output(GP_PHY2_1P8_SEL, 1);
	SETUP_IOMUX_PADS(enet_gpio_pads);

	udelay(1000);	/* 1 ms minimum reset pulse */
#ifdef CONFIG_RGMII1
	gpio_set_value(GP_RGMII1_PHY_RESET, 1);
#endif
#ifdef CONFIG_RGMII2
	gpio_set_value(GP_RGMII2_PHY_RESET, 1);
#endif
	/* strap hold time, 2 fails, 3 works, so 5 should be safe */
	udelay(5);

	SETUP_IOMUX_PADS(enet_pads);

	reg = readl(&anatop->pll_enet);
	reg |= BM_ANADIG_PLL_ENET_REF_25M_ENABLE;
	writel(reg, &anatop->pll_enet);
	return enable_fec_anatop_clock(ENET_125MHZ);
}

int board_eth_init(bd_t *bis)
{
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_fec();

#ifdef CONFIG_FEC_MXC
	bus = fec_get_miibus(ENET_MDIO_BASE, -1);
	if (!bus)
		goto usbeth;
	/* scan phy 4(RGMII1) or 5(RGMII2) */
#ifdef CONFIG_RGMII1
	phydev = phy_find_by_mask(bus, (1 << 4), PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		free(bus);
		goto usbeth;
	}
	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, 0, ENET_BASE_ADDR, bus, phydev);
	if (ret) {
		printf("FEC0 MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
		goto usbeth;
	}
#endif
#ifdef CONFIG_RGMII2
	phydev = phy_find_by_mask(bus, (1 << 5), PHY_INTERFACE_MODE_RGMII);
	if (!phydev)
		goto usbeth;
	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, 1, ENET2_BASE_ADDR, bus, phydev);
	if (ret) {
		printf("FEC1 MXC: %s:failed\n", __func__);
		free(phydev);
	}
#endif
usbeth:
#endif

#ifdef CONFIG_CI_UDC
	/* For otg ethernet*/
	usb_eth_initialize(bis);
#endif
	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	int val;

        /*
         * Ar803x phy SmartEEE feature cause link status generates glitch,
         * which cause ethernet link down/up issue, so disable SmartEEE
         */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x3);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x805d);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4003);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val & ~(1 << 8));

	/* rgmii tx clock delay enable */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, (val|0x0100));

	phydev->supported = phydev->drv->features;
	return 0;
}

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : (cs >> 8) ? (cs >> 8) : -1;
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;
	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
			port * 4);
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

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

static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{.esdhc_base = USDHC2_BASE_ADDR, .max_bus_width = 4},
	{.esdhc_base = USDHC4_BASE_ADDR, .max_bus_width = 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int gp_cd = (cfg->esdhc_base == USDHC2_BASE_ADDR) ? GP_USDHC2_CD : -1;

	if (gp_cd < 0)
		return 1;	/* emmc is present */
	gpio_direction_input(gp_cd);
	return !gpio_get_value(gp_cd);
}

int board_mmc_init(bd_t *bis)
{
	int ret0, ret1;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
	ret0 = fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
	ret1 = fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
	return ret0 ? ret0 : ret1;
}

#ifdef CONFIG_CMD_FBPANEL
#if 0
void board_enable_lvds(const struct display_info_t *di, int enable)
{
	gpio_direction_output(GP_BACKLIGHT_LVDS, enable);
	gpio_direction_output(GP_LVDS_ENABLE, enable);
}

void board_enable_lcd(const struct display_info_t *di, int enable)
{
	if (enable)
		SETUP_IOMUX_PADS(rgb_pads);
	else
		SETUP_IOMUX_PADS(rgb_gpio_pads);
	gpio_direction_output(GP_BACKLIGHT_RGB, enable);
}
#endif
/* hdmi settings */
#define _IMX_VD_1280_720M_60(_mode, _detect, _bus, _addr, _flags) \
{\
	.bus	= _bus,\
	.addr	= _addr,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = _flags,\
	.mode	= {\
		.name           = "1280x720M@60",\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 720,\
		.pixclock       = 1000000000000ULL/((1280+216+72+80)*(720+22+3+5)*60),\
		.left_margin    = 220,\
		.right_margin   = 110,\
		.upper_margin   = 20,\
		.lower_margin   = 5,\
		.hsync_len      = 40,\
		.vsync_len      = 5,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define _IMX_VD_1920_1080M_60(_mode, _detect, _bus, _addr, _flags) \
{\
	.bus	= _bus,\
	.addr	= _addr,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = _flags,\
	.mode	= {\
		.name           = "1920x1080M@60",\
		.refresh        = 60,\
		.xres           = 1920,\
		.yres           = 1080,\
		.pixclock       = 1000000000000ULL/((1920+148+88+44)*(1080+36+4+5)*60),\
		.left_margin    = 148,\
		.right_margin   = 88,\
		.upper_margin   = 36,\
		.lower_margin   = 4,\
		.hsync_len      = 44,\
		.vsync_len      = 5,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}


#define _IMX_VD_1024_768M_60(_mode, _detect, _bus, _addr, _flags) \
{\
	.bus	= _bus,\
	.addr	= _addr,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? fbp_detect_i2c : NULL,\
	.enable	= fbp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = _flags,\
	.mode	= {\
		.name           = "1024x768M@60",\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 768,\
		.pixclock       = 1000000000000ULL/((1024+220+40+60)*(768+21+7+10)*60),\
		.left_margin    = 220,\
		.right_margin   = 40,\
		.upper_margin   = 21,\
		.lower_margin   = 7,\
		.hsync_len      = 60,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}


static const struct display_info_t displays[] = {
	/* hdmi/lcd */
	_IMX_VD_1280_720M_60(LCD, 1, 2, 50, 0),
	_IMX_VD_1920_1080M_60(LCD, 0, 2, 50, 0),
	_IMX_VD_1024_768M_60(LCD, 0, 2, 50, 0),

	/* ft5x06 */
	IMX_VD38_HANNSTAR7(LVDS, 1, 2),
	IMX_VD38_AUO_B101EW05(LVDS, 0, 2),
	IMX_VD38_LG1280_800(LVDS, 0, 2),
	IMX_VD38_DT070BTFT(LVDS, 0, 2),
	IMX_VD38_WSVGA(LVDS, 0, 2),

	/* ili210x */
	IMX_VD41_AMP1024_600(LVDS, 1, 2),

	/* egalax_ts */
	IMX_VD04_HANNSTAR(LVDS, 1, 2),
	IMX_VD04_LG9_7(LVDS, 0, 2),

	/* fusion7 specific touchscreen */
	IMX_VD10_FUSION7(LCD, 1, 2),

	IMX_VD_SHARP_LQ101K1LY04(LVDS, 0, 0),
	IMX_VD_WXGA_J(LVDS, 0, 0),
	IMX_VD_WXGA(LVDS, 0, 0),
	IMX_VD_WVGA(LVDS, 0, 0),
	IMX_VD_AA065VE11(LVDS, 0, 0),
	IMX_VD_VGA(LVDS, 0, 0),

	/* tsc2004 */
	IMX_VD48_CLAA_WVGA(LCD, 1, 2),
	IMX_VD48_SHARP_WVGA(LCD, 0, 2),
	IMX_VD48_DC050WX(LCD, 0, 2),
	IMX_VD48_QVGA(LCD, 0, 2),
	IMX_VD48_AT035GT_07ET3(LCD, 0, 2),

	IMX_VD_LSA40AT9001(LCD, 0, 0),
};

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}

#endif

static const unsigned short gpios_out_low[] = {
	GP_RGMII1_PHY_RESET,
	GP_RGMII2_PHY_RESET,
	GP_BACKLIGHT_LVDS,
	GP_BACKLIGHT_RGB,
	GP_USB_HUB_RESET,
	GP_USB_OTG_PWR,
	GP_LVDS_ENABLE,
	GP_PCIE_RESET,
	GP_SGTL5000_MUTE,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,
	GP_CAN1_STANDBY,
	GP_CAN2_STANDBY,
};

static const unsigned short gpios_in[] = {
	GP_RGMII1_PHY_INT,
	GP_RGMII2_PHY_INT,
	GP_USDHC2_CD,
	GP_PCIE_WAKE,
	GP_PCIE_DISABLE,
	GP_PWM3,
	GP_SGTL5000_HP_DETECT,
	GP_SGTL5000_MIC_DETECT,
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

int board_late_init(void)
{
	int cpurev = get_cpu_rev();
	setenv("cpu",get_imx_type((cpurev & 0xFF000) >> 12));
	if (!getenv("board"))
		setenv("board", "nitrogen6sx");
	if (!getenv("uboot_defconfig"))
		setenv("uboot_defconfig", CONFIG_DEFCONFIG);
#ifdef CONFIG_CMD_FBPANEL
	board_video_skip();
#endif
	return 0;
}

static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",        MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"mmc1",        MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,          0},
};

int misc_init_r(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}
int checkboard(void)
{
	puts("Board: Nitrogen6sx\n");
	return 0;
}
