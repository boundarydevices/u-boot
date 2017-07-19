/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2016, Boundary Devices <info@boundarydevices.com>
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
#include <asm/imx-common/spi.h>
#include <mmc.h>
#include <fsl_esdhc.h>
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
#include <usb/ehci-ci.h>
#include "../padctrl.h"

DECLARE_GLOBAL_DATA_PTR;

#define AUD_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define CEC_PAD_CTRL    (PAD_CTL_PUS_22K_UP |                   \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
	PAD_CTL_ODE)

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
	IOMUX_PAD_CTRL(ENET_RXD0__GPIO1_IO27, WEAK_PULLDN),
#define GPIRQ_ENET_PHY		IMX_GPIO_NR(1, 28)
	IOMUX_PAD_CTRL(ENET_TX_EN__GPIO1_IO28, WEAK_PULLUP),

	/* FLEXCAN */
	IOMUX_PAD_CTRL(KEY_COL2__FLEXCAN1_TX, WEAK_PULLUP),
	IOMUX_PAD_CTRL(KEY_ROW2__FLEXCAN1_RX, WEAK_PULLUP),
#define GP_FLEXCAN_STANDBY	IMX_GPIO_NR(1, 4)
	IOMUX_PAD_CTRL(GPIO_4__GPIO1_IO04, WEAK_PULLUP_OUTPUT),

	/* GPIO output assignments */
#define GP_POWER_OFF		IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO_3__GPIO1_IO03, WEAK_PULLDN_OUTPUT),	/* 0 is on */
#define GP_RBL			IMX_GPIO_NR(3, 0)
	IOMUX_PAD_CTRL(EIM_DA0__GPIO3_IO00, WEAK_PULLDN_OUTPUT),
#define GP_FAN1			IMX_GPIO_NR(3, 1)
	IOMUX_PAD_CTRL(EIM_DA1__GPIO3_IO01, WEAK_PULLDN_OUTPUT),
#define GP_FAN2			IMX_GPIO_NR(3, 2)
	IOMUX_PAD_CTRL(EIM_DA2__GPIO3_IO02, WEAK_PULLDN_OUTPUT),
#define GP_FAN3			IMX_GPIO_NR(3, 3)
	IOMUX_PAD_CTRL(EIM_DA3__GPIO3_IO03, WEAK_PULLDN_OUTPUT),
#define GP_TIME_ACTIVE		IMX_GPIO_NR(3, 4)
	IOMUX_PAD_CTRL(EIM_DA4__GPIO3_IO04, WEAK_PULLDN_OUTPUT),
#define GP_PRG_L3		IMX_GPIO_NR(3, 5)
	IOMUX_PAD_CTRL(EIM_DA5__GPIO3_IO05, WEAK_PULLDN_OUTPUT),
#define GP_PRG_L2		IMX_GPIO_NR(3, 6)
	IOMUX_PAD_CTRL(EIM_DA6__GPIO3_IO06, WEAK_PULLDN_OUTPUT),
#define GP_PRG_L1		IMX_GPIO_NR(3, 7)
	IOMUX_PAD_CTRL(EIM_DA7__GPIO3_IO07, WEAK_PULLDN_OUTPUT),
#define GP_RESET_RBL		IMX_GPIO_NR(6, 2)
	IOMUX_PAD_CTRL(CSI0_DAT16__GPIO6_IO02, WEAK_PULLDN_OUTPUT),
#define GP_TEST_RBL		IMX_GPIO_NR(6, 3)
	IOMUX_PAD_CTRL(CSI0_DAT17__GPIO6_IO03, WEAK_PULLDN_OUTPUT),
#define GP_ENABLE_BSL_CB	IMX_GPIO_NR(6, 4)
	IOMUX_PAD_CTRL(CSI0_DAT18__GPIO6_IO04, WEAK_PULLDN_OUTPUT),
#define GP_ENABLE_BSL_OB	IMX_GPIO_NR(6, 5)
	IOMUX_PAD_CTRL(CSI0_DAT19__GPIO6_IO05, WEAK_PULLDN_OUTPUT),
#define GP_BSL_CB_ON_OFF	IMX_GPIO_NR(4, 8)
	IOMUX_PAD_CTRL(KEY_COL1__GPIO4_IO08, WEAK_PULLDN_OUTPUT),
#define GP_CALIBRATION_FLAG	IMX_GPIO_NR(4, 9)
	IOMUX_PAD_CTRL(KEY_ROW1__GPIO4_IO09, WEAK_PULLDN_OUTPUT),

	/* GPIO_KEYS assignments */
#define GP_GPIOKEY_SW1		IMX_GPIO_NR(2, 18)
	IOMUX_PAD_CTRL(EIM_A20__GPIO2_IO18, WEAK_PULLUP),
#define GP_GPIOKEY_SW2		IMX_GPIO_NR(2, 19)
	IOMUX_PAD_CTRL(EIM_A19__GPIO2_IO19, WEAK_PULLUP),
#define GP_GPIOKEY_SW3		IMX_GPIO_NR(2, 20)
	IOMUX_PAD_CTRL(EIM_A18__GPIO2_IO20, WEAK_PULLUP),
#define GP_GPIOKEY_SW4		IMX_GPIO_NR(2, 21)
	IOMUX_PAD_CTRL(EIM_A17__GPIO2_IO21, WEAK_PULLUP),
#define GP_GPIOKEY_SW5		IMX_GPIO_NR(2, 22)
	IOMUX_PAD_CTRL(EIM_A16__GPIO2_IO22, WEAK_PULLUP),
#define GP_GPIOKEY_POWER	IMX_GPIO_NR(1, 2)
	IOMUX_PAD_CTRL(GPIO_2__GPIO1_IO02, WEAK_PULLUP),

#define GP_GPIOKEY_CH_ON_RBL	IMX_GPIO_NR(5, 27)
	IOMUX_PAD_CTRL(CSI0_DAT9__GPIO5_IO27, WEAK_PULLUP),
#define GP_GPIOKEY_SG_ON_RBL	IMX_GPIO_NR(5, 28)
	IOMUX_PAD_CTRL(CSI0_DAT10__GPIO5_IO28, WEAK_PULLUP),
#define GP_GPIOKEY_DOOR_CLOSED	IMX_GPIO_NR(5, 29)
	IOMUX_PAD_CTRL(CSI0_DAT11__GPIO5_IO29, WEAK_PULLUP),

	/* hdmi_cec */
	IOMUX_PAD_CTRL(EIM_A25__HDMI_TX_CEC_LINE, CEC_PAD_CTRL),

	/* Hog Test points */
#define GP_TP74			IMX_GPIO_NR(2, 7)
	IOMUX_PAD_CTRL(NANDF_D7__GPIO2_IO07, WEAK_PULLUP),
#define GP_TP84			IMX_GPIO_NR(2, 30)
	IOMUX_PAD_CTRL(EIM_EB2__GPIO2_IO30, WEAK_PULLUP),
#define GP_TP85			IMX_GPIO_NR(2, 31)
	IOMUX_PAD_CTRL(EIM_EB3__GPIO2_IO31, WEAK_PULLUP),
#define GP_TP_R218		IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO_3__GPIO1_IO03, WEAK_PULLUP),

	/* i2c1_rv4172 rtc */
#define GPIRQ_RTC_RV4162	IMX_GPIO_NR(7, 12)
	IOMUX_PAD_CTRL(GPIO_17__GPIO7_IO12, WEAK_PULLUP),

	/* i2c1_sgtl5000 */
	IOMUX_PAD_CTRL(GPIO_0__CCM_CLKO1, OUTPUT_40OHM),	/* SGTL5000 sys_mclk */
#define GP_TDA7491P_GAIN0	IMX_GPIO_NR(5, 4)
	IOMUX_PAD_CTRL(EIM_A24__GPIO5_IO04, WEAK_PULLDN_OUTPUT),
#define GP_TDA7491P_GAIN1	IMX_GPIO_NR(6, 6)
	IOMUX_PAD_CTRL(EIM_A23__GPIO6_IO06, WEAK_PULLDN_OUTPUT),
#define GP_TDA7491P_STBY	IMX_GPIO_NR(6, 31)
	IOMUX_PAD_CTRL(EIM_BCLK__GPIO6_IO31, WEAK_PULLDN_OUTPUT),
#define GP_TDA7491P_MUTE	IMX_GPIO_NR(5, 0)
	IOMUX_PAD_CTRL(EIM_WAIT__GPIO5_IO00, WEAK_PULLDN_OUTPUT),
#define GPIRQ_MIC_DET		IMX_GPIO_NR(7, 8)
	IOMUX_PAD_CTRL(SD3_RST__GPIO7_IO08, WEAK_PULLUP),

	/* i2c2 AR1021 */
#define GPIRQ_AR1021		IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO_7__GPIO1_IO07, WEAK_PULLDN),	/* High active */
#define GP_AR1021_5WIRE		IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO_8__GPIO1_IO08, WEAK_PULLUP),

	/* i2c2 ov5640 Mipi Camera */
	IOMUX_PAD_CTRL(NANDF_CS2__CCM_CLKO2, OUTPUT_40OHM),
#define GP_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(6, 7)
	IOMUX_PAD_CTRL(NANDF_CLE__GPIO6_IO07, WEAK_PULLUP),
#define GP_OV5640_MIPI_RESET	IMX_GPIO_NR(6, 8)
	IOMUX_PAD_CTRL(NANDF_ALE__GPIO6_IO08, WEAK_PULLDN),

	/* i2c3 J6 */
#define GPIRQ_I2C3_J6		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO_9__GPIO1_IO09, WEAK_PULLUP),

#define GPIRQ_I2C3_RGB		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, WEAK_PULLUP),

	/* PCIe */
#define GP_PCIE_RESET		IMX_GPIO_NR(4, 7)
	IOMUX_PAD_CTRL(KEY_ROW0__GPIO4_IO07, WEAK_PULLDN),
#define GP_PCIE_DISABLE		IMX_GPIO_NR(4, 6)
	IOMUX_PAD_CTRL(KEY_COL0__GPIO4_IO06, WEAK_PULLDN),

	/* PWM1 - Backlight on RGB connector: J15, pin 37 */
#define GP_BACKLIGHT_RGB	IMX_GPIO_NR(1, 21)
	IOMUX_PAD_CTRL(SD1_DAT3__GPIO1_IO21, WEAK_PULLDN),

	/*
	 * PWM4 - Backlight on LVDS connector: J4, pin 3
	 * 0 is bright, 1 is dim
	 */
#define GP_BACKLIGHT_LVDS	IMX_GPIO_NR(1, 18)
	IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, WEAK_PULLUP),
	/* 0 is 8 bit */
#define GP_8BIT_LVDS		IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(KEY_ROW4__GPIO4_IO15, WEAK_PULLDN_OUTPUT),
#define GP_BACKLIGHT_LVDS_EN	IMX_GPIO_NR(7, 13)
	IOMUX_PAD_CTRL(GPIO_18__GPIO7_IO13, WEAK_PULLDN),

	/* reg_usbotg_vbus */
#define GP_REG_USBOTG		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLDN),

	/* UART1 */
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),
#define GP_UART1_RX_EN		IMX_GPIO_NR(3, 14)
	IOMUX_PAD_CTRL(EIM_DA14__GPIO3_IO14, WEAK_PULLDN_OUTPUT),	/* RS485 RX Enable: pull down */
#define GP_UART1_TX_EN		IMX_GPIO_NR(3, 15)
	IOMUX_PAD_CTRL(EIM_DA15__GPIO3_IO15, WEAK_PULLDN_OUTPUT),	/* RS485 DEN: pull down */
#define GP_UART1_RS485_EN	IMX_GPIO_NR(3, 13)
	IOMUX_PAD_CTRL(EIM_DA13__GPIO3_IO13, WEAK_PULLDN_OUTPUT),	/* RS485/!RS232 Select: pull down (rs232) */
#define GP_UART1_AON		IMX_GPIO_NR(3, 12)
	IOMUX_PAD_CTRL(EIM_DA12__GPIO3_IO12, WEAK_PULLDN_OUTPUT),	/* ON: pull down */
#define GP_UART1_RS485_TERM	IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(GPIO_19__GPIO4_IO05, WEAK_PULLDN_OUTPUT),	/* pull down */

	/* UART2 */
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),

	/* UART3 */
	IOMUX_PAD_CTRL(EIM_D24__UART3_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D25__UART3_RX_DATA, UART_PAD_CTRL),

	/* USBH1 */
	IOMUX_PAD_CTRL(EIM_D30__USB_H1_OC, WEAK_PULLUP),

	/* USBOTG */
	IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, WEAK_PULLUP),
	IOMUX_PAD_CTRL(KEY_COL4__USB_OTG_OC, WEAK_PULLUP),

	/* USDHC3 - sdcard */
#define GP_USDHC3_CD		IMX_GPIO_NR(7, 0)
	IOMUX_PAD_CTRL(SD3_DAT5__GPIO7_IO00, WEAK_PULLUP),
#define GP_USDHC3_POWER_EN	IMX_GPIO_NR(1, 30)
	IOMUX_PAD_CTRL(ENET_TXD0__GPIO1_IO30, WEAK_PULLUP),	/* Pullup so that bmod mmc0 works */

	/* USDHC4 - emmc */
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT4__SD4_DATA4, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT5__SD4_DATA5, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT6__SD4_DATA6, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT7__SD4_DATA7, USDHC_PAD_CTRL),
#define GP_EMMC_RESET		IMX_GPIO_NR(2, 6)
	IOMUX_PAD_CTRL(NANDF_D6__GPIO2_IO06, WEAK_PULLUP),
};

static const iomux_v3_cfg_t sd3_usdhc3_pads[] = {
	IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
};

static const iomux_v3_cfg_t sd3_gpio_pads[] = {
#define GP_USDHC3_CLK	IMX_GPIO_NR(7, 3)
	IOMUX_PAD_CTRL(SD3_CLK__GPIO7_IO03, WEAK_PULLDN),
#define GP_USDHC3_CMD	IMX_GPIO_NR(7, 2)
	IOMUX_PAD_CTRL(SD3_CMD__GPIO7_IO02, WEAK_PULLDN),
#define GP_USDHC3_DAT0	IMX_GPIO_NR(7, 4)
	IOMUX_PAD_CTRL(SD3_DAT0__GPIO7_IO04, WEAK_PULLDN),
#define GP_USDHC3_DAT1	IMX_GPIO_NR(7, 5)
	IOMUX_PAD_CTRL(SD3_DAT1__GPIO7_IO05, WEAK_PULLDN),
#define GP_USDHC3_DAT2	IMX_GPIO_NR(7, 6)
	IOMUX_PAD_CTRL(SD3_DAT2__GPIO7_IO06, WEAK_PULLDN),
#define GP_USDHC3_DAT3	IMX_GPIO_NR(7, 7)
	IOMUX_PAD_CTRL(SD3_DAT3__GPIO7_IO07, WEAK_PULLDN),
};

static const iomux_v3_cfg_t rgb_pads[] = {
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

#include "../eth.c"

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
//	printf("%s:%p *%p=0x%lx\n", __func__, gd, &gd->ram_size, gd->ram_size);
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
	gpio_set_value(GP_REG_USBOTG, on);
	return 0;
}

#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{.esdhc_base = USDHC3_BASE_ADDR, .max_bus_width = 4},
	{.esdhc_base = USDHC4_BASE_ADDR, .max_bus_width = 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

	if (cfg->esdhc_base == USDHC4_BASE_ADDR)
		return 1;	/* eMMC always present */
	return !gpio_get_value(GP_USDHC3_CD);
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
			mdelay(2);
			SETUP_IOMUX_PADS(sd3_usdhc3_pads);
			gpio_set_value(GP_USDHC3_POWER_EN, 1);
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
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : -1;
}
#endif

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
	gpio_set_value(GP_8BIT_LVDS,
			(di->pixfmt == IPU_PIX_FMT_RGB666) ? 1 : 0);
	gpio_set_value(GP_BACKLIGHT_LVDS, enable ^ 1);
	gpio_set_value(GP_BACKLIGHT_LVDS_EN, enable);
}

void board_enable_lcd(const struct display_info_t *di, int enable)
{
	if (enable)
		SETUP_IOMUX_PADS(rgb_pads);
	else
		SETUP_IOMUX_PADS(rgb_gpio_pads);
	gpio_set_value(GP_BACKLIGHT_RGB, enable);
}

static const struct display_info_t displays[] = {
	/* lvds */
	VD_WVGA_TX23D200_18(LVDS, NULL, 0, 0x00),
	VD_WVGA_TX23D200_24(LVDS, NULL, 0, 0x00),

	/* hdmi */
	VD_1280_720M_60(HDMI, fbp_detect_i2c, 1, 0x50),
	VD_1920_1080M_60(HDMI, NULL, 1, 0x50),
	VD_1024_768M_60(HDMI, NULL, 1, 0x50),

	/* tsc2004 */
	VD_CLAA_WVGA(LCD, fbp_detect_i2c, 2, 0x48),
	VD_SHARP_WVGA(LCD, NULL, 2, 0x48),
	VD_DC050WX(LCD, NULL, 2, 0x48),

	/* fusion7 specific touchscreen */
	VD_FUSION7(LCD, fbp_detect_i2c, 2, 0x10),
};

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}
#endif

static const unsigned short gpios_out_low[] = {
	GP_RGMII_PHY_RESET,
	GP_POWER_OFF,
	GP_RBL,
	GP_FAN1,
	GP_FAN2,
	GP_FAN3,
	GP_TIME_ACTIVE,
	GP_PRG_L3,
	GP_PRG_L2,
	GP_PRG_L1,
	GP_RESET_RBL,
	GP_TEST_RBL,
	GP_ENABLE_BSL_CB,
	GP_ENABLE_BSL_OB,
	GP_BSL_CB_ON_OFF,
	GP_CALIBRATION_FLAG,
	GP_TDA7491P_GAIN0,
	GP_TDA7491P_GAIN1,
	GP_TDA7491P_STBY,
	GP_TDA7491P_MUTE,
	GP_OV5640_MIPI_RESET,	/* camera reset */
	GP_PCIE_RESET,
	GP_BACKLIGHT_RGB,
	GP_8BIT_LVDS,
	GP_BACKLIGHT_LVDS_EN,
	GP_REG_USBOTG,		/* disable USB otg power */
	GP_EMMC_RESET,		/* hold in reset */
	GP_UART1_RX_EN,
	GP_UART1_TX_EN,
	GP_UART1_RS485_EN,
	GP_UART1_AON,
	GP_UART1_RS485_TERM,
	GP_USDHC3_CLK,
	GP_USDHC3_CMD,
	GP_USDHC3_DAT0,
	GP_USDHC3_DAT1,
	GP_USDHC3_DAT2,
	GP_USDHC3_DAT3,
	GP_USDHC3_POWER_EN,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,	/* SS1 of spi nor */
	GP_FLEXCAN_STANDBY,
	GP_OV5640_MIPI_POWER_DOWN,	/* camera power down */
	GP_BACKLIGHT_LVDS,
};

static const unsigned short gpios_in[] = {
	GPIRQ_ENET_PHY,
	GP_GPIOKEY_SW1,
	GP_GPIOKEY_SW2,
	GP_GPIOKEY_SW3,
	GP_GPIOKEY_SW4,
	GP_GPIOKEY_SW5,
	GP_GPIOKEY_POWER,
	GP_GPIOKEY_CH_ON_RBL,
	GP_GPIOKEY_SG_ON_RBL,
	GP_GPIOKEY_DOOR_CLOSED,
	GPIRQ_RTC_RV4162,
	GPIRQ_MIC_DET,
	GPIRQ_AR1021,
	GP_AR1021_5WIRE,
	GPIRQ_I2C3_J6,
	GP_PCIE_DISABLE,
	GP_TP74,
	GP_TP84,
	GP_TP85,
	GP_TP_R218,
	GP_USDHC3_CD,
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
	SETUP_IOMUX_PADS(sd3_gpio_pads);
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

void board_poweroff(void)
{
	/*
	 * make all sd3 lines low so that voltage drops quicker.
	 * Without this 10ms delay was not enough, now 2 is enough.
	 */
	SETUP_IOMUX_PADS(sd3_gpio_pads);
	gpio_set_value(GP_USDHC3_POWER_EN, 0);
	mdelay(2);
	gpio_set_value(GP_POWER_OFF, 1);
	mdelay(500);
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
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, ARRAY_SIZE(displays));
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: Ash\n");
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"sw1",		GP_GPIOKEY_SW1,		'1'},
	{"sw2",		GP_GPIOKEY_SW2,		'2'},
	{"sw3",		GP_GPIOKEY_SW3,		'3'},
	{"sw4",		GP_GPIOKEY_SW4,		'4'},
	{"sw5",		GP_GPIOKEY_SW5,		'5'},
	{"power",	GP_GPIOKEY_POWER,	'P'},
	{"ch",		GP_GPIOKEY_CH_ON_RBL,	'C'},
	{"sg",		GP_GPIOKEY_SG_ON_RBL,	'S'},
	{"door",	GP_GPIOKEY_DOOR_CLOSED, 'D'},
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

	setenv("cpu", get_imx_type((cpurev & 0xFF000) >> 12));
	if (!getenv("board"))
		setenv("board","ash");
	setenv("uboot_defconfig", CONFIG_DEFCONFIG);
	return 0;
}

static int _do_poweroff(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	board_poweroff();
	return 0;
}

U_BOOT_CMD(
	poweroff, 70, 0, _do_poweroff,
	"power down board",
	""
);
