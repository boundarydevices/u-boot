/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
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
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <splash.h>
#include <usb/ehci-ci.h>
#include "../common/bd_common.h"
#include "../common/padctrl.h"

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
	IOMUX_PAD_CTRL(CSI0_DAT4__AUD3_TXC, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT5__AUD3_TXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT6__AUD3_TXFS, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT7__AUD3_RXD, AUD_PAD_CTRL),

	/* bt_rfkill */
#define GP_BT_RFKILL_RESET	IMX_GPIO_NR(6, 16)
	IOMUX_PAD_CTRL(NANDF_CS3__GPIO6_IO16, WEAK_PULLDN),

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

	/* GPIO_KEYS assignments for J14 */
#define GP_GPIOKEY_BACK		IMX_GPIO_NR(2, 2)
	IOMUX_PAD_CTRL(NANDF_D2__GPIO2_IO02, BUTTON_PAD_CTRL),
#define GP_GPIOKEY_HOME		IMX_GPIO_NR(2, 4)
	IOMUX_PAD_CTRL(NANDF_D4__GPIO2_IO04, BUTTON_PAD_CTRL),
#define GP_GPIOKEY_MENU		IMX_GPIO_NR(2, 1)
	IOMUX_PAD_CTRL(NANDF_D1__GPIO2_IO01, BUTTON_PAD_CTRL),
#define GP_GPIOKEY_SEARCH	IMX_GPIO_NR(2, 3)
	IOMUX_PAD_CTRL(NANDF_D3__GPIO2_IO03, BUTTON_PAD_CTRL),
#define GP_GPIOKEY_VOL_DOWN	IMX_GPIO_NR(7, 1)
	IOMUX_PAD_CTRL(SD3_DAT4__GPIO7_IO01, BUTTON_PAD_CTRL),
#define GP_GPIOKEY_VOL_UP	IMX_GPIO_NR(7, 13)
	IOMUX_PAD_CTRL(GPIO_18__GPIO7_IO13, BUTTON_PAD_CTRL),

	/* hdmi_cec */
	IOMUX_PAD_CTRL(EIM_A25__HDMI_TX_CEC_LINE, CEC_PAD_CTRL),

	/* Hogs */
#define GP_GPIO_1		IMX_GPIO_NR(4, 23)
	IOMUX_PAD_CTRL(DISP0_DAT2__GPIO4_IO23, WEAK_PULLUP),
#define GP_GPIO_2		IMX_GPIO_NR(4, 21)
	IOMUX_PAD_CTRL(DISP0_DAT0__GPIO4_IO21, WEAK_PULLUP),
#define GP_GPIO_3		IMX_GPIO_NR(4, 22)
	IOMUX_PAD_CTRL(DISP0_DAT1__GPIO4_IO22, WEAK_PULLUP),
#define GP_GPIO_4		IMX_GPIO_NR(4, 24)
	IOMUX_PAD_CTRL(DISP0_DAT3__GPIO4_IO24, WEAK_PULLUP),
#define GP_GPIO_5		IMX_GPIO_NR(4, 25)
	IOMUX_PAD_CTRL(DISP0_DAT4__GPIO4_IO25, WEAK_PULLUP),
#define GP_GPIO_6		IMX_GPIO_NR(4, 26)
	IOMUX_PAD_CTRL(DISP0_DAT5__GPIO4_IO26, WEAK_PULLUP),
#define GP_GPIO_7		IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(SD1_DAT1__GPIO1_IO17, WEAK_PULLUP),
#define GP_GPIO_8		IMX_GPIO_NR(4, 31)
	IOMUX_PAD_CTRL(DISP0_DAT10__GPIO4_IO31, WEAK_PULLUP),
#define GP_GPIO_9		IMX_GPIO_NR(5, 12)
	IOMUX_PAD_CTRL(DISP0_DAT18__GPIO5_IO12, WEAK_PULLUP),
#define GP_GPIO_10		IMX_GPIO_NR(5, 5)
	IOMUX_PAD_CTRL(DISP0_DAT11__GPIO5_IO05, WEAK_PULLUP),
#define GP_GPIO_11		IMX_GPIO_NR(5, 11)
	IOMUX_PAD_CTRL(DISP0_DAT17__GPIO5_IO11, WEAK_PULLUP),
#define GP_GPIO_12		IMX_GPIO_NR(5, 6)
	IOMUX_PAD_CTRL(DISP0_DAT12__GPIO5_IO06, WEAK_PULLUP),
#define GP_GPIO_13		IMX_GPIO_NR(5, 10)
	IOMUX_PAD_CTRL(DISP0_DAT16__GPIO5_IO10, WEAK_PULLUP),
#define GP_GPIO_14		IMX_GPIO_NR(5, 7)
	IOMUX_PAD_CTRL(DISP0_DAT13__GPIO5_IO07, WEAK_PULLUP),
#define GP_GPIO_15		IMX_GPIO_NR(5, 9)
	IOMUX_PAD_CTRL(DISP0_DAT15__GPIO5_IO09, WEAK_PULLUP),
#define GP_GPIO_16		IMX_GPIO_NR(5, 8)
	IOMUX_PAD_CTRL(DISP0_DAT14__GPIO5_IO08, WEAK_PULLUP),

#define GP_BT_CLK_REQ		IMX_GPIO_NR(6, 8)
	IOMUX_PAD_CTRL(NANDF_ALE__GPIO6_IO08, WEAK_PULLUP),
#define GP_BT_HOST_WAKE		IMX_GPIO_NR(6, 7)
	IOMUX_PAD_CTRL(NANDF_CLE__GPIO6_IO07, WEAK_PULLUP),
#define GP_WIFI_QOW		IMX_GPIO_NR(2, 5)
	IOMUX_PAD_CTRL(NANDF_D5__GPIO2_IO05, WEAK_PULLUP),

	/* Hog Test points */
#define GP_TP71			IMX_GPIO_NR(1, 30)
	IOMUX_PAD_CTRL(ENET_TXD0__GPIO1_IO30, WEAK_PULLUP),
#define GP_TP74			IMX_GPIO_NR(2, 7)
	IOMUX_PAD_CTRL(NANDF_D7__GPIO2_IO07, WEAK_PULLUP),
#define GP_TP101		IMX_GPIO_NR(3, 30)
	IOMUX_PAD_CTRL(EIM_D30__GPIO3_IO30, WEAK_PULLUP),
#define GP_TP102		IMX_GPIO_NR(5, 0)
	IOMUX_PAD_CTRL(EIM_WAIT__GPIO5_IO00, WEAK_PULLUP),

	/* i2c1a_rv4172 rtc */
#define GPIRQ_RTC_RV4162	IMX_GPIO_NR(1, 4)
	IOMUX_PAD_CTRL(GPIO_4__GPIO1_IO04, WEAK_PULLUP),

	/* Rev 0, needs to be gpio input */
#define GP_I2C1_ALT_SCL	IMX_GPIO_NR(4, 30)
	IOMUX_PAD_CTRL(DISP0_DAT9__GPIO4_IO30, WEAK_PULLUP),
#define GP_I2C1_ALT_SDA	IMX_GPIO_NR(4, 29)
	IOMUX_PAD_CTRL(DISP0_DAT8__GPIO4_IO29, WEAK_PULLUP),

	/* I2C1b - scl alternate */
	IOMUX_PAD_CTRL(CSI0_DAT9__GPIO5_IO27, WEAK_PULLUP),
	/* I2C1b - sda alternate */
	IOMUX_PAD_CTRL(CSI0_DAT8__GPIO5_IO26, WEAK_PULLUP),

	/* I2C1a -  WM8960 */
	IOMUX_PAD_CTRL(GPIO_0__CCM_CLKO1, WEAK_PULLDN),

	/* I2C2 - DS90UB927QSQ serializer 2 */
	/*
	 * Used to enable LP8860 on deserializer board,
	 * DISP1_CONTRAST on schematic
	 */
#define GP_SER2_GPIO5		IMX_GPIO_NR(2, 20)
	IOMUX_PAD_CTRL(EIM_A18__GPIO2_IO20, WEAK_PULLUP),
#define GP_SER2_GPIO6		IMX_GPIO_NR(2, 19)
	IOMUX_PAD_CTRL(EIM_A19__GPIO2_IO19, WEAK_PULLUP),
#define GP_SER2_FAN_OK		IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO_8__GPIO1_IO08, WEAK_PULLUP),
#define GPIRQ_SER2		IMX_GPIO_NR(2, 31)
	IOMUX_PAD_CTRL(EIM_EB3__GPIO2_IO31, WEAK_PULLUP),

	/* I2C2 - serializer 2, gt911 touch controller */
#define GPIRQ_I2C2_SER2_TOUCH	IMX_GPIO_NR(1, 16)		/* GP0 */
	IOMUX_PAD_CTRL(SD1_DAT0__GPIO1_IO16, WEAK_PULLDN),
#define GP_I2C2_GT911_RESET	IMX_GPIO_NR(1, 19)		/* GP3 */
	IOMUX_PAD_CTRL(SD1_DAT2__GPIO1_IO19, WEAK_PULLDN),

	/* I2C2 - serializer 2, LP8860 backlight */
#define GP_LVDS2_LP8860_RESET	IMX_GPIO_NR(2, 23)		/* GP1 */
	IOMUX_PAD_CTRL(EIM_CS0__GPIO2_IO23, WEAK_PULLDN),

	/* I2C3 - DS90UB927QSQ serializer 1 */
	/*
	 * Used to enable LP8860 on deserializer board,
	 * DISP0_CONTRAST on schematic
	 */
#define GP_SER1_GPIO5		IMX_GPIO_NR(2, 22)
	IOMUX_PAD_CTRL(EIM_A16__GPIO2_IO22, WEAK_PULLUP),
#define GP_SER1_GPIO6		IMX_GPIO_NR(2, 21)
	IOMUX_PAD_CTRL(EIM_A17__GPIO2_IO21, WEAK_PULLUP),
#define GP_SER1_FAN_OK		IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(GPIO_19__GPIO4_IO05, WEAK_PULLUP),
#define GPIRQ_SER1		IMX_GPIO_NR(2, 30)
	IOMUX_PAD_CTRL(EIM_EB2__GPIO2_IO30, WEAK_PULLUP),

	/* I2C3 - serializer 1, gt911 touch controller */
#define GPIRQ_I2C3_SER1_TOUCH	IMX_GPIO_NR(1, 9)		/* GP0 */
	IOMUX_PAD_CTRL(GPIO_9__GPIO1_IO09, WEAK_PULLDN),
#define GP_I2C3_GT911_RESET	IMX_GPIO_NR(1, 18)		/* GP3 */
	IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, WEAK_PULLDN),

	/* I2C3 - serializer 1, LP8860 backlight */
#define GP_LVDS_LP8860_RESET	IMX_GPIO_NR(2, 0)		/* GP1 */
	IOMUX_PAD_CTRL(NANDF_D0__GPIO2_IO00, WEAK_PULLDN),

	/* PCIe */
#define GP_PCIE_RESET		IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO_3__GPIO1_IO03, WEAK_PULLDN),
#define GP_PCIE_DISABLE		IMX_GPIO_NR(1, 2)
	IOMUX_PAD_CTRL(GPIO_2__GPIO1_IO02, WEAK_PULLDN),



	/* reg_usbotg_vbus */
#define GP_REG_USBOTG		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLDN),

	/* reg_wlan_en */
#define GP_REG_WLAN_EN		IMX_GPIO_NR(6, 15)
	IOMUX_PAD_CTRL(NANDF_CS2__GPIO6_IO15, WEAK_PULLDN),

	/* UART1 */
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),

	/* UART2 */
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),

	/* UART3 for wl1271 */
	IOMUX_PAD_CTRL(EIM_D24__UART3_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D25__UART3_RX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D23__UART3_CTS_B, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D31__UART3_RTS_B, UART_PAD_CTRL),

	/* UART4 - J58, J63 (RS485 half duplex)  */
	IOMUX_PAD_CTRL(KEY_COL0__UART4_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_ROW0__UART4_RX_DATA, UART_PAD_CTRL),
#define GP_UART4_TX_EN		IMX_GPIO_NR(4, 10)
	IOMUX_PAD_CTRL(KEY_COL2__GPIO4_IO10, WEAK_PULLDN),


	/* UART5 - J54, J63 (RS485 half duplex)  */
	IOMUX_PAD_CTRL(CSI0_DAT14__UART5_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT15__UART5_RX_DATA, UART_PAD_CTRL),
	/* RS485 RX Enable */
#define GP_UART5_RX_EN		IMX_GPIO_NR(4, 9)
	IOMUX_PAD_CTRL(KEY_ROW1__GPIO4_IO09, WEAK_PULLDN),
	/* RS485 TX Enable */
#define GP_UART5_TX_EN		IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO_7__GPIO1_IO07, WEAK_PULLDN),
	/* RS485/RS232 Select 2.5V */
#define GP_UART5_RS485_EN	IMX_GPIO_NR(4, 8)
	IOMUX_PAD_CTRL(KEY_COL1__GPIO4_IO08, WEAK_PULLDN),
	/* ON - meaning depends on others */
#define GP_UART5_AON		IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(KEY_ROW4__GPIO4_IO15, WEAK_PULLDN),

	/* USBH1 */
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
//	IOMUX_PAD_CTRL(SD1_CLK__OSC32K_32K_OUT, OUTPUT_40OHM),	/* slow clock */
#define GPIRQ_WIFI		IMX_GPIO_NR(6, 14)
	IOMUX_PAD_CTRL(NANDF_CS1__GPIO6_IO14, WEAK_PULLDN),
#define GP_WIFI_WAKE		IMX_GPIO_NR(6, 10)
	IOMUX_PAD_CTRL(NANDF_RB0__GPIO6_IO10, WEAK_PULLUP),

	/* USDHC3 - sdcard */
	IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
#define GP_USDHC3_CD		IMX_GPIO_NR(7, 0)
	IOMUX_PAD_CTRL(SD3_DAT5__GPIO7_IO00, WEAK_PULLUP),

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
	IOMUX_PAD_CTRL(NANDF_D6__GPIO2_IO06, WEAK_PULLDN),

};

static const struct i2c_pads_info i2c_pads[] = {
	/* I2C1a, RV4162(rtc, 0x68) */
	/* I2C1b, wm8960 */
	I2C_PADS_INFO_ENTRY(I2C1, EIM_D21, 3, 21, EIM_D28, 3, 28, I2C_PAD_CTRL),
	/* I2C2 PCIe, hdmi edid, serializer 2(0x0c) */
	I2C_PADS_INFO_ENTRY(I2C2, KEY_COL3, 4, 12, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
	/* I2C3, serializer 1 */
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
	{.esdhc_base = USDHC3_BASE_ADDR, .bus_width = 4,
			.gp_cd = GP_USDHC3_CD},
	{.esdhc_base = USDHC4_BASE_ADDR, .bus_width = 8,
			.gp_reset = GP_EMMC_RESET},
};
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : -1;
}
#endif

#ifdef CONFIG_CMD_FBPANEL
static unsigned char setup_serializer_remote[] = {
	0x0c, 0x03, 0xda,	/* passthough i2c accesses to de-serialized/backlight */
	0x0c, 0x07, 0x5a,	/* setup backlight lp8860 address */
	0x0c, 0x08, 0x5a,
	0x0c, 0x77, 0xba,	/* setup gt911 touch controller address */
	0x0c, 0x70, 0xba,
	0x0c, 0x0d, 0x05,	/* gpio0 output from de-serializer */
};

static unsigned char setup_serializer_data[] = {
	0x2c, 0x1d, 0x03,
	0x0c, 0x0f, 0x03,	/* gpio3 output to de-serializer */
	0x2c, 0x1f, 0x05,
#if 1
	0x0c, 0x0e, 0x03,	/* gpio1 output to de-serializer */
	0x2c, 0x1e, 0x05,
#else
	0x2c, 0x1e, 0x01,	/* gpio1 local to de-serializer, low */
	0x2c, 0x1e, 0x09,	/* gpio1 local to de-serializer, high */
#endif
};

static unsigned char enable_backlight_data[] = {
	0x2d, 0x00, 0xff,	/* 100% brightness */
	0x2d, 0x01, 0xff,
};

void write_i2c_table(unsigned char *p, int size)
{
	int ret;
	int i;

	for (i = 0; i < size; i += 3, p += 3) {
		int retry = 0;
		while (1) {
			ret = i2c_write(p[0], p[1], 1, &p[2], 1);
			if (!ret)
				break;
			if (retry++ > 10) {
				printf("error writing 0x%02x:0x%02x = 0x%02x\n",
					p[0], p[1], p[2]);
				break;
			}
			mdelay(100);
		}
	}
}

void enable_backlight(const struct display_info_t *di, int enable, int gp_lp8860)
{
	if (di->addr_num == 0x0c) {
		/* enable lp8860 backlight */
		int ret = i2c_set_bus_num(di->bus & 0xff);

		if (ret)
			return;

		gpio_direction_output(gp_lp8860, 0);
		if (!enable)
			return;

		write_i2c_table(setup_serializer_remote,
			sizeof(setup_serializer_remote));
		write_i2c_table(setup_serializer_data,
			sizeof(setup_serializer_data));
		mdelay(2);
		gpio_direction_output(gp_lp8860, enable);
		mdelay(60);

		write_i2c_table(enable_backlight_data,
			sizeof(enable_backlight_data));
	}
}

void board_enable_lvds(const struct display_info_t *di, int enable)
{
	enable_backlight(di, enable, GP_LVDS_LP8860_RESET);
}

void board_enable_lvds2(const struct display_info_t *di, int enable)
{
	enable_backlight(di, enable, GP_LVDS2_LP8860_RESET);
}

int fbp_detect_serializer(struct display_info_t const *di)
{
	int ret;
	int gp = di->bus >> 8;

	if (gp)
		gpio_set_value(gp, 1);
	ret = i2c_set_bus_num(di->bus & 0xff);
	if (ret == 0) {
		int gp_lp8860 = (di->fbtype == FB_LVDS2) ? GP_LVDS2_LP8860_RESET :
				GP_LVDS_LP8860_RESET;
		ret = i2c_probe(di->addr_num);
		if (!ret) {
			gpio_direction_output(gp_lp8860, 0);
			write_i2c_table(setup_serializer_remote,
				sizeof(setup_serializer_remote));
			ret = i2c_probe(0x2c);
			if (!ret)
				write_i2c_table(setup_serializer_data,
						sizeof(setup_serializer_data));
		}
	}
	if (gp)
		gpio_set_value(gp, 0);
	return (ret == 0);
}

static const struct display_info_t displays[] = {
	/* hdmi */
	VD_1280_720M_60(HDMI, fbp_detect_i2c, 1, 0x50),
	VD_1920_1080M_60(HDMI, NULL, 1, 0x50),
	VD_1024_768M_60(HDMI, NULL, 1, 0x50),

	/* 0x0c is a serializer */
	VD_TFC_A9700LTWV35TC_C1(LVDS, fbp_detect_serializer, 2, 0x0c, FBTS_GOODIX),
	VD_TFC_A9700LTWV35TC_C1(LVDS2, fbp_detect_serializer, 1, 0x0c, FBTS_GOODIX2),
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

static const unsigned short gpios_out_low[] = {
	GP_BT_RFKILL_RESET, 	/* disable bluetooth */
	GP_EMMC_RESET,		/* hold in reset */
	GP_RGMII_PHY_RESET,
	GP_LVDS2_LP8860_RESET,	/* Serializer2 gpio1 */
	GP_LVDS_LP8860_RESET,	/* Serializer1 gpio1 */
	GP_PCIE_RESET,
	GP_REG_USBOTG,		/* disable USB otg power */
	GP_REG_WLAN_EN,		/* disable wireless */
	GP_USB_HUB_RESET,	/* disable hub */
	GP_UART4_TX_EN,
	GP_UART5_RX_EN,		/* power down uart5 */
	GP_UART5_TX_EN,
	GP_UART5_RS485_EN,
	GP_UART5_AON,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,	/* SS1 of spi nor */
};

static const unsigned short gpios_in[] = {
	GPIRQ_ENET_PHY,
	GP_GPIOKEY_BACK,
	GP_GPIOKEY_HOME,
	GP_GPIOKEY_MENU,
	GP_GPIOKEY_SEARCH,
	GP_GPIOKEY_VOL_DOWN,
	GP_GPIOKEY_VOL_UP,
	GP_GPIO_1,
	GP_GPIO_2,
	GP_GPIO_3,
	GP_GPIO_4,
	GP_GPIO_5,
	GP_GPIO_6,
	GP_GPIO_7,
	GP_GPIO_8,
	GP_GPIO_9,
	GP_GPIO_10,
	GP_GPIO_11,
	GP_GPIO_12,
	GP_GPIO_13,
	GP_GPIO_14,
	GP_GPIO_15,
	GP_GPIO_16,
	GP_BT_CLK_REQ,
	GP_BT_HOST_WAKE,
	GP_WIFI_QOW,
	GP_TP71,
	GP_TP74,
	GP_TP101,
	GP_TP102,
	GPIRQ_RTC_RV4162,
	GP_I2C1_ALT_SCL,
	GP_I2C1_ALT_SDA,
	GPIRQ_I2C2_SER2_TOUCH,
	GP_SER2_GPIO5,
	GP_SER2_GPIO6,
	GP_SER2_FAN_OK,
	GP_I2C2_GT911_RESET,
	GPIRQ_SER2,
	GPIRQ_I2C3_SER1_TOUCH,
	GP_SER1_GPIO5,
	GP_SER1_GPIO6,
	GP_SER1_FAN_OK,
	GP_I2C3_GT911_RESET,
	GPIRQ_SER1,
	GP_PCIE_DISABLE,
	GPIRQ_WIFI,
	GP_WIFI_WAKE,
	GP_USDHC3_CD,
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
	{"search",	GP_GPIOKEY_SEARCH,	'S', 1},
	{"volup",	GP_GPIOKEY_VOL_UP,	'V', 1},
	{"voldown",	GP_GPIOKEY_VOL_DOWN,	'v', 1},
	{NULL, 0, 0, 0},
};

#ifdef CONFIG_CMD_BMODE
const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},	/* 8-bit eMMC */
	{NULL,		0},
};
#endif
