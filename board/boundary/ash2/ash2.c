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
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/spi.h>
#include <mmc.h>
#include <fsl_esdhc.h>
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
	IOMUX_PAD_CTRL(CSI0_DAT7__AUD3_RXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT4__AUD3_TXC, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT5__AUD3_TXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT6__AUD3_TXFS, AUD_PAD_CTRL),

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
#define GP_GPIOKEY_SW6		IMX_GPIO_NR(2, 17)
	IOMUX_PAD_CTRL(EIM_A21__GPIO2_IO17, WEAK_PULLUP),
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
#define GP_TP_R5		IMX_GPIO_NR(2, 7)
	IOMUX_PAD_CTRL(EIM_LBA__GPIO2_IO27, WEAK_PULLUP),
#define GP_TP74			IMX_GPIO_NR(2, 7)
	IOMUX_PAD_CTRL(NANDF_D7__GPIO2_IO07, WEAK_PULLUP),
#define GP_TP84			IMX_GPIO_NR(2, 30)
	IOMUX_PAD_CTRL(EIM_EB2__GPIO2_IO30, WEAK_PULLUP),
#define GP_TP85			IMX_GPIO_NR(2, 31)
	IOMUX_PAD_CTRL(EIM_EB3__GPIO2_IO31, WEAK_PULLUP),

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

	/*
	 * PWM4 - Backlight on LVDS connector: J4, pin 3
	 * 0 is bright, 1 is dim
	 */
#define GP_BACKLIGHT_LVDS	IMX_GPIO_NR(1, 18)
	IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, PAD_CTRL_PWM),
	/* 0 is 8 bit */
#define GP_8BIT_LVDS		IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(KEY_ROW4__GPIO4_IO15, WEAK_PULLDN_OUTPUT),
#define GP_LVDS_BKL_EN		IMX_GPIO_NR(7, 13)
	IOMUX_PAD_CTRL(GPIO_18__GPIO7_IO13, WEAK_PULLDN),

	/* reg_usbotg_vbus */
#define GP_REG_USBOTG		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLDN),

	/* reg_wlan_en */
#define GP_REG_WLAN_EN		IMX_GPIO_NR(2, 5)
	IOMUX_PAD_CTRL(NANDF_D5__GPIO2_IO05, WEAK_PULLDN),

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
	IOMUX_PAD_CTRL(EIM_D23__UART3_CTS_B, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D31__UART3_RTS_B, UART_PAD_CTRL),

	/* UART4  */
	IOMUX_PAD_CTRL(KEY_COL0__UART4_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_ROW0__UART4_RX_DATA, UART_PAD_CTRL),

	/* USBH1 */
	IOMUX_PAD_CTRL(EIM_D30__USB_H1_OC, WEAK_PULLUP),

	/* USBOTG */
	IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, WEAK_PULLUP),
	IOMUX_PAD_CTRL(KEY_COL4__USB_OTG_OC, WEAK_PULLUP),

	/* USDHC2  */
	IOMUX_PAD_CTRL(SD2_CLK__SD2_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__SD2_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT0__SD2_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT1__SD2_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT2__SD2_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT3__SD2_DATA3, USDHC_PAD_CTRL),
//	IOMUX_PAD_CTRL(SD1_CLK__OSC32K_32K_OUT, OUTPUT_40OHM),	/* slow clock */

	/* USDHC2 - wlan */
#define GPIRQ_WIFI		IMX_GPIO_NR(6, 14)
	IOMUX_PAD_CTRL(NANDF_CS1__GPIO6_IO14, WEAK_PULLDN),
#define GP_WIFI_WAKE		IMX_GPIO_NR(2, 1)
	IOMUX_PAD_CTRL(NANDF_D1__GPIO2_IO01, WEAK_PULLUP),
#define GP_WIFI_QOW		IMX_GPIO_NR(2, 3)
	IOMUX_PAD_CTRL(NANDF_D3__GPIO2_IO03, WEAK_PULLUP),
#define GP_BT_HOST_WAKE		IMX_GPIO_NR(6, 10)
	IOMUX_PAD_CTRL(NANDF_RB0__GPIO6_IO10, WEAK_PULLDN),
#define GP_BT_CLK_REQ		IMX_GPIO_NR(2, 6)
	IOMUX_PAD_CTRL(NANDF_D6__GPIO2_IO06, WEAK_PULLUP),

	/* USDHC3 - sdcard */
#define GP_USDHC3_CD		IMX_GPIO_NR(7, 0)
	IOMUX_PAD_CTRL(SD3_DAT5__GPIO7_IO00, WEAK_PULLUP),
#define GP_USDHC3_POWER_EN	IMX_GPIO_NR(1, 30)
	IOMUX_PAD_CTRL(ENET_TXD0__GPIO1_IO30, WEAK_PULLUP),	/* Pullup so that bmod mmc0 works */
};

static const iomux_v3_cfg_t sd3_usdhc3_pads[] = {
	IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
};

/* This powers down the sd card faster */
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

static const struct i2c_pads_info i2c_pads[] = {
	/* I2C1, SGTL5000 */
	I2C_PADS_INFO_ENTRY(I2C1, EIM_D21, 3, 21, EIM_D28, 3, 28, I2C_PAD_CTRL),
	/* I2C2 Camera, MIPI */
	I2C_PADS_INFO_ENTRY(I2C2, KEY_COL3, 4, 12, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
	/* I2C3, J15 - RGB connector */
	I2C_PADS_INFO_ENTRY(I2C3, GPIO_5, 1, 05, GPIO_16, 7, 11, I2C_PAD_CTRL),
};
#define I2C_BUS_CNT	3

int power_init_board(void)
{
	mdelay(3);
	SETUP_IOMUX_PADS(sd3_usdhc3_pads);
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
struct fsl_esdhc_cfg board_usdhc_cfg[] = {
	{.esdhc_base = USDHC3_BASE_ADDR, .bus_width = 4,
			.gp_cd = GP_USDHC3_CD, .gp_reset = GP_USDHC3_POWER_EN},
};
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : -1;
}
#endif

#ifdef CONFIG_CMD_FBPANEL
static int sw_vals = -1;

static ulong swm00[] = { CONFIG_SWM_DISPLAY_00_A, CONFIG_SWM_DISPLAY_00_B };
static ulong swm24[] = { CONFIG_SWM_DISPLAY_24_A, CONFIG_SWM_DISPLAY_24_B };

int board_detect_lvds(const struct display_info_t *di)
{
	unsigned sw = sw_vals & 0x3f;
	unsigned swl = sw >> 5;
	ulong swm = 1 << (sw & 0x1f);
	ulong mask;
	char buf[16];

	snprintf(buf, sizeof(buf), "swm%02x_%c", di->addr_num, 'a' + swl);

	mask = env_get_hex(buf, (di->addr_num == 0x24) ?
			swm24[swl] :  swm00[swl]);
	return (swm & mask) ? 1 : 0;
}

void board_enable_lvds(const struct display_info_t *di, int enable)
{
	if (di->addr_num == 0x24) {
		/* This is a backlight enable for this panel */
		gpio_set_value(GP_8BIT_LVDS, enable);
	} else {
		gpio_set_value(GP_8BIT_LVDS,
			(di->pixfmt == IPU_PIX_FMT_RGB666) ? 1 : 0);
	}
	gpio_set_value(GP_BACKLIGHT_LVDS, enable ^
			((di->fbflags & FBF_BKLIT_LOW_ACTIVE) ? 1 : 0));
}

static const struct display_info_t displays[] = {
	/* lvds */
	VD_WVGA_TX23D200_18L(LVDS, board_detect_lvds, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_WVGA_TX23D200_18H(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_WVGA_TX23D200_24L(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_WVGA_TX23D200_24H(LVDS, NULL, fbp_bus_gp(0, 0, GP_LVDS_BKL_EN, 0), 0x00),
	VD_AM_1280800P2TZQW(LVDS, board_detect_lvds, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x24),

	/* hdmi */
	VD_1280_720M_60(HDMI, fbp_detect_i2c, 1, 0x50),
	VD_1920_1080M_60(HDMI, NULL, 1, 0x50),
	VD_1024_768M_60(HDMI, NULL, 1, 0x50),
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

static const unsigned short gpios_out_low[] = {
	GP_BT_RFKILL_RESET,
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
	GP_8BIT_LVDS,
	GP_LVDS_BKL_EN,
	GP_REG_USBOTG,		/* disable USB otg power */
	GP_REG_WLAN_EN,
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
	GP_GPIOKEY_SW6,
	GP_GPIOKEY_POWER,
	GP_GPIOKEY_CH_ON_RBL,
	GP_GPIOKEY_SG_ON_RBL,
	GP_GPIOKEY_DOOR_CLOSED,
	GPIRQ_RTC_RV4162,
	GPIRQ_MIC_DET,
	GPIRQ_AR1021,
	GP_AR1021_5WIRE,
	GPIRQ_I2C3_J6,
	GP_TP_R5,
	GP_TP74,
	GP_TP84,
	GP_TP85,
	GP_USDHC3_CD,
	GPIRQ_WIFI,
	GP_WIFI_WAKE,
	GP_WIFI_QOW,
	GP_BT_HOST_WAKE,
	GP_BT_CLK_REQ,
};

int board_early_init_f(void)
{
	set_gpios_in(gpios_in, ARRAY_SIZE(gpios_in));
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	SETUP_IOMUX_PADS(sd3_gpio_pads);
	SETUP_IOMUX_PADS(init_pads);
	return 0;
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

const struct button_key board_buttons[] = {
	{"sw1",		GP_GPIOKEY_SW1,		'1', 1},
	{"sw2",		GP_GPIOKEY_SW2,		'2', 1},
	{"sw3",		GP_GPIOKEY_SW3,		'3', 1},
	{"sw4",		GP_GPIOKEY_SW4,		'4', 1},
	{"sw5",		GP_GPIOKEY_SW5,		'5', 1},
	{"sw6",		GP_GPIOKEY_SW6,		'6', 1},
	{"power",	GP_GPIOKEY_POWER,	'P', 1},
	{"ch",		GP_GPIOKEY_CH_ON_RBL,	'C', 1},
	{"sg",		GP_GPIOKEY_SG_ON_RBL,	'S', 1},
	{"door",	GP_GPIOKEY_DOOR_CLOSED, 'D', 1},
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
	unsigned sw = read_keys_int(6);

	sw_vals = sw;
	common_board_init(i2c_pads, I2C_BUS_CNT, IOMUXC_GPR1_OTG_ID_GPIO1,
			displays, display_cnt, 0);
	return 0;
}

#ifdef CONFIG_CMD_BMODE
const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},	/* 8-bit eMMC */
	{NULL,		0},
};
#endif

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
