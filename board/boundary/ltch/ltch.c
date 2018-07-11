/*
 * Copyright (C) 2018, Boundary Devices <info@boundarydevices.com>
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

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

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

	/* CAN1 - TJA1040T */
	IOMUX_PAD_CTRL(KEY_COL2__FLEXCAN1_TX, WEAK_PULLUP),
	IOMUX_PAD_CTRL(KEY_ROW2__FLEXCAN1_RX, WEAK_PULLUP),
#define GP_CAN1_STANDBY		IMX_GPIO_NR(1, 2)
	IOMUX_PAD_CTRL(GPIO_2__GPIO1_IO02, WEAK_PULLUP_OUTPUT),

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

	/* GP - Hogs */
#define GP_BT_CLK_REQ		IMX_GPIO_NR(2, 0)
	IOMUX_PAD_CTRL(NANDF_D0__GPIO2_IO00, WEAK_PULLUP),
#define GP_BT_HOST_WAKE		IMX_GPIO_NR(6, 7)
	IOMUX_PAD_CTRL(NANDF_CLE__GPIO6_IO07, WEAK_PULLUP),
#define GP_WIFI_WAKE		IMX_GPIO_NR(2, 1)
	IOMUX_PAD_CTRL(NANDF_D1__GPIO2_IO01, WEAK_PULLUP),
#define GP_WIFI_QOW		IMX_GPIO_NR(2, 3)
	IOMUX_PAD_CTRL(NANDF_D3__GPIO2_IO03, WEAK_PULLUP),

#define GP_LVDS			IMX_GPIO_NR(4, 5)	/* J4 pin 19 */
	IOMUX_PAD_CTRL(GPIO_19__GPIO4_IO05, WEAK_PULLDN),

#define GP_LVDS2		IMX_GPIO_NR(2, 23)	/* J5 pin 19 */
	IOMUX_PAD_CTRL(EIM_CS0__GPIO2_IO23, WEAK_PULLDN),

#define GP_LED3V		IMX_GPIO_NR(5, 7)
	IOMUX_PAD_CTRL(DISP0_DAT13__GPIO5_IO07, WEAK_PULLUP),
#define GP_LED3V_CTRL		IMX_GPIO_NR(5, 5)
	IOMUX_PAD_CTRL(DISP0_DAT11__GPIO5_IO05, WEAK_PULLDN),
#define GP_LED5V		IMX_GPIO_NR(5, 8)
	IOMUX_PAD_CTRL(DISP0_DAT14__GPIO5_IO08, WEAK_PULLUP),
#define GP_LED5V_CTRL		IMX_GPIO_NR(4, 31)
	IOMUX_PAD_CTRL(DISP0_DAT10__GPIO4_IO31, WEAK_PULLDN),

#define GP_TP55			IMX_GPIO_NR(7, 13)
	IOMUX_PAD_CTRL(GPIO_18__GPIO7_IO13, WEAK_PULLUP),
#define GP_TP71			IMX_GPIO_NR(1, 30)
	IOMUX_PAD_CTRL(ENET_TXD0__GPIO1_IO30, WEAK_PULLUP),
#define GP_TP74			IMX_GPIO_NR(2, 7)
	IOMUX_PAD_CTRL(NANDF_D7__GPIO2_IO07, WEAK_PULLUP),
#define GP_TP124		IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(SD1_DAT0__GPIO1_IO16, WEAK_PULLUP),
#define GP_TP125		IMX_GPIO_NR(7, 1)
	IOMUX_PAD_CTRL(SD3_DAT4__GPIO7_IO01, WEAK_PULLUP),
#define GP_TP151		IMX_GPIO_NR(2, 24)
	IOMUX_PAD_CTRL(EIM_CS1__GPIO2_IO24, WEAK_PULLUP),
#define GP_TP152		IMX_GPIO_NR(6, 9)
	IOMUX_PAD_CTRL(NANDF_WP_B__GPIO6_IO09, WEAK_PULLUP),
#define GP_TP153		IMX_GPIO_NR(6, 4)
	IOMUX_PAD_CTRL(CSI0_DAT18__GPIO6_IO04, WEAK_PULLUP),

	/* i2c1 - TCA9546APWR,  a - wm8960, b - touch, c - pcie, d - J6 */
	/* A0/A1 are i2c address selects, 0x70 - 0x73 */
#define GP_I2C1_TCA9546_A0	IMX_GPIO_NR(4, 19)
	IOMUX_PAD_CTRL(DI0_PIN3__GPIO4_IO19, WEAK_PULLDN),
#define GP_I2C1_TCA9546_A1	IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, WEAK_PULLDN),
#define GP_I2C1_TCA9546_RESET	IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO_8__GPIO1_IO08, WEAK_PULLDN),

	/* i2c1a wm8960 - Amplifier Mute, TDA7491LP13TR */
#define GP_WM8960_AMP_STDBY	IMX_GPIO_NR(4, 7)		/* Low is standby */
	IOMUX_PAD_CTRL(KEY_ROW0__GPIO4_IO07, WEAK_PULLDN),
#define GP_WM8960_AMP_MUTE		IMX_GPIO_NR(4, 21)		/* Low is muted */
	IOMUX_PAD_CTRL(DISP0_DAT0__GPIO4_IO21, WEAK_PULLDN),
#define GP_WM8960_AMP_G0	IMX_GPIO_NR(4, 25)
	IOMUX_PAD_CTRL(DISP0_DAT4__GPIO4_IO25, WEAK_PULLDN),
#define GP_WM8960_AMP_G1	IMX_GPIO_NR(4, 24)
	IOMUX_PAD_CTRL(DISP0_DAT3__GPIO4_IO24, WEAK_PULLDN),
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(7, 8)
	IOMUX_PAD_CTRL(SD3_RST__GPIO7_IO08, WEAK_PULLUP),
	IOMUX_PAD_CTRL(GPIO_0__CCM_CLKO1, OUTPUT_40OHM),	/* mclk */

	/* i2c1b touch J7, pin 4 */
#define GPIRQ_I2C1B_J7	IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO_9__GPIO1_IO09, WEAK_PULLUP),

	/* i2c1d touch J6, pin 4 */
#define GPIRQ_I2C1D_J6		IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO_7__GPIO1_IO07, WEAK_PULLUP),

	/* i2c2 - TCA9546APWR, a : DAC, b: ADC, c: RV4162, d: J14 */
#define GP_I2C2_TCA9546_RESET	IMX_GPIO_NR(1, 4)
	IOMUX_PAD_CTRL(GPIO_4__GPIO1_IO04, WEAK_PULLDN),

	/* i2c2b ISL28022FUZ ADC */
#define GPIRQ_ISL28022		IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(KEY_ROW4__GPIO4_IO15, WEAK_PULLUP),

	/* i2c2c rv4162 rtc */
#define GPIRQ_RTC_RV4162	IMX_GPIO_NR(4, 6)
	IOMUX_PAD_CTRL(KEY_COL0__GPIO4_IO06, WEAK_PULLUP),

	/* i2c2d J14, pin 4 */
#define GPIRQ_I2C2D_J14		IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO_3__GPIO1_IO03, WEAK_PULLUP),

#define GPIRQ_I2C3_J22	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(SD1_DAT1__GPIO1_IO17, WEAK_PULLUP),

	/* PCIe */
#define GP_PCIE_RESET		IMX_GPIO_NR(6, 31)
	IOMUX_PAD_CTRL(EIM_BCLK__GPIO6_IO31, WEAK_PULLDN),
#define GP_PCIE_DISABLE		IMX_GPIO_NR(2, 27)
	IOMUX_PAD_CTRL(EIM_LBA__GPIO2_IO27, WEAK_PULLDN),

	/* PWM2 - Backlight on LVDS2 connector: J5, pin 20 */
#define GP_BACKLIGHT_LVDS2	IMX_GPIO_NR(1, 19)
	IOMUX_PAD_CTRL(SD1_DAT2__GPIO1_IO19, WEAK_PULLDN),

	/* PWM4 - Backlight on LVDS connector: J4, pin 20 */
#define GP_BACKLIGHT_LVDS	IMX_GPIO_NR(1, 18)
	IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, WEAK_PULLDN),

	/* reg_usbotg_vbus */
#define GP_REG_USBOTG		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLDN),

	/* reg_wlan_en */
#define GP_REG_WLAN_EN	IMX_GPIO_NR(6, 15)
	IOMUX_PAD_CTRL(NANDF_CS2__GPIO6_IO15, WEAK_PULLDN),

	/* UART1 */
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),

	/* UART2 */
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),

	/* UART3 for bluetooth */
	IOMUX_PAD_CTRL(EIM_D24__UART3_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D25__UART3_RX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D23__UART3_CTS_B, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D31__UART3_RTS_B, UART_PAD_CTRL),

	/* UART5 */
	IOMUX_PAD_CTRL(KEY_COL1__UART5_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_ROW1__UART5_RX_DATA, UART_PAD_CTRL),
#define GP_UART5_TX_EN		IMX_GPIO_NR(6, 5)
	IOMUX_PAD_CTRL(CSI0_DAT19__GPIO6_IO05, WEAK_PULLDN),

	/* USBH1 */
#define GP_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
	IOMUX_PAD_CTRL(GPIO_17__GPIO7_IO12, WEAK_PULLDN),

	/* USBOTG */
	IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, WEAK_PULLUP),
	IOMUX_PAD_CTRL(KEY_COL4__USB_OTG_OC, WEAK_PULLUP),

	/* USDHC2 - Silex */
	IOMUX_PAD_CTRL(SD2_CLK__SD2_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__SD2_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT0__SD2_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT1__SD2_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT2__SD2_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT3__SD2_DATA3, USDHC_PAD_CTRL),
//	IOMUX_PAD_CTRL(SD1_CLK__OSC32K_32K_OUT, OUTPUT_40OHM),	/* slow clock */

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

	/* Silex */
#define GPIRQ_WIFI		IMX_GPIO_NR(6, 11)
	IOMUX_PAD_CTRL(NANDF_CS0__GPIO6_IO11, WEAK_PULLUP),
};

static const struct i2c_pads_info i2c_pads[] = {
	/* I2C1, wm8960 */
	I2C_PADS_INFO_ENTRY(I2C1, EIM_D21, 3, 21, EIM_D28, 3, 28, I2C_PAD_CTRL),
	/* I2C2  */
	I2C_PADS_INFO_ENTRY(I2C2, KEY_COL3, 4, 12, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
	/* I2C3, J15 - RGB connector */
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

void board_enable_lvds(const struct display_info_t *di, int enable)
{
	if (enable)
		mdelay(100);
	gpio_set_value(GP_BACKLIGHT_LVDS, enable);
}

void board_enable_lvds2(const struct display_info_t *di, int enable)
{
	if (enable)
		mdelay(100);
	gpio_set_value(GP_BACKLIGHT_LVDS2, enable);
}

int board_fbp_detect_i2c(struct display_info_t const *di)
{
	int ret;
	int gp = di->bus >> 8;
	u8 orig_i2c_bus;

	if (gp)
		gpio_set_value(gp, 1);
	orig_i2c_bus = i2c_get_bus_num();
	ret = i2c_set_bus_num(di->bus & 0xff);
	if (ret == 0) {
		ret = i2c_write(0x70, (di->fbtype == FB_LVDS2) ? 8 : 2,
				1, NULL, 0);
		if (!ret) {
			ret = i2c_probe(di->addr_num);
			i2c_write(0x70, 0, 1, NULL, 0);
		}
	}
	i2c_set_bus_num(orig_i2c_bus);
	if (gp)
		gpio_set_value(gp, 0);
	return (ret == 0);
}

static const struct display_info_t displays[] = {
	/* ft5x06 */
	VD_PM9598(LVDS2, board_fbp_detect_i2c, fbp_bus_gp(0, GP_I2C1_TCA9546_RESET, 0, 0), 0x38, FBTS_FT5X06),
	VD_PM9598(LVDS, board_fbp_detect_i2c, fbp_bus_gp(0, GP_I2C1_TCA9546_RESET, 0, 0), 0x38, FBTS_FT5X06),
	VD_DT070BTFT(LVDS2, NULL, fbp_bus_gp(0, GP_I2C1_TCA9546_RESET, 0, 0), 0x38, FBTS_FT5X06),
	VD_DT070BTFT(LVDS, NULL, fbp_bus_gp(0, GP_I2C1_TCA9546_RESET, 0, 0), 0x38, FBTS_FT5X06),
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

static const unsigned short gpios_out_low[] = {
	GP_BT_RFKILL_RESET, 	/* disable bluetooth */
	GP_CAN1_STANDBY,
	GP_RGMII_PHY_RESET,
	GP_I2C1_TCA9546_A0,
	GP_I2C1_TCA9546_A1,
	GP_I2C1_TCA9546_RESET,
	GP_WM8960_AMP_STDBY,
	GP_WM8960_AMP_MUTE,
	GP_WM8960_AMP_G0,
	GP_WM8960_AMP_G1,
	GP_I2C2_TCA9546_RESET,
	GP_PCIE_RESET,
	GP_PCIE_DISABLE,
	GP_BACKLIGHT_LVDS2,
	GP_BACKLIGHT_LVDS,
	GP_REG_USBOTG,		/* disable USB otg power */
	GP_REG_WLAN_EN,		/* disable wireless */
	GP_UART5_TX_EN,
	GP_USB_HUB_RESET,	/* disable hub */
	GP_EMMC_RESET,		/* hold in reset */
	GP_LED3V_CTRL,
	GP_LED5V_CTRL,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,	/* SS1 of spi nor */
	GP_LED3V,
	GP_LED5V,
};

static const unsigned short gpios_in[] = {
	GPIRQ_ENET_PHY,
	GP_BT_CLK_REQ,
	GP_BT_HOST_WAKE,
	GP_WIFI_WAKE,
	GP_WIFI_QOW,
	GP_TP55,
	GP_TP71,
	GP_TP74,
	GP_TP124,
	GP_TP125,
	GP_TP151,
	GP_TP152,
	GP_TP153,
	GP_WM8960_MIC_DET,
	GPIRQ_I2C1B_J7,
	GPIRQ_I2C1D_J6,
	GPIRQ_ISL28022,
	GPIRQ_RTC_RV4162,
	GPIRQ_I2C2D_J14,
	GP_LVDS,
	GP_LVDS2,
	GPIRQ_I2C3_J22,
	GP_USDHC3_CD,
	GPIRQ_WIFI,
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
	{"tp55",	GP_TP55,	't', 1},
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
