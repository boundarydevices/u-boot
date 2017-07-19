/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2015, Boundary Devices <info@boundarydevices.com>
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
#include <usb/ehci-ci.h>
#include "spi_display.h"
#include "../padctrl.h"

DECLARE_GLOBAL_DATA_PTR;

#define AUD_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define CEC_PAD_CTRL	(PAD_CTL_PUS_22K_UP | 			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
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
	/* AUDMUX3 */
	IOMUX_PAD_CTRL(CSI0_DAT7__AUD3_RXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT4__AUD3_TXC, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT5__AUD3_TXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT6__AUD3_TXFS, AUD_PAD_CTRL),

	/* AUDMUX5 - from TC358743 hdmi input */
	IOMUX_PAD_CTRL(KEY_ROW1__AUD5_RXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D24__AUD5_RXFS, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D25__AUD5_RXC, AUD_PAD_CTRL),


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

	/* GPIO_KEYS assignments for J14 */
#define GP_GPIOKEY_POWER		IMX_GPIO_NR(1, 2)
	IOMUX_PAD_CTRL(GPIO_2__GPIO1_IO02, WEAK_PULLUP),
#define GP_GPIOKEY_1		IMX_GPIO_NR(3, 7)
	IOMUX_PAD_CTRL(EIM_DA7__GPIO3_IO07, WEAK_PULLUP),
#define GP_GPIOKEY_2		IMX_GPIO_NR(3, 8)
	IOMUX_PAD_CTRL(EIM_DA8__GPIO3_IO08, WEAK_PULLUP),
#define GP_GPIOKEY_3		IMX_GPIO_NR(3, 9)
	IOMUX_PAD_CTRL(EIM_DA9__GPIO3_IO09, WEAK_PULLUP),
#define GP_VBUS_GOOD		IMX_GPIO_NR(3, 15)
	IOMUX_PAD_CTRL(EIM_DA15__GPIO3_IO15, WEAK_PULLUP),
#define GP_SPARE		IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(SD1_DAT0__GPIO1_IO16, WEAK_PULLUP),

#define GP_MAIN_POWER_EN	IMX_GPIO_NR(1, 19)		/* low is on */
	IOMUX_PAD_CTRL(SD1_DAT2__GPIO1_IO19, WEAK_PULLDN),

	/* hdmi cec - tc358743 and micro hdmi connector */
	IOMUX_PAD_CTRL(EIM_A25__HDMI_TX_CEC_LINE, CEC_PAD_CTRL),

	/* hog  - test points */
#define GP_TP4		IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, WEAK_PULLUP),
#define GP_TP5		IMX_GPIO_NR(4, 17)
	IOMUX_PAD_CTRL(DI0_PIN15__GPIO4_IO17, WEAK_PULLUP),
#define GP_TP6		IMX_GPIO_NR(1, 18)
	IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, WEAK_PULLUP),
#define GP_TP7		IMX_GPIO_NR(1, 30)
	IOMUX_PAD_CTRL(ENET_TXD0__GPIO1_IO30, WEAK_PULLUP),
#define GP_TP8		IMX_GPIO_NR(3, 13)
	IOMUX_PAD_CTRL(EIM_DA13__GPIO3_IO13, WEAK_PULLUP),
#define GP_TP9		IMX_GPIO_NR(3, 14)
	IOMUX_PAD_CTRL(EIM_DA14__GPIO3_IO14, WEAK_PULLUP),
#define GP_TP10		IMX_GPIO_NR(2, 7)
	IOMUX_PAD_CTRL(NANDF_D7__GPIO2_IO07, WEAK_PULLUP),

	/* i2c1_wm8960 - Amplifier Mute */
	IOMUX_PAD_CTRL(GPIO_0__CCM_CLKO1, OUTPUT_40OHM),

	/* i2c2 TC3587 mipi hdmi input */
#define GP_TC3587_RESET		IMX_GPIO_NR(6, 1)
	IOMUX_PAD_CTRL(CSI0_DAT15__GPIO6_IO01, WEAK_PULLDN),
#define GPIRQ_TC3587		IMX_GPIO_NR(6, 0)
	IOMUX_PAD_CTRL(CSI0_DAT14__GPIO6_IO00, WEAK_PULLUP),
	IOMUX_PAD_CTRL(GPIO_3__CCM_CLKO2, OUTPUT_40OHM),
	/* micro hdmi power detect */
#define GP_HDMI_POWER_DET	IMX_GPIO_NR(4, 8)
	IOMUX_PAD_CTRL(KEY_COL1__GPIO4_IO08, WEAK_PULLDN),


	/* i2c3 - J8 */
#define GPIRQ_J8	IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO_9__GPIO1_IO09, WEAK_PULLUP),
#define GP_J8_RESET	IMX_GPIO_NR(7, 12)
	IOMUX_PAD_CTRL(GPIO_17__GPIO7_IO12, WEAK_PULLDN),


	/* I2C3 MAX77818 */
#define GP_MAX77818_INOKB	IMX_GPIO_NR(3, 4)	/* INOKB, WCHG_VALID_INT */
	IOMUX_PAD_CTRL(EIM_DA4__GPIO3_IO04, WEAK_PULLUP),
#define GP_MAX77818_WCINOKB	IMX_GPIO_NR(3, 5)	/* WCINOKB, WCHG_INT */
	IOMUX_PAD_CTRL(EIM_DA5__GPIO3_IO05, WEAK_PULLUP),
#define GP_MAX77818_INTB	IMX_GPIO_NR(3, 6)	/* INTB, CHG_INT */
	IOMUX_PAD_CTRL(EIM_DA6__GPIO3_IO06, WEAK_PULLUP),

	/* lcd */
#define GP_SPI_DISPLAY_RESET	IMX_GPIO_NR(3, 29)
	IOMUX_PAD_CTRL(EIM_D29__GPIO3_IO29, WEAK_PULLDN),
#define GP_SPI_DISPLAY_STANDBY	IMX_GPIO_NR(3, 31)
	IOMUX_PAD_CTRL(EIM_D31__GPIO3_IO31, WEAK_PULLUP),

	/* PWM1 - Backlight on RGB connector: J15, pin 37 */
#define GP_BACKLIGHT_RGB	IMX_GPIO_NR(1, 21)
	IOMUX_PAD_CTRL(SD1_DAT3__GPIO1_IO21, WEAK_PULLDN),

	/* PWM3  */
#define GP_BACKLIGHT_SPI	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(SD1_DAT1__GPIO1_IO17, WEAK_PULLDN),

	/* reg_usbh1_vbus */
#define GP_REG_5V		IMX_GPIO_NR(3, 3)
	IOMUX_PAD_CTRL(EIM_DA3__GPIO3_IO03, WEAK_PULLDN),

	/* reg_usbh1_vbus */
#define GP_REG_USBH1		IMX_GPIO_NR(2, 19)
	IOMUX_PAD_CTRL(EIM_A19__GPIO2_IO19, WEAK_PULLDN),

	/* reg_usbotg_vbus */
#define GP_REG_USBOTG		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLDN),

	/* reg_wlan_en */
#define GP_REG_WLAN_EN		IMX_GPIO_NR(6, 14)
	IOMUX_PAD_CTRL(NANDF_CS1__GPIO6_IO14, WEAK_PULLDN),

	/* UART1 */
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),

	/* UART2 */
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),

	/* UART4  */
	IOMUX_PAD_CTRL(KEY_ROW0__UART4_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_COL0__UART4_RX_DATA, UART_PAD_CTRL),

	/* USBH1 */
	IOMUX_PAD_CTRL(EIM_D30__USB_H1_OC, WEAK_PULLUP),

	/* USBOTG */
	IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, WEAK_PULLUP),
	IOMUX_PAD_CTRL(KEY_COL4__USB_OTG_OC, WEAK_PULLUP),

	/* USDHC2 - Murata */
	IOMUX_PAD_CTRL(SD2_CLK__SD2_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__SD2_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT0__SD2_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT1__SD2_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT2__SD2_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT3__SD2_DATA3, USDHC_PAD_CTRL),
//	IOMUX_PAD_CTRL(SD1_CLK__OSC32K_32K_OUT, OUTPUT_40OHM),	/* slow clock */

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

	/* murata wifi */
#define GPIRQ_WLAN		IMX_GPIO_NR(6, 11)
	IOMUX_PAD_CTRL(NANDF_CS0__GPIO6_IO11, WEAK_PULLDN),
#define GP_WLAN_CLK_REQ		IMX_GPIO_NR(6, 15)
	IOMUX_PAD_CTRL(NANDF_CS2__GPIO6_IO15, WEAK_PULLDN),
};

static const iomux_v3_cfg_t rgb_pads[] = {
	IOMUX_PAD_CTRL(DI0_DISP_CLK__IPU1_DI0_DISP_CLK, RGB_PAD_CTRL),
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
	gpio_set_value(GP_REG_5V, 1);
	return 0;
}

int board_ehci_power(int port, int on)
{
	gpio_set_value(port ? GP_REG_USBH1 : GP_REG_USBOTG, on);
	return 0;
}

#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{.esdhc_base = USDHC4_BASE_ADDR, .max_bus_width = 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

	if (cfg->esdhc_base == USDHC4_BASE_ADDR)
		return 1;	/* eMMC always present */
	return 0;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
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
void board_enable_lcd(const struct display_info_t *di, int enable)
{
	if (enable) {
		SETUP_IOMUX_PADS(rgb_pads);
#ifdef CONFIG_MXC_SPI_DISPLAY
		if (di->fbflags & FBF_SPI)
			enable_spi_rgb(di);
#endif
		mdelay(100); /* let panel sync up before enabling backlight */
		gpio_direction_output(GP_BACKLIGHT_RGB, enable);
		gpio_direction_output(GP_BACKLIGHT_SPI, enable);
	} else {
		gpio_direction_output(GP_BACKLIGHT_RGB, enable);
		gpio_direction_output(GP_BACKLIGHT_SPI, enable);
		SETUP_IOMUX_PADS(rgb_gpio_pads);
	}
}


static const struct display_info_t displays[] = {
#ifdef CONFIG_MXC_SPI_DISPLAY
	VD_A030JN01_YUV720(LCD, NULL, 1, 1),
	VD_A030JN01_UPS051(LCD, NULL, 1, 2),
	VD_AUO_G050(LCD, NULL, 1, 0),
#endif
};

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}
#endif

static const unsigned short gpios_out_low[] = {
	GP_MAIN_POWER_EN,
	GP_TC3587_RESET,
	GP_J8_RESET,
	GP_SPI_DISPLAY_RESET,
	GP_SPI_DISPLAY_STANDBY,
	GP_BACKLIGHT_RGB,
	GP_BACKLIGHT_SPI,
	GP_REG_5V,
	GP_REG_USBH1,
	GP_REG_USBOTG,
	GP_REG_WLAN_EN,
	GP_EMMC_RESET,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,	/* SS1 of spi nor */
	GP_ECSPI2_CS,
};

static const unsigned short gpios_in[] = {
	GP_GPIOKEY_POWER,
	GP_GPIOKEY_1,
	GP_GPIOKEY_2,
	GP_GPIOKEY_3,
	GP_VBUS_GOOD,
	GP_SPARE,
	GP_TP4,
	GP_TP5,
	GP_TP6,
	GP_TP7,
	GP_TP8,
	GP_TP9,
	GP_TP10,
	GPIRQ_TC3587,
	GP_HDMI_POWER_DET,
	GPIRQ_J8,
	GP_MAX77818_INOKB,
	GP_MAX77818_WCINOKB,
	GP_MAX77818_INTB,
	GP_WLAN_CLK_REQ,
	GPIRQ_WLAN,
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

void board_poweroff(void)
{
	struct snvs_regs *snvs = (struct snvs_regs *)(SNVS_BASE_ADDR);

	/* Doing both gpio1:19 and pmic_req makes the board not turn on again */
//	gpio_set_value(GP_MAIN_POWER_EN, 1);
	writel(0x60, &snvs->lpcr);
	mdelay(500);
}

int board_init(void)
{
	int i;
	int ret;
	struct i2c_pads_info *p = i2c_pads + i2c_get_info_entry_offset();
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	u8 orig_i2c_bus;
	u8 val8;
	u8 buf[2];

	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	for (i = 0; i < 3; i++) {
	        setup_i2c(i, CONFIG_SYS_I2C_SPEED, 0x7f, p);
		p += I2C_PADS_INFO_ENTRY_SPACING;
	}
	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(2);
#define I2C_ADDR_CHARGER	0x69
	val8 = 0x7f;	/* 4.0A source */
	i2c_write(I2C_ADDR_CHARGER, 0xc0, 1, &val8, 1);
	val8 = 0x0c;	/* Protection allow 0xb9 write */
	i2c_write(I2C_ADDR_CHARGER, 0xbd, 1, &val8, 1);
	val8 = 0x14;	/* 1A charge */
	i2c_write(I2C_ADDR_CHARGER, 0xb9, 1, &val8, 1);
	val8 = 0x27;	/* enable charging from otg */
	i2c_write(I2C_ADDR_CHARGER, 0xc3, 1, &val8, 1);
	val8 = 0x5;	/* enable charging mode */
	i2c_write(I2C_ADDR_CHARGER, 0xb7, 1, &val8, 1);

#define I2C_ADDR_FUELGAUGE	0x36
#define MAX77823_REG_VCELL	0x09
	ret = i2c_read(I2C_ADDR_FUELGAUGE, MAX77823_REG_VCELL, 1, buf, 2);
	if (!ret) {
		u32 v = (buf[1] << 8) | buf[0];

		v = (v >> 3) * 625;
		if (v < 3000000) {
			printf("voltage = %d uV too low, powering off\n", v);
			board_poweroff();
		}
	} else {
		printf("error reading battery voltage\n");
	}
	i2c_set_bus_num(orig_i2c_bus);

	gpio_set_value(GP_REG_5V, 1);
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, ARRAY_SIZE(displays));
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: H4\n");
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"search",	GP_GPIOKEY_POWER,	'S'},
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
		setenv("board","h4");
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
