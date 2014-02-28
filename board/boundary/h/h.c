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
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <i2c.h>
#include <input.h>
#include <netdev.h>
#include <usb/ehci-ci.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |	       \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |	       \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |	       \
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |	       \
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED	  |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define RGB_PAD_CTRL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define WEAK_PULLUP	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLDOWN	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_SRE_SLOW)

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)

/*
 *
 */
static const iomux_v3_cfg_t init_pads[] = {
	/* ECSPI1 pads */
	IOMUX_PAD_CTRL(EIM_D17__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D18__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D16__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS		IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(EIM_D19__GPIO3_IO19, SPI_PAD_CTRL),

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
	IOMUX_PAD_CTRL(EIM_D23__GPIO3_IO23, WEAK_PULLUP),
	/* pin 42 PHY nRST */
#define GP_PHY_RESET		IMX_GPIO_NR(1, 27)
	IOMUX_PAD_CTRL(ENET_RXD0__GPIO1_IO27, WEAK_PULLUP),

	/* i2c mux pads */
#define GP_I2C_EN_MIPI		IMX_GPIO_NR(2, 16)
	IOMUX_PAD_CTRL(EIM_A22__GPIO2_IO16, WEAK_PULLDOWN),	/* mipi I2C enable */
#define GP_I2C_EN_LVDS0		IMX_GPIO_NR(2, 21)
	IOMUX_PAD_CTRL(EIM_A17__GPIO2_IO21, WEAK_PULLDOWN),	/* LVDS0 I2C enable */
#define GP_I2C_EN_LVDS1		IMX_GPIO_NR(2, 22)
	IOMUX_PAD_CTRL(EIM_A16__GPIO2_IO22, WEAK_PULLDOWN),	/* LVDS1 I2C enable */
#define GP_I2C_EN_RTC		IMX_GPIO_NR(2, 23)
	IOMUX_PAD_CTRL(EIM_CS0__GPIO2_IO23, WEAK_PULLDOWN),	/* RTC I2C enable */
#define GP_I2C_EN_AR1020	IMX_GPIO_NR(7, 13)
	IOMUX_PAD_CTRL(GPIO_18__GPIO7_IO13, WEAK_PULLDOWN),	/* AR1020 I2C enable */

	/* Broadcom bcm4330 pads on nitrogen6x */
#define GP_WL_EN		IMX_GPIO_NR(6, 7)	/* NANDF_CLE - active high */
        IOMUX_PAD_CTRL(NANDF_CLE__GPIO6_IO07, OUTPUT_40OHM),	/* wlan regulator enable */
#define GP_WL_WAKE_IRQ		IMX_GPIO_NR(6, 14)	/* NANDF_CS1 - active low */
	IOMUX_PAD_CTRL(NANDF_CS1__GPIO6_IO14, WEAK_PULLDOWN),	/* wlan wake irq */
#define GP_WL_BT_REG_EN		IMX_GPIO_NR(6, 15)	/* NANDF_CS2 - active high */
	IOMUX_PAD_CTRL(NANDF_CS2__GPIO6_IO15, OUTPUT_40OHM),	/* bt regulator enable */
#define GP_WL_BT_WAKE_IRQ	IMX_GPIO_NR(6, 16)	/* NANDF_CS3 - active low */
	IOMUX_PAD_CTRL(NANDF_CS3__GPIO6_IO16, OUTPUT_40OHM),	/* bt wake irq */
	IOMUX_PAD_CTRL(NANDF_D2__GPIO2_IO02, OUTPUT_40OHM),		/* bt wake */
#define GP_WL_BT_RESET		IMX_GPIO_NR(6, 8)	/* NANDF_ALE - active low */
	IOMUX_PAD_CTRL(NANDF_ALE__GPIO6_IO08, OUTPUT_40OHM),	/* bt reset */
#define GP_WL_CLK_REQ_IRQ	IMX_GPIO_NR(6, 9)	/* NANDF_WP_B - active low */

#define GP_RGB_BACKLIGHT_PWM		IMX_GPIO_NR(1, 21)
	IOMUX_PAD_CTRL(SD1_DAT3__GPIO1_IO21, WEAK_PULLDOWN),
#define GP_LVDS0_BACKLIGHT_PWM		IMX_GPIO_NR(1, 18)
	IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, WEAK_PULLDOWN),
#define GP_LVDS1_BACKLIGHT_PWM		IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(SD1_DAT1__GPIO1_IO17, WEAK_PULLDOWN),
#define GP_RGB_MIRROR_H			IMX_GPIO_NR(2, 25)
	IOMUX_PAD_CTRL(EIM_OE__GPIO2_IO25, WEAK_PULLUP),	/* DI0 display left/right mirror */
#define GP_RGB_MIRROR_V			IMX_GPIO_NR(2, 27)
	IOMUX_PAD_CTRL(EIM_LBA__GPIO2_IO27, WEAK_PULLUP),	/* DI0 display up/down mirror */
#define GP_LVDS0_12V_5V_BL_SELECT	IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(GPIO_19__GPIO4_IO05, WEAK_PULLDOWN),	/* LVDS0 12v/5v select, 0 - 5v, 1 - 12v */
#define GP_RGB_LVDS1_12V_5V_BL_SELECT	IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO_7__GPIO1_IO07, WEAK_PULLDOWN),	/* rgb/LVDS1 12v/5v select, 0 - 5v, 1 - 12v */
#define GP_12V_POWER_EN			IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, WEAK_PULLDOWN),	/* 12v power enable */

	/* UART1 */
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),

	/* UART2 */
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),

	/* USB */
#define GP_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
	IOMUX_PAD_CTRL(GPIO_17__GPIO7_IO12, WEAK_PULLUP),	/* Hub reset */
	IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, USDHC_PAD_CTRL), /* USBOTG ID pin */
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLUP),	/* usbotg power */
	IOMUX_PAD_CTRL(KEY_COL4__USB_OTG_OC, WEAK_PULLUP),	/* USBOTG OC pin */
	IOMUX_PAD_CTRL(EIM_RW__GPIO2_IO26, WEAK_PULLUP),	/* Rev1 usb power */
	IOMUX_PAD_CTRL(EIM_D20__GPIO3_IO20, WEAK_PULLUP),	/* Rev1 usb power */
	IOMUX_PAD_CTRL(EIM_A25__GPIO5_IO02, WEAK_PULLUP),	/* Rev1 usb power */

	/* USDHC3 - sdcard */
	IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
#define GP_USDHC3_CD		IMX_GPIO_NR(7, 0)
	IOMUX_PAD_CTRL(SD3_DAT5__GPIO7_IO00, WEAK_PULLUP), /* CD */

	/* USDHC4  */
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL),
#define GP_USDHC4_CD		IMX_GPIO_NR(2, 6)
	IOMUX_PAD_CTRL(NANDF_D6__GPIO2_IO06, WEAK_PULLUP), /* CD */
};

#ifdef CONFIG_FEC_MXC
static const iomux_v3_cfg_t enet_pads1[] = {
	/* pin 35 - 1 (PHY_AD2) on reset */
#define GP_PHY_AD2		IMX_GPIO_NR(6, 30)
	IOMUX_PAD_CTRL(RGMII_RXC__GPIO6_IO30, WEAK_PULLUP),
	/* pin 32 - 1 - (MODE0) all */
#define GP_PHY_MODE0		IMX_GPIO_NR(6, 25)
	IOMUX_PAD_CTRL(RGMII_RD0__GPIO6_IO25, WEAK_PULLUP),
	/* pin 31 - 1 - (MODE1) all */
#define GP_PHY_MODE1		IMX_GPIO_NR(6, 27)
	IOMUX_PAD_CTRL(RGMII_RD1__GPIO6_IO27, WEAK_PULLUP),
	/* pin 28 - 1 - (MODE2) all */
#define GP_PHY_MODE2		IMX_GPIO_NR(6, 28)
	IOMUX_PAD_CTRL(RGMII_RD2__GPIO6_IO28, WEAK_PULLUP),
	/* pin 27 - 1 - (MODE3) all */
#define GP_PHY_MODE3		IMX_GPIO_NR(6, 29)
	IOMUX_PAD_CTRL(RGMII_RD3__GPIO6_IO29, WEAK_PULLUP),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
#define GP_PHY_CLK125		IMX_GPIO_NR(6, 24)
	IOMUX_PAD_CTRL(RGMII_RX_CTL__GPIO6_IO24, WEAK_PULLUP),
};

static const iomux_v3_cfg_t enet_pads2[] = {
	IOMUX_PAD_CTRL(RGMII_RXC__RGMII_RXC, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD0__RGMII_RD0, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD1__RGMII_RD1, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD2__RGMII_RD2, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD3__RGMII_RD3, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RX_CTL__RGMII_RX_CTL, ENET_PAD_CTRL),
};
#endif

#ifdef CONFIG_CMD_FBPANEL
static const iomux_v3_cfg_t rgb_pads[] = {
	IOMUX_PAD_CTRL(DI0_DISP_CLK__IPU1_DI0_DISP_CLK, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN15__IPU1_DI0_PIN15, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN2__IPU1_DI0_PIN02, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN3__IPU1_DI0_PIN03, RGB_PAD_CTRL),
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
#endif

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

static struct i2c_pads_info i2c_pads[] = {
	/* I2C1, SGTL5000 */
	I2C_PADS_INFO_ENTRY(I2C1, EIM_D21, 3, 21, EIM_D28, 3, 28, I2C_PAD_CTRL),
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
		gpio_direction_output(GP_USB_HUB_RESET, 0);
		mdelay(2);
		gpio_set_value(GP_USB_HUB_RESET, 1);
	}
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
	int gp = (cfg->esdhc_base == USDHC3_BASE_ADDR) ? GP_USDHC3_CD : GP_USDHC4_CD;

	return !gpio_get_value(gp);
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
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : -1;
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

static void setup_iomux_enet(void)
{
	gpio_direction_output(GP_PHY_RESET, 0);
	gpio_direction_output(GP_PHY_AD2, 1);
	gpio_direction_output(GP_PHY_MODE0, 1);
	gpio_direction_output(GP_PHY_MODE1, 1);
	gpio_direction_output(GP_PHY_MODE2, 1);
	gpio_direction_output(GP_PHY_MODE3, 1);
	gpio_direction_output(GP_PHY_CLK125, 1);
	SETUP_IOMUX_PADS(enet_pads1);

	/* Need delay 10ms according to KSZ9021 spec */
	udelay(1000 * 10);
	gpio_set_value(GP_PHY_RESET, 1);
	SETUP_IOMUX_PADS(enet_pads2);
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
	if (!getenv("eth1addr"))
		setenv("eth1addr", getenv("usbnet_devaddr"));
	usb_eth_initialize(bis);
#endif
	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
void board_enable_lcd(const struct display_info_t *di, int enable)
{
	if (enable)
		SETUP_IOMUX_PADS(rgb_pads);
	else
		SETUP_IOMUX_PADS(rgb_gpio_pads);
	gpio_direction_output(GP_RGB_BACKLIGHT_PWM, enable);
}

void board_enable_lvds(const struct display_info_t *di, int enable)
{
	gpio_direction_output(GP_LVDS0_BACKLIGHT_PWM, enable);
	gpio_direction_output(GP_LVDS1_BACKLIGHT_PWM, enable ^ 1);
}

void board_enable_lvds2(const struct display_info_t *di, int enable)
{
	gpio_direction_output(GP_LVDS0_BACKLIGHT_PWM, enable ^ 1);
	gpio_direction_output(GP_LVDS1_BACKLIGHT_PWM, enable);
}

static const struct display_info_t displays[] = {
	VD_1024_600(LVDS, fbp_detect_i2c, 2, 0x04),
	VD_INNOLUX_WVGA(LVDS2, fbp_detect_i2c, 2, 0x48),
};
#endif

static const unsigned short gpios_out_low[] = {
	/* Disable wl1271 */
	GP_WL_EN,
	GP_WL_BT_REG_EN,
	GP_WL_BT_RESET,
	GP_RGB_MIRROR_V,
	GP_LVDS0_12V_5V_BL_SELECT,
	GP_RGB_LVDS1_12V_5V_BL_SELECT,
	GP_12V_POWER_EN,
	GP_I2C_EN_MIPI,
	GP_I2C_EN_LVDS0,
	GP_I2C_EN_LVDS1,
	GP_I2C_EN_RTC,
	GP_I2C_EN_AR1020,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,
	GP_RGB_MIRROR_H,
};

static const unsigned short gpios_in[] = {
	GP_WL_WAKE_IRQ,
	GP_WL_BT_WAKE_IRQ,
	GP_WL_CLK_REQ_IRQ,
	GP_RGB_BACKLIGHT_PWM,
	GP_LVDS0_BACKLIGHT_PWM,
	GP_LVDS1_BACKLIGHT_PWM,
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
	struct i2c_pads_info *p = i2c_pads + i2c_get_info_entry_offset();
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, p);
	p += I2C_PADS_INFO_ENTRY_SPACING;
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, p);

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, ARRAY_SIZE(displays));
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: Boundary H\n");

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
		setenv("board", "h");
	setenv("uboot_defconfig", CONFIG_DEFCONFIG);
	return 0;
}
