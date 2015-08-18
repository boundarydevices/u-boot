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
#include <asm/imx-common/video.h>
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
#include <usb/ehci-fsl.h>

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

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	NEW_PAD_CTRL(MX6_PAD_SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart2_pads[] = {
	NEW_PAD_CTRL(MX6_PAD_EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),
};

#define PC(a) NEW_PAD_CTRL(a, I2C_PAD_CTRL)

/* I2C1, SGTL5000 */
static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = PC(MX6_PAD_EIM_D21__I2C1_SCL),
		.gpio_mode = PC(MX6_PAD_EIM_D21__GPIO3_IO21),
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = PC(MX6_PAD_EIM_D28__I2C1_SDA),
		.gpio_mode = PC(MX6_PAD_EIM_D28__GPIO3_IO28),
		.gp = IMX_GPIO_NR(3, 28)
	}
};

/* I2C3, J15 - RGB connector */
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = PC(MX6_PAD_GPIO_5__I2C3_SCL),
		.gpio_mode = PC(MX6_PAD_GPIO_5__GPIO1_IO05),
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = PC(MX6_PAD_GPIO_16__I2C3_SDA),
		.gpio_mode = PC(MX6_PAD_GPIO_16__GPIO7_IO11),
		.gp = IMX_GPIO_NR(7, 11)
	}
};

#define GP_SD3_CD		IMX_GPIO_NR(7, 0)

static iomux_v3_cfg_t const usdhc3_pads[] = {
	NEW_PAD_CTRL(MX6_PAD_SD3_CLK__SD3_CLK, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD3_DAT5__GPIO7_IO00, NO_PAD_CTRL), /* CD */
};

#define GP_SD4_CD		IMX_GPIO_NR(2, 6)

static iomux_v3_cfg_t const usdhc4_pads[] = {
	NEW_PAD_CTRL(MX6_PAD_SD4_CLK__SD4_CLK, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD4_CMD__SD4_CMD, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_NANDF_D6__GPIO2_IO06, NO_PAD_CTRL), /* CD */
};

#define GP_PHY_AD2		IMX_GPIO_NR(6, 30)
#define GP_PHY_MODE0		IMX_GPIO_NR(6, 25)
#define GP_PHY_MODE1		IMX_GPIO_NR(6, 27)
#define GP_PHY_MODE2		IMX_GPIO_NR(6, 28)
#define GP_PHY_MODE3		IMX_GPIO_NR(6, 29)
#define GP_PHY_CLK125		IMX_GPIO_NR(6, 24)
#define GP_PHY_RESET		IMX_GPIO_NR(1, 27)

#ifdef CONFIG_FEC_MXC
static iomux_v3_cfg_t const enet_pads1[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* pin 35 - 1 (PHY_AD2) on reset */
	MX6_PAD_RGMII_RXC__GPIO6_IO30		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 32 - 1 - (MODE0) all */
	MX6_PAD_RGMII_RD0__GPIO6_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 31 - 1 - (MODE1) all */
	MX6_PAD_RGMII_RD1__GPIO6_IO27		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 28 - 1 - (MODE2) all */
	MX6_PAD_RGMII_RD2__GPIO6_IO28		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 27 - 1 - (MODE3) all */
	MX6_PAD_RGMII_RD3__GPIO6_IO29		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	MX6_PAD_RGMII_RX_CTL__GPIO6_IO24	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 42 PHY nRST */
	MX6_PAD_EIM_D23__GPIO3_IO23		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_ENET_RXD0__GPIO1_IO27		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const enet_pads2[] = {
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
};
#endif

/* Broadcom bcm4330 pads on nitrogen6x */
static iomux_v3_cfg_t const init_pads[] = {
        NEW_PAD_CTRL(MX6_PAD_NANDF_CLE__GPIO6_IO07, OUTPUT_40OHM),	/* wlan regulator enable */
	NEW_PAD_CTRL(MX6_PAD_NANDF_CS1__GPIO6_IO14, WEAK_PULLDOWN),	/* wlan wake irq */
	NEW_PAD_CTRL(MX6_PAD_NANDF_CS2__GPIO6_IO15, OUTPUT_40OHM),	/* bt regulator enable */
	NEW_PAD_CTRL(MX6_PAD_NANDF_CS3__GPIO6_IO16, OUTPUT_40OHM),	/* bt wake irq */
	NEW_PAD_CTRL(MX6_PAD_NANDF_D2__GPIO2_IO02, OUTPUT_40OHM),		/* bt wake */
	NEW_PAD_CTRL(MX6_PAD_NANDF_ALE__GPIO6_IO08, OUTPUT_40OHM),	/* bt reset */

#define GP_RGB_BACKLIGHT_PWM		IMX_GPIO_NR(1, 21)
	NEW_PAD_CTRL(MX6_PAD_SD1_DAT3__GPIO1_IO21, NO_PAD_CTRL),
#define GP_LVDS0_BACKLIGHT_PWM		IMX_GPIO_NR(1, 18)
	NEW_PAD_CTRL(MX6_PAD_SD1_CMD__GPIO1_IO18, NO_PAD_CTRL),
#define GP_LVDS1_BACKLIGHT_PWM		IMX_GPIO_NR(1, 17)
	NEW_PAD_CTRL(MX6_PAD_SD1_DAT1__GPIO1_IO17, NO_PAD_CTRL),
#define GP_RGB_MIRROR_H			IMX_GPIO_NR(2, 25)
	NEW_PAD_CTRL(MX6_PAD_EIM_OE__GPIO2_IO25, NO_PAD_CTRL),	/* DI0 display left/right mirror */
#define GP_RGB_MIRROR_V			IMX_GPIO_NR(2, 27)
	NEW_PAD_CTRL(MX6_PAD_EIM_LBA__GPIO2_IO27, NO_PAD_CTRL),	/* DI0 display up/down mirror */
#define GP_LVDS0_12V_5V_BL_SELECT	IMX_GPIO_NR(4, 5)
	NEW_PAD_CTRL(MX6_PAD_GPIO_19__GPIO4_IO05,	WEAK_PULLDOWN),	/* LVDS0 12v/5v select, 0 - 5v, 1 - 12v */
#define GP_RGB_LVDS1_12V_5V_BL_SELECT	IMX_GPIO_NR(1, 7)
	NEW_PAD_CTRL(MX6_PAD_GPIO_7__GPIO1_IO07, WEAK_PULLDOWN),	/* rgb/LVDS1 12v/5v select, 0 - 5v, 1 - 12v */
#define GP_12V_POWER_EN			IMX_GPIO_NR(4, 20)
	NEW_PAD_CTRL(MX6_PAD_DI0_PIN4__GPIO4_IO20, WEAK_PULLDOWN),	/* 12v power enable */
};

#define GP_WL_EN		IMX_GPIO_NR(6, 7)	/* NANDF_CLE - active high */
#define GP_WL_WAKE_IRQ		IMX_GPIO_NR(6, 14)	/* NANDF_CS1 - active low */
#define GP_WL_BT_REG_EN		IMX_GPIO_NR(6, 15)	/* NANDF_CS2 - active high */
#define GP_WL_BT_WAKE_IRQ	IMX_GPIO_NR(6, 16)	/* NANDF_CS3 - active low */
#define GP_WL_BT_RESET		IMX_GPIO_NR(6, 8)	/* NANDF_ALE - active low */
#define GP_WL_CLK_REQ_IRQ	IMX_GPIO_NR(6, 9)	/* NANDF_WP_B - active low */

static void setup_iomux_enet(void)
{
	gpio_direction_output(GP_PHY_RESET, 0);
	gpio_direction_output(GP_PHY_AD2, 1);
	gpio_direction_output(GP_PHY_MODE0, 1);
	gpio_direction_output(GP_PHY_MODE1, 1);
	gpio_direction_output(GP_PHY_MODE2, 1);
	gpio_direction_output(GP_PHY_MODE3, 1);
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));
	gpio_direction_output(GP_PHY_CLK125, 1);

	/* Need delay 10ms according to KSZ9021 spec */
	udelay(1000 * 10);
	gpio_set_value(GP_PHY_RESET, 1);
	imx_iomux_v3_setup_multiple_pads(enet_pads2, ARRAY_SIZE(enet_pads2));
}

#define GP_USB_HUB_RESET	IMX_GPIO_NR(7, 12)

static iomux_v3_cfg_t const usb_pads[] = {
	NEW_PAD_CTRL(MX6_PAD_GPIO_17__GPIO7_IO12, WEAK_PULLUP),	/* Hub reset */
	NEW_PAD_CTRL(MX6_PAD_GPIO_1__USB_OTG_ID, USDHC_PAD_CTRL), /* USBOTG ID pin */
	NEW_PAD_CTRL(MX6_PAD_EIM_D22__GPIO3_IO22, WEAK_PULLUP),	/* usbotg power */
	MX6_PAD_KEY_COL4__USB_OTG_OC,			/* USBOTG OC pin */
	NEW_PAD_CTRL(MX6_PAD_EIM_RW__GPIO2_IO26, WEAK_PULLUP),	/* Rev1 usb power */
	NEW_PAD_CTRL(MX6_PAD_EIM_D20__GPIO3_IO20, WEAK_PULLUP),	/* Rev1 usb power */
	NEW_PAD_CTRL(MX6_PAD_EIM_A25__GPIO5_IO02, WEAK_PULLUP),	/* Rev1 usb power */
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));

	/* Reset USB hub */
	gpio_direction_output(GP_USB_HUB_RESET, 0);
	mdelay(2);
	gpio_set_value(GP_USB_HUB_RESET, 1);

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
	int gp = (cfg->esdhc_base == USDHC3_BASE_ADDR) ? GP_SD3_CD : GP_SD4_CD;

	gpio_direction_input(gp);
	return !gpio_get_value(gp);
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
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			break;
		case 1:
		       imx_iomux_v3_setup_multiple_pads(
			       usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
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
	return (bus == 0 && cs == 0) ? (IMX_GPIO_NR(3, 19)) : -1;
}

static iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS1 */
	NEW_PAD_CTRL(MX6_PAD_EIM_D19__GPIO3_IO19, SPI_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_EIM_D17__ECSPI1_MISO, SPI_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_EIM_D18__ECSPI1_MOSI, SPI_PAD_CTRL),
	NEW_PAD_CTRL(MX6_PAD_EIM_D16__ECSPI1_SCLK, SPI_PAD_CTRL),
};

static void setup_spi(void)
{
	gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));
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

#if defined(CONFIG_VIDEO_IPUV3)

static iomux_v3_cfg_t const rgb_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00,
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01,
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02,
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03,
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04,
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05,
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06,
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07,
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08,
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09,
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10,
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11,
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12,
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13,
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14,
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15,
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16,
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17,
	MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18,
	MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19,
	MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20,
	MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21,
	MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22,
	MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23,
};

void board_enable_lcd(const struct display_info_t *di)
{
	imx_iomux_v3_setup_multiple_pads(rgb_pads, ARRAY_SIZE(rgb_pads));
	gpio_direction_output(GP_RGB_BACKLIGHT_PWM, 1);
}

void board_enable_lvds(const struct display_info_t *di)
{
	gpio_direction_output(GP_LVDS0_BACKLIGHT_PWM, 1);
	gpio_direction_output(GP_LVDS1_BACKLIGHT_PWM, 0);
}

void board_enable_lvds2(const struct display_info_t *di)
{
	gpio_direction_output(GP_LVDS0_BACKLIGHT_PWM, 0);
	gpio_direction_output(GP_LVDS1_BACKLIGHT_PWM, 1);
}

const struct display_info_t displays[] = {
	IMX_VD04_1024_600(LVDS, 1, 2),
	IMX_VD48_INNOLUX_WVGA(LVDS2, 1, 2),
};

size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;

	reg = readl(&mxc_ccm->chsccdr);
	reg &= ~(MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK |
		 MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK);
	reg |= (CHSCCDR_PODF_DIVIDE_BY_3 << MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET)
	      |(CHSCCDR_IPU_PRE_CLK_540M_PFD << MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);
}
#endif

static unsigned gpios_out_low[] = {
	/* Disable wl1271 */
	GP_WL_EN,
	GP_WL_BT_REG_EN,
	GP_WL_BT_RESET,
	GP_RGB_MIRROR_V,
	GP_LVDS0_12V_5V_BL_SELECT,
	GP_RGB_LVDS1_12V_5V_BL_SELECT,
	GP_12V_POWER_EN,
};

static unsigned gpios_out_high[] = {
	GP_RGB_MIRROR_H,
};

static unsigned short gpios_in[] = {
	GP_WL_WAKE_IRQ,
	GP_WL_BT_WAKE_IRQ,
	GP_WL_CLK_REQ_IRQ,
	GP_RGB_BACKLIGHT_PWM,
	GP_LVDS0_BACKLIGHT_PWM,
	GP_LVDS1_BACKLIGHT_PWM,
};

static void set_gpios_in(unsigned short *p, int cnt)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_input(*p++);
}

static void set_gpios(unsigned *p, int cnt, int val)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_output(*p++, val);
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	set_gpios_in(gpios_in, ARRAY_SIZE(gpios_in));
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

#if defined(CONFIG_VIDEO_IPUV3)
	imx_setup_display();
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

#define GP_I2C_EN_MIPI		IMX_GPIO_NR(2, 16)
#define GP_I2C_EN_LVDS0		IMX_GPIO_NR(2, 21)
#define GP_I2C_EN_LVDS1		IMX_GPIO_NR(2, 22)
#define GP_I2C_EN_RTC		IMX_GPIO_NR(2, 23)
#define GP_I2C_EN_AR1020	IMX_GPIO_NR(7, 13)

static iomux_v3_cfg_t const i2c_mux_pads[] = {
	NEW_PAD_CTRL(MX6_PAD_EIM_A22__GPIO2_IO16, WEAK_PULLDOWN),	/* mipi I2C enable */
	NEW_PAD_CTRL(MX6_PAD_EIM_A17__GPIO2_IO21, WEAK_PULLDOWN),	/* LVDS0 I2C enable */
	NEW_PAD_CTRL(MX6_PAD_EIM_A16__GPIO2_IO22, WEAK_PULLDOWN),	/* LVDS1 I2C enable */
	NEW_PAD_CTRL(MX6_PAD_EIM_CS0__GPIO2_IO23, WEAK_PULLDOWN),	/* RTC I2C enable */
	NEW_PAD_CTRL(MX6_PAD_GPIO_18__GPIO7_IO13, WEAK_PULLDOWN),	/* AR1020 I2C enable */
};

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	imx_iomux_v3_setup_multiple_pads(i2c_mux_pads,
					 ARRAY_SIZE(i2c_mux_pads));
	gpio_direction_output(GP_I2C_EN_MIPI, 0);
	gpio_direction_output(GP_I2C_EN_LVDS0, 0);
	gpio_direction_output(GP_I2C_EN_LVDS1, 0);
	gpio_direction_output(GP_I2C_EN_RTC, 0);
	gpio_direction_output(GP_I2C_EN_AR1020, 0);
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

#ifdef CONFIG_CMD_SATA
	setup_sata();
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
	if (!getenv("uboot_defconfig"))
		setenv("uboot_defconfig", CONFIG_DEFCONFIG);
	return 0;
}
