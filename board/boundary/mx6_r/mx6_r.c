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
#include <spi.h>
#include <input.h>
#include <splash.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLDOWN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1, SGTL5000 */
static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | PC,
		.gp = IMX_GPIO_NR(3, 28)
	}
};

/* I2C2 Camera, MIPI */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

/* I2C3, J15 - RGB connector */
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_5__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_5__GPIO1_IO05 | PC,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_16__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO_16__GPIO7_IO11 | PC,
		.gp = IMX_GPIO_NR(7, 11)
	}
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_DI0_PIN4__GPIO4_IO20   | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

/* Button assignments for J14 */
static iomux_v3_cfg_t const button_pads[] = {
	/* GPIO_KEYS - J4  */
	NEW_PAD_CTRL(MX6_PAD_EIM_DA0__GPIO3_IO00, WEAK_PULLUP),	/* pin 1 - back */
	NEW_PAD_CTRL(MX6_PAD_EIM_DA1__GPIO3_IO01, WEAK_PULLUP),	/* pin 2 - Home(left) */
	NEW_PAD_CTRL(MX6_PAD_EIM_DA2__GPIO3_IO02, WEAK_PULLUP),	/* pin 3 - Menu */
	NEW_PAD_CTRL(MX6_PAD_EIM_DA3__GPIO3_IO03, WEAK_PULLUP),	/* pin 4 - Down */
	NEW_PAD_CTRL(MX6_PAD_GPIO_18__GPIO7_IO13, WEAK_PULLUP),	/* pin 5 - up */
	NEW_PAD_CTRL(MX6_PAD_GPIO_19__GPIO4_IO05, WEAK_PULLUP),	/* pin 6 - right */
	NEW_PAD_CTRL(MX6_PAD_KEY_COL2__GPIO4_IO10, WEAK_PULLUP),	/* pin 7 - NC */
	NEW_PAD_CTRL(MX6_PAD_KEY_ROW2__GPIO4_IO11, WEAK_PULLUP),	/* pin 8 - NC */
	NEW_PAD_CTRL(MX6_PAD_CSI0_DAT15__GPIO6_IO01, WEAK_PULLDOWN),/* pin 9 inverted, Main power off request */
	NEW_PAD_CTRL(MX6_PAD_NANDF_CS0__GPIO6_IO11, WEAK_PULLUP), /* pin 10 - NC */
	NEW_PAD_CTRL(MX6_PAD_SD1_DAT1__GPIO1_IO17, WEAK_PULLUP),	/* pin 11 - NC */
};

int board_eth_init(bd_t *bis)
{
#ifdef CONFIG_CI_UDC
	/* For otg ethernet*/
	usb_eth_initialize(bis);
#endif
	return 0;
}

static iomux_v3_cfg_t const usb_pads[] = {
	MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),
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
	gpio_direction_output(IMX_GPIO_NR(7, 12), 0);
	mdelay(2);
	gpio_set_value(IMX_GPIO_NR(7, 12), 1);

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC4_BASE_ADDR},
	{USDHC3_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int gp_cd = (cfg->esdhc_base == USDHC3_BASE_ADDR) ? CONFIG_SDHC3_CD :
			CONFIG_SDHC4_CD;

	if (gp_cd >= 0) {
		gpio_direction_input(gp_cd);
		return !gpio_get_value(gp_cd);
	}
	return 1;	/* eMMC is always present */
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
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			break;
		case 0:
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
static iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS1 */
	MX6_PAD_EIM_D19__GPIO3_IO19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

static void setup_spi(void)
{
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

static void setup_buttons(void)
{
	imx_iomux_v3_setup_multiple_pads(button_pads,
					 ARRAY_SIZE(button_pads));
}

#if defined(CONFIG_VIDEO_IPUV3)

static iomux_v3_cfg_t const backlight_pads[] = {
	/* Backlight on RGB connector: J15 */
	MX6_PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define RGB_BACKLIGHT_GP IMX_GPIO_NR(1, 21)
};

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

struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};


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

#ifdef CONFIG_MXC_SPI_DISPLAY
static iomux_v3_cfg_t const ecspi2_pads[] = {
	MX6_PAD_CSI0_DAT8__ECSPI2_SCLK  | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT9__ECSPI2_MOSI  | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__GPIO5_IO29   | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

static int spi_display_read(struct spi_slave *spi, u8 addr, u8 reg, u8 *data, size_t data_len)
{
	u8 cmd[2];
	int ret;

	cmd[0] = addr;
	cmd[1] = reg;
	ret = spi_xfer(spi, 2 * 8, cmd, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret) {
		debug("%s: Failed to select reg 0x%x, %d\n", __func__, reg, ret);
		return ret;
	}
	udelay(3);
	cmd[0] = addr | 3;
	ret = spi_xfer(spi, 1 * 8, cmd, NULL, SPI_XFER_BEGIN);
	if (ret) {
		debug("%s: Failed to start read for reg 0x%x, %d\n", __func__, reg, ret);
		return ret;
	}
	ret = spi_xfer(spi, data_len * 8, NULL, data, SPI_XFER_END);
	if (ret) {
		debug("%s: Failed to read data for reg 0x%x, %d\n", __func__, reg, ret);
		return ret;
	}
	return ret;
}

static int spi_display_cmds(struct spi_slave *spi, u8 addr, u8 *cmds)
{
	u8 cmd_buf[16];
	int ret = 0;

	printf("%s\n", __func__);
	while (*cmds) {
		u8 reg = *cmds++;
		size_t len = *cmds++;

		cmd_buf[0] = addr;
		cmd_buf[1] = reg;
		ret = spi_xfer(spi, 2 * 8, cmd_buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
		if (ret) {
			debug("%s: Failed to select reg 0x%x, %d\n", __func__, reg, ret);
			return ret;
		}
		udelay(3);
		cmd_buf[0] = addr | 2;
		memcpy(&cmd_buf[1], cmds, len);
		cmds += len;
		ret = spi_xfer(spi, (len + 1) * 8, cmd_buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
		if (ret) {
			debug("%s: Failed to write for reg 0x%x, %d\n", __func__, reg, ret);
			return ret;
		}
	}
	return ret;
}

static u8 display_init_cmds[] = {
/* Display Mode Setting */
	0x36, 1, 0x08,
	0x3a, 1, 0x70,
	0xb1, 3, 0x0e, 0x28, 0x0a,
	0xb2, 2, 0x00, 0xc8,
	0xb3, 1, 0x00,
	0xb4, 1, 0x04,
	0xb5, 5, 0x42, 0x10, 0x10, 0x00, 0x20,
	0xb6, 6, 0x0b, 0x0f, 0x3c, 0x13, 0x13, 0xe8,
	0xb7, 5, 0x46, 0x06, 0x0c, 0x00, 0x00,
/* Power Setting */
	0xc0, 2, 0x01, 0x11,
	0xc3, 5, 0x07, 0x03, 0x04, 0x04, 0x04,
	0xc4, 6, 0x12, 0x24, 0x18, 0x18, 0x02, 0x49,
	0xc5, 1, 0x6f,
	0xc6, 2, 0x41, 0x63,
/* Gamma Setting */
	0xd0, 9, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,
	0xd2, 9, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,
	0xd4, 9, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,
	0xd1, 9, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,
	0xd3, 9, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,
	0xd5, 9, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,
/* Sleep out */
	0x11, 0,
	0
};

static u8 display_on_cmds[] = {
	0x29, 0,
	0
};

static void enable_spi_rgb(struct display_info_t const *dev)
{
	unsigned cs_gpio = IMX_GPIO_NR(5, 29);
	struct spi_slave *spi;
	int ret;

	printf("%s\n", __func__);
	gpio_direction_output(IMX_GPIO_NR(1, 21), 1);	/* set sd1_dat3 pwm1 high */
	gpio_direction_output(RGB_BACKLIGHT_GP, 1);
	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, dev->bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(dev->bus, cs_gpio << 8, 1000000, SPI_MODE_3);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		goto free_bus;
	}

	/*
	 * Initialization sequence
	 * 1. Display Mode Settings
	 * 2. Power Settings
	 * 3. Gamma Settings
	 * 4. Sleep Out
	 * 5. Wait >= 7 frame
	 * 6. Display on
	 */
	ret = spi_display_cmds(spi, dev->addr, display_init_cmds);
	if (ret) {
		printf("%s: Failed to display_init_cmds %d\n", __func__, ret);
		goto release_bus;
	}
	mdelay(200);
	ret = spi_display_cmds(spi, dev->addr, display_on_cmds);
	if (ret) {
		printf("%s: Failed to display_on_cmds %d\n", __func__, ret);
		goto release_bus;
	}
	ret = 1;

	/* Release spi bus */
release_bus:
	spi_release_bus(spi);
free_bus:
	spi_free_slave(spi);
	enable_spi_clk(0, dev->bus);
	return;
}

/*
 * Return 1 for successful detection of display
 */
static int detect_spi(struct display_info_t const *dev)
{
	int ret;
	unsigned cs_gpio = IMX_GPIO_NR(5, 29);
	unsigned reset_gpio = IMX_GPIO_NR(2, 5);	/* nandf_d5 */
	struct spi_slave *spi;
	u8 data[8];

	printf("%s\n", __func__);
	gpio_direction_output(cs_gpio, 1);
	gpio_direction_output(reset_gpio, 1);
	imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));
	udelay(1);
	gpio_direction_output(reset_gpio, 0);
	udelay(2000);
	gpio_direction_output(reset_gpio, 1);
	udelay(1);
	enable_spi_clk(1, dev->bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(dev->bus, cs_gpio << 8, 1000000, SPI_MODE_3);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return 0;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		ret = 0;
		goto free_bus;
	}

	/* Read the dispctl1 */
	ret = spi_display_read(spi, dev->addr, 0xb5, data, 2);
	if (ret) {
		printf("%s: Failed to read dispctl1, %d\n", __func__, ret);
		ret = 0;
		goto release_bus;
	}
	debug("%s: *(0x%02x) = 0x%02x 0x%02x\n", __func__, 0xb5, data[0], data[1]);

	if ((data[1] == 0) || (data[1] == 0xff)) {
		ret = 0;
		goto release_bus;
	}
	/*
	 * Initialization sequence
	 * 1. Display Mode Settings
	 * 2. Power Settings
	 * 3. Gamma Settings
	 * 4. Sleep Out
	 * 5. Wait >= 7 frame
	 * 6. Display on
	 */
	ret = spi_display_cmds(spi, dev->addr, display_init_cmds);
	if (ret) {
		printf("%s: Failed to init %d\n", __func__, ret);
		ret = 0;
		goto release_bus;
	}
	ret = 1;

	/* Release spi bus */
release_bus:
	spi_release_bus(spi);
free_bus:
	spi_free_slave(spi);
	enable_spi_clk(0, dev->bus);
	return ret;
}
#endif

static void enable_rgb(struct display_info_t const *dev)
{
	gpio_direction_output(RGB_BACKLIGHT_GP, 1);
}

static struct display_info_t const displays[] = {
#ifdef CONFIG_MXC_SPI_DISPLAY
{
	.bus	= 1,
	.addr	= 0x70,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_spi,
	.enable	= enable_spi_rgb,
	.mode	= {
		.name           = "LB043",
		.refresh        = 57,
		.xres           = 480,
		.yres           = 800,
		.pixclock       = 37037,
		.left_margin    = 40,
		.right_margin   = 60,
		.upper_margin   = 10,
		.lower_margin   = 10,
		.hsync_len      = 20,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
	},
},
#endif
{
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
} },
};

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel;

	imx_iomux_v3_setup_multiple_pads(
		rgb_pads,
		 ARRAY_SIZE(rgb_pads));

	panel = getenv("panel");
	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays+i;
			if (dev->detect && dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			panel = displays[0].mode.name;
			printf("No panel detected: default to %s\n", panel);
			i = 0;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else {
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
		}
	} else {
		printf("unsupported panel %s\n", panel);
		ret = -EINVAL;
	}

	if (!ret)
		splash_screen_prepare();

	return (0 != ret);
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
	     |IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
			|IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

	/* backlights off until needed */
	imx_iomux_v3_setup_multiple_pads(backlight_pads,
					 ARRAY_SIZE(backlight_pads));
	gpio_direction_input(RGB_BACKLIGHT_GP);
}
#endif

static iomux_v3_cfg_t const init_pads[] = {
	/* WL12XX_WL_IRQ_GP */
	NEW_PAD_CTRL(MX6_PAD_NANDF_CS1__GPIO6_IO14, WEAK_PULLDOWN),
	/* WL12XX_WL_ENABLE_GP */
	NEW_PAD_CTRL(MX6_PAD_NANDF_CS2__GPIO6_IO15, OUTPUT_40OHM),
	/* WL12XX_BT_ENABLE_GP */
	NEW_PAD_CTRL(MX6_PAD_NANDF_CS3__GPIO6_IO16, OUTPUT_40OHM),
	/* USB otg power */
	NEW_PAD_CTRL(MX6_PAD_EIM_D22__GPIO3_IO22, OUTPUT_40OHM),
	/* Main power */
	NEW_PAD_CTRL(MX6_PAD_SD1_DAT0__GPIO1_IO16, OUTPUT_40OHM),
	/* Mipi Camera Reset */
	NEW_PAD_CTRL(MX6_PAD_CSI0_DATA_EN__GPIO5_IO20, OUTPUT_40OHM),
	/* Mipi Powerdown */
	NEW_PAD_CTRL(MX6_PAD_CSI0_VSYNC__GPIO5_IO21, OUTPUT_40OHM),
	/* LED control */
	NEW_PAD_CTRL(MX6_PAD_EIM_D20__GPIO3_IO20, OUTPUT_40OHM),
	/* 5VEN */
	NEW_PAD_CTRL(MX6_PAD_NANDF_WP_B__GPIO6_IO09, OUTPUT_40OHM),
	/* 3P3VEN */
	NEW_PAD_CTRL(MX6_PAD_EIM_RW__GPIO2_IO26, OUTPUT_40OHM),
	/* 5P4VEN */
	NEW_PAD_CTRL(MX6_PAD_NANDF_D1__GPIO2_IO01, OUTPUT_40OHM),
	/* ??? function */
	NEW_PAD_CTRL(MX6_PAD_EIM_D29__GPIO3_IO29, OUTPUT_40OHM),
	/* BAT status */
	NEW_PAD_CTRL(MX6_PAD_NANDF_D2__GPIO2_IO02, WEAK_PULLUP),
	/* Misc outputs */
	NEW_PAD_CTRL(MX6_PAD_GPIO_3__GPIO1_IO03, OUTPUT_40OHM),
};

#define WL12XX_WL_IRQ_GP	IMX_GPIO_NR(6, 14)

static unsigned gpios_out_low[] = {
	/* Disable wl1271 */
	IMX_GPIO_NR(6, 15),	/* disable wireless */
	IMX_GPIO_NR(6, 16), 	/* disable bluetooth */
	IMX_GPIO_NR(3, 22),	/* disable USB otg power */
	IMX_GPIO_NR(5, 20),	/* ov5640 mipi camera reset */
	IMX_GPIO_NR(3, 20),	/* turn led off */
	/* No 5VEN */
	IMX_GPIO_NR(6, 9),
	/* No 3P3VEN */
	IMX_GPIO_NR(2, 26),
	/* No 5P4VEN */
	IMX_GPIO_NR(2, 1),
	IMX_GPIO_NR(1, 3),	/* Unknown function */
};

static unsigned gpios_out_high[] = {
	IMX_GPIO_NR(1, 16),	/* Enable main power */
	IMX_GPIO_NR(5, 21),	/* ov5640 mipi camera power down */
	IMX_GPIO_NR(3, 29),	/* Unknown function */
};

static void set_gpios(unsigned *p, int cnt, int val)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_output(*p++, val);
}

int board_early_init_f(void)
{
	setup_iomux_uart();

	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	gpio_direction_input(WL12XX_WL_IRQ_GP);

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	setup_buttons();

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
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	imx_iomux_v3_setup_multiple_pads(
		usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	return 0;
}

static char const *board_type = "uninitialized";

int checkboard(void)
{
	puts("Board: " CONFIG_BOARD_NAME "\n");
	board_type = CONFIG_BOARD_NAME;
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
	unsigned char	active_low;
};

static struct button_key const buttons[] = {
	{"back",	IMX_GPIO_NR(3, 0),	'B', 1},
	{"up",		IMX_GPIO_NR(3, 1),	'U', 1},
	{"menu",	IMX_GPIO_NR(3, 2),	'M', 1},
	{"left",	IMX_GPIO_NR(3, 3),	'L', 1},
	{"right",	IMX_GPIO_NR(7, 13),	'R', 1},
	{"down",	IMX_GPIO_NR(4, 5),	'D', 1},
	{"power",	IMX_GPIO_NR(6, 1),	'P', 0},
};

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed
 */
static int read_keys(char *buf)
{
	int i, numpressed = 0;
	int val;

	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		val = gpio_get_value(buttons[i].gpnum);
		if (buttons[i].active_low)
			val = !val;
		if (val)
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
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
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
	setenv("board",board_type);
	return 0;
}

void board_poweroff(void)
{
	/* Turn off main power */
	gpio_direction_output(IMX_GPIO_NR(1, 16), 0);
	udelay(1000000);
}

static int do_poweroff(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	board_poweroff();
	return 0;
}

U_BOOT_CMD(poweroff, 1, 1, do_poweroff, "Turn off power", "");
