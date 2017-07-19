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
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <spi.h>
#include <input.h>
#include <splash.h>
#include <usb/ehci-ci.h>
#include "../padctrl.h"

DECLARE_GLOBAL_DATA_PTR;

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define RGB_PAD_CTRL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

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
	/* bt_rfkill */
#define GP_BT_RFKILL_RESET	IMX_GPIO_NR(6, 16)
	IOMUX_PAD_CTRL(NANDF_CS3__GPIO6_IO16, WEAK_PULLDN),

	/* ECSPI1 */
	IOMUX_PAD_CTRL(EIM_D17__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D18__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D16__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(EIM_D19__GPIO3_IO19, WEAK_PULLUP),

	/* ECSPI2 */
	IOMUX_PAD_CTRL(CSI0_DAT8__ECSPI2_SCLK, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT9__ECSPI2_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT10__ECSPI2_MISO, SPI_PAD_CTRL),
#define GP_ECSPI2_CS		IMX_GPIO_NR(5, 29)
	IOMUX_PAD_CTRL(CSI0_DAT11__GPIO5_IO29, WEAK_PULLUP),
#define GP_SPI_DISPLAY_RESET	IMX_GPIO_NR(2, 5)
	IOMUX_PAD_CTRL(NANDF_D5__GPIO2_IO05, WEAK_PULLUP),

	/* gpio_Keys - Button assignments for J14 */
#define GP_GPIOKEY_BACK		IMX_GPIO_NR(3, 0)
	IOMUX_PAD_CTRL(EIM_DA0__GPIO3_IO00, WEAK_PULLUP),	/* pin 1 - back */
#define GP_GPIOKEY_UP		IMX_GPIO_NR(3, 1)
	IOMUX_PAD_CTRL(EIM_DA1__GPIO3_IO01, WEAK_PULLUP),	/* pin 2 - up */
#define GP_GPIOKEY_MENU		IMX_GPIO_NR(3, 2)
	IOMUX_PAD_CTRL(EIM_DA2__GPIO3_IO02, WEAK_PULLUP),	/* pin 3 - Menu */
#define GP_GPIOKEY_LEFT		IMX_GPIO_NR(3, 3)
	IOMUX_PAD_CTRL(EIM_DA3__GPIO3_IO03, WEAK_PULLUP),	/* pin 4 - Left */
#define GP_GPIOKEY_RIGHT	IMX_GPIO_NR(7, 13)
	IOMUX_PAD_CTRL(GPIO_18__GPIO7_IO13, WEAK_PULLUP),	/* pin 5 - Right */
#define GP_GPIOKEY_DOWN		IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(GPIO_19__GPIO4_IO05, WEAK_PULLUP),	/* pin 6 - Down */
	IOMUX_PAD_CTRL(KEY_COL2__GPIO4_IO10, WEAK_PULLUP),	/* pin 7 - NC */
	IOMUX_PAD_CTRL(KEY_ROW2__GPIO4_IO11, WEAK_PULLUP),	/* pin 8 - NC */
#define GP_GPIOKEY_POWER	IMX_GPIO_NR(6, 1)
	IOMUX_PAD_CTRL(CSI0_DAT15__GPIO6_IO01, WEAK_PULLDN),	/* pin 9 inverted, Main power off request */
	IOMUX_PAD_CTRL(NANDF_CS0__GPIO6_IO11, WEAK_PULLUP),	/* pin 10 - NC */
	IOMUX_PAD_CTRL(SD1_DAT1__GPIO1_IO17, WEAK_PULLUP),	/* pin 11 - NC */

#define GP_MAIN_POWER			IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(SD1_DAT0__GPIO1_IO16, OUTPUT_40OHM),

	/* i2c2 ov5640 mipi Camera controls */
#define GP_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(5, 21)
	IOMUX_PAD_CTRL(CSI0_VSYNC__GPIO5_IO21, OUTPUT_40OHM),
#define GP_OV5640_MIPI_RESET		IMX_GPIO_NR(5, 20)
	IOMUX_PAD_CTRL(CSI0_DATA_EN__GPIO5_IO20, OUTPUT_40OHM),

#define GP_LED			IMX_GPIO_NR(3, 20)
	IOMUX_PAD_CTRL(EIM_D20__GPIO3_IO20, OUTPUT_40OHM),
	/* BAT status */
	IOMUX_PAD_CTRL(NANDF_D2__GPIO2_IO02, WEAK_PULLUP),

	/* Misc outputs */
#define GP_L1			IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO_3__GPIO1_IO03, OUTPUT_40OHM),

#define GP_L2			IMX_GPIO_NR(3, 29)
	IOMUX_PAD_CTRL(EIM_D29__GPIO3_IO29, OUTPUT_40OHM),

	/* PWM1 - Backlight on RGB connector: J15 */
#define GP_BACKLIGHT_RGB	IMX_GPIO_NR(1, 21)
	IOMUX_PAD_CTRL(SD1_DAT3__GPIO1_IO21, WEAK_PULLDN),

	/* reg 3.3v */
#define GP_REG_3P3V		IMX_GPIO_NR(2, 26)
	IOMUX_PAD_CTRL(EIM_RW__GPIO2_IO26, OUTPUT_40OHM),

	/* reg 5v */
#define GP_REG_5V		IMX_GPIO_NR(6, 9)
	IOMUX_PAD_CTRL(NANDF_WP_B__GPIO6_IO09, OUTPUT_40OHM),

	/* reg 5.4v */
#define GP_REG_5P4V		IMX_GPIO_NR(2, 1)
	IOMUX_PAD_CTRL(NANDF_D1__GPIO2_IO01, OUTPUT_40OHM),

	/* reg_usbotg_vbus */
#define GP_REG_USBOTG		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLDN),

	/* reg_wlan_en */
#define GP_REG_WLAN_EN		IMX_GPIO_NR(6, 15)
	IOMUX_PAD_CTRL(NANDF_CS2__GPIO6_IO15, WEAK_PULLDN),

	/* UART1 */
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),

	/* UART2 */
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),

	/* USBH1 */
#define GP_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
	IOMUX_PAD_CTRL(GPIO_17__GPIO7_IO12, WEAK_PULLDN),

	/* USDHC2 - TiWi wl1271 */
	IOMUX_PAD_CTRL(SD2_CLK__SD2_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__SD2_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT0__SD2_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT1__SD2_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT2__SD2_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT3__SD2_DATA3, USDHC_PAD_CTRL),

	/* USDHC3 - eMMC */
	IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT4__SD3_DATA4, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT5__SD3_DATA5, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT6__SD3_DATA6, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT7__SD3_DATA7, USDHC_PAD_CTRL),

	/* USDHC4 - sdcard */
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL),
#define GP_USDHC4_CD		IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, NO_PAD_CTRL), /* CD */

	/* wl1271 */
#define GPIRQ_WL1271_WL		IMX_GPIO_NR(6, 14)
	IOMUX_PAD_CTRL(NANDF_CS1__GPIO6_IO14, WEAK_PULLDN),
};

#ifdef CONFIG_CMD_FBPANEL
static iomux_v3_cfg_t const rgb_pads[] = {
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
#endif

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
	{.esdhc_base = USDHC4_BASE_ADDR, .max_bus_width = 4},
	{.esdhc_base = USDHC3_BASE_ADDR, .max_bus_width = 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

	if (cfg->esdhc_base == USDHC3_BASE_ADDR)
		return 1;	/* eMMC is always present */
	return !gpio_get_value(GP_USDHC4_CD);
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);

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
#ifdef CONFIG_MXC_SPI_DISPLAY
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
	unsigned cs_gpio = GP_ECSPI2_CS;
	struct spi_slave *spi;
	int ret;

	printf("%s\n", __func__);
	gpio_direction_output(GP_BACKLIGHT_RGB, 1);
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
	unsigned cs_gpio = GP_ECSPI2_CS;
	unsigned reset_gpio = GP_SPI_DISPLAY_RESET;	/* nandf_d5 */
	struct spi_slave *spi;
	u8 data[8];

	printf("%s\n", __func__);
	gpio_direction_output(cs_gpio, 1);
	gpio_direction_output(reset_gpio, 1);
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

void board_enable_lcd(const struct display_info_t *di, int enable)
{
	if (enable) {
		SETUP_IOMUX_PADS(rgb_pads);
#ifdef CONFIG_MXC_SPI_DISPLAY
		if (di->fbflags & FBF_SPI)
			return enable_spi_rgb(di);
#endif
	} else {
		SETUP_IOMUX_PADS(rgb_gpio_pads);
	}
	gpio_direction_output(GP_BACKLIGHT_RGB, enable);
}

static const struct display_info_t displays[] = {
#ifdef CONFIG_MXC_SPI_DISPLAY
	VDF_LB043(LCD, "LB043", RGB24, FBF_MODESTR | FBF_SPI, detect_spi, 1, 0x70),
#endif
	/* hdmi */
	VD_1280_720M_60(HDMI, fbp_detect_i2c, 1, 0x50),
	VD_1920_1080M_60(HDMI, NULL, 1, 0x50),
	VD_1024_768M_60(HDMI, NULL, 1, 0x50),

	/* tsc2004 */
	VD_CLAA_WVGA(LCD, fbp_detect_i2c, 2, 0x48),
	VD_QVGA(LCD, NULL, 2, 0x48),
};

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}
#endif

static const unsigned short gpios_out_low[] = {
	/* Disable wl1271 */
	GP_REG_WLAN_EN,
	GP_BT_RFKILL_RESET,
	GP_REG_USBOTG,
	GP_OV5640_MIPI_RESET,
	GP_LED,
	GP_REG_5V,
	GP_REG_3P3V,
	GP_REG_5P4V,
	GP_L1,
};

static const unsigned short gpios_out_high[] = {
	GP_MAIN_POWER,
	GP_ECSPI1_NOR_CS,
	GP_OV5640_MIPI_POWER_DOWN,
	GP_L2,
};

static const unsigned short gpios_in[] = {
	GP_BACKLIGHT_RGB,
	GPIRQ_WL1271_WL,
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
	int i;
	struct i2c_pads_info *p = i2c_pads + i2c_get_info_entry_offset();

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	for (i = 0; i < 3; i++) {
	        setup_i2c(i, CONFIG_SYS_I2C_SPEED, 0x7f, p);
		p += I2C_PADS_INFO_ENTRY_SPACING;
	}
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
	puts("Board: " CONFIG_BOARD_NAME "\n");
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
	unsigned char	active_low;
};

static struct button_key const buttons[] = {
	{"back",	GP_GPIOKEY_BACK,	'B', 1},
	{"up",		GP_GPIOKEY_UP,		'U', 1},
	{"menu",	GP_GPIOKEY_MENU,	'M', 1},
	{"left",	GP_GPIOKEY_LEFT,	'L', 1},
	{"right",	GP_GPIOKEY_RIGHT,	'R', 1},
	{"down",	GP_GPIOKEY_DOWN,	'D', 1},
	{"power",	GP_GPIOKEY_POWER,	'P', 0},
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

	setenv("cpu", get_imx_type((cpurev & 0xFF000) >> 12));
	if (!getenv("board"))
		setenv("board", CONFIG_BOARD_NAME);
	setenv("uboot_defconfig", CONFIG_DEFCONFIG);
	return 0;
}

void board_poweroff(void)
{
	/* Turn off main power */
	gpio_direction_output(GP_MAIN_POWER, 0);
	udelay(1000000);
}

static int _do_poweroff(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	board_poweroff();
	return 0;
}

U_BOOT_CMD(poweroff, 1, 1, _do_poweroff, "Turn off power", "");
