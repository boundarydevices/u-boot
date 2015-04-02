/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/spi.h>
#include <asm/imx-common/video.h>
#include <spi.h>
#include "spi_display.h"

#define DI0_PAD_CTRL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#if defined(CONFIG_MX6QDL)
#define IOMUX_PAD_CTRL(name, ctrl)	NEW_PAD_CTRL(MX6Q_PAD_##name, ctrl), \
					NEW_PAD_CTRL(MX6DL_PAD_##name, ctrl)
#else
#define IOMUX_PAD_CTRL(name, ctrl)	NEW_PAD_CTRL(MX6_PAD_##name, ctrl)
#endif

#if 0
/* RH320240T-3X5WP-A2, Display Mode Setting */
#define SPI_FREQ	1000000
#define SPI_MOSI_PAD_CTRL  (SPI_PAD_CTRL | PAD_CTL_PUS_22K_UP | PAD_CTL_ODE)
#else
/* RH320240T-3X5AP-A, Display Mode Setting */
#define SPI_FREQ	1000000
#define SPI_MOSI_PAD_CTRL  (SPI_PAD_CTRL | PAD_CTL_PUS_22K_UP)
#endif


#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLDOWN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

static iomux_v3_cfg_t const spi_display_pads[] = {
	/* ECSPI2 */
	IOMUX_PAD_CTRL(CSI0_DAT8__ECSPI2_SCLK, SPI_PAD_CTRL),	    /* P5:pin 10, P6:pin 49 */
	IOMUX_PAD_CTRL(CSI0_DAT9__ECSPI2_MOSI, SPI_MOSI_PAD_CTRL),  /* P5:pin 12, P6:pin 50 */
	IOMUX_PAD_CTRL(CSI0_DAT10__ECSPI2_MISO, SPI_MOSI_PAD_CTRL), /* P5:pin 11, P6:pin 50 */
#define GP_ECSPI2_CS		IMX_GPIO_NR(5, 29)
	IOMUX_PAD_CTRL(CSI0_DAT11__GPIO5_IO29, SPI_PAD_CTRL),	    /* P5:pin  9, P6:pin 43 */
#define GP_LCD_DISABLE		IMX_GPIO_NR(2, 4)
	IOMUX_PAD_CTRL(NANDF_D4__GPIO2_IO04, WEAK_PULLUP),	    /* P5:pin 42, P6:pin 48 (SHUT) */
#define GP_TOUCH_IRQ	IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, WEAK_PULLUP),

	/* DI0 */
	IOMUX_PAD_CTRL(DI0_DISP_CLK__IPU1_DI0_DISP_CLK, DI0_PAD_CTRL),
//	IOMUX_PAD_CTRL(DI0_PIN15__IPU1_DI0_PIN15, DI0_PAD_CTRL),	/* DRDY */
	IOMUX_PAD_CTRL(DI0_PIN2__IPU1_DI0_PIN02, DI0_PAD_CTRL),		/* HSYNC */
	IOMUX_PAD_CTRL(DI0_PIN3__IPU1_DI0_PIN03, DI0_PAD_CTRL),		/* VSYNC */
	IOMUX_PAD_CTRL(DISP0_DAT0__IPU1_DISP0_DATA00, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT1__IPU1_DISP0_DATA01, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT2__IPU1_DISP0_DATA02, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT3__IPU1_DISP0_DATA03, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT4__IPU1_DISP0_DATA04, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT5__IPU1_DISP0_DATA05, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT6__IPU1_DISP0_DATA06, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT7__IPU1_DISP0_DATA07, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT8__IPU1_DISP0_DATA08, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT9__IPU1_DISP0_DATA09, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT10__IPU1_DISP0_DATA10, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT11__IPU1_DISP0_DATA11, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT12__IPU1_DISP0_DATA12, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT13__IPU1_DISP0_DATA13, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT14__IPU1_DISP0_DATA14, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT15__IPU1_DISP0_DATA15, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT16__IPU1_DISP0_DATA16, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT17__IPU1_DISP0_DATA17, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT18__IPU1_DISP0_DATA18, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT19__IPU1_DISP0_DATA19, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT20__IPU1_DISP0_DATA20, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT21__IPU1_DISP0_DATA21, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT22__IPU1_DISP0_DATA22, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT23__IPU1_DISP0_DATA23, DI0_PAD_CTRL),
};

static int spi_display_cmds(struct spi_slave *spi, u8 *cmds)
{
	u8 buf[4];
	int ret = 0;

	debug("%s\n", __func__);
	while (1) {
		uint reg = (cmds[0] << 8) | cmds[1];
		uint len = cmds[2];
		uint val;

		if (!len && !reg)
			break;
		cmds += 3;
		do {
			buf[0] = 0x70;
			buf[1] = reg >> 8;
			buf[2] = reg;
			ret = spi_xfer(spi, 3 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				debug("%s: Failed to select reg 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			udelay(2);
			if (!len) {
				debug("spi: reg:%04x\n", reg);
				break;
			}
			val = (cmds[0] << 8) | cmds[1];
			cmds += 2;
			buf[0] = 0x72;
			buf[1] = val >> 8;
			buf[2] = val;
			ret = spi_xfer(spi, 3 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				debug("%s: Failed to write val 0x%x=0x%x, %d\n", __func__, reg, val, ret);
				return ret;
			}
			debug("spi: reg:%04x=%04x\n", reg, val);
			udelay(2);
			reg++;
		} while (--len);
	}
	return ret;
}

int spi_read_register(struct spi_slave *spi, int reg)
{
	int val;
	int ret;
	u8 buf[4];
	u8 rx_buf[4];

	buf[0] = 0x70;
	buf[1] = reg >> 8;
	buf[2] = reg;
	ret = spi_xfer(spi, 3 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret) {
		debug("%s: Failed to select reg 0x%x, %d\n", __func__, reg, ret);
		return ret;
	}
	udelay(2);
	buf[0] = 0x73;
	buf[1] = 0xff;
	buf[2] = 0xff;
	rx_buf[0] = 0xff;
	rx_buf[1] = 0xff;
	rx_buf[2] = 0xff;
	ret = spi_xfer(spi, 3 * 8, buf, rx_buf, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret) {
		debug("%s: Failed to read val from reg 0x%x %d\n", __func__, reg, ret);
		return ret;
	}
	val = (rx_buf[1] << 8) | rx_buf[2];
	debug("spi: (0x%02x)reg:%04x=%04x\n", rx_buf[0], reg, val);
	return val;
}

#define A(reg, cnt) (reg >> 8), (reg & 0xff), cnt
#define VAL(val) (val >> 8), (val & 0xff)

static u8 display_init_cmds[] = {
/* RH320240T-3X5WP-A2, Display Mode Setting */
#if 1
	A(0x0001, 5), VAL(0x7300), VAL(0x0200), VAL(0x6164), VAL(0x04c7), VAL(0xfc80),
	A(0x000a, 1), VAL(0x4008),
	A(0x000d, 2), VAL(0x3229), VAL(0x3200),
#else
//defaults
//	A(0x0001, 5), VAL(0x7300), VAL(0x0200), VAL(0x6464), VAL(0x0447), VAL(0xb4c4),
//	A(0x000a, 1), VAL(0x4008),
//	A(0x000d, 2), VAL(0x3229), VAL(0x1200),
#endif
	A(0, 0)
};

void enable_spi_rgb(struct display_info_t const *dev)
{
	unsigned cs_gpio = GP_ECSPI2_CS;
	struct spi_slave *spi;
	int ret;

	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, dev->bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(dev->bus, cs_gpio << 8, SPI_FREQ, SPI_MODE_3);
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
	ret = spi_display_cmds(spi, display_init_cmds);
	if (ret) {
		printf("%s: Failed to display_init_cmds %d\n", __func__, ret);
		goto release_bus;
	}

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
int detect_spi(struct display_info_t const *dev)
{
	unsigned cs_gpio = GP_ECSPI2_CS;
	unsigned disable_gpio = GP_LCD_DISABLE;

	debug("%s\n", __func__);
	gpio_direction_output(cs_gpio, 1);
	gpio_direction_output(disable_gpio, 1);
	SETUP_IOMUX_PADS(spi_display_pads);
	udelay(200);
	gpio_direction_output(disable_gpio, 0);
	mdelay(200);
	return 1;
}

static int do_spid(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned cs_gpio = GP_ECSPI2_CS;
	struct spi_slave *spi;
	int ret = 0;
	int arg = 2;
	int bus = 1;
	int index = 3;
	uint reg;
	u8 buf[80];

	if (argc < 2)
		return 1;
	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(bus, cs_gpio << 8, SPI_FREQ, SPI_MODE_3);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return 1;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		goto exit1;
	}

	reg = simple_strtoul(argv[1], NULL, 16);
	buf[0] = reg >> 8;
	buf[1] = reg;
	buf[2] = argc - arg;
	while (arg < argc) {
		int val = simple_strtoul(argv[arg], NULL, 16);;
		buf[index++] = val >> 8;
		buf[index++] = val;
		if (index >= ARRAY_SIZE(buf) - 4)
			break;
		arg++;
	}
	buf[index++] = 0;
	buf[index++] = 0;
	buf[index++] = 0;
	spi_display_cmds(spi, buf);
	spi_release_bus(spi);
exit1:
	spi_free_slave(spi);
	enable_spi_clk(0, bus);
	return ret ? 1 : 0;
}

U_BOOT_CMD(
	spid, 70, 0, do_spid,
	"write cmd, data to spi display",
	"reg16 [word]"
);

static int do_spidr(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned cs_gpio = GP_ECSPI2_CS;
	struct spi_slave *spi;
	int ret = 0;
	int bus = 1;
	uint reg;
	int val;

	if (argc != 2)
		return 1;

	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(bus, cs_gpio << 8, SPI_FREQ, SPI_MODE_3);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return 1;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		goto exit1;
	}

	reg = simple_strtoul(argv[1], NULL, 16);
	val = spi_read_register(spi, reg);
	printf("reg %04x=0x%04x\n", reg, val);
	spi_release_bus(spi);
exit1:
	spi_free_slave(spi);
	enable_spi_clk(0, bus);
	return ret ? 1 : 0;
}

U_BOOT_CMD(
	spidr, 70, 0, do_spidr,
	"read data from spi display",
	"reg16"
);
