/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <command.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/spi.h>
#include <asm/mach-imx/video.h>
#include <linux/delay.h>
#include <spi.h>
#include "spi_display.h"

#define GP_ECSPI2_CS		IMX_GPIO_NR(5, 29)
#define SPI_FREQ	1000000

static int spi_display_cmds(struct spi_slave *spi, u8 *cmds)
{
	u8 buf[4];
	int ret = 0;

	pr_debug("%s\n", __func__);
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
				pr_debug("%s: Failed to select reg 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			udelay(2);
			if (!len) {
				pr_debug("spi: reg:%04x\n", reg);
				break;
			}
			val = (cmds[0] << 8) | cmds[1];
			cmds += 2;
			buf[0] = 0x72;
			buf[1] = val >> 8;
			buf[2] = val;
			ret = spi_xfer(spi, 3 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				pr_debug("%s: Failed to write val 0x%x=0x%x, %d\n", __func__, reg, val, ret);
				return ret;
			}
			pr_debug("spi: reg:%04x=%04x\n", reg, val);
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
		pr_debug("%s: Failed to select reg 0x%x, %d\n", __func__, reg, ret);
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
		pr_debug("%s: Failed to read val from reg 0x%x %d\n", __func__, reg, ret);
		return ret;
	}
	val = (rx_buf[1] << 8) | rx_buf[2];
	pr_debug("spi: (0x%02x)reg:%04x=%04x\n", rx_buf[0], reg, val);
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
		pr_debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
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

static int do_spid(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
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
		pr_debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
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

static int do_spidr(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
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
		pr_debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
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
