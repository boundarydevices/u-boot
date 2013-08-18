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

#define RGB_PAD_CTRL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define SPI_MOSI_R_PAD_CTRL	SPI_PAD_CTRL | PAD_CTL_ODE | PAD_CTL_PUS_22K_UP

#define GP_SPI_DISPLAY_RESET	IMX_GPIO_NR(4, 20)

#define WEAK_PULLDN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLDN_OUTPUT (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

struct spi_display_info {
	int mode;
	int speed_r;
	int speed_w;
	int reset_active_low;
	int cs_gpio;
	int reset_gpio;
	u8 *init_cmds;
	u8 *on_cmds;
	const iomux_v3_cfg_t *spi_mosi_r_pads;
	const iomux_v3_cfg_t *spi_mosi_w_pads;
	const iomux_v3_cfg_t *spi_ss0_pad;
	const iomux_v3_cfg_t *spi_ss0_gpio_pad;
	int (*spi_write_rtn)(struct spi_slave *spi, u8 *cmds);
	int (*spi_read_rtn)(struct spi_slave *spi, int reg);
	int (*spi_setup_rtn)(struct spi_display_info *di);
	int (*spi_exit_rtn)(struct spi_display_info *di);
};

static iomux_v3_cfg_t const spi_mosi_r_pads[] = {
	IOMUX_PAD_CTRL(CSI0_DAT9__ECSPI2_MOSI, SPI_MOSI_R_PAD_CTRL),
};

static iomux_v3_cfg_t const spi_mosi_w_pads[] = {
	IOMUX_PAD_CTRL(CSI0_DAT9__ECSPI2_MOSI, SPI_PAD_CTRL),
};

static iomux_v3_cfg_t const spi_ss0_pad[] = {
	IOMUX_PAD_CTRL(CSI0_DAT11__ECSPI2_SS0, SPI_PAD_CTRL),
};

static iomux_v3_cfg_t const spi_ss0_gpio_pad[] = {
#define GP_ECSPI2_CS		IMX_GPIO_NR(5, 29)
	IOMUX_PAD_CTRL(CSI0_DAT11__GPIO5_IO29, SPI_PAD_CTRL),
};

static int AUO_G050_spi_write_rtn(struct spi_slave *spi, u8 *cmds)
{
	u8 buf[4];
	int ret = 0;

	pr_debug("%s\n", __func__);
	while (1) {
		uint reg = (cmds[0] << 8) | cmds[1];
		uint len = cmds[2];

		if (!len && !reg)
			break;
		cmds += 3;
		do {
			buf[0] = 0x20;
			buf[1] = reg >> 8;
			ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				pr_debug("%s: Failed to select reg1 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			udelay(2);
			buf[0] = 0;
			buf[1] = reg;
			ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				pr_debug("%s: Failed to select reg2 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			udelay(2);
			if (!len) {
				pr_debug("spi: reg:%04x\n", reg);
				break;
			}
			buf[0] = 0x40;
			buf[1] = *cmds++;
			ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				pr_debug("%s: Failed to select reg3 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			pr_debug("spi: reg:%04x %02x\n", reg, buf[1]);
			udelay(2);
			reg++;
		} while (--len);
	}
	return ret;
}

static int AUO_G050_spi_read_rtn(struct spi_slave *spi, int reg)
{
	u8 buf[4];
	u8 rbuf[4];
	int ret = 0;

	buf[0] = 0x20;
	buf[1] = reg >> 8;
	ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret) {
		pr_debug("%s: Failed to select reg1 0x%x, %d\n", __func__, reg, ret);
		return ret;
	}
	udelay(2);
	buf[0] = 0;
	buf[1] = reg;
	ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret) {
		pr_debug("%s: Failed to select reg2 0x%x, %d\n", __func__, reg, ret);
		return ret;
	}
	udelay(2);
	buf[0] = 0xC0;
	buf[1] = 0xff;
	ret = spi_xfer(spi, 2 * 8, buf, rbuf, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret) {
		pr_debug("%s: Failed to select reg3 0x%x, %d\n", __func__, reg, ret);
		return ret;
	}
	pr_debug("spi: reg:0x%04x: %02x %02x\n", reg, rbuf[0], rbuf[1]);
	udelay(2);
	return rbuf[1];
}

#define A(reg, cnt) (reg >> 8), (reg & 0xff), cnt

static u8 AUO_G050_display_init_cmds[] = {
/* Display Mode Setting */
	A(0xf000, 5), 0x55, 0xaa, 0x52, 0x08, 0x00,
	A(0xb100, 2), 0x0c, 0x00,
	A(0xbc00, 3), 0x05, 0x05, 0x05,
	A(0xb700, 2), 0x22, 0x22,
	A(0xb800, 4), 0x01, 0x03, 0x03, 0x03,
	A(0xc803, 1), 0x96,
	A(0xc805, 1), 0x96,
	A(0xc807, 1), 0x96,
	A(0xc809, 1), 0x96,
	A(0xc80b, 1), 0x2a,
	A(0xc80c, 1), 0x2a,
	A(0xc80f, 1), 0x2a,
	A(0xc810, 1), 0x2a,
	A(0xf000, 5), 0x55, 0xaa, 0x52, 0x08, 0x01,
	A(0xb900, 3), 0x34, 0x34, 0x34,
	A(0xba00, 3), 0x14, 0x14, 0x14,
	A(0xbe00, 2), 0x00, 0x8c,
	A(0xb000, 3), 0x00, 0x00, 0x00,
	A(0xb800, 3), 0x24, 0x24, 0x24,
	A(0xbc00, 3), 0x00, 0x88, 0x01,
	A(0xbd00, 3), 0x00, 0x88, 0x01,
	A(0xd100, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0xd200, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0xd300, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0xd400, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0xd500, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0xd600, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0x1100, 0),	/* exit sleep mode, wait 120 ms */
	A(0, 0)
};

static u8 AUO_G050_display_on_cmds[] = {
	A(0x2900, 0),
	A(0, 0)
};

/* *************************************************** */

static int A030JN01_spi_write_rtn(struct spi_slave *spi, u8 *cmds)
{
	u8 buf[4];
	int ret = 0;

	while (1) {
		uint reg = (cmds[0] << 8) | cmds[1];
		uint len = cmds[2];

		if (!len && !reg)
			break;
		cmds += 3;
		do {
			buf[0] = reg + (reg & 0x40);
			buf[1] = *cmds++;
			ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				pr_debug("%s: Failed 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			pr_debug("spi: reg:%02x %02x\n", reg, buf[1]);
			udelay(2);
			reg++;
		} while (--len);
	}
	return ret;
}

static int A030JN01_spi_read_rtn(struct spi_slave *spi, int reg)
{
	u8 buf[4];
	u8 rbuf[4];
	int ret = 0;

	buf[0] = (reg + (reg & 0x40)) | 0x40;
	buf[1] = 0xff;
	ret = spi_xfer(spi, 2 * 8, buf, rbuf, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret) {
		pr_debug("%s: Failed 0x%x, %d\n", __func__, reg, ret);
		return ret;
	}
	pr_debug("spi: reg:0x%02x: %02x %02x\n", reg, rbuf[0], rbuf[1]);
	return rbuf[1];
}

static u8 A030JN01_display_YUV720_init_cmds[] = {
/* Display Mode Setting */
	A(4, 2), 0x6b, 0x5f,
	A(0, 0)
};

static u8 A030JN01_display_UPS051_init_cmds[] = {
	A(5, 1), 0x5f,
	A(0, 0)
};

static u8 A030JN01_display_on_cmds[] = {
	A(0, 0)
};

/* *************************************************** */

static iomux_v3_cfg_t const KD024FM_spi_pads[] = {
#define GP_KD024FM_CS		IMX_GPIO_NR(5, 13)
	IOMUX_PAD_CTRL(DISP0_DAT19__GPIO5_IO13, WEAK_PULLUP),	/* Pin 8 CS */
#define GP_KD024FM_IM2		IMX_GPIO_NR(5, 17)
	IOMUX_PAD_CTRL(DISP0_DAT23__GPIO5_IO17, WEAK_PULLUP),
#define GP_KD024FM_IM1		IMX_GPIO_NR(5, 16)
	IOMUX_PAD_CTRL(DISP0_DAT22__GPIO5_IO16, WEAK_PULLUP),
#define GP_KD024FM_IM0		IMX_GPIO_NR(5, 15)
	IOMUX_PAD_CTRL(DISP0_DAT21__GPIO5_IO15, WEAK_PULLDN),
#define GP_KD024FM_RESET	IMX_GPIO_NR(5, 14)
	IOMUX_PAD_CTRL(DISP0_DAT20__GPIO5_IO14, WEAK_PULLDN),
#define GP_KD024FM_RS		IMX_GPIO_NR(4, 25)
	IOMUX_PAD_CTRL(DISP0_DAT4__GPIO4_IO25, WEAK_PULLUP),	/* Pin 10 RS ** */
	IOMUX_PAD_CTRL(DISP0_DAT0__ECSPI3_SCLK, WEAK_PULLUP),	/* Pin 9 SCL */
	IOMUX_PAD_CTRL(DISP0_DAT1__ECSPI3_MOSI, WEAK_PULLUP),	/* Pin 16 SDA1 */
	IOMUX_PAD_CTRL(DISP0_DAT2__ECSPI3_MISO, WEAK_PULLUP),	/* Pin 35 SDO */
#define GP_KD024FM_PCLK		IMX_GPIO_NR(4, 16)
	IOMUX_PAD_CTRL(DI0_DISP_CLK__GPIO4_IO16, WEAK_PULLUP),
#define GP_KD024FM_HSYNC	IMX_GPIO_NR(4, 18)
	IOMUX_PAD_CTRL(DI0_PIN2__GPIO4_IO18, WEAK_PULLUP),	/* HSYNC */
#define GP_KD024FM_VSYNC	IMX_GPIO_NR(4, 19)
	IOMUX_PAD_CTRL(DI0_PIN3__GPIO4_IO19, WEAK_PULLUP),	/* VSYNC */
#define GP_KD024FM_DRDY		IMX_GPIO_NR(4, 17)
	IOMUX_PAD_CTRL(DI0_PIN15__GPIO4_IO17, WEAK_PULLUP),	/* DRDY */
};

static iomux_v3_cfg_t const KD024FM_display_pads[] = {
	IOMUX_PAD_CTRL(DISP0_DAT0__IPU1_DISP0_DATA00, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT1__IPU1_DISP0_DATA01, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT2__IPU1_DISP0_DATA02, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT3__IPU1_DISP0_DATA03, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT4__IPU1_DISP0_DATA04, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_DISP_CLK__IPU1_DI0_DISP_CLK, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN15__IPU1_DI0_PIN15, RGB_PAD_CTRL),	/* DRDY */
	IOMUX_PAD_CTRL(DI0_PIN2__IPU1_DI0_PIN02, RGB_PAD_CTRL),		/* HSYNC */
	IOMUX_PAD_CTRL(DI0_PIN3__IPU1_DI0_PIN03, RGB_PAD_CTRL),		/* VSYNC */
};

static int KD024FM_spi_write_rtn(struct spi_slave *spi, u8 *cmds)
{
	u8 buf[4];
	int ret = 0;

	while (1) {
		uint reg = cmds[1];
		uint len = cmds[2];

		if (!len && !reg)
			break;
		cmds += 3;
		gpio_direction_output(GP_KD024FM_RS, 0);
		buf[0] = reg;
		ret = spi_xfer(spi, 1 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
		if (ret) {
			printf("%s: Failed 0x%x, %d\n", __func__, reg, ret);
			return ret;
		}
		if (!len) {
			pr_debug("spi: reg:%02x\n", reg);
			continue;
		}

		gpio_set_value(GP_KD024FM_RS, 1);
		while (len--) {
			buf[0] = *cmds++;
			ret = spi_xfer(spi, 1 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				printf("%s: Failed 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			pr_debug("spi: reg:%02x %02x\n", reg, buf[0]);
			udelay(2);
		}
	}
	return ret;
}

static int KD024FM_spi_read_rtn(struct spi_slave *spi, int reg)
{
	printf("%s: undefined\n", __func__);
	return 0;
}

static int KD024FM_spi_setup_rtn(struct spi_display_info *di)
{
	gpio_direction_output(di->cs_gpio, 1);
	gpio_direction_output(GP_KD024FM_IM2, 1);
	gpio_direction_output(GP_KD024FM_IM1, 1);
	gpio_direction_output(GP_KD024FM_IM0, 0);
	gpio_direction_output(di->reset_gpio, 1);
	gpio_direction_output(GP_KD024FM_RS, 0);
	gpio_direction_output(GP_KD024FM_HSYNC, 1);
	gpio_direction_output(GP_KD024FM_VSYNC, 1);
	gpio_direction_output(GP_KD024FM_DRDY, 1);
	gpio_direction_output(GP_KD024FM_PCLK, 1);
	SETUP_IOMUX_PADS(KD024FM_spi_pads);
	return 0;
}

static int KD024FM_spi_exit_rtn(struct spi_display_info *di)
{
	SETUP_IOMUX_PADS(KD024FM_display_pads);
	return 0;
}


static u8 KD024FM_display_init_cmds[] = {
	A(0x3a, 1), 0x05,	/* ST7789S Frame rate setting */
	A(0x36, 1), 0x00,
	A(0xb2, 5), 0x00, 0x00, 0x00, 0x33, 0x33,
	A(0xb7, 1), 0x35,
	A(0xb8, 3), 0x2f, 0x2b, 0x2f,	/* ST7789S Power setting */
	A(0xbb, 1), 0x24,
	A(0xc0, 1), 0x2c,
	A(0xc3, 1), 0x10,
	A(0xc4, 1), 0x20,
	A(0xc6, 1), 0x11,
	A(0xd0, 2), 0xa4, 0xa1,
	A(0xe8, 1), 0x03,
	A(0xe9, 3), 0x0d, 0x12, 0x00,
	/* ST7789S gamma setting */
	A(0xe0, 14), 0xd0, 0x00, 0x00, 0x08, 0x11, 0x1a, 0x2b, 0x33, 0x42, 0x26, 0x12, 0x21, 0x2f, 0x11,
	A(0xe1, 14), 0xd0, 0x02, 0x09, 0x0d, 0x0d, 0x27, 0x2b, 0x33, 0x42, 0x17, 0x12, 0x11, 0x2f, 0x31,
	A(0x21, 0),
	/* SET RGB Interface */
	A(0xb0, 3), 0x11, 0x00, 0x00,
	/* set DE mode ; SET Hs,Vs,DE,DOTCLK signal polarity */
	A(0xb1, 3), 0xc0, 0x08, 0x14,
	A(0x3a, 1), 0x88,		/* 18 Bit RGB, 0x55 means 16 Bit RGB */
	A(0x11, 0),
	A(0, 0)
};

static u8 KD024FM_display_on_cmds[] = {
	A(0x29, 0),
	A(0x2c, 0),
	A(0, 0)
};

struct spi_display_info spi_di[] = {
	{ .mode = SPI_MODE_0, .speed_r = 10000, .speed_w = 1000000, .reset_active_low = 1,
			.cs_gpio = GP_ECSPI2_CS, .reset_gpio = GP_SPI_DISPLAY_RESET,
			.init_cmds = AUO_G050_display_init_cmds, .on_cmds = AUO_G050_display_on_cmds,
			.spi_mosi_r_pads = spi_mosi_r_pads,
			.spi_mosi_w_pads = spi_mosi_w_pads,
			.spi_ss0_pad = spi_ss0_pad,
			.spi_ss0_gpio_pad = spi_ss0_gpio_pad,
			.spi_write_rtn = AUO_G050_spi_write_rtn, .spi_read_rtn = AUO_G050_spi_read_rtn},
	{ .mode = SPI_MODE_3, .speed_r = 10000, .speed_w = 10000, .reset_active_low = 1,
			.cs_gpio = GP_ECSPI2_CS, .reset_gpio = GP_SPI_DISPLAY_RESET,
			.init_cmds = A030JN01_display_YUV720_init_cmds, .on_cmds = A030JN01_display_on_cmds,
			.spi_mosi_r_pads = spi_mosi_r_pads,
			.spi_mosi_w_pads = spi_mosi_w_pads,
			.spi_ss0_pad = spi_ss0_pad,
			.spi_ss0_gpio_pad = spi_ss0_gpio_pad,
			.spi_write_rtn = A030JN01_spi_write_rtn, .spi_read_rtn = A030JN01_spi_read_rtn},
	{ .mode = SPI_MODE_3, .speed_r = 10000, .speed_w = 10000, .reset_active_low = 1,
			.cs_gpio = GP_ECSPI2_CS, .reset_gpio = GP_SPI_DISPLAY_RESET,
			.init_cmds = A030JN01_display_UPS051_init_cmds, .on_cmds = A030JN01_display_on_cmds,
			.spi_mosi_r_pads = spi_mosi_r_pads,
			.spi_mosi_w_pads = spi_mosi_w_pads,
			.spi_ss0_pad = spi_ss0_pad,
			.spi_ss0_gpio_pad = spi_ss0_gpio_pad,
			.spi_write_rtn = A030JN01_spi_write_rtn, .spi_read_rtn = A030JN01_spi_read_rtn},
	{ .mode = SPI_MODE_0, .speed_r = 10000, .speed_w = 10000, .reset_active_low = 1,
			.cs_gpio = GP_KD024FM_CS, .reset_gpio = GP_KD024FM_RESET,
			.init_cmds = KD024FM_display_init_cmds, .on_cmds = KD024FM_display_on_cmds,
			.spi_write_rtn = KD024FM_spi_write_rtn, .spi_read_rtn = KD024FM_spi_read_rtn,
			.spi_setup_rtn = KD024FM_spi_setup_rtn, .spi_exit_rtn = KD024FM_spi_exit_rtn},
};

const struct display_info_t *g_dev;

/*
 * Return 1 for successful detection of display
 */
int detect_spi(struct display_info_t const *dev)
{
	return 1;
}

static void init_spi(struct display_info_t const *dev)
{
	struct spi_display_info *di = &spi_di[dev->addr];
	unsigned reset_gpio = di->reset_gpio;
	int reset_val = di->reset_active_low ? 0 : 1;

	pr_debug("%s\n", __func__);
	gpio_direction_output(di->cs_gpio, 1);
	gpio_direction_output(reset_gpio, reset_val ^ 1);
	gpio_direction_output(reset_gpio, reset_val);
	udelay(200);
	gpio_direction_output(reset_gpio, reset_val ^ 1);
	mdelay(200);
}

void enable_spi_rgb(struct display_info_t const *dev)
{
	struct spi_display_info *di = &spi_di[dev->addr];
	struct spi_slave *spi;
	int ret;

	g_dev = dev;
	if (di->spi_setup_rtn)
		di->spi_setup_rtn(di);
	init_spi(dev);
	gpio_direction_output(di->cs_gpio, 1);
	if (di->spi_mosi_w_pads)
		imx_iomux_v3_setup_multiple_pads(di->spi_mosi_w_pads, 1);

	enable_spi_clk(1, dev->bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(dev->bus, di->spi_ss0_pad ? 0 : di->cs_gpio << 8,
			di->speed_w, di->mode);
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
	if (di->spi_ss0_pad)
		imx_iomux_v3_setup_multiple_pads(di->spi_ss0_pad, 1);
	ret = di->spi_write_rtn(spi, di->init_cmds);
	if (ret) {
		printf("%s: Failed to display_init_cmds %d\n", __func__, ret);
		goto release_bus;
	}
	mdelay(200);
	ret = di->spi_write_rtn(spi, di->on_cmds);
	if (ret) {
		printf("%s: Failed to display_on_cmds %d\n", __func__, ret);
		goto release_bus;
	}
	ret = 1;
	if (di->spi_ss0_gpio_pad)
		imx_iomux_v3_setup_multiple_pads(di->spi_ss0_gpio_pad, 1);

	/* Release spi bus */
release_bus:
	spi_release_bus(spi);
free_bus:
	spi_free_slave(spi);
	enable_spi_clk(0, dev->bus);
	if (di->spi_exit_rtn)
		di->spi_exit_rtn(di);
	return;
}

static int do_spid(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	struct spi_slave *spi;
	int display_index = g_dev ? g_dev->addr : 0;
	int bus = g_dev ? g_dev->bus : 1;
	struct spi_display_info *di = &spi_di[display_index];
	int ret = 0;
	int arg = 2;
	uint reg;
	u8 buf[80];

	if (argc < 2)
		return 1;
	if (di->spi_setup_rtn)
		di->spi_setup_rtn(di);
	gpio_direction_output(di->cs_gpio, 1);
	if (di->spi_mosi_w_pads)
		imx_iomux_v3_setup_multiple_pads(di->spi_mosi_w_pads, 1);

	enable_spi_clk(1, bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(bus, di->spi_ss0_pad ? 0 : di->cs_gpio << 8,
			di->speed_w, di->mode);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return 1;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		pr_debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		goto free_bus;
	}

	if (argc > ARRAY_SIZE(buf) - 3)
		argc = ARRAY_SIZE(buf) - 3;

	reg = simple_strtoul(argv[1], NULL, 16);
	buf[0] = reg >> 8;
	buf[1] = reg;
	buf[2] = argc - arg;
	while (arg < argc) {
		buf[arg + 1] = simple_strtoul(argv[arg], NULL, 16);
		arg++;
	}
	arg++;
	buf[arg++] = 0;
	buf[arg++] = 0;
	buf[arg++] = 0;
	if (di->spi_ss0_pad)
		imx_iomux_v3_setup_multiple_pads(di->spi_ss0_pad, 1);
	di->spi_write_rtn(spi, buf);
	if (di->spi_ss0_gpio_pad)
		imx_iomux_v3_setup_multiple_pads(di->spi_ss0_gpio_pad, 1);
	spi_release_bus(spi);
free_bus:
	if (di->spi_exit_rtn)
		di->spi_exit_rtn(di);
	spi_free_slave(spi);
	enable_spi_clk(0, bus);
	return ret ? 1 : 0;
}

U_BOOT_CMD(
	spid, 70, 0, do_spid,
	"write cmd, data to spi display",
	"reg16 [byte]"
);

static int do_spidr(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	struct spi_slave *spi;
	int display_index = g_dev ? g_dev->addr : 0;
	int bus = g_dev ? g_dev->bus : 1;
	struct spi_display_info *di = &spi_di[display_index];
	int ret = 0;
	uint reg;
	int val;

	if (argc != 2)
		return CMD_RET_USAGE;
	if (di->spi_setup_rtn)
		di->spi_setup_rtn(di);
	gpio_direction_output(di->cs_gpio, 1);
	if (di->spi_mosi_r_pads)
		imx_iomux_v3_setup_multiple_pads(di->spi_mosi_r_pads, 1);

	enable_spi_clk(1, bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(bus, di->spi_ss0_pad ? 0 : di->cs_gpio << 8, di->speed_r, di->mode);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return 1;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		pr_debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		goto free_bus;
	}

	reg = simple_strtoul(argv[1], NULL, 16);
	if (di->spi_ss0_pad)
		imx_iomux_v3_setup_multiple_pads(di->spi_ss0_pad, 1);
	val = di->spi_read_rtn(spi, reg);
	if (di->spi_ss0_gpio_pad)
		imx_iomux_v3_setup_multiple_pads(di->spi_ss0_gpio_pad, 1);
	printf("spidr: reg:0x%x = 0x%x\n", reg, val);
	spi_release_bus(spi);
free_bus:
	if (di->spi_exit_rtn)
		di->spi_exit_rtn(di);
	spi_free_slave(spi);
	enable_spi_clk(0, bus);
	return ret ? 1 : 0;
}

U_BOOT_CMD(
	spidr, 70, 0, do_spidr,
	"read spi display register",
	"reg16"
);
