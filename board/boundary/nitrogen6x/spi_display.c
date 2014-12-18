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


static iomux_v3_cfg_t const spi_display_pads[] = {
	/* ECSPI2 */
	IOMUX_PAD_CTRL(CSI0_DAT8__ECSPI2_SCLK, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT9__ECSPI2_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT10__ECSPI2_MISO, SPI_PAD_CTRL),
#define GP_ECSPI2_CS		IMX_GPIO_NR(5, 29)
	IOMUX_PAD_CTRL(CSI0_DAT11__GPIO5_IO29, SPI_PAD_CTRL),
#define GP_SPI_DISPLAY_RESET	IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, SPI_PAD_CTRL),
#define GP_BACKLIGHT		IMX_GPIO_NR(1, 21)		/* PWM1 */
	IOMUX_PAD_CTRL(SD1_DAT3__GPIO1_IO21, SPI_PAD_CTRL),

	/* DI0 */
	IOMUX_PAD_CTRL(DI0_DISP_CLK__IPU1_DI0_DISP_CLK, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN15__IPU1_DI0_PIN15, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN2__IPU1_DI0_PIN02, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN3__IPU1_DI0_PIN03, DI0_PAD_CTRL),
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

		if (!len && !reg)
			break;
		cmds += 3;
		do {
			buf[0] = 0x20;
			buf[1] = reg >> 8;
			ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				debug("%s: Failed to select reg1 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			udelay(2);
			buf[0] = 0;
			buf[1] = reg;
			ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				debug("%s: Failed to select reg2 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			udelay(2);
			if (!len) {
				debug("spi: reg:%04x\n", reg);
				break;
			}
			buf[0] = 0x40;
			buf[1] = *cmds++;
			ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				debug("%s: Failed to select reg3 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			debug("spi: reg:%04x %02x\n", reg, buf[1]);
			udelay(2);
			reg++;
		} while (--len);
	}
	return ret;
}

#define A(reg, cnt) (reg >> 8), (reg & 0xff), cnt

static u8 display_init_cmds[] = {
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

static u8 display_on_cmds[] = {
	A(0x2900, 0),
	A(0, 0)
};

void enable_spi_rgb(struct display_info_t const *dev)
{
	unsigned cs_gpio = GP_ECSPI2_CS;
	struct spi_slave *spi;
	int ret;

	gpio_direction_output(GP_BACKLIGHT, 1);
	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, dev->bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(dev->bus, cs_gpio << 8, 1000000, SPI_MODE_0);
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
	mdelay(200);
	ret = spi_display_cmds(spi, display_on_cmds);
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
int detect_spi(struct display_info_t const *dev)
{
	unsigned cs_gpio = GP_ECSPI2_CS;
	unsigned reset_gpio = GP_SPI_DISPLAY_RESET;

	debug("%s\n", __func__);
	gpio_direction_output(cs_gpio, 1);
	gpio_direction_output(reset_gpio, 1);
	SETUP_IOMUX_PADS(spi_display_pads);
	gpio_direction_output(reset_gpio, 0);
	udelay(200);
	gpio_direction_output(reset_gpio, 1);
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
	uint reg;
	u8 buf[80];

	if (argc < 2)
		return 1;
	gpio_direction_output(GP_BACKLIGHT, 1);
	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(bus, cs_gpio << 8, 1000000, SPI_MODE_0);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return 1;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
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
	spi_display_cmds(spi, buf);
	spi_release_bus(spi);
free_bus:
	spi_free_slave(spi);
	enable_spi_clk(0, bus);
	return ret ? 1 : 0;
}

U_BOOT_CMD(
	spid, 70, 0, do_spid,
	"write cmd, data to spi display",
	"reg16 [byte]"
);

