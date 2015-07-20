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

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define SPI_MOSI_R_PAD_CTRL	SPI_PAD_CTRL | PAD_CTL_ODE | PAD_CTL_PUS_22K_UP

#define GP_SPI_DISPLAY_RESET	IMX_GPIO_NR(4, 20)

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

struct spi_display_info {
	int mode;
	int speed_r;
	int speed_w;
	int reset_active_low;
	u8 *init_cmds;
	u8 *on_cmds;
	int (*spi_write_rtn)(struct spi_slave *spi, u8 *cmds);
	int (*spi_read_rtn)(struct spi_slave *spi, int reg);
};

struct spi_display_info spi_di[] = {
	{ .mode = SPI_MODE_0, .speed_r = 10000, .speed_w = 1000000, .reset_active_low = 1,
			.init_cmds = AUO_G050_display_init_cmds, .on_cmds = AUO_G050_display_on_cmds,
			.spi_write_rtn = AUO_G050_spi_write_rtn, .spi_read_rtn = AUO_G050_spi_read_rtn},
	{ .mode = SPI_MODE_3, .speed_r = 10000, .speed_w = 10000, .reset_active_low = 1,
			.init_cmds = A030JN01_display_YUV720_init_cmds, .on_cmds = A030JN01_display_on_cmds,
			.spi_write_rtn = A030JN01_spi_write_rtn, .spi_read_rtn = A030JN01_spi_read_rtn},
	{ .mode = SPI_MODE_3, .speed_r = 10000, .speed_w = 10000, .reset_active_low = 1,
			.init_cmds = A030JN01_display_UPS051_init_cmds, .on_cmds = A030JN01_display_on_cmds,
			.spi_write_rtn = A030JN01_spi_write_rtn, .spi_read_rtn = A030JN01_spi_read_rtn},
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
	unsigned cs_gpio = GP_ECSPI2_CS;
	unsigned reset_gpio = GP_SPI_DISPLAY_RESET;
	struct spi_display_info *di = &spi_di[dev->addr];
	int reset_val = di->reset_active_low ? 0 : 1;

	pr_debug("%s\n", __func__);
	gpio_direction_output(cs_gpio, 1);
	gpio_direction_output(reset_gpio, reset_val ^ 1);
	gpio_direction_output(reset_gpio, reset_val);
	udelay(200);
	gpio_direction_output(reset_gpio, reset_val ^ 1);
	mdelay(200);
}

void enable_spi_rgb(struct display_info_t const *dev)
{
	unsigned cs_gpio = GP_ECSPI2_CS;
	struct spi_slave *spi;
	int ret;
	struct spi_display_info *di = &spi_di[dev->addr];

	g_dev = dev;
	init_spi(dev);
	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, dev->bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(dev->bus, 0, di->speed_w, di->mode);
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
	SETUP_IOMUX_PADS(spi_ss0_pad);
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
	SETUP_IOMUX_PADS(spi_ss0_gpio_pad);

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
	int display_index = g_dev ? g_dev->addr : 0;
	int bus = g_dev ? g_dev->bus : 1;
	struct spi_display_info *di = &spi_di[display_index];
	int ret = 0;
	int arg = 2;
	uint reg;
	u8 buf[80];

	if (argc < 2)
		return 1;
	gpio_direction_output(cs_gpio, 1);
	SETUP_IOMUX_PADS(spi_mosi_w_pads);

	enable_spi_clk(1, bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(bus, 0, di->speed_w, di->mode);
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
	SETUP_IOMUX_PADS(spi_ss0_pad);
	di->spi_write_rtn(spi, buf);
	SETUP_IOMUX_PADS(spi_ss0_gpio_pad);
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

static int do_spidr(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned cs_gpio = GP_ECSPI2_CS;
	struct spi_slave *spi;
	int display_index = g_dev ? g_dev->addr : 0;
	int bus = g_dev ? g_dev->bus : 1;
	struct spi_display_info *di = &spi_di[display_index];
	int ret = 0;
	uint reg;
	int val;

	if (argc != 2)
		return CMD_RET_USAGE;
	gpio_direction_output(cs_gpio, 1);
	SETUP_IOMUX_PADS(spi_mosi_r_pads);

	enable_spi_clk(1, bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(bus, 0, di->speed_r, di->mode);
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
	SETUP_IOMUX_PADS(spi_ss0_pad);
	val = di->spi_read_rtn(spi, reg);
	SETUP_IOMUX_PADS(spi_ss0_gpio_pad);
	printf("spidr: reg:0x%x = 0x%x\n", reg, val);
	spi_release_bus(spi);
free_bus:
	spi_free_slave(spi);
	enable_spi_clk(0, bus);
	return ret ? 1 : 0;
}

U_BOOT_CMD(
	spidr, 70, 0, do_spidr,
	"read spi display register",
	"reg16"
);
