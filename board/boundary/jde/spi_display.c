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

#define GP_SPI_DISPLAY_RESET	IMX_GPIO_NR(3, 4)

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
	IOMUX_PAD_CTRL(CSI_DATA02__ECSPI2_MOSI, SPI_MOSI_R_PAD_CTRL),
};

static iomux_v3_cfg_t const spi_mosi_w_pads[] = {
	IOMUX_PAD_CTRL(CSI_DATA02__ECSPI2_MOSI, SPI_PAD_CTRL),
};

static iomux_v3_cfg_t const spi_ss0_pad[] = {
	IOMUX_PAD_CTRL(CSI_DATA01__ECSPI2_SS0, SPI_PAD_CTRL),
};

static iomux_v3_cfg_t const spi_ss0_gpio_pad[] = {
#define GP_ECSPI2_CS		IMX_GPIO_NR(5, 29)
	IOMUX_PAD_CTRL(CSI_DATA01__GPIO4_IO22, SPI_PAD_CTRL),
};

#define A(reg, cnt) (reg >> 8), (reg & 0xff), cnt

/* *************************************************** */

static int AM320240_spi_write_rtn(struct spi_slave *spi, u8 *cmds)
{
	u8 buf[4];
	int ret = 0;

	while (1) {
		uint reg = (cmds[0] << 8) | cmds[1];
		uint len = cmds[2];

		if (!len)
			break;
		cmds += 3;
		do {
			buf[0] = (reg << 2) | 3;
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

static int AM320240_spi_read_rtn(struct spi_slave *spi, int reg)
{
	u8 buf[4];
	u8 rbuf[4];
	int ret = 0;

	buf[0] = (reg << 2) | 1;
	buf[1] = 0xff;
	ret = spi_xfer(spi, 2 * 8, buf, rbuf, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret) {
		pr_debug("%s: Failed 0x%x, %d\n", __func__, reg, ret);
		return ret;
	}
	pr_debug("spi: reg:0x%02x: %02x %02x\n", reg, rbuf[0], rbuf[1]);
	return rbuf[1];
}

static u8 AM320240_display_init_cmds[] = {
/* Display Mode Setting */
	A(0x0e, 2), 0x63, 0xa4,
	A(0x02, 1), 0x03,
	A(0, 0)
};

static u8 AM320240_display_on_cmds[] = {
	A(0, 0)
};

/* *************************************************** */

struct spi_display_info spi_di[] = {
	{ .mode = SPI_MODE_3, .speed_r = 10000, .speed_w = 10000, .reset_active_low = 1,
		.cs_gpio = GP_ECSPI2_CS,
		.reset_gpio = GP_SPI_DISPLAY_RESET,
		.init_cmds = AM320240_display_init_cmds,
		.on_cmds = AM320240_display_on_cmds,
		.spi_mosi_r_pads = spi_mosi_r_pads,
		.spi_mosi_w_pads = spi_mosi_w_pads,
		.spi_ss0_pad = spi_ss0_pad,
		.spi_ss0_gpio_pad = spi_ss0_gpio_pad,
		.spi_write_rtn = AM320240_spi_write_rtn,
		.spi_read_rtn = AM320240_spi_read_rtn},
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
	"reg byte .."
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
	"reg"
);
