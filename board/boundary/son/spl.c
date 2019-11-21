/*
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <hang.h>
#include <init.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/ddr.h>
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define I2C_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = IMX8MQ_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = IMX8MQ_PAD_I2C1_SCL__GPIO5_IO14 | PC,
		.gp = IMX_GPIO_NR(5, 14),
	},
	.sda = {
		.i2c_mode = IMX8MQ_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = IMX8MQ_PAD_I2C1_SDA__GPIO5_IO15 | PC,
		.gp = IMX_GPIO_NR(5, 15),
	},
};

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | \
			 PAD_CTL_FSEL2)
#define USDHC_GPIO_PAD_CTRL (PAD_CTL_PUE | PAD_CTL_DSE1)

static iomux_v3_cfg_t const init_pads[] = {
#define GP_I2C1_PCA9546_RESET		IMX_GPIO_NR(1, 8)
	IMX8MQ_PAD_GPIO1_IO08__GPIO1_IO8 | MUX_PAD_CTRL(0x49),

	IMX8MQ_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA4__USDHC1_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA5__USDHC1_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA6__USDHC1_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA7__USDHC1_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IMX8MQ_PAD_SD1_RESET_B__GPIO2_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),

	IMX8MQ_PAD_SD2_CLK__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD2_CMD__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD2_DATA0__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD2_DATA1__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD2_DATA2__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD2_DATA3__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
#define GP_USDHC2_CD	IMX_GPIO_NR(2, 12)
	IMX8MQ_PAD_SD2_CD_B__GPIO2_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define GP_USDHC2_VSEL		IMX_GPIO_NR(3, 20)
	IMX8MQ_PAD_SAI5_RXC__GPIO3_IO20 | MUX_PAD_CTRL(0x16),
};

struct fsl_esdhc_cfg board_usdhc_cfg[] = {
	{.esdhc_base = USDHC1_BASE_ADDR, .bus_width = 8,
			.gp_reset = GP_EMMC_RESET},
	{.esdhc_base = USDHC2_BASE_ADDR, .bus_width = 4,
			.gp_cd = GP_USDHC2_CD},
};

int power_init_boundary(void)
{
	int ret;
	unsigned char buf[4];

	i2c_set_bus_num(0);
#define PF8100	0x08
#define SW1_VOLT	0x51
#define SW2_VOLT	0x59
#define SW3_CONFIG2	0x5e
#define SW3_VOLT	0x61
#define SW4_CONFIG2	0x66
#define SW4_VOLT	0x69
#define SW5_VOLT	0x71
#define SW6_VOLT	0x79

	buf[0] = 0x50;	/* (.90-.4)*160=.50*160=80=0x50  80/160+.4=.90 */
	/* dram */
	ret = i2c_write(PF8100, SW2_VOLT, 1, buf, 1);
	if (ret)
		return ret;
	/* aux_0p9 */
	ret = i2c_write(PF8100, SW4_VOLT, 1, buf, 1);
	if (ret)
		return ret;
	/* arm */
	ret = i2c_write(PF8100, SW3_VOLT, 1, buf, 1);
	if (ret)
		return ret;
	/* soc */
	ret = i2c_write(PF8100, SW1_VOLT, 1, buf, 1);
	if (ret)
		return ret;
	/*
	 * Make sw3 a 180 phase shift from sw4,
	 * in case pmic not programmed for dual mode
	 */
	ret = i2c_read(PF8100, SW4_CONFIG2, 1, buf, 1);
	if (!ret) {
		buf[0] ^= 4;	/* 180 degree phase */
		ret = i2c_write(PF8100, SW3_CONFIG2, 1, buf, 1);
	}

	/* vpu/gpu */
	buf[0] = 0x50;	/* (.90-.4)*160=.50*160=80=0x50  80/160+.4=.90 */
	ret = i2c_write(PF8100, SW5_VOLT, 1, buf, 1);

	/* DRAM_1P1V */
	buf[0] = 0x70;	/* (1.10-.4)*160=.70*160=112=0x70  112/160+.4=1.10 */
	ret = i2c_write(PF8100, SW6_VOLT, 1, buf, 1);
	return ret;
}

void spl_board_init(void)
{
#ifndef CONFIG_SPL_USB_SDP_SUPPORT
	/* Serial download mode */
	if (is_usb_boot()) {
		puts("Back to ROM, SDP\n");
		restore_boot_params();
	}
#endif
	puts("Normal Boot\n");
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	pr_debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	int ret;

	arch_cpu_init();

	init_uart_clk(0);

	board_early_init_f();

	preloader_console_init();

	ret = spl_early_init();
	if (ret) {
		printf("spl_init() failed: %d\n", ret);
		hang();
	}

	enable_tzc380();
	gpio_request(GP_USDHC2_VSEL, "usdhc2_vsel");
	gpio_direction_output(GP_USDHC2_VSEL, 0);
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	/* Adjust pmic voltage to 1.0V for 800M */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

	power_init_boundary();

	/* DDR initialization */
	spl_dram_init();
}
