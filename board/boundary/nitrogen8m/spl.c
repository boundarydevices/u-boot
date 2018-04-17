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
};

struct fsl_esdhc_cfg board_usdhc_cfg[] = {
	{.esdhc_base = USDHC1_BASE_ADDR, .bus_width = 8,
			.gp_reset = GP_EMMC_RESET},
};

#define GP_ARM_DRAM_VSEL		IMX_GPIO_NR(3, 24)
#define GP_DRAM_1P1_VSEL		IMX_GPIO_NR(2, 11)
#define GP_SOC_GPU_VPU_VSEL		IMX_GPIO_NR(2, 20)

#define I2C_MUX_ADDR		0x70
#define I2C_FAN53555_ADDR	0x60
void ddr_voltage_init(void)
{
	u8 val8;

	gpio_set_value(GP_I2C1_PCA9546_RESET, 1);
	gpio_set_value(GP_ARM_DRAM_VSEL, 0);
	gpio_set_value(GP_DRAM_1P1_VSEL, 0);
	gpio_set_value(GP_SOC_GPU_VPU_VSEL, 0);
	printf("Setting voltages\n");
	/*
	 * 9e (1e = 30) default .9 V
	 * 0.6V to 1.23V in 10 MV steps
	 */

	/* Enable I2C1A, ARM/DRAM */
	i2c_write(I2C_MUX_ADDR, 1, 1, NULL, 0);
	/*
	 * .6 + .40 = 1.00
	 */
	val8 = 0x80 + 40;
	i2c_write(I2C_FAN53555_ADDR, 0, 1, &val8, 1);
	i2c_write(I2C_FAN53555_ADDR, 1, 1, &val8, 1);

	/* Enable I2C1B, DRAM 1.1V */
	i2c_write(I2C_MUX_ADDR, 2, 1, NULL, 0);
	/*
	 * .6 + .50 = 1.10
	 */
	val8 = 0x80 + 50;
	i2c_write(I2C_FAN53555_ADDR, 0, 1, &val8, 1);
	i2c_write(I2C_FAN53555_ADDR, 1, 1, &val8, 1);

	/* Enable I2C1C, soc/gpu/vpu */
	i2c_write(I2C_MUX_ADDR, 4, 1, NULL, 0);
	/*
	 * .6 + .30 = .90
	 */
	val8 = 0x80 + 30;
	i2c_write(I2C_FAN53555_ADDR, 0, 1, &val8, 1);
	i2c_write(I2C_FAN53555_ADDR, 1, 1, &val8, 1);

	/* Disable I2C1A-I2C1D */
	i2c_write(I2C_MUX_ADDR, 0, 1, NULL, 0);
}

int power_init_board(void)
{
	/* Nitrogen8M I2C write */
	ddr_voltage_init();
	return 0;
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
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	/* Adjust pmic voltage to 1.0V for 800M */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

	power_init_board();

	/* DDR initialization */
	spl_dram_init();
}
