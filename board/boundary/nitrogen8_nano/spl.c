/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <hang.h>
#include <init.h>
#include <i2c.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#if defined(CONFIG_IMX8MM)
#include <asm/arch/imx8mm_pins.h>
#elif defined(CONFIG_IMX8MN)
#include <asm/arch/imx8mn_pins.h>
#endif
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/ddr.h>
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define I2C_PAD_CTRL	(PAD_CTL_DSE3 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)

struct i2c_pads_info i2c_pad_info1[] = {
{
	.scl = {
		.i2c_mode = IOMUX_PAD_CTRL(I2C1_SCL__I2C1_SCL, I2C_PAD_CTRL),
		.gpio_mode = IOMUX_PAD_CTRL(I2C1_SCL__GPIO5_IO14, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 14),
	},
	.sda = {
		.i2c_mode = IOMUX_PAD_CTRL(I2C1_SDA__I2C1_SDA, I2C_PAD_CTRL),
		.gpio_mode = IOMUX_PAD_CTRL(I2C1_SDA__GPIO5_IO15, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 15),
	},
},
{
	.scl = {
		.i2c_mode = IOMUX_PAD_CTRL(I2C2_SCL__I2C2_SCL, I2C_PAD_CTRL),
		.gpio_mode = IOMUX_PAD_CTRL(I2C2_SCL__GPIO5_IO16, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 16),
	},
	.sda = {
		.i2c_mode = IOMUX_PAD_CTRL(I2C2_SDA__I2C2_SDA, I2C_PAD_CTRL),
		.gpio_mode = IOMUX_PAD_CTRL(I2C2_SDA__GPIO5_IO17, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 17),
	},
},
{
	.scl = {
		.i2c_mode = IOMUX_PAD_CTRL(I2C3_SCL__I2C3_SCL, I2C_PAD_CTRL),
		.gpio_mode = IOMUX_PAD_CTRL(I2C3_SCL__GPIO5_IO18, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 18),
	},
	.sda = {
		.i2c_mode = IOMUX_PAD_CTRL(I2C3_SDA__I2C3_SDA, I2C_PAD_CTRL),
		.gpio_mode = IOMUX_PAD_CTRL(I2C3_SDA__GPIO5_IO19, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 19),
	},
},
{
	.scl = {
		.i2c_mode = IOMUX_PAD_CTRL(I2C4_SCL__I2C4_SCL, I2C_PAD_CTRL),
		.gpio_mode = IOMUX_PAD_CTRL(I2C4_SCL__GPIO5_IO20, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 20),
	},
	.sda = {
		.i2c_mode = IOMUX_PAD_CTRL(I2C4_SDA__I2C4_SDA, I2C_PAD_CTRL),
		.gpio_mode = IOMUX_PAD_CTRL(I2C4_SDA__GPIO5_IO21, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 21),
	},
},
};

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE |PAD_CTL_PE | \
			 PAD_CTL_FSEL2)
#define USDHC_GPIO_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_DSE1)

static iomux_v3_cfg_t const init_pads[] = {
	IOMUX_PAD_CTRL(SD1_CLK__USDHC1_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_CMD__USDHC1_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA0__USDHC1_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA1__USDHC1_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA2__USDHC1_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA3__USDHC1_DATA3, USDHC_PAD_CTRL),

	IOMUX_PAD_CTRL(NAND_WE_B__USDHC3_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_WP_B__USDHC3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA04__USDHC3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA05__USDHC3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA06__USDHC3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA07__USDHC3_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_RE_B__USDHC3_DATA4, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_CE2_B__USDHC3_DATA5, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_CE3_B__USDHC3_DATA6, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_CLE__USDHC3_DATA7, USDHC_PAD_CTRL),
#define GP_EMMC_RESET	IMX_GPIO_NR(3, 16)
	IOMUX_PAD_CTRL(NAND_READY_B__GPIO3_IO16, 0x41),

	IOMUX_PAD_CTRL(SD2_CLK__USDHC2_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__USDHC2_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA0__USDHC2_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA1__USDHC2_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA2__USDHC2_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA3__USDHC2_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_RESET_B__GPIO2_IO19, USDHC_GPIO_PAD_CTRL),
#define GP_USDHC2_VSEL		IMX_GPIO_NR(3, 2)
	IOMUX_PAD_CTRL(NAND_CE1_B__GPIO3_IO2, 0x16),
#define GP_USDHC2_CD		IMX_GPIO_NR(2, 12)
	IOMUX_PAD_CTRL(SD2_CD_B__GPIO2_IO12, USDHC_GPIO_PAD_CTRL),
};

struct fsl_esdhc_cfg board_usdhc_cfg[] = {
	{.esdhc_base = USDHC1_BASE_ADDR, .bus_width = 1, .gp_cd = -1},	/* keep numbering correct */
	{.esdhc_base = USDHC2_BASE_ADDR, .bus_width = 4, .gp_cd = GP_USDHC2_CD},
	{.esdhc_base = USDHC3_BASE_ADDR, .bus_width = 8, .gp_reset = GP_EMMC_RESET},
};

int power_init_boundary(void)
{
	int ret;
	unsigned char buf[4];

	i2c_set_bus_num(0);
#define PF8100	0x08
#define SW2_VOLT	0x59
#define SW3_CONFIG2	0x5e
#define SW3_VOLT	0x61
#define SW4_CONFIG2	0x66
#define SW4_VOLT	0x69
#define SW5_VOLT	0x71

	buf[0] = 0x50;	/* (.90-.4)*160=.50*160=80=0x50  80/160+.4=.90 gpu/dram/arm */
	ret = i2c_write(PF8100, SW2_VOLT, 1, buf, 1);
	if (ret)
		return ret;
	ret = i2c_write(PF8100, SW4_VOLT, 1, buf, 1);
	if (ret)
		return ret;
	ret = i2c_write(PF8100, SW3_VOLT, 1, buf, 1);
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

	buf[0] = 0x40;	/* (.80-.4)*160=.40*160=64=0x40  64/160+.4=.80 vpu */
	ret = i2c_write(PF8100, SW5_VOLT, 1, buf, 1);
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
	int i;

	arch_cpu_init();

	init_uart_clk(1);

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

	for (i = 0; i < ARRAY_SIZE(i2c_pad_info1); i++)
		setup_i2c(i, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1[i]);

	power_init_boundary();
	/* DDR initialization */
	spl_dram_init();
}
