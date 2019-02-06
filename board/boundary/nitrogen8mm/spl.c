/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/ddr.h>
#include <asm/arch/imx8m_ddr.h>

DECLARE_GLOBAL_DATA_PTR;

void spl_dram_init(void)
{
	/* ddr train */
	ddr_init(&lpddr4_timing);
}

#define I2C_PAD_CTRL	(PAD_CTL_DSE3 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
struct i2c_pads_info i2c_pad_info1[] = {
{
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C1_SCL_I2C1_SCL | PC,
		.gpio_mode = IMX8MM_PAD_I2C1_SCL_GPIO5_IO14 | PC,
		.gp = IMX_GPIO_NR(5, 14),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C1_SDA_I2C1_SDA | PC,
		.gpio_mode = IMX8MM_PAD_I2C1_SDA_GPIO5_IO15 | PC,
		.gp = IMX_GPIO_NR(5, 15),
	},
},
{
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C2_SCL_I2C2_SCL | PC,
		.gpio_mode = IMX8MM_PAD_I2C2_SCL_GPIO5_IO16 | PC,
		.gp = IMX_GPIO_NR(5, 16),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C2_SDA_I2C2_SDA | PC,
		.gpio_mode = IMX8MM_PAD_I2C2_SDA_GPIO5_IO17 | PC,
		.gp = IMX_GPIO_NR(5, 17),
	},
},
{
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C3_SCL_I2C3_SCL | PC,
		.gpio_mode = IMX8MM_PAD_I2C3_SCL_GPIO5_IO18 | PC,
		.gp = IMX_GPIO_NR(5, 18),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C3_SDA_I2C3_SDA | PC,
		.gpio_mode = IMX8MM_PAD_I2C3_SDA_GPIO5_IO19 | PC,
		.gp = IMX_GPIO_NR(5, 19),
	},
},
{
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C4_SCL_I2C4_SCL | PC,
		.gpio_mode = IMX8MM_PAD_I2C4_SCL_GPIO5_IO20 | PC,
		.gp = IMX_GPIO_NR(5, 20),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C4_SDA_I2C4_SDA | PC,
		.gpio_mode = IMX8MM_PAD_I2C4_SDA_GPIO5_IO21 | PC,
		.gp = IMX_GPIO_NR(5, 21),
	},
},
};

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE |PAD_CTL_PE | \
			 PAD_CTL_FSEL2)
#define USDHC_GPIO_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_DSE1)

static iomux_v3_cfg_t const usdhc1_pads[] = {
	IMX8MM_PAD_SD1_CLK_USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_CMD_USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA0_USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA1_USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA2_USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA3_USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA4_USDHC1_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA5_USDHC1_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA6_USDHC1_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA7_USDHC1_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IMX8MM_PAD_SD1_RESET_B_GPIO2_IO10 | MUX_PAD_CTRL(0x41),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	IMX8MM_PAD_SD2_CLK_USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_CMD_USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA0_USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA1_USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA2_USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA3_USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_RESET_B_GPIO2_IO19 | MUX_PAD_CTRL(USDHC_GPIO_PAD_CTRL),
#define GP_USDHC2_VSEL		IMX_GPIO_NR(3, 2)
	IMX8MM_PAD_NAND_CE1_B_GPIO3_IO2 | MUX_PAD_CTRL(0x16),
#define USDHC2_CD_GPIO		IMX_GPIO_NR(2, 12)
	IMX8MM_PAD_SD2_CD_B_GPIO2_IO12 | MUX_PAD_CTRL(USDHC_GPIO_PAD_CTRL),
};

static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{.esdhc_base = USDHC1_BASE_ADDR, .bus_width = 8, .gp_reset = GP_EMMC_RESET},
	{.esdhc_base = USDHC2_BASE_ADDR, .bus_width = 4,},
};

int board_mmc_init(bd_t *bis)
{
	int i, ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc1                    USDHC2
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			gpio_request(GP_EMMC_RESET, "emmc_reset");
			gpio_direction_output(GP_EMMC_RESET, 0);
			udelay(500);
			gpio_direction_output(GP_EMMC_RESET, 1);
			break;
		case 1:
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			gpio_request(USDHC2_CD_GPIO, "usdhc2 cd");
			gpio_direction_input(USDHC2_CD_GPIO);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		return 1;
	case USDHC2_BASE_ADDR:
		ret = gpio_get_value(USDHC2_CD_GPIO);
		return ret ? 0 : 1;
	}
	printf("c\n");

	return 0;
}

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

	gpio_request(GP_USDHC2_VSEL, "usdhc2_vsel");
	gpio_direction_output(GP_USDHC2_VSEL, 0);
	imx_iomux_v3_setup_multiple_pads(
		usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
	return ret;
}

void spl_board_init(void)
{
	int i;
	enable_tzc380();

	for (i = 0; i < ARRAY_SIZE(i2c_pad_info1); i++)
		setup_i2c(i, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1[i]);

	malloc(sizeof(int));

	power_init_boundary();
	/* DDR initialization */
	spl_dram_init();

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
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	/* Clear global data */
	memset((void *)gd, 0, sizeof(gd_t));

	arch_cpu_init();

	board_early_init_f();

	timer_init();

	preloader_console_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	board_init_r(NULL, 0);
}
