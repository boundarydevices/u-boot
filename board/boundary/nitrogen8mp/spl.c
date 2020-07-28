/*
 * Copyright 2018-2019 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/ddr.h>
#include <asm/arch/imx8mp_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/io.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>

#include <errno.h>
#include <fsl_esdhc_imx.h>
#include <hang.h>
#include <init.h>
#include <mmc.h>
#include <power/pca9450.h>
#include <power/pmic.h>
#include <spl.h>
#include "../common/bd_common.h"


DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_SPL_BOOTROM_SUPPORT
int spl_board_boot_device(enum boot_device boot_dev_spl)
{
	return BOOT_DEVICE_BOOTROM;
}
#endif

void spl_board_init(void)
{
	puts("Normal Boot\n");
}

#define I2C_PAD_CTRL (PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)
struct i2c_pads_info i2c_pad_info1 = {
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
};

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE |PAD_CTL_PE | \
			 PAD_CTL_FSEL2)
#define USDHC_GPIO_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_DSE1)
#define USDHC_CD_PAD_CTRL (PAD_CTL_PE |PAD_CTL_PUE |PAD_CTL_HYS | PAD_CTL_DSE4)


static iomux_v3_cfg_t const init_pads[] = {
	IOMUX_PAD_CTRL(SD1_CLK__USDHC1_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_CMD__USDHC1_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA0__USDHC1_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA1__USDHC1_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA2__USDHC1_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA3__USDHC1_DATA3, USDHC_PAD_CTRL),
#define GP_USDHC1_CD	IMX_GPIO_NR(2, 11)
	IOMUX_PAD_CTRL(SD1_STROBE__GPIO2_IO11, 0x1c4),

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
#define GP_EMMC_RESET		IMX_GPIO_NR(3, 1)
	IOMUX_PAD_CTRL(NAND_CE0_B__GPIO3_IO01, 0x140),

};

struct fsl_esdhc_cfg board_usdhc_cfg[] = {
	{.esdhc_base = USDHC1_BASE_ADDR, .bus_width = 4, .gp_cd = GP_USDHC1_CD},
	{.esdhc_base = USDHC2_BASE_ADDR, .bus_width = 4, .gp_cd = -1},
	{.esdhc_base = USDHC3_BASE_ADDR, .bus_width = 8, .gp_reset = GP_EMMC_RESET},
};

#ifdef CONFIG_POWER
#define I2C_PMIC	0
int power_init_board(void)
{
	struct pmic *p;
	int ret;

	ret = power_pca9450_init(I2C_PMIC);
	if (ret)
		printf("power init failed");
	p = pmic_get("PCA9450");
	pmic_probe(p);

	/* BUCKxOUT_DVS0/1 control BUCK123 output */
	pmic_reg_write(p, PCA9450_BUCK123_DVS, 0x29);

	/*
	 * increase VDD_SOC to typical value 0.95V before first
	 * DRAM access, set DVS1 to 0.85v for suspend.
	 * Enable DVS control through PMIC_STBY_REQ and
	 * set B1_ENMODE=1 (ON by PMIC_ON_REQ=H)
	 */
	pmic_reg_write(p, PCA9450_BUCK1OUT_DVS0, 0x1C);
	pmic_reg_write(p, PCA9450_BUCK1OUT_DVS1, 0x14);
	pmic_reg_write(p, PCA9450_BUCK1CTRL, 0x59);

	/* Kernel uses OD/OD freq for SOC */
	/* To avoid timing risk from SOC to ARM,increase VDD_ARM to OD voltage 0.95v */
	pmic_reg_write(p, PCA9450_BUCK2OUT_DVS0, 0x1C);

	/* set WDOG_B_CFG to cold reset */
	pmic_reg_write(p, PCA9450_RESET_CTRL, 0xA1);

	return 0;
}
#endif

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

	board_early_init_f();

	preloader_console_init();

	ret = spl_early_init();
	if (ret) {
		pr_debug("spl_init() failed: %d\n", ret);
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
