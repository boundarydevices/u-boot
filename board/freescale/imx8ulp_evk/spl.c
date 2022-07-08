// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 */

#include <common.h>
#include <init.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx8ulp-pins.h>
#include <asm/mach-imx/s400_api.h>
#include <fsl_sec.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <asm/arch/ddr.h>
#include <asm/arch/rdc.h>
#include <asm/arch/upower.h>
#include <asm/mach-imx/s400_api.h>
#include <asm/arch/clock.h>
#include <asm/arch/pcc.h>

DECLARE_GLOBAL_DATA_PTR;

void spl_dram_init(void)
{
	/* Reboot in dual boot setting no need to init ddr again */
	bool ddr_enable = pcc_clock_is_enable(5, LPDDR4_PCC5_SLOT);
	if (!ddr_enable) {
		init_clk_ddr();
		ddr_init(&dram_timing);
	} else {
		/* reinit pfd/pfddiv and lpavnic except pll4*/
		cgc2_pll4_init(false);
	}
}

u32 spl_boot_device(void)
{
	return BOOT_DEVICE_BOOTROM;
}

int power_init_board(void)
{
	u32 pmic_reg;

	/* PMIC set bucks1-4 to PWM mode */
	upower_pmic_i2c_read(0x10, &pmic_reg);
	upower_pmic_i2c_read(0x14, &pmic_reg);
	upower_pmic_i2c_read(0x21, &pmic_reg);
	upower_pmic_i2c_read(0x2e, &pmic_reg);

	upower_pmic_i2c_write(0x10, 0x3d);
	upower_pmic_i2c_write(0x14, 0x7d);
	upower_pmic_i2c_write(0x21, 0x7d);
	upower_pmic_i2c_write(0x2e, 0x3d);

	upower_pmic_i2c_read(0x10, &pmic_reg);
	upower_pmic_i2c_read(0x14, &pmic_reg);
	upower_pmic_i2c_read(0x21, &pmic_reg);
	upower_pmic_i2c_read(0x2e, &pmic_reg);

	if (IS_ENABLED(CONFIG_IMX8ULP_LD_MODE)) {
		/* Set buck3 to 0.9v LD */
		upower_pmic_i2c_write(0x22, 0x18);
	} else if (IS_ENABLED(CONFIG_IMX8ULP_ND_MODE)) {
		/* Set buck3 to 1.0v ND */
		upower_pmic_i2c_write(0x22, 0x20);
	} else {
		/* Set buck3 to 1.1v OD */
		upower_pmic_i2c_write(0x22, 0x28);

	}

	return 0;
}

void spl_board_init(void)
{
	struct udevice *dev;
	u32 res;
	int ret;

	ret = arch_cpu_init_dm();
	if (ret)
		return;

	board_early_init_f();

	preloader_console_init();

	puts("Normal Boot\n");

	/* After AP set iomuxc0, the i2c can't work, Need M33 to set it now */

	/* Load the lposc fuse for single boot to work around ROM issue,
	 *  The fuse depends on S400 to read.
	 */
	if (is_soc_rev(CHIP_REV_1_0) && get_boot_mode() == SINGLE_BOOT)
		load_lposc_fuse();

	upower_init();

	power_init_board();

	clock_init_late();

	/* DDR initialization */
	spl_dram_init();

	/* This must place after upower init, so access to MDA and MRC are valid */
	/* Init XRDC MDA  */
	xrdc_init_mda();

	/* Init XRDC MRC for VIDEO, DSP domains */
	xrdc_init_mrc();

	/* Call it after PS16 power up */
	set_lpav_qos();

	/* Asks S400 to release CAAM for A35 core */
	ret = ahab_release_caam(7, &res);
	if (!ret) {

		/* Only two UCLASS_MISC devicese are present on the platform. There
		 * are MU and CAAM. Here we initialize CAAM once it's released by
		 * S400 firmware..
		 */
		if (IS_ENABLED(CONFIG_FSL_CAAM)) {
			ret = uclass_get_device_by_driver(UCLASS_MISC, DM_DRIVER_GET(caam_jr), &dev);
			if (ret)
				printf("Failed to initialize %s: %d\n", dev->name, ret);
		}
	}
}

void board_init_f(ulong dummy)
{
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	timer_init();

	arch_cpu_init();

	board_init_r(NULL, 0);
}
