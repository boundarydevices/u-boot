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
#include <asm/arch/s400_api.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <asm/arch/ddr.h>
#include <asm/arch/rdc.h>
#include <asm/arch/upower.h>

DECLARE_GLOBAL_DATA_PTR;

void spl_dram_init(void)
{
	init_clk_ddr();
	ddr_init(&dram_timing);
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

	/* Set buck3 to 1.1v OD */
	upower_pmic_i2c_write(0x22, 0x28);
	return 0;
}

void spl_board_init(void)
{
	struct udevice *dev;
	u32 res;
	int ret;

	uclass_find_first_device(UCLASS_MISC, &dev);

	for (; dev; uclass_find_next_device(&dev)) {
		if (device_probe(dev))
			continue;
	}

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

	/* DDR initialization */
	spl_dram_init();

	/* This must place after upower init, so access to MDA and MRC are valid */
	/* Init XRDC MDA  */
	xrdc_init_mda();

	/* Init XRDC MRC for VIDEO, DSP domains */
	xrdc_init_mrc();

	/* Call it after PS16 power up */
	set_lpav_qos();

	/* Enable A35 access to the CAAM */
	ret = ahab_release_caam(0x7, &res);
	if (ret)
		printf("ahab release caam failed %d, 0x%x\n", ret, res);
}

void board_init_f(ulong dummy)
{
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	timer_init();

	arch_cpu_init();

	board_init_r(NULL, 0);
}
