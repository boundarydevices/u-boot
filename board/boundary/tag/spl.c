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
#include <asm/arch/imx8ulp-pins.h>
#include <asm/mach-imx/s400_api.h>
#include <fsl_sec.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <asm/arch/clock.h>
#include <asm/arch/ddr.h>
#include <asm/arch/pcc.h>
#include <asm/arch/rdc.h>
#include <asm/arch/upower.h>
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

u32 spl_boot_device(void)
{
	return BOOT_DEVICE_BOOTROM;
}

int power_init_board(void)
{
	u32 buck3val;

	if (IS_ENABLED(CONFIG_IMX8ULP_LD_MODE)) {
		/* Set buck3 to 0.9v LD */
		buck3val = 0x18;
	} else if (IS_ENABLED(CONFIG_IMX8ULP_ND_MODE)) {
		/* Set buck3 to 1.0v ND */
		buck3val = 0x20;
	} else {
		/* Set buck3 to 1.1v OD */
		buck3val = 0x28;
	}

	upower_pmic_i2c_write(0x22, buck3val);	/* PMIC_STBY_REQ = L */
	upower_pmic_i2c_write(0x2a, buck3val);	/* PMIC_STBY_REQ = H */
	upower_pmic_i2c_write(0x1D, 0x20); 	/* BUCK2 DVS: 1.0V (0.6+(0x20*12.5)) = 1.000 */

	return 0;
}

void display_ele_fw_version(void)
{
	u32 fw_version, sha1, res;
	int ret;

	ret = ahab_get_fw_version(&fw_version, &sha1, &res);
	if (ret) {
		printf("ahab get firmware version failed %d, 0x%x\n", ret, res);
	} else {
		printf("ELE firmware version %u.%u.%u-%x",
		       (fw_version & (0x00ff0000)) >> 16,
		       (fw_version & (0x0000ff00)) >> 8,
		       (fw_version & (0x000000ff)), sha1);
		((fw_version & (0x80000000)) >> 31) == 1 ? puts("-dirty\n") : puts("\n");
	}
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

	display_ele_fw_version();

	/* Load the lposc fuse to work around ROM issue,
	 *  The fuse depends on S400 to read.
	 */
	if (is_soc_rev(CHIP_REV_1_0))
		load_lposc_fuse();

	upower_init();

	power_init_board();

	clock_init_late();

	/* This must place after upower init, so access to MDA and MRC are valid */
	/* Init XRDC MDA  */
	xrdc_init_mda();

	/* Init XRDC MRC for VIDEO, DSP domains */
	xrdc_init_mrc();

	xrdc_init_pdac_msc();

	/* DDR initialization */
	spl_dram_init();

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
				printf("Failed to initialize caam_jr: %d\n", ret);
		}
	}

	/*
	 * RNG start only available on the A1 soc revision.
	 * Check some JTAG register for the SoC revision.
	 */
	if (!is_soc_rev(CHIP_REV_1_0)) {
		ret = ahab_start_rng();
		if (ret)
			printf("Fail to start RNG: %d\n", ret);
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
