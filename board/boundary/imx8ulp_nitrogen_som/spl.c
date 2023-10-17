// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 Boundary Devices
 */

#include <common.h>
#include <init.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/imx8ulp-pins.h>
#include <asm/mach-imx/ele_api.h>
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

DECLARE_GLOBAL_DATA_PTR;
extern struct dram_timing_info2 dram_timing_ch2;

void spl_dram_init(void)
{
	/* Reboot in dual boot setting no need to init ddr again */
	bool ddr_enable = pcc_clock_is_enable(5, LPDDR4_PCC5_SLOT);

	if (!ddr_enable) {
		init_clk_ddr();
		ddr_init(&dram_timing_ch2);
	} else {
		/* reinit pfd/pfddiv and lpavnic except pll4*/
		cgc2_pll4_init(false);
	}
}

u32 spl_boot_device(void)
{
	return BOOT_DEVICE_BOOTROM;
}

#define PMIC_I2C_PAD_CTRL	(PAD_CTL_PUS_UP | PAD_CTL_SRE_SLOW | PAD_CTL_ODE)
#define PMIC_MODE_PAD_CTRL	(PAD_CTL_PUS_UP)

static iomux_cfg_t const pmic_pads[] = {
	IMX8ULP_PAD_PTB7__PMIC0_MODE2 | MUX_PAD_CTRL(PMIC_MODE_PAD_CTRL),
	IMX8ULP_PAD_PTB8__PMIC0_MODE1 | MUX_PAD_CTRL(PMIC_MODE_PAD_CTRL),
	IMX8ULP_PAD_PTB9__PMIC0_MODE0 | MUX_PAD_CTRL(PMIC_MODE_PAD_CTRL),
	IMX8ULP_PAD_PTB11__PMIC0_SCL | MUX_PAD_CTRL(PMIC_I2C_PAD_CTRL),
	IMX8ULP_PAD_PTB10__PMIC0_SDA | MUX_PAD_CTRL(PMIC_I2C_PAD_CTRL),
};

void setup_iomux_pmic(void)
{
	imx8ulp_iomux_setup_multiple_pads(pmic_pads, ARRAY_SIZE(pmic_pads));
}

int power_init_board(void)
{
	/* Set buck2 ramp-up speed 1us */
	upower_pmic_i2c_write(0x14, 0x39);
	/* Set buck3 ramp-up speed 1us */
	upower_pmic_i2c_write(0x21, 0x39);
	/* Set buck3out min limit 0.625v */
	upower_pmic_i2c_write(0x2d, 0x2);

	if (IS_ENABLED(CONFIG_IMX8ULP_ND_MODE)) {
		/* Set buck3 to 1.0v ND */
		upower_pmic_i2c_write(0x22, 0x20);
	} else {
		/* Set buck3 to 1.1v OD */
		upower_pmic_i2c_write(0x22, 0x28);
	}

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
		       (fw_version & (0x0000fff0)) >> 4,
		       (fw_version & (0x0000000f)), sha1);
		((fw_version & (0x80000000)) >> 31) == 1 ? puts("-dirty\n") : puts("\n");
	}
}

void spl_board_init(void)
{
	u32 res;
	int ret;
	struct udevice *dev;

	ret = imx8ulp_dm_post_init();
	if (ret)
		return;

	board_early_init_f();

	preloader_console_init();

	puts("Normal Boot\n");

	display_ele_fw_version();

	/* Set iomuxc0 for pmic when m33 is not booted */
	if (!m33_image_booted())
		setup_iomux_pmic();

	/* Load the lposc fuse to work around ROM issue. The fuse depends on S400 to read. */
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
		if (((res >> 8) & 0xff) == ELE_NON_SECURE_STATE_FAILURE_IND)
			printf("Warning: CAAM is in non-secure state, 0x%x\n", res);

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
