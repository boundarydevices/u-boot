// SPDX-License-Identifier: GPL-2.0+
/*
 * Board specific initialization for Carbon AM62 OSM module
 *
 * Copyright (C) 2024 Ezurio
 *
 */

#include <cpu_func.h>
#include <display_options.h>
#include <env.h>
#include <spl.h>
#include <init.h>
#include <video.h>
#include <splash.h>
#include <k3-ddrss.h>
#include <fdt_support.h>
#include <fdt_simplefb.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <dm/uclass.h>
#include <power/regulator.h>

DECLARE_GLOBAL_DATA_PTR;

#if CONFIG_IS_ENABLED(SPLASH_SCREEN)
static struct splash_location default_splash_locations[] = {
	{
		.name		= "mmc",
		.storage	= SPLASH_STORAGE_MMC,
		.flags		= SPLASH_STORAGE_FS,
		.devpart	= "1:1",
	},
};

int splash_screen_prepare(void)
{
	return splash_source_load(default_splash_locations,
				ARRAY_SIZE(default_splash_locations));
}
#endif

int board_init(void)
{
	struct udevice *regulator;
	int ret;

	// Ensure that mmc1 is selected to the SDCARD or M.2 device correctly.
	// U-boot does not support vin-supply so we have to manually enable this regulator.
	// See this regulator in the device-tree for selection.
	ret = regulator_get_by_platname("vdd_mmc1_sel", &regulator);
	if (ret) {
		debug("%s vdd_mmc1_sel init fail! ret %d\n", __func__, ret);
		goto out;
	}

	ret = regulator_set_enable(regulator, true);
	if (ret)
		debug("%s vdd_mmc1_sel set fail! ret %d\n", __func__, ret);
out:

	return 0;
}

int dram_init(void)
{
	gd->ram_size = get_ram_size((long *)CFG_SYS_SDRAM_BASE, SZ_2G);

#if defined(CONFIG_TARGET_CARBON_AM62_R5)
	puts("DDR RAM detected Size: ");
	print_size(gd->ram_size, "\n");
#endif

	return 0;
}

#if CONFIG_IS_ENABLED(BOARD_LATE_INIT)
int board_late_init(void)
{

	// ti_set_fdt_env(NULL, NULL);
	return 0;
}
#endif

#if defined(CONFIG_SPL_BUILD)
void spl_board_init(void)
{
	u32 val;

	/* We have 32k crystal, so lets enable it */
	val = readl(MCU_CTRL_LFXOSC_CTRL);
	val &= ~(MCU_CTRL_LFXOSC_32K_DISABLE_VAL);
	writel(val, MCU_CTRL_LFXOSC_CTRL);
	/* Add any TRIM needed for the crystal here.. */
	/* Make sure to mux up to take the SoC 32k from the crystal */
	writel(MCU_CTRL_DEVICE_CLKOUT_LFOSC_SELECT_VAL,
	       MCU_CTRL_DEVICE_CLKOUT_32K_CTRL);

	enable_caches();
	if (IS_ENABLED(CONFIG_SPL_SPLASH_SCREEN) && IS_ENABLED(CONFIG_SPL_BMP))
		splash_display();
}
#endif

#if defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, struct bd_info *bd)
{
	int ret = -1;

	if (IS_ENABLED(CONFIG_FDT_SIMPLEFB))
		ret = fdt_simplefb_enable_and_mem_rsv(blob);

	/* If simplefb is not enabled and video is active, then at least reserve
	 * the framebuffer region to preserve the splash screen while OS is booting
	 */
	if (IS_ENABLED(CONFIG_VIDEO) && IS_ENABLED(CONFIG_OF_LIBFDT)) {
		if (ret && video_is_active())
			return fdt_add_fb_mem_rsv(blob);
	}

	return 0;
}
#endif
