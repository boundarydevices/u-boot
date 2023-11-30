// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <dm/uclass.h>
#include <env.h>
#include <g_dnl.h>
#include <fastboot.h>
#include <mmc.h>
#include <usb.h>
#include <wdt.h>

static int do_fastboot_usb(uintptr_t buf_addr, size_t buf_size)
{
	struct udevice *wdt = NULL;
	struct udevice *udc;
	int ret;

	uclass_first_device(UCLASS_WDT, &wdt);
	if (!wdt) {
		printf("Cannot load Watchdog driver\n");
		return -1;
	}

	ret = udc_device_get_by_index(0, &udc);
	if (ret) {
		printf("USB init failed: %d\n", ret);
		return -1;
	}

	g_dnl_clear_detach();
	ret = g_dnl_register("usb_dnl_fastboot");
	if (ret)
		return ret;

	if (!g_dnl_board_usb_cable_connected()) {
		printf("USB cable not detected\n");
		ret = -1;
		goto exit;
	}

	while (1) {
		if (g_dnl_detach())
			break;

		wdt_reset(wdt);
		dm_usb_gadget_handle_interrupts(udc);
	}

exit:
	g_dnl_unregister();
	g_dnl_clear_detach();
	udc_device_put(udc);

	return 0;
}

static int mmc_boot_prepare(void)
{
	struct udevice *dev = NULL;
	struct mmc *mmc;
	int ret;

	uclass_first_device(UCLASS_MMC, &dev);
	if (!dev) {
		printf("Cannot load MMC driver\n");
		return -1;
	}

	mmc = find_mmc_device(CONFIG_FASTBOOT_FLASH_MMC_DEV);
	if (!mmc) {
		printf("No MMC device found\n");
		return -1;
	}

	ret = mmc_init(mmc);
	if (ret) {
		printf("MMC init failed\n");
		return -1;
	}

	/* enable BOOT1 partition */
	ret = mmc_set_part_conf(mmc, 1, 1, 0);
	if (ret)
		printf("Cannot set partition config\n");

	/*
	 * BOOT_BUS_WIDTH
	 * 0 : x1(sdr) buswidth
	 *
	 * RESET_BOOT_BUS_CONDITIONS
	 * 0 : Reset bus width to x1, SDR, Backward compatible
	 *
	 * BOOT_MODE
	 * 0 : Use SDR + Backward compatible timing in boot operation
	 */
	ret = mmc_set_boot_bus_width(mmc, 0, 0, 0);
	if (ret)
		printf("Cannot set boot bus witdth\n");

	/* enable reset function */
	if (mmc->ext_csd[EXT_CSD_RST_N_FUNCTION] == 0) {
		ret = mmc_set_rst_n_function(mmc, 1);
		if (ret)
			printf("Cannot disable rst function config\n");
	}

	return 0;
}

void spl_board_init(void)
{
	uintptr_t buf_addr = (uintptr_t)NULL;
	size_t buf_size = 0;

	if (mmc_boot_prepare())
		printf("MMC boot prepare failed\n");

	/* always return slot a */
	env_set_default(NULL, 0);
	if (env_set("current_slot", "a"))
		printf("Cannot set current_slot env\n");

	fastboot_init(NULL, 0);

	printf("Waiting fastboot commands ...\n");
	do_fastboot_usb(buf_addr, buf_size);
}
