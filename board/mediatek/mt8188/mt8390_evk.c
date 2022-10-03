// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 MediaTek Inc.
 * Author: Chris-QJ Chen <chris-qj.chen@mediatek.com>
 */

#include <common.h>
#include <dm.h>
#include <net.h>
#include <asm/io.h>
int board_init(void)
{
	struct udevice *dev;
	int ret;

	if (CONFIG_IS_ENABLED(USB_GADGET)) {
		ret = uclass_get_device(UCLASS_USB_GADGET_GENERIC, 0, &dev);
		if (ret) {
			pr_err("%s: Cannot find USB device\n", __func__);
			return ret;
		}
	}

	if (CONFIG_IS_ENABLED(USB_ETHER))
		usb_ether_init();

	printf("Disabling WDT\n");
	writel(0, 0x10007000);

	printf("Enabling SCP SRAM\n");
	for (unsigned int val = 0xFFFFFFFF; val != 0U;) {
		val = val >> 1;
		writel(val, 0x1072102C);
	}

	return 0;
}
