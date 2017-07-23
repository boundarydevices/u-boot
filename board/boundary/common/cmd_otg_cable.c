/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <asm/arch/crm_regs.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <i2c.h>
#include <linux/delay.h>
#include "bd_common.h"

#define ANADIG_USB1_CHRG_DETECT_CHK_CONTACT	BIT(18)
#define ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B	BIT(19)
#define ANADIG_USB1_CHRG_DETECT_EN_B		BIT(20)

#define ANADIG_USB1_CHRG_DET_STAT_PLUG_CONTACT	BIT(0)
#define ANADIG_USB1_CHRG_DET_STAT_CHRG_DETECTED	BIT(1)

int otg_power_detect(void)
{
	int ret = 0;
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u32 val;
	int i = 0;

	/* turn on comparator, change threshold to 4.6V*/
	writel(BIT(20) | 6, &mxc_ccm->usb1_vbus_detect_set);
	writel(1, &mxc_ccm->usb1_vbus_detect_clr);

	/* Enable charger detect, contact detect */
	writel(ANADIG_USB1_CHRG_DETECT_EN_B, &mxc_ccm->usb1_chrg_detect_clr);
	writel(ANADIG_USB1_CHRG_DETECT_CHK_CONTACT |
		ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,
		&mxc_ccm->usb1_chrg_detect_set);

	/* determine type of cable */
	/* Check if plug is connected */
	while (1) {
		val = readl(&mxc_ccm->usb1_vbus_det_stat);
		if (!(val & 0x0e))
			break;
		if (val & 0x8) {
			ret = 1;
			break;
		}
		val = readl(&mxc_ccm->usb1_chrg_det_stat);
		if (!(val & ANADIG_USB1_CHRG_DET_STAT_PLUG_CONTACT))
			break;
		i++;
		if (i >= 10) {
			ret = 1;
			break;
		}
		udelay(5000);
	}
	/* Disable charger detect */
	writel(ANADIG_USB1_CHRG_DETECT_EN_B |
		ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,
		&mxc_ccm->usb1_chrg_detect_set);
	return ret;
}

static int do_otg_cable(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	return otg_power_detect() ? 0 : 1;
}

U_BOOT_CMD(
	otg_cable, 1, 1, do_otg_cable,
	"Tests for otg cable connected",
	"Returns 0 (true) to shell if cable is connected."
);
