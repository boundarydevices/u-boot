/*
 * Copyright (C) 2016-2017 Socionext Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <linux/io.h>

#include "../init.h"
#include "sbc-regs.h"

void uniphier_pxs2_sbc_init(void)
{
	uniphier_sbc_init_savepin();

	/* necessary for ROM boot ?? */
	/* system bus output enable */
	writel(0x17, PC0CTRL);
}
