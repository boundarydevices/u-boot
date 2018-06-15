/*
 * Copyright (C) 2011-2014 Panasonic Corporation
 * Copyright (C) 2015-2016 Socionext Inc.
 *   Author: Masahiro Yamada <yamada.masahiro@socionext.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <linux/io.h>

#include "../init.h"
#include "bcu-regs.h"

#define ch(x) ((x) >= 32 ? 0 : (x) < 0 ? 0x11111111 : 0x11111111 << (x))

void uniphier_sld3_bcu_init(const struct uniphier_board_data *bd)
{
	int shift;

	writel(0x11111111, BCSCR2); /* 0x80000000-0x9fffffff: IPPC/IPPD-bus */
	writel(0x11111111, BCSCR3); /* 0xa0000000-0xbfffffff: IPPC/IPPD-bus */
	writel(0x11111111, BCSCR4); /* 0xc0000000-0xdfffffff: IPPC/IPPD-bus */
	/*
	 * 0xe0000000-0xefffffff: Ex-bus
	 * 0xf0000000-0xfbffffff: ASM bus
	 * 0xfc000000-0xffffffff: OCM bus
	 */
	writel(0x24440000, BCSCR5);

	/* Specify DDR channel */
	shift = bd->dram_ch[0].size / 0x04000000 * 4;
	writel(ch(shift), BCIPPCCHR2); /* 0x80000000-0x9fffffff */

	shift -= 32;
	writel(ch(shift), BCIPPCCHR3); /* 0xa0000000-0xbfffffff */

	shift -= 32;
	writel(ch(shift), BCIPPCCHR4); /* 0xc0000000-0xdfffffff */
}
