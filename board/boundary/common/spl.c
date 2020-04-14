/*
 * Copyright 2020 Boundary Devices
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/ddr.h>
#include "bd_common.h"

void spl_dram_init(void)
{
	/* ddr train */
	ddr_init(&dram_timing);
}
