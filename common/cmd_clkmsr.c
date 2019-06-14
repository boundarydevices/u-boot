/*
* Copyright (C) 2017 Amlogic, Inc. All rights reserved.
* *
This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* *
This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
* *
You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
* *
Description:
*/

#include <common.h>
#include <command.h>
#include <asm/cpu_id.h>
#include <asm/arch/io.h>
#include <asm/arch/regs.h>
#include <asm/arch/clock.h>

static int do_clkmsr(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int index = 0xff;

	if (argc ==  2)
		index = simple_strtoul(argv[1], NULL, 10);

	clk_msr(index);

	return 0;
}

static int do_ringmsr(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int index = 0xff;

	if (argc < 2)
		return CMD_RET_USAGE;

	if (argc ==  2)
		index = simple_strtoul(argv[1], NULL, 10);
#ifdef CONFIG_RING
	ring_msr(index);
#else
	printf("error: this prj not support get ring %x\n", index);
#endif
	return 0;
}

static char ringmsr_help_text[] =
	"ringmsr x\n"
	"  - for get chip ring info\n"
	"  - x:   \n"
	"  G12A/G12B/TL1:\n"
	"  0x0: print all ring info and set voltage\n"
	"  0x1-0x7: get efuse corner info\n"
	"  0xff: print all ring info and no set voltage\n"
	"  0xfe: print all efuse corner info\n";

U_BOOT_CMD(
		clkmsr, 2, 1, do_clkmsr,
		"Amlogic measure clock",
		"	- measure PLL clock.\n"
		"\n"
		"clkmsr [index]"
		"\n"
);

U_BOOT_CMD(
		ringmsr, 2, 1, do_ringmsr,
		"Amlogic measure ring",
		ringmsr_help_text
);
