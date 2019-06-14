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

/* mem write function with mask setting */

#include <common.h>
#include <command.h>
#include <watchdog.h>
#include <asm/io.h>

#define wr_reg(addr, data)	writel(data, addr)
#define rd_reg(addr)	readl(addr)

int do_mem_mw_mask(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char *endp;
	unsigned long reg_add=0;
	unsigned int wr_reg_value=0;
	unsigned int rd_reg_value=0;
	unsigned int wr_reg_and_mask_1=0xffffffff;
	if (argc < 4) {
		//cmd_usage(cmdtp);
		return -1;
	}
	reg_add = simple_strtoul(argv[1], &endp, 10);
	wr_reg_value = simple_strtoul(argv[2], &endp, 10);
	wr_reg_and_mask_1 = simple_strtoul(argv[3], &endp, 10);
	rd_reg_value = (rd_reg(reg_add));
	wr_reg(reg_add,(rd_reg_value&wr_reg_and_mask_1)|(wr_reg_value&(~wr_reg_and_mask_1)) );
	printf("mwm 0x%lx:[0x%8x] with [0x%8x] ok\n", reg_add, rd_reg_value, rd_reg(reg_add));
	return 0;
}

U_BOOT_CMD(
	mwm,	30,	1,	do_mem_mw_mask,
	"mw mask function",
	"[reg] [value] [mask]\n"
	"    //write [reg] with [value], skip [mask] bits\n"
	"eg: mwm 0x01000000 0xaaaabbbb 0x0000ffff\n"
	"    then read 0x01000000 = 0xaaaaXXXX //skip mask bits\n"
);