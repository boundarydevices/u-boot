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

#define FUNCID_UPDATE_TEE_TRACE_LEVEL        0xb2000016

#define __asmeq(x, y) ".ifnc " x "," y " ; .err ; .endif\n\t"

static int atoi(const char* str)
{
	int ret = 0;

	if (!str)
		return ret;

	while ( *str != '\0') {
		if (('0' <= *str) && (*str <= '9')) {
			ret = ret * 10 + (*str++ - '0');
		} else {
			ret = 0;
			break;
		}
	}

	return ret;
}

static uint32_t update_log_level(uint32_t log_level)
{
	register uint32_t x0 asm("x0") = FUNCID_UPDATE_TEE_TRACE_LEVEL;
	register uint32_t x1 asm("x1") = log_level;
	do {
			asm volatile(
				__asmeq("%0", "x0")
				__asmeq("%1", "x0")
				__asmeq("%2", "x1")
				"smc    #0\n"
				: "=r"(x0)
				: "r"(x0), "r"(x1));
	} while (0);

	return x0;
}

int exec(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	uint32_t log_level = 0;
	uint32_t updated_log_level = 0;

	if (argc != 2) {
		printf("BL33: parameter error,\n"
				"Usage:\ntee_log_level [log_level],\n"
				"the min log level is 1, and the max log level is related to"
				" Secure OS version\n");
		return 0;
	}

	log_level = atoi(argv[1]);
	updated_log_level = update_log_level(log_level);
	if (log_level < updated_log_level) {
		printf("BL33: the min log level is %d\n", updated_log_level);
	} else if (log_level > updated_log_level) {
		printf("BL33: the max log level is %d\n", updated_log_level);
	}
	printf("BL33: the updated log level is %d\n", updated_log_level);

	return 0;
}

/* -------------------------------------------------------------------- */
U_BOOT_CMD(
		tee_log_level, CONFIG_SYS_MAXARGS, 0, exec,
		"update tee log level",
		"[log_level],\nthe min log level is 1, and the max log level is"
		" related to Secure OS version"
);
