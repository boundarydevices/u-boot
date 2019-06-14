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
#include <amlogic/storage_if.h>

#ifndef PAGE_SIZE
#define PAGE_SIZE 4096
#endif

unsigned char w_buf[PAGE_SIZE];
extern int has_instaboot_part(void);
static int do_wipeisb(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int rc = 0;
	u64 partSz = 0;

	if (!has_instaboot_part())
		return 0;
	rc = store_get_partititon_size((unsigned char*)"instaboot", &partSz);
	if (rc || !partSz) {
	    printf("can not get instaboot part size\n");
	    return -1;
	}

	memset(w_buf, 0, PAGE_SIZE);
	rc = store_write_ops((unsigned char*)"instaboot",
		w_buf, 0, PAGE_SIZE);
	if (rc) {
	    printf("wipe instaboot header error\n");
	    return -1;
	}
	return 0;
}

U_BOOT_CMD(
   wipeisb,         //command name
   1,               //maxargs
   0,               //repeatable
   do_wipeisb,   //command function
   "wipeisb",
   "wipe the insaboot image header"
);

