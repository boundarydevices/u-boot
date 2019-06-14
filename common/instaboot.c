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
#include <amlogic/instaboot.h>
#include <partition_table.h>

#ifndef PAGE_SIZE
#define PAGE_SIZE 4096
#endif
struct instaboot_header {
	char reserved[PAGE_SIZE - 10 -
		sizeof(struct new_utsname) - sizeof(u32)];
	struct new_utsname uts;
	u32 version_code;
	char	sig[10];
} __attribute__((packed));

static struct instaboot_header ib_header;

int has_instaboot_part(void)
{
	struct partitions *part_table = NULL;
	int i, name_len, ret = 0;
	int part_num = get_partitions_table(&part_table);
	name_len = strlen("instaboot");
	for (i = 0; i < part_num; i++) {
		if (!strncmp(part_table[i].name, "instaboot", name_len)) {
			ret = 1;
			break;
		}
	}
	return ret;
}
int get_instaboot_header(struct instaboot_info* ib_info)
{
	int rc = 0;
	u64 partSz = 0;

	if (!has_instaboot_part())
		return -1;
	rc = store_get_partititon_size((unsigned char*)"instaboot", &partSz);
	if (rc || !partSz) {
	    //printf("can not get instaboot part size\n");
	    return -1;
	}
	rc = store_read_ops((unsigned char*)"instaboot",
		(unsigned char*)&ib_header, 0, PAGE_SIZE);
	if (rc) {
	    printf("read instaboo header error\n");
	    return -1;
	}
	if (!strncmp(ib_header.sig, INSTABOOT_SIG, strlen(INSTABOOT_SIG))) {
		memcpy(&ib_info->uts, &ib_header.uts, sizeof(struct new_utsname));
		ib_info->version_code = ib_header.version_code;
	} else {
		printf("signature: %s\n", ib_header.sig);
		return -1;
	}

	return 0;
}