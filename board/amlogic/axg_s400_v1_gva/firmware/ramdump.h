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

#ifndef __RAM_DUMP_H__
#define __RAM_DUMP_H__

#include <config.h>
#ifdef CONFIG_MDUMP_COMPRESS
#define CONFIG_COMPRESS_SECTION		4

#if CONFIG_COMPRESS_SECTION > 8
#error ---> CONFIG_COMPRESS_SECTION out of range, max should be 8
#endif
/*
 * Full Memory lay out for RAM compress:
 *
 *              DDR_TOP -> +--------+
 *                         |        |
 *                         |        |
 *                         |   4    |
 *                         |        |
 *                         |        |
 *                         |~~~~~~~~| <- store compressing data
 *                         |~~~~~~~~|
 *                         |~~~~~~~~|
 *                         |~~~~~~~~|
 *                         |~~~~~~~~|
 *                         |~~~~~~~~|
 *                         |~~~~~~~~|
 *                         |~~~~~~~~|
 *                         |~~~~~~~~|
 *      COMPRESSED_DATA -> +--------+
 *                         |        |
 *                         |   3    |
 *                         |        |
 *       BL2_IGNORE_END -> +--------+ -- IGNORE_SIZE
 *                         ||||||||||
 *                         ||||2|||||
 *                         ||||||||||
 *      BL2_IGNORE_ADDR -> +--------+
 *                         |        |
 *                         |   1    |
 *                         |        |
 *  COMPRESS_START_ADDR -> +--------+
 */
#define CONFIG_DDR_TOTAL_SIZE		(CONFIG_DDR_SIZE << 20)
#define CONFIG_COMPRESSED_DATA_ADDR	(0x08000000)
#define CONFIG_COMPRESSED_DATA_ADDR1	(0x08000000)

#define CONFIG_COMPRESS_START_ADDR	(0x00000000)
#define CONFIG_BL2_IGNORE_ADDR		(0x05000000)
#define CONFIG_BL2_IGNORE_SIZE		(0x00300000)
#define CONFIG_SEG4_ADDR		(CONFIG_BL2_IGNORE_ADDR + \
					 CONFIG_BL2_IGNORE_SIZE)

enum {
	RAM_COMPRESS_NORMAL = 1,
	RAM_COMPRESS_COPY   = 2,
	RAM_COMPRESS_SET    = 3		/* set ram content to same vale */
};

struct ram_compress_section {
	void *phy_addr;
	unsigned int section_size;
	unsigned int section_index :  8;
	unsigned int compress_type :  8;
	unsigned int set_value     : 16;
};

struct ram_compress_full {
	void *store_phy_addr;
	unsigned int full_memsize;
	unsigned int section_count;
	struct ram_compress_section sections[CONFIG_COMPRESS_SECTION];
};

#endif
#endif /* __RAM_DUMP_H__ */
